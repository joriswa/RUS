#!/usr/bin/env python3
"""
Enhanced Parameter Tuning System for STOMP and Hauser Algorithms
================================================================

This module provides advanced parameter optimization capabilities using proven libraries:
- scikit-optimize for Bayesian optimization
- Optuna for hyperparameter optimization
- Ray Tune for distributed optimization
- Hyperopt for TPE optimization

The system automatically tunes parameters for both STOMP and Hauser algorithms
to optimize trajectory planning performance across multiple scenarios.

Dependencies:
- scikit-optimize
- optuna
- ray[tune]
- hyperopt
- pandas
- numpy
- matplotlib
- seaborn

Author: PathPlanner Team
Date: December 2024
"""

import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import seaborn as sns
import json
import yaml
import subprocess
import tempfile
import os
import time
from pathlib import Path
from typing import Dict, List, Tuple, Optional, Union, Callable
import logging
from concurrent.futures import ThreadPoolExecutor, ProcessPoolExecutor
import warnings

# Optimization libraries
try:
    from skopt import gp_minimize, forest_minimize, gbrt_minimize
    from skopt.space import Real, Integer, Categorical
    from skopt.utils import use_named_args
    from skopt.plots import plot_convergence, plot_objective, plot_evaluations
    SKOPT_AVAILABLE = True
except ImportError:
    SKOPT_AVAILABLE = False
    print("Warning: scikit-optimize not available. Install with: pip install scikit-optimize")

try:
    import optuna
    from optuna.samplers import TPESampler, CmaEsSampler
    from optuna.pruners import MedianPruner, HyperbandPruner
    OPTUNA_AVAILABLE = True
except ImportError:
    OPTUNA_AVAILABLE = False
    print("Warning: Optuna not available. Install with: pip install optuna")

try:
    import ray
    from ray import tune
    from ray.tune.schedulers import ASHAScheduler, HyperBandScheduler
    from ray.tune.suggest.skopt import SkOptSearch
    from ray.tune.suggest.optuna import OptunaSearch
    RAY_AVAILABLE = True
except ImportError:
    RAY_AVAILABLE = False
    print("Warning: Ray Tune not available. Install with: pip install ray[tune]")

try:
    from hyperopt import fmin, tpe, hp, STATUS_OK, Trials
    HYPEROPT_AVAILABLE = True
except ImportError:
    HYPEROPT_AVAILABLE = False
    print("Warning: Hyperopt not available. Install with: pip install hyperopt")

# Setup logging
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')
logger = logging.getLogger(__name__)

class ParameterSpace:
    """Defines parameter spaces for optimization algorithms."""
    
    @staticmethod
    def get_stomp_space():
        """Get STOMP parameter space definition."""
        if SKOPT_AVAILABLE:
            return [
                Real(0.001, 0.5, name='exploration_constant', prior='log-uniform'),
                Integer(5, 100, name='num_noisy_trajectories'),
                Integer(2, 20, name='num_best_samples'),
                Integer(20, 1000, name='max_iterations'),
                Real(0.1, 0.8, name='learning_rate'),
                Real(1.0, 50.0, name='temperature'),
                Real(0.01, 0.2, name='dt'),
                Categorical([True, False], name='adaptive_sampling'),
                Categorical([True, False], name='early_termination')
            ]
        else:
            # Return dict format for other optimizers
            return {
                'exploration_constant': (0.001, 0.5, 'log-uniform'),
                'num_noisy_trajectories': (5, 100, 'int'),
                'num_best_samples': (2, 20, 'int'),
                'max_iterations': (20, 1000, 'int'),
                'learning_rate': (0.1, 0.8, 'uniform'),
                'temperature': (1.0, 50.0, 'uniform'),
                'dt': (0.01, 0.2, 'uniform'),
                'adaptive_sampling': [True, False],
                'early_termination': [True, False]
            }
    
    @staticmethod
    def get_hauser_space():
        """Get Hauser/RRT parameter space definition."""
        if SKOPT_AVAILABLE:
            return [
                Categorical(['RRT', 'RRTConnect', 'RRTStar', 'InformedRRTStar'], name='algorithm'),
                Real(0.01, 0.5, name='step_size'),
                Real(0.01, 0.8, name='goal_bias'),
                Integer(100, 5000, name='max_iterations'),
                Integer(10, 500, name='shortcut_iterations'),
                Real(0.0001, 0.01, name='tolerance_threshold'),
                Categorical([True, False], name='adaptive_shortcutting'),
                Real(0.1, 2.0, name='max_velocity_near_patient'),
                Categorical([True, False], name='enforce_gentle_motion'),
                # Simulated Annealing IK parameters
                Real(100.0, 5000.0, name='sa_t_max'),
                Real(0.01, 10.0, name='sa_t_min'),
                Real(0.8, 0.99, name='sa_cooling_rate'),
                Integer(100, 2000, name='sa_max_iterations')
            ]
        else:
            return {
                'algorithm': ['RRT', 'RRTConnect', 'RRTStar', 'InformedRRTStar'],
                'step_size': (0.01, 0.5, 'uniform'),
                'goal_bias': (0.01, 0.8, 'uniform'),
                'max_iterations': (100, 5000, 'int'),
                'shortcut_iterations': (10, 500, 'int'),
                'tolerance_threshold': (0.0001, 0.01, 'log-uniform'),
                'adaptive_shortcutting': [True, False],
                'max_velocity_near_patient': (0.1, 2.0, 'uniform'),
                'enforce_gentle_motion': [True, False],
                # Simulated Annealing IK parameters
                'sa_t_max': (100.0, 5000.0, 'uniform'),
                'sa_t_min': (0.01, 10.0, 'uniform'),
                'sa_cooling_rate': (0.8, 0.99, 'uniform'),
                'sa_max_iterations': (100, 2000, 'int')
            }

class TrajectoryEvaluator:
    """Evaluates trajectory planning algorithms with given parameters."""
    
    def __init__(self, cpp_executable_path: str, scenario_configs: List[Dict]):
        """
        Initialize evaluator.
        
        Args:
            cpp_executable_path: Path to compiled C++ parameter tuning executable
            scenario_configs: List of test scenario configurations
        """
        self.cpp_executable = Path(cpp_executable_path)
        self.scenario_configs = scenario_configs
        self.temp_dir = Path(tempfile.mkdtemp())
        
        # Validate scenario_1 resources
        scenario_1_dir = Path(__file__).parent.parent.parent / "res" / "scenario_1"
        self.obstacles_file = scenario_1_dir / "obstacles.xml"
        self.urdf_file = scenario_1_dir / "panda_US.urdf"
        self.poses_file = scenario_1_dir / "scan_poses.csv"
        
        if not self.cpp_executable.exists():
            raise FileNotFoundError(f"C++ executable not found: {cpp_executable_path}")
        
        if not self.obstacles_file.exists():
            logger.warning(f"Obstacles file not found: {self.obstacles_file}")
        if not self.urdf_file.exists():
            logger.warning(f"URDF file not found: {self.urdf_file}")
        if not self.poses_file.exists():
            logger.warning(f"Poses file not found: {self.poses_file}")
    
    def evaluate_parameters(self, algorithm: str, parameters: Dict) -> Dict:
        """
        Evaluate algorithm with given parameters.
        
        Args:
            algorithm: 'STOMP' or 'Hauser'
            parameters: Parameter dictionary
            
        Returns:
            Evaluation metrics dictionary
        """
        # Create parameter configuration file
        config_file = self.temp_dir / f"config_{algorithm}_{int(time.time() * 1000000)}.yaml"
        
        config_data = {
            'algorithm': algorithm,
            'parameters': parameters,
            'scenarios': [],
            'evaluation_settings': {
                'num_runs_per_scenario': 5,
                'timeout_seconds': 30,
                'output_trajectories': False
            },
            'resources': {
                'obstacles_file': str(self.obstacles_file),
                'urdf_file': str(self.urdf_file),
                'poses_file': str(self.poses_file)
            }
        }
        
        # Process scenarios to ensure proper YAML format
        for scenario in self.scenario_configs:
            scenario_data = {
                'name': scenario['name'],
                'description': scenario['description'],
                'difficulty': scenario['difficulty'],
                'start_config': scenario['start_config'],
                'environment': scenario['environment'],
                'urdf': scenario['urdf']
            }
            
            # Add target poses if they exist
            if 'target_poses' in scenario and scenario['target_poses']:
                scenario_data['target_poses'] = []
                for pose in scenario['target_poses']:
                    pose_data = {
                        'position': pose['position'],
                        'orientation': pose['orientation'],
                        'contact': pose['contact'],
                        'distance': pose['distance'],
                        'index': pose['index']
                    }
                    scenario_data['target_poses'].append(pose_data)
            
            config_data['scenarios'].append(scenario_data)
        
        with open(config_file, 'w') as f:
            yaml.dump(config_data, f)
        
        try:
            # Run C++ evaluator
            result = subprocess.run([
                str(self.cpp_executable),
                '--config', str(config_file),
                '--output-format', 'json'
            ], capture_output=True, text=True, timeout=300)
            
            if result.returncode != 0:
                logger.error(f"C++ evaluator failed: {result.stderr}")
                # Save failing config for debugging
                debug_file = f"debug_config_{int(time.time() * 1000000)}.yaml"
                config_file.rename(debug_file)
                logger.error(f"Saved failing config to: {debug_file}")
                return self._get_failure_metrics()
            
            # Parse results
            try:
                metrics = json.loads(result.stdout)
                return self._process_metrics(metrics)
            except json.JSONDecodeError:
                logger.error("Failed to parse C++ evaluator output")
                logger.error(f"Output was: {result.stdout}")
                return self._get_failure_metrics()
                
        except subprocess.TimeoutExpired:
            logger.warning("Evaluation timeout")
            return self._get_failure_metrics()
        finally:
            # Cleanup (only if evaluation succeeded)
            if config_file.exists():
                config_file.unlink()
    
    def _process_metrics(self, raw_metrics: Dict) -> Dict:
        """Process raw metrics into standardized format."""
        processed = {
            'success_rate': raw_metrics.get('success_rate', 0.0),
            'avg_planning_time_ms': raw_metrics.get('avg_planning_time_ms', 10000.0),
            'avg_path_length': raw_metrics.get('avg_path_length', float('inf')),
            'avg_smoothness_score': raw_metrics.get('avg_smoothness_score', 0.0),
            'avg_safety_score': raw_metrics.get('avg_safety_score', 0.0),
            'avg_min_clearance': raw_metrics.get('avg_min_clearance', 0.0),
            'consistency_score': raw_metrics.get('consistency_score', 0.0),
            'computational_efficiency': raw_metrics.get('computational_efficiency', 0.0)
        }
        
        # Compute composite objective (lower is better)
        weights = {
            'time_weight': 0.3,
            'success_weight': 0.25,
            'path_weight': 0.2,
            'safety_weight': 0.15,
            'smoothness_weight': 0.1
        }
        
        # Normalize and combine metrics
        time_penalty = min(processed['avg_planning_time_ms'] / 1000.0, 10.0)  # Cap at 10s
        success_bonus = processed['success_rate']
        path_penalty = min(processed['avg_path_length'] / 10.0, 5.0)  # Cap at 5 rad
        safety_bonus = processed['avg_safety_score']
        smoothness_bonus = processed['avg_smoothness_score']
        
        composite_objective = (
            weights['time_weight'] * time_penalty +
            weights['success_weight'] * (1.0 - success_bonus) +
            weights['path_weight'] * path_penalty +
            weights['safety_weight'] * (1.0 - safety_bonus) +
            weights['smoothness_weight'] * (1.0 - smoothness_bonus)
        )
        
        processed['composite_objective'] = composite_objective
        return processed
    
    def _get_failure_metrics(self) -> Dict:
        """Return metrics for failed evaluations."""
        return {
            'success_rate': 0.0,
            'avg_planning_time_ms': 30000.0,  # Timeout penalty
            'avg_path_length': float('inf'),
            'avg_smoothness_score': 0.0,
            'avg_safety_score': 0.0,
            'avg_min_clearance': 0.0,
            'consistency_score': 0.0,
            'computational_efficiency': 0.0,
            'composite_objective': 10.0  # High penalty
        }

class EnhancedParameterOptimizer:
    """Enhanced parameter optimizer using multiple optimization libraries."""
    
    def __init__(self, 
                 cpp_executable_path: str,
                 scenario_configs: List[Dict],
                 output_dir: str = "enhanced_tuning_results"):
        """
        Initialize the enhanced parameter optimizer.
        
        Args:
            cpp_executable_path: Path to C++ parameter evaluation executable
            scenario_configs: List of test scenario configurations
            output_dir: Directory to save results
        """
        self.evaluator = TrajectoryEvaluator(cpp_executable_path, scenario_configs)
        self.output_dir = Path(output_dir)
        self.output_dir.mkdir(exist_ok=True)
        
        # Results storage
        self.optimization_results = {}
        self.evaluation_history = []
        
        logger.info(f"Enhanced Parameter Optimizer initialized")
        logger.info(f"Available optimizers: {self._get_available_optimizers()}")
    
    def _get_available_optimizers(self) -> List[str]:
        """Get list of available optimization libraries."""
        optimizers = []
        if SKOPT_AVAILABLE:
            optimizers.extend(['gaussian_process', 'random_forest', 'gradient_boosting'])
        if OPTUNA_AVAILABLE:
            optimizers.append('optuna_tpe')
        if RAY_AVAILABLE:
            optimizers.append('ray_tune')
        if HYPEROPT_AVAILABLE:
            optimizers.append('hyperopt_tpe')
        return optimizers
    
    def optimize_stomp_parameters(self, 
                                 optimizer: str = 'auto',
                                 n_calls: int = 100,
                                 n_jobs: int = 1) -> Dict:
        """
        Optimize STOMP parameters using specified optimizer.
        
        Args:
            optimizer: Optimization method ('auto', 'gaussian_process', 'optuna_tpe', etc.)
            n_calls: Number of optimization iterations
            n_jobs: Number of parallel jobs
            
        Returns:
            Optimization results dictionary
        """
        logger.info(f"Starting STOMP parameter optimization with {optimizer}")
        
        if optimizer == 'auto':
            optimizer = self._select_best_optimizer()
        
        space = ParameterSpace.get_stomp_space()
        
        if optimizer == 'gaussian_process' and SKOPT_AVAILABLE:
            result = self._optimize_with_skopt(space, 'STOMP', gp_minimize, n_calls, n_jobs)
        elif optimizer == 'random_forest' and SKOPT_AVAILABLE:
            result = self._optimize_with_skopt(space, 'STOMP', forest_minimize, n_calls, n_jobs)
        elif optimizer == 'gradient_boosting' and SKOPT_AVAILABLE:
            result = self._optimize_with_skopt(space, 'STOMP', gbrt_minimize, n_calls, n_jobs)
        elif optimizer == 'optuna_tpe' and OPTUNA_AVAILABLE:
            # Force dict format for optuna by temporarily setting SKOPT_AVAILABLE to False
            original_skopt = globals().get('SKOPT_AVAILABLE', False)
            globals()['SKOPT_AVAILABLE'] = False
            space = ParameterSpace.get_stomp_space()
            globals()['SKOPT_AVAILABLE'] = original_skopt
            result = self._optimize_with_optuna(space, 'STOMP', n_calls)
        elif optimizer == 'ray_tune' and RAY_AVAILABLE:
            result = self._optimize_with_ray(space, 'STOMP', n_calls)
        elif optimizer == 'hyperopt_tpe' and HYPEROPT_AVAILABLE:
            result = self._optimize_with_hyperopt(space, 'STOMP', n_calls)
        else:
            raise ValueError(f"Optimizer {optimizer} not available or not supported")
        
        self.optimization_results['stomp'] = result
        self._save_results('stomp', result)
        return result
    
    def optimize_hauser_parameters(self, 
                                  optimizer: str = 'auto',
                                  n_calls: int = 100,
                                  n_jobs: int = 1) -> Dict:
        """
        Optimize Hauser parameters using specified optimizer.
        
        Args:
            optimizer: Optimization method
            n_calls: Number of optimization iterations
            n_jobs: Number of parallel jobs
            
        Returns:
            Optimization results dictionary
        """
        logger.info(f"Starting Hauser parameter optimization with {optimizer}")
        
        if optimizer == 'auto':
            optimizer = self._select_best_optimizer()
        
        space = ParameterSpace.get_hauser_space()
        
        if optimizer == 'gaussian_process' and SKOPT_AVAILABLE:
            result = self._optimize_with_skopt(space, 'Hauser', gp_minimize, n_calls, n_jobs)
        elif optimizer == 'random_forest' and SKOPT_AVAILABLE:
            result = self._optimize_with_skopt(space, 'Hauser', forest_minimize, n_calls, n_jobs)
        elif optimizer == 'gradient_boosting' and SKOPT_AVAILABLE:
            result = self._optimize_with_skopt(space, 'Hauser', gbrt_minimize, n_calls, n_jobs)
        elif optimizer == 'optuna_tpe' and OPTUNA_AVAILABLE:
            # Force dict format for optuna by temporarily setting SKOPT_AVAILABLE to False
            original_skopt = globals().get('SKOPT_AVAILABLE', False)
            globals()['SKOPT_AVAILABLE'] = False
            space = ParameterSpace.get_hauser_space()
            globals()['SKOPT_AVAILABLE'] = original_skopt
            result = self._optimize_with_optuna(space, 'Hauser', n_calls)
        elif optimizer == 'ray_tune' and RAY_AVAILABLE:
            result = self._optimize_with_ray(space, 'Hauser', n_calls)
        elif optimizer == 'hyperopt_tpe' and HYPEROPT_AVAILABLE:
            result = self._optimize_with_hyperopt(space, 'Hauser', n_calls)
        else:
            raise ValueError(f"Optimizer {optimizer} not available or not supported")
        
        self.optimization_results['hauser'] = result
        self._save_results('hauser', result)
        return result
    
    def _select_best_optimizer(self) -> str:
        """Automatically select the best available optimizer."""
        if OPTUNA_AVAILABLE:
            return 'optuna_tpe'  # Generally most robust
        elif SKOPT_AVAILABLE:
            return 'gaussian_process'  # Good for continuous spaces
        elif RAY_AVAILABLE:
            return 'ray_tune'  # Good for distributed optimization
        elif HYPEROPT_AVAILABLE:
            return 'hyperopt_tpe'
        else:
            raise RuntimeError("No optimization libraries available")
    
    def _optimize_with_skopt(self, space, algorithm: str, minimize_func, n_calls: int, n_jobs: int):
        """Optimize using scikit-optimize."""
        
        @use_named_args(space)
        def objective(**params):
            metrics = self.evaluator.evaluate_parameters(algorithm, params)
            self.evaluation_history.append({
                'algorithm': algorithm,
                'parameters': params,
                'metrics': metrics,
                'objective': metrics['composite_objective']
            })
            return metrics['composite_objective']
        
        result = minimize_func(
            func=objective,
            dimensions=space,
            n_calls=n_calls,
            n_jobs=n_jobs,
            random_state=42,
            acq_func='EI'  # Expected Improvement
        )
        
        best_params = dict(zip([dim.name for dim in space], result.x))
        
        return {
            'optimizer': 'scikit-optimize',
            'best_parameters': best_params,
            'best_objective': float(result.fun),
            'convergence_data': {
                'func_vals': [float(x) for x in result.func_vals],
                'x_iters': result.x_iters
            },
            'n_evaluations': len(result.func_vals)
        }
    
    def _optimize_with_optuna(self, space_dict, algorithm: str, n_trials: int):
        """Optimize using Optuna."""
        
        def objective(trial):
            params = {}
            for param_name, param_config in space_dict.items():
                if isinstance(param_config, list):  # Categorical
                    params[param_name] = trial.suggest_categorical(param_name, param_config)
                elif isinstance(param_config, tuple) and len(param_config) >= 3:
                    if param_config[2] == 'int':
                        params[param_name] = trial.suggest_int(param_name, int(param_config[0]), int(param_config[1]))
                    elif param_config[2] == 'log-uniform':
                        params[param_name] = trial.suggest_float(param_name, param_config[0], param_config[1], log=True)
                    else:  # uniform
                        params[param_name] = trial.suggest_float(param_name, param_config[0], param_config[1])
                elif isinstance(param_config, tuple) and len(param_config) >= 2:
                    params[param_name] = trial.suggest_float(param_name, param_config[0], param_config[1])
                else:
                    # Fallback for unexpected formats
                    params[param_name] = trial.suggest_float(param_name, 0.1, 1.0)
            
            metrics = self.evaluator.evaluate_parameters(algorithm, params)
            self.evaluation_history.append({
                'algorithm': algorithm,
                'parameters': params,
                'metrics': metrics,
                'objective': metrics['composite_objective']
            })
            return metrics['composite_objective']
        
        study = optuna.create_study(
            direction='minimize',
            sampler=TPESampler(seed=42),
            pruner=MedianPruner(n_startup_trials=10, n_warmup_steps=5)
        )
        
        study.optimize(objective, n_trials=n_trials)
        
        return {
            'optimizer': 'optuna',
            'best_parameters': study.best_params,
            'best_objective': float(study.best_value),
            'convergence_data': {
                'values': [float(trial.value) for trial in study.trials if trial.value is not None],
                'trial_numbers': [trial.number for trial in study.trials if trial.value is not None]
            },
            'n_evaluations': len(study.trials)
        }
    
    def _optimize_with_ray(self, space_dict, algorithm: str, num_samples: int):
        """Optimize using Ray Tune."""
        if not RAY_AVAILABLE:
            raise ImportError("Ray Tune not available")
        
        # Initialize Ray
        if not ray.is_initialized():
            ray.init(ignore_reinit_error=True)
        
        # Convert space to Ray Tune format
        config = {}
        for param_name, param_config in space_dict.items():
            if isinstance(param_config, list):  # Categorical
                config[param_name] = tune.choice(param_config)
            elif param_config[2] == 'int':
                config[param_name] = tune.randint(param_config[0], param_config[1] + 1)
            elif param_config[2] == 'log-uniform':
                config[param_name] = tune.loguniform(param_config[0], param_config[1])
            else:  # uniform
                config[param_name] = tune.uniform(param_config[0], param_config[1])
        
        def trainable(config):
            metrics = self.evaluator.evaluate_parameters(algorithm, config)
            self.evaluation_history.append({
                'algorithm': algorithm,
                'parameters': config,
                'metrics': metrics,
                'objective': metrics['composite_objective']
            })
            tune.report(objective=metrics['composite_objective'])
        
        analysis = tune.run(
            trainable,
            config=config,
            num_samples=num_samples,
            scheduler=ASHAScheduler(metric="objective", mode="min"),
            verbose=1
        )
        
        best_trial = analysis.get_best_trial("objective", "min")
        
        return {
            'optimizer': 'ray_tune',
            'best_parameters': best_trial.config,
            'best_objective': float(best_trial.last_result["objective"]),
            'convergence_data': {
                'trial_ids': [trial.trial_id for trial in analysis.trials],
                'objectives': [trial.last_result.get("objective", float('inf')) for trial in analysis.trials]
            },
            'n_evaluations': len(analysis.trials)
        }
    
    def _optimize_with_hyperopt(self, space_dict, algorithm: str, max_evals: int):
        """Optimize using Hyperopt."""
        
        # Convert space to Hyperopt format
        hp_space = {}
        for param_name, param_config in space_dict.items():
            if isinstance(param_config, list):  # Categorical
                hp_space[param_name] = hp.choice(param_name, param_config)
            elif param_config[2] == 'int':
                hp_space[param_name] = hp.quniform(param_name, param_config[0], param_config[1], 1)
            elif param_config[2] == 'log-uniform':
                hp_space[param_name] = hp.lognormal(param_name, 
                                                   np.log(param_config[0]), 
                                                   np.log(param_config[1]/param_config[0]))
            else:  # uniform
                hp_space[param_name] = hp.uniform(param_name, param_config[0], param_config[1])
        
        def objective(params):
            # Convert integer parameters
            for param_name, param_config in space_dict.items():
                if not isinstance(param_config, list) and param_config[2] == 'int':
                    params[param_name] = int(params[param_name])
            
            metrics = self.evaluator.evaluate_parameters(algorithm, params)
            self.evaluation_history.append({
                'algorithm': algorithm,
                'parameters': params,
                'metrics': metrics,
                'objective': metrics['composite_objective']
            })
            return {'loss': metrics['composite_objective'], 'status': STATUS_OK}
        
        trials = Trials()
        best = fmin(fn=objective,
                   space=hp_space,
                   algo=tpe.suggest,
                   max_evals=max_evals,
                   trials=trials,
                   rstate=np.random.RandomState(42))
        
        return {
            'optimizer': 'hyperopt',
            'best_parameters': best,
            'best_objective': float(min(trials.losses())),
            'convergence_data': {
                'losses': [float(loss) for loss in trials.losses()],
                'trial_numbers': list(range(len(trials.losses())))
            },
            'n_evaluations': len(trials.losses())
        }
    
    def compare_optimizers(self, 
                          algorithms: List[str] = ['STOMP', 'Hauser'],
                          optimizers: List[str] = None,
                          n_calls: int = 50) -> Dict:
        """
        Compare different optimizers on the same problem.
        
        Args:
            algorithms: List of algorithms to optimize
            optimizers: List of optimizers to compare (None = all available)
            n_calls: Number of evaluations per optimizer
            
        Returns:
            Comparison results
        """
        if optimizers is None:
            optimizers = self._get_available_optimizers()
        
        comparison_results = {}
        
        for algorithm in algorithms:
            algorithm_results = {}
            
            for optimizer in optimizers:
                try:
                    logger.info(f"Running {optimizer} for {algorithm}")
                    
                    if algorithm == 'STOMP':
                        result = self.optimize_stomp_parameters(optimizer, n_calls, n_jobs=1)
                    else:
                        result = self.optimize_hauser_parameters(optimizer, n_calls, n_jobs=1)
                    
                    algorithm_results[optimizer] = result
                    
                except Exception as e:
                    logger.error(f"Failed to run {optimizer} for {algorithm}: {e}")
                    continue
            
            comparison_results[algorithm] = algorithm_results
        
        # Save comparison results
        comparison_file = self.output_dir / 'optimizer_comparison.json'
        with open(comparison_file, 'w') as f:
            json.dump(comparison_results, f, indent=2)
        
        self._generate_comparison_plots(comparison_results)
        
        return comparison_results
    
    def _save_results(self, algorithm: str, result: Dict):
        """Save optimization results."""
        results_file = self.output_dir / f'{algorithm}_optimization_results.json'
        with open(results_file, 'w') as f:
            json.dump(result, f, indent=2)
        
        # Save evaluation history
        history_file = self.output_dir / f'{algorithm}_evaluation_history.csv'
        algorithm_history = [entry for entry in self.evaluation_history 
                           if entry['algorithm'] == algorithm]
        
        if algorithm_history:
            df_data = []
            for entry in algorithm_history:
                row = {
                    'algorithm': entry['algorithm'],
                    'objective': entry['objective']
                }
                row.update(entry['parameters'])
                row.update(entry['metrics'])
                df_data.append(row)
            
            df = pd.DataFrame(df_data)
            df.to_csv(history_file, index=False)
        
        logger.info(f"Results saved to {results_file}")
    
    def _generate_comparison_plots(self, comparison_results: Dict):
        """Generate comparison visualization plots."""
        fig, axes = plt.subplots(2, 2, figsize=(15, 12))
        fig.suptitle('Optimizer Comparison Results', fontsize=16, fontweight='bold')
        
        for i, (algorithm, results) in enumerate(comparison_results.items()):
            if i >= 2:  # Only plot first 2 algorithms
                break
                
            # Convergence comparison
            ax1 = axes[i, 0]
            for optimizer, result in results.items():
                if 'convergence_data' in result:
                    if 'func_vals' in result['convergence_data']:
                        y_data = result['convergence_data']['func_vals']
                    elif 'values' in result['convergence_data']:
                        y_data = result['convergence_data']['values']
                    elif 'losses' in result['convergence_data']:
                        y_data = result['convergence_data']['losses']
                    else:
                        continue
                    
                    # Compute running minimum
                    running_min = np.minimum.accumulate(y_data)
                    ax1.plot(running_min, label=optimizer, linewidth=2)
            
            ax1.set_xlabel('Evaluation Number')
            ax1.set_ylabel('Best Objective Value')
            ax1.set_title(f'{algorithm} Convergence Comparison')
            ax1.legend()
            ax1.grid(True, alpha=0.3)
            
            # Final performance comparison
            ax2 = axes[i, 1]
            optimizer_names = list(results.keys())
            best_objectives = [results[opt]['best_objective'] for opt in optimizer_names]
            
            bars = ax2.bar(optimizer_names, best_objectives, 
                          color=plt.cm.Set3(np.linspace(0, 1, len(optimizer_names))))
            ax2.set_ylabel('Best Objective Value')
            ax2.set_title(f'{algorithm} Final Performance')
            ax2.tick_params(axis='x', rotation=45)
            
            # Add value labels on bars
            for bar, value in zip(bars, best_objectives):
                ax2.text(bar.get_x() + bar.get_width()/2, bar.get_height() + 0.01,
                        f'{value:.4f}', ha='center', va='bottom')
        
        plt.tight_layout()
        plt.savefig(self.output_dir / 'optimizer_comparison.png', dpi=300, bbox_inches='tight')
        plt.close()
        
        logger.info("Comparison plots saved")
    
    def generate_comprehensive_report(self):
        """Generate comprehensive optimization report."""
        report = {
            'timestamp': time.strftime('%Y-%m-%d %H:%M:%S'),
            'optimization_results': self.optimization_results,
            'total_evaluations': len(self.evaluation_history),
            'available_optimizers': self._get_available_optimizers(),
            'recommendations': self._generate_recommendations()
        }
        
        # Save comprehensive report
        report_file = self.output_dir / 'comprehensive_optimization_report.json'
        with open(report_file, 'w') as f:
            json.dump(report, f, indent=2, default=str)
        
        # Generate markdown report
        self._generate_markdown_report(report)
        
        logger.info(f"Comprehensive report saved to {report_file}")
        return report
    
    def _generate_recommendations(self) -> Dict:
        """Generate optimization recommendations."""
        recommendations = {
            'best_stomp_config': None,
            'best_hauser_config': None,
            'optimizer_recommendations': {},
            'general_guidelines': []
        }
        
        if 'stomp' in self.optimization_results:
            recommendations['best_stomp_config'] = self.optimization_results['stomp']['best_parameters']
        
        if 'hauser' in self.optimization_results:
            recommendations['best_hauser_config'] = self.optimization_results['hauser']['best_parameters']
        
        # Optimizer recommendations based on performance
        available_optimizers = self._get_available_optimizers()
        if 'optuna_tpe' in available_optimizers:
            recommendations['optimizer_recommendations']['general'] = 'optuna_tpe'
        elif 'gaussian_process' in available_optimizers:
            recommendations['optimizer_recommendations']['general'] = 'gaussian_process'
        
        recommendations['general_guidelines'] = [
            "Use Bayesian optimization for expensive function evaluations",
            "Start with 50-100 evaluations for initial exploration",
            "Validate best parameters on independent test scenarios",
            "Consider multi-objective optimization for trade-offs"
        ]
        
        return recommendations
    
    def _generate_markdown_report(self, report: Dict):
        """Generate markdown format report."""
        markdown_content = f"""# Enhanced Parameter Optimization Report

Generated: {report['timestamp']}

## Summary

Total function evaluations: {report['total_evaluations']}
Available optimizers: {', '.join(report['available_optimizers'])}

## Best Configurations

### STOMP
"""
        
        if 'stomp' in self.optimization_results:
            result = self.optimization_results['stomp']
            markdown_content += f"""
- **Optimizer**: {result['optimizer']}
- **Best Objective**: {result['best_objective']:.6f}
- **Evaluations**: {result['n_evaluations']}
- **Best Parameters**:
"""
            for param, value in result['best_parameters'].items():
                markdown_content += f"  - {param}: {value}\n"
        
        if 'hauser' in self.optimization_results:
            result = self.optimization_results['hauser']
            markdown_content += f"""
### Hauser

- **Optimizer**: {result['optimizer']}
- **Best Objective**: {result['best_objective']:.6f}
- **Evaluations**: {result['n_evaluations']}
- **Best Parameters**:
"""
            for param, value in result['best_parameters'].items():
                markdown_content += f"  - {param}: {value}\n"
        
        markdown_content += f"""
## Recommendations

"""
        for guideline in report['recommendations']['general_guidelines']:
            markdown_content += f"- {guideline}\n"
        
        # Save markdown report
        markdown_file = self.output_dir / 'optimization_report.md'
        with open(markdown_file, 'w') as f:
            f.write(markdown_content)

def load_scenario_1_poses():
    """Load poses from scenario_1/scan_poses.csv."""
    import pandas as pd
    poses_file = Path(__file__).parent.parent.parent / "res" / "scenario_1" / "scan_poses.csv"
    
    if not poses_file.exists():
        logger.warning(f"Poses file not found: {poses_file}, using default poses")
        return []
    
    try:
        # Read CSV without header, columns are: x,y,z,qx,qy,qz,qw,contact,distance,index
        df = pd.read_csv(poses_file, header=None)
        poses = []
        
        for _, row in df.iterrows():
            pose_data = {
                'position': [float(row[0]), float(row[1]), float(row[2])],
                'orientation': [float(row[6]), float(row[3]), float(row[4]), float(row[5])],  # w,x,y,z format
                'contact': bool(row[7]),
                'distance': float(row[8]),
                'index': int(row[9])
            }
            poses.append(pose_data)
        
        logger.info(f"Loaded {len(poses)} poses from scenario_1")
        return poses
    
    except Exception as e:
        logger.error(f"Error loading poses: {e}")
        return []

def create_default_scenarios():
    """Create default test scenarios using scenario_1 data."""
    scenario_1_poses = load_scenario_1_poses()
    
    # Default robot start configuration (from existing parameter_tuning_main.cpp)
    start_config = [-0.785398, 0.196349, -0.196349, -1.047197, 0.0, 1.570796, 0.785398]
    
    scenarios = []
    
    if scenario_1_poses:
        # Create scenarios using actual scan poses
        # Group poses for different complexity levels
        num_poses = len(scenario_1_poses)
        
        # Simple scenario: first few poses
        scenarios.append({
            'name': 'scenario_1_simple',
            'description': 'Simple ultrasound scanning motion using first 3 poses',
            'difficulty': 1,
            'start_config': start_config,
            'target_poses': scenario_1_poses[:3],
            'environment': '../../res/scenario_1/obstacles.xml',
            'urdf': '../../res/scenario_1/panda_US.urdf'
        })
        
        # Medium scenario: subset of poses
        mid_poses = scenario_1_poses[:min(8, num_poses)]
        scenarios.append({
            'name': 'scenario_1_medium',
            'description': 'Medium complexity scanning with 8 poses',
            'difficulty': 2,
            'start_config': start_config,
            'target_poses': mid_poses,
            'environment': '../../res/scenario_1/obstacles.xml',
            'urdf': '../../res/scenario_1/panda_US.urdf'
        })
        
        # Complex scenario: all poses
        scenarios.append({
            'name': 'scenario_1_complete',
            'description': 'Complete ultrasound scanning sequence',
            'difficulty': 3,
            'start_config': start_config,
            'target_poses': scenario_1_poses,
            'environment': '../../res/scenario_1/obstacles.xml',
            'urdf': '../../res/scenario_1/panda_US.urdf'
        })
        
        # High precision scenario: poses requiring high accuracy
        precision_poses = [pose for pose in scenario_1_poses if pose['contact']]
        if precision_poses:
            scenarios.append({
                'name': 'scenario_1_precision',
                'description': 'High precision contact-based poses',
                'difficulty': 4,
                'start_config': start_config,
                'target_poses': precision_poses,
                'environment': '../../res/scenario_1/obstacles.xml',
                'urdf': '../../res/scenario_1/panda_US.urdf'
            })
    
    else:
        # Fallback scenarios if poses can't be loaded
        logger.warning("Using fallback scenarios - scenario_1 poses not available")
        scenarios = [
            {
                'name': 'fallback_simple',
                'description': 'Basic point-to-point motion',
                'difficulty': 1,
                'start_config': start_config,
                'goal_config': [0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785],
                'environment': '../../res/scenario_1/obstacles.xml',
                'urdf': '../../res/scenario_1/panda_US.urdf'
            }
        ]
    
    return scenarios

def main():
    """Main function for enhanced parameter tuning."""
    import argparse
    
    parser = argparse.ArgumentParser(description='Enhanced Parameter Tuning System')
    parser.add_argument('--cpp-executable', required=True,
                       help='Path to compiled C++ parameter evaluation executable')
    parser.add_argument('--algorithm', choices=['STOMP', 'Hauser', 'both'], default='both',
                       help='Algorithm to optimize')
    parser.add_argument('--optimizer', default='auto',
                       help='Optimization method to use')
    parser.add_argument('--n-calls', type=int, default=100,
                       help='Number of optimization iterations')
    parser.add_argument('--compare-optimizers', action='store_true',
                       help='Compare different optimization methods')
    parser.add_argument('--output-dir', default='enhanced_tuning_results',
                       help='Output directory for results')
    
    args = parser.parse_args()
    
    # Create scenario configurations using scenario_1 data
    scenarios = create_default_scenarios()
    
    if not scenarios:
        logger.error("No valid scenarios could be created")
        return 1
    
    logger.info(f"Created {len(scenarios)} test scenarios:")
    for scenario in scenarios:
        logger.info(f"  - {scenario['name']}: {scenario['description']}")
    
    # Initialize optimizer
    optimizer = EnhancedParameterOptimizer(
        cpp_executable_path=args.cpp_executable,
        scenario_configs=scenarios,
        output_dir=args.output_dir
    )
    
    try:
        if args.compare_optimizers:
            # Compare different optimizers
            algorithms = ['STOMP', 'Hauser'] if args.algorithm == 'both' else [args.algorithm]
            comparison_results = optimizer.compare_optimizers(algorithms, n_calls=args.n_calls)
            print("\nOptimizer comparison completed!")
            
        else:
            # Run optimization for specified algorithm(s)
            if args.algorithm in ['STOMP', 'both']:
                stomp_result = optimizer.optimize_stomp_parameters(args.optimizer, args.n_calls)
                print(f"\nSTOMP optimization completed! Best objective: {stomp_result['best_objective']:.6f}")
                
            if args.algorithm in ['Hauser', 'both']:
                hauser_result = optimizer.optimize_hauser_parameters(args.optimizer, args.n_calls)
                print(f"\nHauser optimization completed! Best objective: {hauser_result['best_objective']:.6f}")
        
        # Generate comprehensive report
        report = optimizer.generate_comprehensive_report()
        print(f"\nComprehensive report generated in: {args.output_dir}")
        
    except Exception as e:
        logger.error(f"Optimization failed: {e}")
        return 1
    
    return 0

if __name__ == "__main__":
    exit(main())