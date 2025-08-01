#!/usr/bin/env python3
"""
STOMP Success Rate & Speed Optimizer using Optuna

Optimizes STOMP parameters for high success rate and fast execution time
using soft constraints instead of hard thresholds. Uses proven working
architecture from optuna_stomp_optimizer.py.

Target: Success rate ~= 0.95, Planning time ~= 1000ms
"""

import json
import subprocess
import time
import logging
from pathlib import Path
from typing import Dict, Any, Tuple
import optuna
from optuna.visualization import plot_optimization_history, plot_param_importances


# Setup logging
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(message)s')
logger = logging.getLogger(__name__)


class STOMPSuccessSpeedOptimizer:
    """
    STOMP optimizer focusing on success rate and speed with soft constraints
    """
    
    def __init__(self, scenario_config: Dict[str, str], n_startup_trials: int = 10):
        self.scenario_config = scenario_config
        self.n_startup_trials = n_startup_trials
        self.results_dir = Path('results')
        self.results_dir.mkdir(exist_ok=True)
        
        # Create Optuna study
        study_name = f"stomp_optimization_{int(time.time())}"
        self.study = optuna.create_study(
            direction='minimize',  # Minimize deviation from targets
            study_name=study_name,
            storage=f'sqlite:///optuna_stomp_study.db',
            load_if_exists=True
        )
        
        logger.info(f"Initialized STOMP Success & Speed Optimizer")
        logger.info(f"Study: {study_name}")
        logger.info(f"Target: Success rate ~= 0.95, Planning time ~= 1000ms")

    def suggest_parameters(self, trial: optuna.Trial) -> Dict[str, Any]:
        """Define parameter search space optimized for speed"""
        params = {
            'temperature': trial.suggest_float('temperature', 5.0, 35.0, log=False),
            'learning_rate': trial.suggest_float('learning_rate', 0.1, 0.6, log=False),
            'max_iterations': trial.suggest_int('max_iterations', 50, 150, step=10),  # Speed-focused range
            'N': trial.suggest_int('N', 30, 100, step=5),  # Smaller for speed
            'num_noisy_trajectories': trial.suggest_int('num_noisy_trajectories', 16, 50, step=4),
            'num_best_samples': trial.suggest_int('num_best_samples', 4, 20, step=2),
            'obstacle_cost_weight': trial.suggest_float('obstacle_cost_weight', 0.5, 8.0, log=False),
            'constraint_cost_weight': trial.suggest_float('constraint_cost_weight', 0.5, 8.0, log=False),
            # Joint standard deviations - optimize each joint independently
            'joint_std_dev_0': trial.suggest_float('joint_std_dev_0', 0.02, 0.15, log=False),
            'joint_std_dev_1': trial.suggest_float('joint_std_dev_1', 0.02, 0.15, log=False),
            'joint_std_dev_2': trial.suggest_float('joint_std_dev_2', 0.02, 0.15, log=False),
            'joint_std_dev_3': trial.suggest_float('joint_std_dev_3', 0.02, 0.15, log=False),
            'joint_std_dev_4': trial.suggest_float('joint_std_dev_4', 0.02, 0.15, log=False),
            'joint_std_dev_5': trial.suggest_float('joint_std_dev_5', 0.02, 0.15, log=False),
            'joint_std_dev_6': trial.suggest_float('joint_std_dev_6', 0.02, 0.15, log=False),
        }
        
        return params

    def create_config_file(self, trial: optuna.Trial) -> str:
        """Create YAML config file for this trial"""
        params = self.suggest_parameters(trial)
        
        # Generate unique filenames
        timestamp = int(time.time() * 1000) % 1000000
        config_file = f'/tmp/trial_{trial.number}_{timestamp}.yaml'
        output_file = self.results_dir / f'trial_{trial.number}_{timestamp}.json'
        
        # Store filenames in trial attributes for cleanup
        trial.set_user_attr('config_file', config_file)
        trial.set_user_attr('output_file', str(output_file))
        
        # Create YAML content
        yaml_content = f'''csv_file: "{self.scenario_config['csv_file']}"
obstacles_file: "{self.scenario_config['obstacles_file']}"
urdf_file: "{self.scenario_config['urdf_file']}"
output_file: "{output_file}"

stomp:
  temperature: {params['temperature']}
  learning_rate: {params['learning_rate']}
  max_iterations: {params['max_iterations']}
  N: {params['N']}
  num_noisy_trajectories: {params['num_noisy_trajectories']}
  num_best_samples: {params['num_best_samples']}
  obstacle_cost_weight: {params['obstacle_cost_weight']}
  constraint_cost_weight: {params['constraint_cost_weight']}
  joint_std_devs: [{params['joint_std_dev_0']}, {params['joint_std_dev_1']}, {params['joint_std_dev_2']}, {params['joint_std_dev_3']}, {params['joint_std_dev_4']}, {params['joint_std_dev_5']}, {params['joint_std_dev_6']}]

evaluation:
  trajectory_pairs: 10  # Smaller for faster evaluation
'''
        
        # Write config file
        with open(config_file, 'w') as f:
            f.write(yaml_content)
        
        return config_file

    def evaluate_parameters(self, trial: optuna.Trial) -> float:
        """
        Evaluate parameters using soft constraints for success rate and speed
        
        Args:
            trial: Optuna trial object
            
        Returns:
            Combined score based on success rate and planning time deviations (lower is better)
        """
        config_file = self.create_config_file(trial)
        output_file = trial.user_attrs['output_file']
        
        try:
            # Execute parameter_evaluator with relative path from project root
            evaluator_path = Path.cwd() / 'apps' / 'ParameterTuning' / 'parameter_evaluator'
            result = subprocess.run([str(evaluator_path), config_file], 
                                  text=True, 
                                  capture_output=True, 
                                  timeout=120)  # Shorter timeout for speed focus
            
            if result.returncode != 0:
                logger.error(f"Trial {trial.number} failed: {result.stderr.strip()}")
                return 100000.0  # High penalty for failures
            
            # Read the structured JSON output
            if not Path(output_file).exists():
                logger.error(f"Trial {trial.number}: Output file not created")
                return 100000.0
                
            with open(output_file, 'r') as f:
                results = json.load(f)
            
            # Extract metrics for soft constraint evaluation
            success_rate = results.get('success_rate', 0.0)
            avg_time = results.get('avg_planning_time_ms', 999999)
            
            # Store additional metrics as trial attributes for analysis
            trial.set_user_attr('success_rate', success_rate)
            trial.set_user_attr('avg_planning_time_ms', avg_time)
            trial.set_user_attr('avg_path_length', results.get('avg_path_length', 0.0))
            trial.set_user_attr('avg_smoothness', results.get('avg_smoothness', 0.0))
            trial.set_user_attr('composite_score', results.get('composite_score', 10.0))
            
            # Soft constraint optimization (no hard thresholds)
            # Target: Success rate ~= 0.95, Time ~= 1000.0ms
            target_success = 0.95
            target_time = 1000.0
            
            # Calculate deviation scores (lower is better)
            success_score = abs(success_rate - target_success) * 10000  # Scale success rate importance
            time_score = abs(avg_time - target_time) / 10.0  # Scale time importance
            
            # Combined score prioritizing success rate
            final_score = success_score + time_score
            
            logger.info(f"Trial {trial.number}: Success={success_rate:.3f}, Time={avg_time:.1f}ms, "
                       f"Score={final_score:.2f}")
            
            return final_score
            
        except subprocess.TimeoutExpired:
            logger.warning(f"Trial {trial.number}: Timeout (>2min)")
            return 50000.0  # Penalty for slow configurations
        except Exception as e:
            logger.error(f"Trial {trial.number}: Error - {e}")
            return 100000.0
        finally:
            # Cleanup temporary files
            config_file_path = Path(trial.user_attrs.get('config_file', ''))
            if config_file_path.exists():
                config_file_path.unlink()

    def optimize(self, n_trials: int = 30) -> Dict[str, Any]:
        """
        Run Bayesian optimization
        
        Args:
            n_trials: Number of optimization trials
            
        Returns:
            Dictionary with optimization results
        """
        logger.info(f"Starting optimization with {n_trials} trials")
        logger.info(f"Startup trials: {self.n_startup_trials}")
        
        # Run optimization
        self.study.optimize(
            self.evaluate_parameters,
            n_trials=n_trials,
            show_progress_bar=True
        )
        
        # Get results
        best_trial = self.study.best_trial
        best_params = best_trial.params
        best_score = best_trial.value
        
        # Log results
        logger.info(f"\n🎯 OPTIMIZATION RESULTS:")
        logger.info(f"Best deviation score: {best_score:.4f} (lower = closer to targets)")
        logger.info(f"Success rate: {best_trial.user_attrs.get('success_rate', 'N/A')}")
        logger.info(f"Planning time: {best_trial.user_attrs.get('avg_planning_time_ms', 'N/A')}ms")
        logger.info(f"Best parameters: {best_params}")
        
        # Save results
        self.save_best_config(best_trial)
        self.generate_plots()
        
        return {
            'best_score': best_score,
            'best_params': best_params,
            'best_trial': best_trial,
            'study': self.study
        }

    def save_best_config(self, best_trial: optuna.Trial):
        """Save the best parameters to a reusable config file"""
        
        best_config = f'''# Optimal STOMP parameters found by Optuna soft constraint optimization
# Trial {best_trial.number} - Deviation Score: {best_trial.value:.4f}
# Target: Success Rate ~= 0.95, Planning Time ~= 1000ms
csv_file: "{self.scenario_config['csv_file']}"
obstacles_file: "{self.scenario_config['obstacles_file']}"
urdf_file: "{self.scenario_config['urdf_file']}"
output_file: "optimal_results.json"

stomp:
  temperature: {best_trial.params['temperature']}
  learning_rate: {best_trial.params['learning_rate']}
  max_iterations: {best_trial.params['max_iterations']}
  N: {best_trial.params['N']}
  num_noisy_trajectories: {best_trial.params['num_noisy_trajectories']}
  num_best_samples: {best_trial.params['num_best_samples']}
  obstacle_cost_weight: {best_trial.params['obstacle_cost_weight']}
  constraint_cost_weight: {best_trial.params['constraint_cost_weight']}
  joint_std_devs: [{best_trial.params['joint_std_dev_0']}, {best_trial.params['joint_std_dev_1']}, {best_trial.params['joint_std_dev_2']}, {best_trial.params['joint_std_dev_3']}, {best_trial.params['joint_std_dev_4']}, {best_trial.params['joint_std_dev_5']}, {best_trial.params['joint_std_dev_6']}]

evaluation:
  trajectory_pairs: 20  # Use more pairs for final validation

# Performance metrics:
# Success Rate: {best_trial.user_attrs.get('success_rate', 'N/A')}
# Avg Planning Time: {best_trial.user_attrs.get('avg_planning_time_ms', 'N/A')}ms
# Deviation Score: {best_trial.value:.4f} (lower = closer to targets)
'''
        
        config_path = Path("optimal_stomp_config.yaml")
        config_path.write_text(best_config)
        logger.info(f"Saved optimal config to: {config_path}")

    def generate_plots(self):
        """Generate optimization visualization plots"""
        try:
            fig1 = plot_optimization_history(self.study)
            fig1.write_html(self.results_dir / "optimization_history.html")
            
            fig2 = plot_param_importances(self.study)
            fig2.write_html(self.results_dir / "parameter_importance.html")
            
            logger.info("Generated visualization plots:")
            logger.info(f"  - {self.results_dir}/optimization_history.html")
            logger.info(f"  - {self.results_dir}/parameter_importance.html")
        except Exception as e:
            logger.warning(f"Could not generate plots: {e}")

    def validate_best_parameters(self, n_validation_runs: int = 3):
        """Validate the best parameters with more trajectory pairs"""
        logger.info(f"Validating best parameters with {n_validation_runs} runs...")
        
        best_params = self.study.best_params
        validation_scores = []
        
        for i in range(n_validation_runs):
            timestamp = int(time.time() * 1000) % 1000000
            output_file = self.results_dir / f'validation_{i}_{timestamp}.json'
            
            config_content = f'''csv_file: "{self.scenario_config['csv_file']}"
obstacles_file: "{self.scenario_config['obstacles_file']}"
urdf_file: "{self.scenario_config['urdf_file']}"
output_file: "{output_file}"

stomp:
  temperature: {best_params['temperature']}
  learning_rate: {best_params['learning_rate']}
  max_iterations: {best_params['max_iterations']}
  N: {best_params['N']}
  num_noisy_trajectories: {best_params['num_noisy_trajectories']}
  num_best_samples: {best_params['num_best_samples']}
  obstacle_cost_weight: {best_params['obstacle_cost_weight']}
  constraint_cost_weight: {best_params['constraint_cost_weight']}
  joint_std_devs: [{best_params['joint_std_dev_0']}, {best_params['joint_std_dev_1']}, {best_params['joint_std_dev_2']}, {best_params['joint_std_dev_3']}, {best_params['joint_std_dev_4']}, {best_params['joint_std_dev_5']}, {best_params['joint_std_dev_6']}]

evaluation:
  trajectory_pairs: 15
'''
            
            config_file = Path(f'/tmp/validation_config_{i}_{timestamp}.yaml')
            config_file.write_text(config_content)
            
            try:
                # Run validation
                evaluator_path = Path.cwd() / 'apps' / 'ParameterTuning' / 'parameter_evaluator'
                result = subprocess.run(
                    [str(evaluator_path), str(config_file)],
                    capture_output=True,
                    text=True,
                    timeout=180
                )
                
                if result.returncode == 0 and Path(output_file).exists():
                    with open(output_file, 'r') as f:
                        results = json.load(f)
                    
                    # Calculate validation score using same soft constraint formula
                    success_rate = results.get('success_rate', 0.0)
                    avg_time = results.get('avg_planning_time_ms', 999999)
                    
                    target_success = 0.95
                    target_time = 1000.0
                    success_score = abs(success_rate - target_success) * 10000
                    time_score = abs(avg_time - target_time) / 10.0
                    score = success_score + time_score
                    
                    validation_scores.append(score)
                    logger.info(f"Validation run {i+1}: Score = {score:.4f} (Success={success_rate:.3f}, Time={avg_time:.1f}ms)")
                else:
                    logger.error(f"Validation run {i+1} failed")
                    validation_scores.append(100000.0)
                    
            except Exception as e:
                logger.error(f"Validation run {i+1} error: {e}")
                validation_scores.append(100000.0)
            finally:
                # Cleanup
                if config_file.exists():
                    config_file.unlink()
        
        avg_validation_score = sum(validation_scores) / len(validation_scores)
        logger.info(f"Average validation score: {avg_validation_score:.4f}")
        
        return validation_scores


def main():
    """Main optimization loop"""
    print("🧠 INTELLIGENT STOMP PARAMETER OPTIMIZATION WITH OPTUNA")
    print("=" * 60)
    
    # Scenario configuration - using the complicated scenario with 127 poses
    scenario_config = {
        'csv_file': '/Users/joris/Uni/MA/Code/PathPlanner_US_wip/apps/ParameterTuning/Data/Ext_Ant_Scan/scan_pose_list_ee.csv',
        'obstacles_file': '/Users/joris/Uni/MA/Code/PathPlanner_US_wip/apps/ParameterTuning/Data/Ext_Ant_Scan/obstacles.xml',
        'urdf_file': '/Users/joris/Uni/MA/Code/PathPlanner_US_wip/res/scenario_1/panda_US.urdf'
    }
    
    # Create optimizer
    optimizer = STOMPSuccessSpeedOptimizer(scenario_config, n_startup_trials=10)
    
    # Run optimization
    results = optimizer.optimize(n_trials=30)
    
    # Validate best parameters
    validation_scores = optimizer.validate_best_parameters(n_validation_runs=3)
    
    print("\n🎯 OPTIMIZATION COMPLETE!")
    print("=" * 40)
    print(f"Best deviation score: {results['best_score']:.4f} (lower = closer to targets)")
    print(f"Validation average: {sum(validation_scores)/len(validation_scores):.4f}")
    print("\nOptimal parameters saved to: optimal_stomp_config.yaml")
    print("Visualizations saved to: results/")
    print("Study database: optuna_stomp_study.db")


if __name__ == "__main__":
    main()
