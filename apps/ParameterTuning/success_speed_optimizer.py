#!/usr/bin/env python3
"""
STOMP Success Rate & Speed Optimizer

Optimizes STOMP parameters for high success rate and fast execution time
using soft constraints instead of hard thresholds.
"""

import optuna
import subprocess
import json
import yaml
import time
import logging
from pathlib import Path
from typing import Dict, Any, Tuple

# Configure logging
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(message)s')
logger = logging.getLogger(__name__)

class STOMPSuccessSpeedOptimizer:
    def __init__(self, scenario_config: Dict[str, str], n_startup_trials: int = 10):
        """
        Initialize STOMP Success & Speed optimizer with soft constraints.
        
        Args:
            scenario_config: Dictionary with csv_file, obstacles_file, urdf_file paths
            n_startup_trials: Number of random trials before TPE sampling
        """
        self.scenario_config = scenario_config
        self.n_startup_trials = n_startup_trials
        
        # Create results directory
        self.results_dir = Path('results')
        self.results_dir.mkdir(exist_ok=True)
        
        # Create Optuna study with unique name
        timestamp = int(time.time())
        study_name = f"stomp_success_speed_{timestamp}"
        storage_name = f"sqlite:///{Path.cwd()}/optuna_success_speed.db"
        
        self.study = optuna.create_study(
            direction="minimize",
            study_name=study_name,
            storage=storage_name,
            sampler=optuna.samplers.TPESampler(n_startup_trials=n_startup_trials, seed=42)
        )
        
        logger.info(f"Created Optuna study: {study_name}")
        logger.info(f"Storage: {storage_name}")

    def objective(self, trial):
        """Optuna objective function with soft constraints for success rate and speed"""
        
        # Suggest parameters with expanded ranges for more exploration
        params = {
            'temperature': trial.suggest_float('temperature', 1.0, 20.0),
            'learning_rate': trial.suggest_float('learning_rate', 0.1, 1.0), 
            'max_iterations': trial.suggest_int('max_iterations', 50, 200),  # Allow more iterations
            'N': trial.suggest_int('N', 30, 100),
            'num_noisy_trajectories': trial.suggest_int('num_noisy_trajectories', 4, 20),
            'num_best_samples': trial.suggest_int('num_best_samples', 3, 15),
            'obstacle_cost_weight': trial.suggest_float('obstacle_cost_weight', 0.5, 8.0),
            'constraint_cost_weight': trial.suggest_float('constraint_cost_weight', 0.5, 10.0),
            'joint_std_dev_0': trial.suggest_float('joint_std_dev_0', 0.02, 0.15),
            'joint_std_dev_1': trial.suggest_float('joint_std_dev_1', 0.02, 0.15), 
            'joint_std_dev_2': trial.suggest_float('joint_std_dev_2', 0.02, 0.15),
            'joint_std_dev_3': trial.suggest_float('joint_std_dev_3', 0.02, 0.15),
            'joint_std_dev_4': trial.suggest_float('joint_std_dev_4', 0.02, 0.15),
            'joint_std_dev_5': trial.suggest_float('joint_std_dev_5', 0.02, 0.15),
            'joint_std_dev_6': trial.suggest_float('joint_std_dev_6', 0.02, 0.15)
        }
        
        # Create unique output filename
        timestamp = int(time.time() * 1000) % 1000000
        output_file = self.results_dir / f"trial_{trial.number}_{timestamp}.json"
        
        config_content = f"""csv_file: "{self.scenario_config['csv_file']}"
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
  trajectory_pairs: 15
"""
        
        config_file = Path(f'/tmp/optuna_success_speed_config_{trial.number}_{timestamp}.yaml')
        config_file.write_text(config_content)
        
        # Store output file path for later reading
        trial.set_user_attr('output_file', str(output_file))
        trial.set_user_attr('config_file', str(config_file))
        
        # Run parameter evaluation
        output_file = trial.user_attrs['output_file']
        
        try:
            # Execute parameter_evaluator with relative path from project root
            evaluator_path = Path.cwd() / 'apps' / 'ParameterTuning' / 'parameter_evaluator'
            result = subprocess.run([str(evaluator_path), config_file], 
                                  text=True, 
                                  capture_output=True, 
                                  timeout=120,  # Shorter timeout for speed focus
                                  cwd=Path.cwd())
            
            if result.returncode != 0:
                logger.error(f"Trial {trial.number} failed: {result.stderr.strip()}")
                return 100000.0  # High penalty for failures
            
            # Read the structured JSON output
            if not Path(output_file).exists():
                logger.error(f"Trial {trial.number}: Output file not created")
                return 100000.0
                
            with open(output_file, 'r') as f:
                results = json.load(f)
            
            # Extract metrics
            success_rate = results.get('success_rate', 0.0)
            avg_time = results.get('avg_planning_time_ms', 999999)
            
            logger.info(f"Trial {trial.number}: Success={success_rate:.3f}, Time={avg_time:.1f}ms")
            
            # Soft constraint optimization (no hard thresholds)
            # Target: Success rate ~= 0.95, Time ~= 1000.0ms
            target_success = 0.95
            target_time = 1000.0
            
            # Calculate deviation scores (lower is better)
            success_score = abs(success_rate - target_success) * 10000  # Scale success rate importance
            time_score = abs(avg_time - target_time) / 10.0  # Scale time importance
            
            logger.info(f"Trial {trial.number}: Success score={success_score:.1f}, Time score={time_score:.1f}")
            
            # Combined score prioritizing success rate
            final_score = success_score + time_score
            logger.info(f"Trial {trial.number}: Final score = {final_score:.2f}")
            
            return final_score
            
        except subprocess.TimeoutExpired:
            logger.error(f"Trial {trial.number} timed out")
            return 100000.0
        except Exception as e:
            logger.error(f"Trial {trial.number} error: {str(e)}")
            return 100000.0
            
    def optimize(self, n_trials: int = 40):
        """Run the optimization with the specified number of trials."""
        logger.info(f"Starting STOMP success/speed optimization with {n_trials} trials")
        logger.info(f"Target: Success rate ~= 0.95, Time ~= 1000.0ms")
        
        self.study.optimize(self.objective, n_trials=n_trials)
        
        # Get best trial
        best_trial = self.study.best_trial
        if best_trial is None:
            logger.warning("No successful trials completed!")
            return None
            
        logger.info("=== OPTIMIZATION COMPLETE ===")
        
        # Extract best results from trial attributes
        best_output_file = best_trial.user_attrs.get('output_file')
        if best_output_file and Path(best_output_file).exists():
            with open(best_output_file, 'r') as f:
                best_results = json.load(f)
            
            success_rate = best_results.get('success_rate', 0.0)
            avg_time = best_results.get('avg_planning_time_ms', 0.0)
            
            logger.info("BEST SOLUTION FOUND:")
            logger.info(f"  Success Rate: {success_rate:.3f}")
            logger.info(f"  Avg Time: {avg_time:.1f}ms")
            logger.info(f"  Trial Number: {best_trial.number}")
            logger.info("  Parameters:")
            for key, value in best_trial.params.items():
                logger.info(f"    {key}: {value}")
                
            # Save best parameters 
            best_config = {
                'csv_file': self.scenario_config['csv_file'],
                'obstacles_file': self.scenario_config['obstacles_file'],
                'urdf_file': self.scenario_config['urdf_file'],
                'output_file': 'optimal_success_speed_results.json',
                'stomp': {
                    'temperature': best_trial.params['temperature'],
                    'learning_rate': best_trial.params['learning_rate'],
                    'max_iterations': best_trial.params['max_iterations'],
                    'N': best_trial.params['N'],
                    'num_noisy_trajectories': best_trial.params['num_noisy_trajectories'],
                    'num_best_samples': best_trial.params['num_best_samples'],
                    'obstacle_cost_weight': best_trial.params['obstacle_cost_weight'],
                    'constraint_cost_weight': best_trial.params['constraint_cost_weight'],
                    'joint_std_devs': [
                        best_trial.params['joint_std_dev_0'],
                        best_trial.params['joint_std_dev_1'],
                        best_trial.params['joint_std_dev_2'],
                        best_trial.params['joint_std_dev_3'],
                        best_trial.params['joint_std_dev_4'],
                        best_trial.params['joint_std_dev_5'],
                        best_trial.params['joint_std_dev_6']
                    ]
                },
                'evaluation': {'trajectory_pairs': 15}
            }
            
            with open('optimal_success_speed_config.yaml', 'w') as f:
                yaml.dump(best_config, f, default_flow_style=False)
            logger.info("Saved best parameters to optimal_success_speed_config.yaml")
            
            return best_results
        else:
            logger.warning("Could not find results for best trial")
            return None
            
    def validate_best_parameters(self, n_validation_runs: int = 3):
        """Validate the best parameters with multiple runs."""
        best_trial = self.study.best_trial
        if best_trial is None:
            logger.error("No best trial found. Run optimization first.")
            return None
            
        logger.info("Validating best parameters with extended evaluation...")
        
        validation_results = []
        
        for run in range(n_validation_runs):
            logger.info(f"Running validation {run + 1}/{n_validation_runs}")
            
            # Create config with longer trajectory_pairs for validation
            timestamp = int(time.time() * 1000) % 1000000
            output_file = self.results_dir / f"validation_{run}_{timestamp}.json"
            
            config_content = f"""csv_file: "{self.scenario_config['csv_file']}"
obstacles_file: "{self.scenario_config['obstacles_file']}"
urdf_file: "{self.scenario_config['urdf_file']}"
output_file: "{output_file}"

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
  trajectory_pairs: 30
"""
            
            config_file = Path(f'/tmp/validation_config_{run}_{timestamp}.yaml')
            config_file.write_text(config_content)
            
            try:
                evaluator_path = Path('/Users/joris/Uni/MA/Code/PathPlanner_US_wip/apps/ParameterTuning/parameter_evaluator')
                result = subprocess.run(
                    [str(evaluator_path), str(config_file)],
                    text=True,
                    capture_output=True,
                    timeout=300,
                    cwd=Path.cwd(),
                )
                
                if result.returncode == 0 and Path(output_file).exists():
                    with open(output_file, 'r') as f:
                        results = json.load(f)
                    
                    success_rate = results.get('success_rate', 0.0)
                    avg_time = results.get('avg_planning_time_ms', 0.0)
                    
                    logger.info(f"Validation run {run + 1}: Success={success_rate:.3f}, Time={avg_time:.1f}ms")
                    validation_results.append((success_rate, avg_time))
                else:
                    logger.warning(f"Validation run {run + 1} failed")
                    validation_results.append((0.0, 0.0))
                    
            except Exception as e:
                logger.error(f"Validation run {run + 1} error: {str(e)}")
                validation_results.append((0.0, 0.0))
            finally:
                if config_file.exists():
                    config_file.unlink()
        
        # Calculate averages
        if validation_results:
            avg_success = sum(r[0] for r in validation_results) / len(validation_results)
            avg_time = sum(r[1] for r in validation_results) / len(validation_results)
            
            logger.info("VALIDATION RESULTS:")
            logger.info(f"  Average Success Rate: {avg_success:.3f}")
            logger.info(f"  Average Time: {avg_time:.1f}ms")
            
            return {'avg_success_rate': avg_success, 'avg_time_ms': avg_time}
        
        return None


if __name__ == "__main__":
    import sys
    
    print("🚀 STOMP SUCCESS RATE & SPEED OPTIMIZER")
    print("=" * 50)
    
    # Configuration with absolute paths
    scenario_config = {
        'csv_file': '/Users/joris/Uni/MA/Code/PathPlanner_US_wip/res/scenario_1/scan_poses.csv',
        'obstacles_file': '/Users/joris/Uni/MA/Code/PathPlanner_US_wip/res/scenario_1/obstacles.xml',
        'urdf_file': '/Users/joris/Uni/MA/Code/PathPlanner_US_wip/res/scenario_1/panda_US.urdf'
    }
    
    # Create optimizer
    optimizer = STOMPSuccessSpeedOptimizer(scenario_config, n_startup_trials=5)
    
    # Run optimization
    import sys
    n_trials = 40 if len(sys.argv) <= 1 else int(sys.argv[1])
    results = optimizer.optimize(n_trials=n_trials)
    
    if results:
        # Validate best parameters
        validation_scores = optimizer.validate_best_parameters(n_validation_runs=3)
        
    print("\n🎯 SUCCESS/SPEED OPTIMIZATION COMPLETE!")
    if results:
        print(f"Best success rate: {results.get('success_rate', 0.0):.3f}")
        print(f"Best avg time: {results.get('avg_planning_time_ms', 0.0):.1f}ms")
