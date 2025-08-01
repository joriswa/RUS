#!/usr/bin/env python3
"""
STOMP Success Rate & Speed Optimizer - FIXED VERSION

This script focuses specifically on optimizing STOMP parameters for:
1. HIGH SUCCESS RATE (primary objective - must be 95%+)
2. FAST EXECUTION TIME (secondary objective - under 1000ms)

Based directly on the working optuna_stomp_optimizer_fixed.py with changed objectives.
"""

import json
import subprocess
import time
import logging
from pathlib import Path
from typing import Dict, Any, Tuple
import optuna

# Setup logging
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(message)s')
logger = logging.getLogger(__name__)

class STOMPSuccessSpeedOptimizer:
    def __init__(self, scenario_config: Dict[str, str], n_startup_trials: int = 10):
        """
        Initialize the STOMP success/speed optimizer
        
        Args:
            scenario_config: Dictionary with csv_file, obstacles_file, urdf_file paths
            n_startup_trials: Number of random trials before Bayesian optimization
        """
        self.scenario_config = scenario_config
        self.n_startup_trials = n_startup_trials
        self.results_dir = Path("results")
        self.results_dir.mkdir(exist_ok=True)
        
        # Success/speed targets
        self.min_success_rate = 0.95  # Must achieve 95%+ success
        self.max_avg_time_ms = 1000.0  # Must be under 1 second average
        
        # Create study with persistent storage
        storage_name = f"sqlite:///{Path.cwd()}/optuna_success_speed.db"
        study_name = f"stomp_success_speed_{int(time.time())}"
        
        self.study = optuna.create_study(
            direction="minimize",  # Minimize execution time while maintaining success
            storage=storage_name,
            study_name=study_name,
            sampler=optuna.samplers.TPESampler(n_startup_trials=n_startup_trials)
        )
        
        logger.info(f"Created Optuna study: {study_name}")
        logger.info(f"Storage: {storage_name}")

    def create_config_file(self, trial: optuna.Trial) -> str:
        """Create YAML config file from Optuna trial parameters"""
        
        # Define parameter search space focused on success rate and speed
        params = {
            'temperature': trial.suggest_float('temperature', 1.0, 15.0, log=False),
            'learning_rate': trial.suggest_float('learning_rate', 0.1, 0.8, log=False),
            'max_iterations': trial.suggest_int('max_iterations', 20, 80, step=5),  # Fewer iterations for speed
            'N': trial.suggest_int('N', 30, 70, step=5),  # Smaller N for speed
            'num_noisy_trajectories': trial.suggest_int('num_noisy_trajectories', 4, 12, step=2),
            'num_best_samples': trial.suggest_int('num_best_samples', 3, 10, step=1),
            'obstacle_cost_weight': trial.suggest_float('obstacle_cost_weight', 0.5, 3.0, log=False),
            'constraint_cost_weight': trial.suggest_float('constraint_cost_weight', 1.0, 8.0, log=False),
            # Joint standard deviations - optimize each joint independently
            'joint_std_dev_0': trial.suggest_float('joint_std_dev_0', 0.02, 0.15, log=False),
            'joint_std_dev_1': trial.suggest_float('joint_std_dev_1', 0.02, 0.15, log=False),
            'joint_std_dev_2': trial.suggest_float('joint_std_dev_2', 0.02, 0.15, log=False),
            'joint_std_dev_3': trial.suggest_float('joint_std_dev_3', 0.02, 0.15, log=False),
            'joint_std_dev_4': trial.suggest_float('joint_std_dev_4', 0.02, 0.15, log=False),
            'joint_std_dev_5': trial.suggest_float('joint_std_dev_5', 0.02, 0.15, log=False),
            'joint_std_dev_6': trial.suggest_float('joint_std_dev_6', 0.02, 0.15, log=False),
        }
        
        # Use fewer trajectory pairs for faster evaluation
        trajectory_pairs = 10  # Balance between speed and statistical significance
        
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
  trajectory_pairs: {trajectory_pairs}
"""
        
        config_file = Path(f'/tmp/optuna_success_speed_config_{trial.number}_{timestamp}.yaml')
        config_file.write_text(config_content)
        
        # Store output file path for later reading
        trial.set_user_attr('output_file', str(output_file))
        trial.set_user_attr('config_file', str(config_file))
        
        return str(config_file)

    def evaluate_parameters(self, trial: optuna.Trial) -> float:
        """
        Evaluate parameters focusing on success rate and execution time
        
        Args:
            trial: Optuna trial object
            
        Returns:
            score: Lower is better (objective to minimize)
        """
        config_file = self.create_config_file(trial)
        output_file = trial.user_attrs['output_file']
        
        try:
            # Execute parameter_evaluator with full absolute path
            evaluator_path = Path('/Users/joris/Uni/MA/Code/PathPlanner_US_wip/apps/ParameterTuning/parameter_evaluator')
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
            
            # Extract metrics
            success_rate = results.get('success_rate', 0.0)
            avg_time = results.get('avg_planning_time_ms', 999999)
            
            # Store metrics as trial attributes for analysis
            trial.set_user_attr('success_rate', success_rate)
            trial.set_user_attr('avg_planning_time_ms', avg_time)
            trial.set_user_attr('avg_path_length', results.get('avg_path_length', 0.0))
            trial.set_user_attr('composite_score', results.get('composite_score', 999))
            
            logger.info(f"Trial {trial.number}: Success={success_rate:.3f}, Time={avg_time:.1f}ms")
            
            # SUCCESS RATE AND SPEED FOCUSED OBJECTIVE
            # Strict success rate requirement
            if success_rate < self.min_success_rate:
                logger.info(f"Trial {trial.number}: REJECTED - Success rate {success_rate:.3f} < {self.min_success_rate}")
                return 100000.0  # High penalty for insufficient success rate
            
            # Speed requirement 
            if avg_time > self.max_avg_time_ms:
                logger.info(f"Trial {trial.number}: SLOW - Time {avg_time:.1f}ms > {self.max_avg_time_ms}ms")
                # Penalize but don't completely reject
                time_penalty = (avg_time / self.max_avg_time_ms) * 1000
                return avg_time + time_penalty
            
            # SUCCESS! Optimize for minimum time while maintaining high success rate
            logger.info(f"Trial {trial.number}: SUCCESS - {success_rate:.3f} success, {avg_time:.1f}ms")
            
            # Primary objective: minimize execution time
            # Secondary objective: maximize success rate
            speed_score = avg_time
            success_penalty = (1.0 - success_rate) * 500  # Small penalty for imperfect success
            
            final_score = speed_score + success_penalty
            logger.info(f"Trial {trial.number}: Final score = {final_score:.2f}")
            
            return final_score
            
        except subprocess.TimeoutExpired:
            logger.warning(f"Trial {trial.number}: Evaluation timed out")
            return 100000.0
        except Exception as e:
            logger.error(f"Trial {trial.number}: Error - {e}")
            return 100000.0
        finally:
            # Clean up config file
            try:
                Path(config_file).unlink()
            except:
                pass

    def optimize(self, n_trials: int = 50) -> Dict[str, Any]:
        """Run Optuna optimization focused on success rate and speed"""
        
        logger.info(f"Starting STOMP success/speed optimization with {n_trials} trials")
        logger.info(f"Target: Success rate >= {self.min_success_rate}, Time <= {self.max_avg_time_ms}ms")
        
        # Run optimization
        self.study.optimize(self.evaluate_parameters, n_trials=n_trials)
        
        # Analyze results
        logger.info("=== OPTIMIZATION COMPLETE ===")
        
        # Find best trials that meet our criteria
        valid_trials = []
        for trial in self.study.trials:
            if (trial.state == optuna.trial.TrialState.COMPLETE and 
                trial.value < 100000.0):  # Exclude failed trials
                
                success_rate = trial.user_attrs.get('success_rate', 0.0)
                avg_time = trial.user_attrs.get('avg_planning_time_ms', float('inf'))
                
                if success_rate >= self.min_success_rate:
                    valid_trials.append((trial, success_rate, avg_time))
        
        if valid_trials:
            # Sort by speed (fastest first)
            valid_trials.sort(key=lambda x: x[2])
            best_trial, best_success, best_time = valid_trials[0]
            
            logger.info(f"BEST SOLUTION FOUND:")
            logger.info(f"  Success Rate: {best_success:.3f}")
            logger.info(f"  Avg Time: {best_time:.1f}ms")
            logger.info(f"  Trial Number: {best_trial.number}")
            
            # Print parameters
            logger.info(f"  Parameters:")
            for key, value in best_trial.params.items():
                logger.info(f"    {key}: {value}")
            
            # Generate config for best parameters
            best_config = {
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
            }
            
            # Save best config to YAML
            config_yaml = f"""# OPTIMAL STOMP PARAMETERS FOR SUCCESS RATE & SPEED
# Success Rate: {best_success:.3f}, Avg Time: {best_time:.1f}ms
# Generated by success_speed_optimizer_fixed.py

temperature: {best_config['temperature']}
learning_rate: {best_config['learning_rate']}
max_iterations: {best_config['max_iterations']}
N: {best_config['N']}
num_noisy_trajectories: {best_config['num_noisy_trajectories']}
num_best_samples: {best_config['num_best_samples']}
obstacle_cost_weight: {best_config['obstacle_cost_weight']}
constraint_cost_weight: {best_config['constraint_cost_weight']}
joint_std_devs: {best_config['joint_std_devs']}
"""
            
            with open("optimal_success_speed_config.yaml", "w") as f:
                f.write(config_yaml)
            logger.info("Saved best parameters to optimal_success_speed_config.yaml")
            
            return {
                'best_params': best_trial.params,
                'success_rate': best_success,
                'avg_time_ms': best_time,
                'trial_number': best_trial.number
            }
            
        else:
            logger.warning("No trials met the success criteria!")
            logger.info("Consider relaxing constraints or adjusting parameter ranges")
            return None


def main():
    """Main optimization loop"""
    print("🚀 STOMP SUCCESS RATE & SPEED OPTIMIZER - FIXED")
    print("=" * 55)
    
    # Scenario configuration
    scenario_config = {
        'csv_file': '/Users/joris/Uni/MA/Code/PathPlanner_US_wip/apps/ParameterTuning/Data/Ext_Ant_Scan/scan_pose_list_ee.csv',
        'obstacles_file': '/Users/joris/Uni/MA/Code/PathPlanner_US_wip/apps/ParameterTuning/Data/Ext_Ant_Scan/obstacles.xml',
        'urdf_file': '/Users/joris/Uni/MA/Code/PathPlanner_US_wip/res/scenario_1/panda_US.urdf'
    }
    
    # Create optimizer
    optimizer = STOMPSuccessSpeedOptimizer(scenario_config, n_startup_trials=5)
    
    # Run optimization
    import sys
    n_trials = 40 if len(sys.argv) <= 1 else int(sys.argv[1])
    results = optimizer.optimize(n_trials=n_trials)
    
    if results:
        print("\n🎯 SUCCESS/SPEED OPTIMIZATION COMPLETE!")
        print(f"Best success rate: {results['success_rate']:.3f}")
        print(f"Best avg time: {results['avg_time_ms']:.1f}ms")
    else:
        print("\n❌ No suitable parameters found. Consider adjusting constraints.")


if __name__ == "__main__":
    main()
