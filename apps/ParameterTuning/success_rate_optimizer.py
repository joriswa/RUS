#!/usr/bin/env python3
"""
STOMP Success Rate Optimizer using Optuna

Simple copy of the working optuna_stomp_optimizer.py with modifications for success rate optimization.
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

class STOMPSuccessRateOptimizer:
    def __init__(self, scenario_config: Dict[str, str], n_startup_trials: int = 10):
        self.scenario_config = scenario_config
        self.n_startup_trials = n_startup_trials
        self.results_dir = Path("results")
        self.results_dir.mkdir(exist_ok=True)
        
        # Create Optuna study
        study_name = f"stomp_success_rate_{int(time.time())}"
        self.study = optuna.create_study(
            direction='maximize',  # Maximize success rate
            study_name=study_name,
            storage=f'sqlite:///optuna_success_rate_study.db',
            load_if_exists=True
        )
        
        logger.info(f"Initialized STOMP Success Rate Optimizer")
        logger.info(f"Study: {study_name}")

    def suggest_parameters(self, trial: optuna.Trial) -> Dict[str, Any]:
        """Define parameter search space with conservative ranges based on working config"""
        params = {
            'temperature': trial.suggest_float('temperature', 8.0, 15.0, log=False),  # Conservative range around working value (10)
            'learning_rate': trial.suggest_float('learning_rate', 0.3, 0.6, log=False),  # Conservative range around working value (0.5)
            'max_iterations': trial.suggest_int('max_iterations', 40, 80, step=10),  # Conservative range around working value (50)
            'N': trial.suggest_int('N', 40, 60, step=5),  # Conservative range around working value (50)
            'num_noisy_trajectories': trial.suggest_int('num_noisy_trajectories', 6, 12, step=2),  # Conservative range around working value (8)
            'num_best_samples': trial.suggest_int('num_best_samples', 3, 6, step=1),  # Conservative range around working value (4)
            'obstacle_cost_weight': trial.suggest_float('obstacle_cost_weight', 1.5, 2.5, log=False),  # Conservative range around working value (2)
            'constraint_cost_weight': trial.suggest_float('constraint_cost_weight', 4.0, 6.0, log=False),  # Conservative range around working value (5)
            'joint_std_dev_0': trial.suggest_float('joint_std_dev_0', 0.06, 0.10, log=False),  # Conservative ranges
            'joint_std_dev_1': trial.suggest_float('joint_std_dev_1', 0.06, 0.10, log=False),
            'joint_std_dev_2': trial.suggest_float('joint_std_dev_2', 0.06, 0.10, log=False),
            'joint_std_dev_3': trial.suggest_float('joint_std_dev_3', 0.06, 0.10, log=False),
            'joint_std_dev_4': trial.suggest_float('joint_std_dev_4', 0.06, 0.10, log=False),
            'joint_std_dev_5': trial.suggest_float('joint_std_dev_5', 0.06, 0.10, log=False),
            'joint_std_dev_6': trial.suggest_float('joint_std_dev_6', 0.06, 0.10, log=False),
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
        yaml_content = f"""csv_file: "{self.scenario_config['csv_file']}"
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
        
        # Write config file
        with open(config_file, 'w') as f:
            f.write(yaml_content)
        
        return config_file

    def evaluate_parameters(self, trial: optuna.Trial) -> float:
        """Evaluate parameters and return success rate"""
        config_file = self.create_config_file(trial)
        output_file = trial.user_attrs['output_file']
        
        try:
            # Execute parameter_evaluator with relative path from project root
            evaluator_path = Path.cwd() / 'apps' / 'ParameterTuning' / 'parameter_evaluator'
            result = subprocess.run([str(evaluator_path), config_file], 
                                  text=True, 
                                  capture_output=True, 
                                  timeout=180)
            
            if result.returncode != 0:
                logger.error(f"Trial {trial.number} failed: {result.stderr.strip()}")
                return 0.0  # Return 0 success rate for failures
            
            # Read the structured JSON output
            if not Path(output_file).exists():
                logger.error(f"Trial {trial.number}: Output file not created")
                return 0.0
                
            with open(output_file, 'r') as f:
                results = json.load(f)
            
            # Extract success rate - this is what we want to maximize
            success_rate = results.get('success_rate', 0.0)
            avg_time = results.get('avg_planning_time_ms', 999999)
            
            # Store additional metrics as trial attributes for analysis
            trial.set_user_attr('success_rate', success_rate)
            trial.set_user_attr('avg_planning_time_ms', avg_time)
            trial.set_user_attr('avg_path_length', results.get('avg_path_length', 0.0))
            trial.set_user_attr('avg_smoothness', results.get('avg_smoothness', 0.0))
            trial.set_user_attr('composite_score', results.get('composite_score', 10.0))
            
            logger.info(f"Trial {trial.number}: Success={success_rate:.3f}, Time={avg_time:.1f}ms")
            
            return success_rate  # Return success rate directly (maximize this)
            
        except subprocess.TimeoutExpired:
            logger.warning(f"Trial {trial.number}: Timeout (>3min)")
            return 0.0  # Return 0 for timeouts
        except Exception as e:
            logger.error(f"Trial {trial.number}: Error - {e}")
            return 0.0
        finally:
            # Cleanup temporary files
            config_file_path = Path(trial.user_attrs.get('config_file', ''))
            if config_file_path.exists():
                config_file_path.unlink()

    def optimize(self, n_trials: int = 30) -> Dict[str, Any]:
        """Run Bayesian optimization"""
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
        logger.info(f"Best success rate: {best_score:.4f}")
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
        best_config = f"""# Optimal STOMP parameters found by Optuna success rate optimization
# Trial {best_trial.number} - Success Rate: {best_trial.value:.4f}
csv_file: "{self.scenario_config['csv_file']}"
obstacles_file: "{self.scenario_config['obstacles_file']}"
urdf_file: "{self.scenario_config['urdf_file']}"
output_file: "optimal_success_results.json"

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
  trajectory_pairs: 20

# Performance metrics:
# Success Rate: {best_trial.user_attrs.get('success_rate', 'N/A')}
# Avg Planning Time: {best_trial.user_attrs.get('avg_planning_time_ms', 'N/A')}ms
"""
        
        config_path = Path("optimal_success_rate_config.yaml")
        config_path.write_text(best_config)
        logger.info(f"Saved optimal config to: {config_path}")

    def generate_plots(self):
        """Generate optimization visualization plots"""
        try:
            fig1 = plot_optimization_history(self.study)
            fig1.write_html(self.results_dir / "success_rate_optimization_history.html")
            
            fig2 = plot_param_importances(self.study)
            fig2.write_html(self.results_dir / "success_rate_parameter_importance.html")
            
            logger.info("Generated visualization plots:")
            logger.info(f"  - {self.results_dir}/success_rate_optimization_history.html")
            logger.info(f"  - {self.results_dir}/success_rate_parameter_importance.html")
        except Exception as e:
            logger.warning(f"Could not generate plots: {e}")

def main():
    """Main optimization loop"""
    print("🎯 STOMP SUCCESS RATE OPTIMIZATION WITH OPTUNA")
    print("=" * 60)
    
    # Scenario configuration - using the complicated scenario with 127 poses
    scenario_config = {
        'csv_file': '/Users/joris/Uni/MA/Code/PathPlanner_US_wip/apps/ParameterTuning/Data/Ext_Ant_Scan/scan_pose_list_ee.csv',
        'obstacles_file': '/Users/joris/Uni/MA/Code/PathPlanner_US_wip/apps/ParameterTuning/Data/Ext_Ant_Scan/obstacles.xml',
        'urdf_file': '/Users/joris/Uni/MA/Code/PathPlanner_US_wip/res/scenario_1/panda_US.urdf'
    }
    
    # Create optimizer
    optimizer = STOMPSuccessRateOptimizer(scenario_config, n_startup_trials=10)
    
    # Run optimization
    results = optimizer.optimize(n_trials=10)  # Quick test with conservative parameters
    
    print("\n🎯 OPTIMIZATION COMPLETE!")
    print("=" * 40)
    print(f"Best success rate: {results['best_score']:.4f}")
    print("\nOptimal parameters saved to: optimal_success_rate_config.yaml")
    print("Visualizations saved to: results/")
    print("Study database: optuna_success_rate_study.db")

if __name__ == "__main__":
    main()
