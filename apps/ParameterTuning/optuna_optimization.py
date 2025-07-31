#!/usr/bin/env python3
"""
Comprehensive STOMP Parameter Optimization using Optuna
With Heavy Penalties for Failing Trajectories
"""

import optuna
import subprocess
import json
import argparse
import yaml
import os
import sys
from pathlib import Path
import logging

# Configure logging
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')
logger = logging.getLogger(__name__)

class STOMPOptimizer:
    def __init__(self, evaluations_per_trial=50):
        self.evaluations_per_trial = evaluations_per_trial
        self.base_config = {
            "csv_file": "/Users/joris/Uni/MA/Code/PathPlanner_US_wip/apps/ParameterTuning/Data/Ext_Ant_Scan/scan_pose_list_ee.csv",
            "obstacles_file": "/Users/joris/Uni/MA/Code/PathPlanner_US_wip/apps/ParameterTuning/Data/Ext_Ant_Scan/obstacles.xml",
            "urdf_file": "/Users/joris/Uni/MA/robot_definition/panda_US.urdf",
            "output_file": "trial_results.json"
        }
        self.working_dir = Path("/Users/joris/Uni/MA/Code/PathPlanner_US_wip")
        self.trial_count = 0
        
    def create_config(self, trial):
        """Create STOMP configuration for this trial"""
        config = self.base_config.copy()
        
        # STOMP parameters to optimize
        config["stomp"] = {
            "temperature": trial.suggest_float("temperature", 1.0, 20.0),
            "learning_rate": trial.suggest_float("learning_rate", 0.1, 1.0),
            "max_iterations": 50,  # Keep fixed for consistency
            "N": 50,  # Keep trajectory points fixed
            "num_noisy_trajectories": trial.suggest_int("num_noisy_trajectories", 4, 12),
            "num_best_samples": trial.suggest_int("num_best_samples", 2, 8),
            "obstacle_cost_weight": trial.suggest_float("obstacle_cost_weight", 0.5, 5.0),
            "constraint_cost_weight": trial.suggest_float("constraint_cost_weight", 0.5, 8.0),
            "joint_std_devs": [
                trial.suggest_float(f"joint_std_dev_{i}", 0.01, 0.15) for i in range(7)
            ]
        }
        
        # Evaluation parameters
        config["evaluation"] = {
            "trajectory_pairs": self.evaluations_per_trial
        }
        
        return config
        
    def run_evaluation(self, config):
        """Run parameter evaluation and return composite score"""
        config_file = self.working_dir / f"trial_config_{self.trial_count}.yaml"
        
        try:
            # Write configuration file
            with open(config_file, 'w') as f:
                yaml.dump(config, f, default_flow_style=False)
            
            # Run parameter evaluator (compiled with Release + O3)
            cmd = [str(self.working_dir / "build/apps/ParameterTuning/parameter_evaluator"), str(config_file)]
            result = subprocess.run(cmd, capture_output=True, text=True, timeout=300)
            
            if result.returncode != 0:
                logger.error(f"Parameter evaluator failed: {result.stderr}")
                return 1000  # Heavy penalty for evaluation failure
            
            # Extract objective value (composite score with heavy penalties)
            lines = result.stdout.strip().split('\n')
            for line in lines:
                if line.startswith('OBJECTIVE_VALUE:'):
                    score = float(line.split(':')[1].strip())
                    
                    # Log trial results
                    logger.info(f"Trial {self.trial_count}: Score = {score:.3f}")
                    if "Success Rate:" in result.stdout:
                        for output_line in lines:
                            if "Success Rate:" in output_line:
                                success_rate = output_line.split(':')[1].strip()
                                logger.info(f"Trial {self.trial_count}: Success Rate = {success_rate}")
                    
                    return score
                    
            logger.error("Could not find OBJECTIVE_VALUE in output")
            return 1000  # Heavy penalty for parsing failure
            
        except subprocess.TimeoutExpired:
            logger.error(f"Trial {self.trial_count} timed out")
            return 1000  # Heavy penalty for timeout
        except Exception as e:
            logger.error(f"Trial {self.trial_count} failed: {e}")
            return 1000  # Heavy penalty for any error
        finally:
            # Clean up config file
            if config_file.exists():
                config_file.unlink()
    
    def objective(self, trial):
        """Optuna objective function"""
        self.trial_count += 1
        logger.info(f"Starting Trial {self.trial_count}/50 with {self.evaluations_per_trial} trajectory evaluations")
        
        # Create configuration for this trial
        config = self.create_config(trial)
        
        # Run evaluation with heavy penalties for failures
        score = self.run_evaluation(config)
        
        logger.info(f"Trial {self.trial_count} completed with score: {score:.3f}")
        
        # Optuna minimizes, so return the composite score (lower is better)
        return score

def main():
    parser = argparse.ArgumentParser(description='STOMP Parameter Optimization with Heavy Failure Penalties')
    parser.add_argument('--trials', type=int, default=50, help='Number of optimization trials')
    parser.add_argument('--evaluations', type=int, default=50, help='Number of trajectory evaluations per trial')
    parser.add_argument('--study-name', type=str, default='stomp_heavy_penalty_optimization', help='Optuna study name')
    
    args = parser.parse_args()
    
    logger.info(f"Starting STOMP optimization with heavy failure penalties:")
    logger.info(f"  - Trials: {args.trials}")
    logger.info(f"  - Evaluations per trial: {args.evaluations}")
    logger.info(f"  - Total trajectory evaluations: {args.trials * args.evaluations}")
    logger.info(f"  - Heavy penalties: 50x multiplier for STOMP failures")
    
    # Create optimizer
    optimizer = STOMPOptimizer(evaluations_per_trial=args.evaluations)
    
    # Create Optuna study
    study = optuna.create_study(
        direction='minimize',  # Minimize composite score (lower is better)
        study_name=args.study_name,
        sampler=optuna.samplers.TPESampler(n_startup_trials=10)  # Bayesian optimization
    )
    
    try:
        # Run optimization
        study.optimize(optimizer.objective, n_trials=args.trials)
        
        # Log best results
        logger.info("=" * 60)
        logger.info("OPTIMIZATION COMPLETED!")
        logger.info(f"Best trial: {study.best_trial.number}")
        logger.info(f"Best score: {study.best_value:.6f}")
        logger.info("Best parameters:")
        for key, value in study.best_params.items():
            logger.info(f"  {key}: {value}")
        
        # Save best configuration
        best_config = optimizer.base_config.copy()
        best_config["stomp"] = {
            "temperature": study.best_params["temperature"],
            "learning_rate": study.best_params["learning_rate"],
            "max_iterations": 50,
            "N": 50,
            "num_noisy_trajectories": study.best_params["num_noisy_trajectories"],
            "num_best_samples": study.best_params["num_best_samples"],
            "obstacle_cost_weight": study.best_params["obstacle_cost_weight"],
            "constraint_cost_weight": study.best_params["constraint_cost_weight"],
            "joint_std_devs": [study.best_params[f"joint_std_dev_{i}"] for i in range(7)]
        }
        best_config["evaluation"] = {"trajectory_pairs": args.evaluations}
        
        # Save optimized configuration
        output_file = Path("/Users/joris/Uni/MA/Code/PathPlanner_US_wip/optimized_heavy_penalty_config.yaml")
        with open(output_file, 'w') as f:
            f.write(f"# Optimized STOMP parameters with heavy failure penalties\n")
            f.write(f"# Best trial: {study.best_trial.number}\n") 
            f.write(f"# Best composite score: {study.best_value:.6f}\n")
            f.write(f"# Total evaluations: {args.trials * args.evaluations}\n\n")
            yaml.dump(best_config, f, default_flow_style=False)
        
        logger.info(f"Best configuration saved to: {output_file}")
        
        # Print comparison with current optimal config
        logger.info("=" * 60)
        logger.info("OPTIMIZATION SUMMARY:")
        logger.info(f"Previous best score: 6.4 (from comprehensive evaluation)")
        logger.info(f"New optimized score: {study.best_value:.6f}")
        improvement = ((6.4 - study.best_value) / 6.4) * 100
        logger.info(f"Improvement: {improvement:.1f}% {'better' if improvement > 0 else 'worse'}")
        
    except KeyboardInterrupt:
        logger.info("Optimization interrupted by user")
        if study.trials:
            logger.info(f"Best result so far: {study.best_value:.6f}")
    except Exception as e:
        logger.error(f"Optimization failed: {e}")
        sys.exit(1)

if __name__ == "__main__":
    main()
