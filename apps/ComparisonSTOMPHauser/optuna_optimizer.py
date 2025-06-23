#!/usr/bin/env python3
"""
Optuna-based Parameter Optimization for Trajectory Planning Algorithms

This script provides comprehensive parameter optimization for:
- STOMP (Stochastic Trajectory Optimization for Motion Planning)
- RRT variants (RRT, RRT*, Informed RRT*, RRT-Connect)
- Hauser algorithm with different path planning backends

Uses Optuna for efficient hyperparameter optimization with support for:
- Single-objective optimization
- Multi-objective optimization (Pareto fronts)
- Parallel trials
- Study persistence
- Advanced pruning strategies
"""

import optuna
import json
import subprocess
import pandas as pd
import numpy as np
import os
import logging
import argparse
import time
from pathlib import Path
from typing import Dict, List, Tuple, Optional, Any
from dataclasses import dataclass, asdict
from enum import Enum

# Configure logging
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')
logger = logging.getLogger(__name__)

class Algorithm(Enum):
    STOMP = "stomp"
    RRT = "rrt"
    RRT_STAR = "rrt_star"
    INFORMED_RRT_STAR = "informed_rrt_star"
    RRT_CONNECT = "rrt_connect"
    HAUSER = "hauser"

class ObjectiveType(Enum):
    SUCCESS_RATE = "success_rate"
    PLANNING_TIME = "planning_time"
    TRAJECTORY_QUALITY = "trajectory_quality"
    COMPOSITE = "composite"

@dataclass
class StompConfig:
    """STOMP algorithm configuration parameters"""
    max_iterations: int = 100
    num_noisy_trajectories: int = 10
    num_best_samples: int = 4
    learning_rate: float = 0.1
    temperature: float = 10.0
    dt: float = 0.1
    joint_std_devs: List[float] = None
    
    def __post_init__(self):
        if self.joint_std_devs is None:
            self.joint_std_devs = [0.1] * 7

@dataclass
class RRTConfig:
    """RRT algorithm configuration parameters"""
    algorithm: str = "RRT"  # RRT, RRTStar, InformedRRTStar, RRTConnect
    step_size: float = 0.05
    goal_bias_probability: float = 0.5
    max_iterations: int = 5000
    custom_cost: bool = False

@dataclass
class HauserConfig:
    """Hauser algorithm configuration parameters"""
    max_iterations: int = 1000
    output_frequency: float = 100.0
    path_planning_algorithm: str = "RRT"
    path_planning_config: RRTConfig = None
    
    def __post_init__(self):
        if self.path_planning_config is None:
            self.path_planning_config = RRTConfig(algorithm=self.path_planning_algorithm)

@dataclass
class OptimizationConfig:
    """Main configuration for parameter optimization"""
    algorithm: Algorithm
    n_trials: int = 100
    n_parallel_jobs: int = 4
    study_name: str = "trajectory_optimization"
    storage_url: str = "sqlite:///optuna_studies.db"
    objectives: List[ObjectiveType] = None
    min_success_rate: float = 0.8  # Constraint: minimum acceptable success rate
    
    def __post_init__(self):
        if self.objectives is None:
            self.objectives = [ObjectiveType.COMPOSITE]

class TrajectoryOptimizer:
    """Main class for trajectory planning parameter optimization using Optuna"""
    
    def __init__(self, config: OptimizationConfig):
        self.config = config
        self.cpp_executable = "./build/apps/ComparisonSTOMPHauser/SimpleConfigurableComparison"
        self.results_dir = Path("optuna_results")
        self.results_dir.mkdir(exist_ok=True)
        
        # Create study
        if len(self.config.objectives) > 1:
            # Multi-objective optimization
            directions = []
            for obj in self.config.objectives:
                if obj in [ObjectiveType.SUCCESS_RATE]:
                    directions.append("maximize")
                else:
                    directions.append("minimize")  # Planning time, trajectory length
            
            self.study = optuna.create_study(
                directions=directions,
                study_name=self.config.study_name,
                storage=self.config.storage_url,
                load_if_exists=True
            )
        else:
            # Single-objective optimization
            direction = "maximize" if self.config.objectives[0] == ObjectiveType.SUCCESS_RATE else "minimize"
            self.study = optuna.create_study(
                direction=direction,
                study_name=self.config.study_name,
                storage=self.config.storage_url,
                load_if_exists=True
            )
    
    def suggest_stomp_parameters(self, trial: optuna.Trial) -> StompConfig:
        """Suggest STOMP parameters for optimization"""
        return StompConfig(
            max_iterations=trial.suggest_int("stomp_max_iterations", 50, 500),
            num_noisy_trajectories=trial.suggest_int("stomp_num_noisy_trajectories", 5, 25),
            num_best_samples=trial.suggest_int("stomp_num_best_samples", 2, 10),
            learning_rate=trial.suggest_float("stomp_learning_rate", 0.01, 0.3, log=True),
            temperature=trial.suggest_float("stomp_temperature", 1.0, 25.0),
            dt=trial.suggest_float("stomp_dt", 0.02, 0.2),
            joint_std_devs=[
                trial.suggest_float(f"stomp_joint_std_dev_{i}", 0.01, 0.3) 
                for i in range(7)
            ]
        )
    
    def suggest_rrt_parameters(self, trial: optuna.Trial) -> RRTConfig:
        """Suggest RRT parameters for optimization"""
        algorithm = trial.suggest_categorical("rrt_algorithm", 
                                            ["RRT", "RRTStar", "InformedRRTStar", "RRTConnect"])
        
        return RRTConfig(
            algorithm=algorithm,
            step_size=trial.suggest_float("rrt_step_size", 0.005, 0.15),
            goal_bias_probability=trial.suggest_float("rrt_goal_bias", 0.05, 0.95),
            max_iterations=trial.suggest_int("rrt_max_iterations", 1000, 15000),
            custom_cost=trial.suggest_categorical("rrt_custom_cost", [True, False])
        )
    
    def suggest_hauser_parameters(self, trial: optuna.Trial) -> HauserConfig:
        """Suggest Hauser parameters for optimization"""
        path_planning_config = self.suggest_rrt_parameters(trial)
        
        return HauserConfig(
            max_iterations=trial.suggest_int("hauser_max_iterations", 200, 3000),
            output_frequency=trial.suggest_float("hauser_output_frequency", 25.0, 200.0),
            path_planning_algorithm=path_planning_config.algorithm,
            path_planning_config=path_planning_config
        )
    
    def create_config_file(self, trial_params: Dict[str, Any], trial_id: int) -> str:
        """Create simple config file for C++ application"""
        config_file = self.results_dir / f"config_trial_{trial_id}.txt"
        
        config_lines = [
            f"algorithm={self.config.algorithm.value}",
            f"num_poses=5",
            f"trials_per_pose=10",
            f"output_directory={self.results_dir / f'trial_{trial_id}'}",
        ]
        
        # Add algorithm-specific parameters
        if self.config.algorithm == Algorithm.STOMP:
            config_lines.extend([
                f"stomp_max_iterations={trial_params.get('max_iterations', 100)}",
                f"stomp_num_noisy_trajectories={trial_params.get('num_noisy_trajectories', 10)}",
                f"stomp_num_best_samples={trial_params.get('num_best_samples', 4)}",
                f"stomp_learning_rate={trial_params.get('learning_rate', 0.1)}",
                f"stomp_temperature={trial_params.get('temperature', 10.0)}",
                f"stomp_dt={trial_params.get('dt', 0.1)}",
            ])
            if 'joint_std_devs' in trial_params:
                joint_std_devs_str = ','.join(map(str, trial_params['joint_std_devs']))
                config_lines.append(f"stomp_joint_std_devs={joint_std_devs_str}")
        
        elif self.config.algorithm in [Algorithm.RRT, Algorithm.RRT_STAR, Algorithm.INFORMED_RRT_STAR, Algorithm.RRT_CONNECT, Algorithm.HAUSER]:
            config_lines.extend([
                f"rrt_step_size={trial_params.get('step_size', 0.1)}",
                f"rrt_goal_bias_probability={trial_params.get('goal_bias_probability', 0.05)}",
                f"rrt_max_iterations={trial_params.get('max_iterations', 5000)}",
                f"rrt_custom_cost={1 if trial_params.get('custom_cost', False) else 0}",
            ])
        
        with open(config_file, 'w') as f:
            f.write('\n'.join(config_lines))
        
        return str(config_file)
    
    def run_cpp_trial(self, config_file: str, trial_id: int) -> Dict[str, float]:
        """Execute C++ comparison application with given configuration"""
        output_dir = self.results_dir / f"trial_{trial_id}"
        output_dir.mkdir(exist_ok=True)
        
        # Run C++ application with simple command line interface
        cmd = [
            self.cpp_executable,
            config_file,
            str(output_dir),
            "5",  # num_poses
            "10"  # trials_per_pose
        ]
        
        try:
            result = subprocess.run(
                cmd, 
                capture_output=True, 
                text=True, 
                timeout=300,  # 5 minute timeout
                cwd="/Users/joris/Uni/MA/Code/PathPlanner_US_wip"
            )
            
            if result.returncode != 0:
                logger.error(f"C++ execution failed: {result.stderr}")
                return {"success_rate": 0.0, "planning_time": 1000.0, "trajectory_quality": 1000.0}
            
            # Parse results
            return self.parse_results(output_dir)
            
        except subprocess.TimeoutExpired:
            logger.warning(f"Trial {trial_id} timed out")
            return {"success_rate": 0.0, "planning_time": 1000.0, "trajectory_quality": 1000.0}
        except Exception as e:
            logger.error(f"Error running trial {trial_id}: {e}")
            return {"success_rate": 0.0, "planning_time": 1000.0, "trajectory_quality": 1000.0}
    
    def parse_results(self, output_dir: Path) -> Dict[str, float]:
        """Parse results from C++ application output"""
        try:
            # Look for CSV results file
            csv_files = list(output_dir.glob("*.csv"))
            if not csv_files:
                logger.error(f"No CSV results found in {output_dir}")
                return {"success_rate": 0.0, "planning_time": 1000.0, "trajectory_quality": 1000.0}
            
            # Read the first CSV file found
            df = pd.read_csv(csv_files[0])
            
            if df.empty:
                return {"success_rate": 0.0, "planning_time": 1000.0, "trajectory_quality": 1000.0}
            
            # Calculate metrics
            success_rate = df['success'].mean() if 'success' in df.columns else 0.0
            avg_planning_time = df[df['success'] == True]['planning_time_ms'].mean() if 'planning_time_ms' in df.columns else 1000.0
            
            # Trajectory quality metrics (lower is better)
            avg_trajectory_duration = df[df['success'] == True]['trajectory_duration_s'].mean() if 'trajectory_duration_s' in df.columns else 1000.0
            avg_trajectory_points = df[df['success'] == True]['trajectory_points'].mean() if 'trajectory_points' in df.columns else 1000.0
            
            # Composite trajectory quality (normalize and combine)
            trajectory_quality = (avg_trajectory_duration / 10.0) + (avg_trajectory_points / 100.0)
            
            # Handle NaN values
            if pd.isna(avg_planning_time):
                avg_planning_time = 1000.0
            if pd.isna(trajectory_quality):
                trajectory_quality = 1000.0
            
            return {
                "success_rate": success_rate,
                "planning_time": avg_planning_time,
                "trajectory_quality": trajectory_quality
            }
            
        except Exception as e:
            logger.error(f"Error parsing results: {e}")
            return {"success_rate": 0.0, "planning_time": 1000.0, "trajectory_quality": 1000.0}
    
    def objective_function(self, trial: optuna.Trial) -> float | List[float]:
        """Objective function for Optuna optimization"""
        trial_id = trial.number
        
        # Suggest parameters based on algorithm
        if self.config.algorithm == Algorithm.STOMP:
            params = asdict(self.suggest_stomp_parameters(trial))
        elif self.config.algorithm in [Algorithm.RRT, Algorithm.RRT_STAR, 
                                     Algorithm.INFORMED_RRT_STAR, Algorithm.RRT_CONNECT]:
            params = asdict(self.suggest_rrt_parameters(trial))
        elif self.config.algorithm == Algorithm.HAUSER:
            params = asdict(self.suggest_hauser_parameters(trial))
        else:
            raise ValueError(f"Unsupported algorithm: {self.config.algorithm}")
        
        # Create configuration file
        config_file = self.create_config_file(params, trial_id)
        
        # Run trial
        results = self.run_cpp_trial(config_file, trial_id)
        
        # Apply constraints
        if results["success_rate"] < self.config.min_success_rate:
            # Penalize trials that don't meet minimum success rate
            if len(self.config.objectives) > 1:
                return [0.0, 1000.0]  # Low success rate, high planning time
            else:
                return 0.0 if self.config.objectives[0] == ObjectiveType.SUCCESS_RATE else 1000.0
        
        # Return objectives
        if len(self.config.objectives) == 1:
            obj_type = self.config.objectives[0]
            if obj_type == ObjectiveType.SUCCESS_RATE:
                return results["success_rate"]
            elif obj_type == ObjectiveType.PLANNING_TIME:
                return results["planning_time"]
            elif obj_type == ObjectiveType.TRAJECTORY_QUALITY:
                return results["trajectory_quality"]
            elif obj_type == ObjectiveType.COMPOSITE:
                # Weighted composite score
                return (0.5 * results["success_rate"] - 
                       0.3 * (results["planning_time"] / 1000.0) - 
                       0.2 * (results["trajectory_quality"] / 10.0))
        else:
            # Multi-objective: return list of objective values
            objectives = []
            for obj_type in self.config.objectives:
                if obj_type == ObjectiveType.SUCCESS_RATE:
                    objectives.append(results["success_rate"])
                elif obj_type == ObjectiveType.PLANNING_TIME:
                    objectives.append(results["planning_time"])
                elif obj_type == ObjectiveType.TRAJECTORY_QUALITY:
                    objectives.append(results["trajectory_quality"])
            return objectives
    
    def optimize(self):
        """Run the optimization study"""
        logger.info(f"Starting optimization for {self.config.algorithm.value}")
        logger.info(f"Number of trials: {self.config.n_trials}")
        logger.info(f"Objectives: {[obj.value for obj in self.config.objectives]}")
        
        # Optimize study
        self.study.optimize(
            self.objective_function,
            n_trials=self.config.n_trials,
            n_jobs=self.config.n_parallel_jobs,
            show_progress_bar=True
        )
        
        # Print results
        self.print_results()
        
        # Save results
        self.save_results()
        
        # Return the study for further analysis
        return self.study
    
    def print_results(self):
        """Print optimization results"""
        logger.info("Optimization completed!")
        
        if len(self.config.objectives) == 1:
            # Single-objective results
            best_trial = self.study.best_trial
            logger.info(f"Best trial: {best_trial.number}")
            logger.info(f"Best value: {best_trial.value}")
            logger.info("Best parameters:")
            for key, value in best_trial.params.items():
                logger.info(f"  {key}: {value}")
        else:
            # Multi-objective results (Pareto front)
            pareto_trials = self.study.best_trials
            logger.info(f"Number of Pareto optimal solutions: {len(pareto_trials)}")
            logger.info("Pareto front solutions:")
            for i, trial in enumerate(pareto_trials[:5]):  # Show top 5
                logger.info(f"  Solution {i+1}: values={trial.values}, trial={trial.number}")
    
    def save_results(self):
        """Save optimization results to files"""
        results_file = self.results_dir / f"optimization_results_{self.config.algorithm.value}.json"
        
        if len(self.config.objectives) == 1:
            best_trial = self.study.best_trial
            results = {
                "algorithm": self.config.algorithm.value,
                "best_trial": best_trial.number,
                "best_value": best_trial.value,
                "best_parameters": best_trial.params,
                "n_trials": len(self.study.trials)
            }
        else:
            pareto_trials = self.study.best_trials
            results = {
                "algorithm": self.config.algorithm.value,
                "pareto_solutions": [
                    {
                        "trial": trial.number,
                        "values": trial.values,
                        "parameters": trial.params
                    }
                    for trial in pareto_trials
                ],
                "n_trials": len(self.study.trials)
            }
        
        with open(results_file, 'w') as f:
            json.dump(results, f, indent=2)
        
        logger.info(f"Results saved to {results_file}")

def main():
    parser = argparse.ArgumentParser(description="Optuna-based trajectory planning optimization")
    parser.add_argument("--algorithm", choices=["stomp", "rrt", "hauser"], required=True,
                       help="Algorithm to optimize")
    parser.add_argument("--trials", type=int, default=100,
                       help="Number of optimization trials")
    parser.add_argument("--jobs", type=int, default=4,
                       help="Number of parallel jobs")
    parser.add_argument("--study-name", default="trajectory_optimization",
                       help="Optuna study name")
    parser.add_argument("--multi-objective", action="store_true",
                       help="Use multi-objective optimization")
    parser.add_argument("--min-success-rate", type=float, default=0.8,
                       help="Minimum required success rate")
    
    args = parser.parse_args()
    
    # Configure objectives
    if args.multi_objective:
        objectives = [ObjectiveType.SUCCESS_RATE, ObjectiveType.PLANNING_TIME]
    else:
        objectives = [ObjectiveType.COMPOSITE]
    
    # Create configuration
    config = OptimizationConfig(
        algorithm=Algorithm(args.algorithm),
        n_trials=args.trials,
        n_parallel_jobs=args.jobs,
        study_name=f"{args.study_name}_{args.algorithm}",
        objectives=objectives,
        min_success_rate=args.min_success_rate
    )
    
    # Run optimization
    optimizer = TrajectoryOptimizer(config)
    optimizer.optimize()

if __name__ == "__main__":
    main()
