#!/usr/bin/env python3
"""
Simplified Parameter Tuning with Actual C++ Evaluator

This script runs parameter tuning using only Optuna optimizer, which has been
verified to work correctly with the C++ evaluator.
"""

import os
import sys
import json
import yaml
import logging
import tempfile
import subprocess
from pathlib import Path
from typing import Dict, List
import time

# Add the current directory to Python path
sys.path.append(str(Path(__file__).parent))

def setup_logging():
    """Setup logging configuration."""
    logging.basicConfig(
        level=logging.INFO,
        format='%(asctime)s - %(name)s - %(levelname)s - %(message)s',
        handlers=[
            logging.FileHandler('simplified_parameter_tuning.log'),
            logging.StreamHandler()
        ]
    )
    return logging.getLogger(__name__)

def load_scenario_poses():
    """Load poses from scenario_1/scan_poses.csv."""
    import pandas as pd
    poses_file = Path(__file__).parent.parent.parent / "res" / "scenario_1" / "scan_poses.csv"
    
    if not poses_file.exists():
        logging.warning(f"Poses file not found: {poses_file}")
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
        
        logging.info(f"Loaded {len(poses)} poses from scenario_1")
        return poses
    
    except Exception as e:
        logging.error(f"Error loading poses: {e}")
        return []

def create_scenarios():
    """Create test scenarios using scenario_1 data."""
    scenario_poses = load_scenario_poses()
    
    # Default robot start configuration
    start_config = [-0.785398, 0.196349, -0.196349, -1.047197, 0.0, 1.570796, 0.785398]
    
    scenarios = []
    
    if scenario_poses:
        # Simple scenario: first 3 poses
        scenarios.append({
            'name': 'scenario_1_simple',
            'description': 'Simple ultrasound scanning motion using first 3 poses',
            'difficulty': 1,
            'start_config': start_config,
            'target_poses': scenario_poses[:3],
            'environment': '../../res/scenario_1/obstacles.xml',
            'urdf': '../../res/scenario_1/panda_US.urdf'
        })
        
        # Medium scenario: 8 poses
        scenarios.append({
            'name': 'scenario_1_medium',
            'description': 'Medium complexity scanning with 8 poses',
            'difficulty': 2,
            'start_config': start_config,
            'target_poses': scenario_poses[:8],
            'environment': '../../res/scenario_1/obstacles.xml',
            'urdf': '../../res/scenario_1/panda_US.urdf'
        })
    
    return scenarios

class SimplifiedEvaluator:
    """Simplified evaluator using only the C++ backend."""
    
    def __init__(self, cpp_executable_path: str):
        self.cpp_executable = Path(cpp_executable_path)
        self.temp_dir = Path(tempfile.mkdtemp())
        
        # Validate paths
        scenario_1_dir = Path(__file__).parent.parent.parent / "res" / "scenario_1"
        self.obstacles_file = scenario_1_dir / "obstacles.xml"
        self.urdf_file = scenario_1_dir / "panda_US.urdf"
        self.poses_file = scenario_1_dir / "scan_poses.csv"
        
        if not self.cpp_executable.exists():
            raise FileNotFoundError(f"C++ executable not found: {cpp_executable_path}")
    
    def evaluate_parameters(self, algorithm: str, parameters: Dict, scenarios: List[Dict]) -> Dict:
        """Evaluate algorithm with given parameters."""
        config_file = self.temp_dir / f"config_{algorithm}_{int(time.time() * 1000000)}.yaml"
        
        config_data = {
            'algorithm': algorithm,
            'parameters': parameters,
            'scenarios': [],
            'evaluation_settings': {
                'num_runs_per_scenario': 3,
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
        for scenario in scenarios:
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
        
        try:
            # Write YAML config
            with open(config_file, 'w') as f:
                yaml.dump(config_data, f, default_flow_style=False)
            
            # Run C++ evaluator
            result = subprocess.run([
                str(self.cpp_executable),
                '--config', str(config_file),
                '--output-format', 'json'
            ], capture_output=True, text=True, timeout=300)
            
            if result.returncode != 0:
                logging.error(f"C++ evaluator failed: {result.stderr}")
                return self._get_failure_metrics()
            
            # Parse results
            try:
                metrics = json.loads(result.stdout)
                return self._process_metrics(metrics)
            except json.JSONDecodeError as e:
                logging.error(f"Failed to parse JSON: {e}")
                logging.error(f"Output was: {result.stdout}")
                return self._get_failure_metrics()
                
        except subprocess.TimeoutExpired:
            logging.warning("Evaluation timeout")
            return self._get_failure_metrics()
        except Exception as e:
            logging.error(f"Evaluation error: {e}")
            return self._get_failure_metrics()
        finally:
            # Cleanup
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
        time_penalty = min(processed['avg_planning_time_ms'] / 1000.0, 10.0)
        success_bonus = processed['success_rate']
        path_penalty = min(processed['avg_path_length'] / 10.0, 5.0) if processed['avg_path_length'] != float('inf') else 5.0
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
            'avg_planning_time_ms': 30000.0,
            'avg_path_length': float('inf'),
            'avg_smoothness_score': 0.0,
            'avg_safety_score': 0.0,
            'composite_objective': 10.0
        }

def get_parameter_space(algorithm: str):
    """Get parameter space for optimization."""
    if algorithm == 'STOMP':
        return {
            'exploration_constant': (0.001, 0.5),
            'num_noisy_trajectories': (5, 100),
            'num_best_samples': (3, 20),
            'max_iterations': (50, 1000),
            'learning_rate': (0.1, 0.7),
            'temperature': (5.0, 50.0),
            'dt': (0.01, 0.2),
            'adaptive_sampling': [True, False],
            'early_termination': [True, False]
        }
    elif algorithm == 'Hauser':
        return {
            'max_deviation': (0.1, 2.0),
            'time_step': (0.01, 0.5),
            'max_iterations': (100, 2000),
            'tolerance': (1e-6, 1e-3),
            'acceleration_limit': (0.5, 5.0),
            'velocity_limit': (0.5, 3.0),
            'interpolation_dt': (0.01, 0.1)
        }
    else:
        raise ValueError(f"Unknown algorithm: {algorithm}")

def optimize_with_optuna(evaluator, algorithm: str, scenarios: List[Dict], n_trials: int = 100):
    """Optimize parameters using Optuna."""
    import optuna
    
    logger = logging.getLogger(__name__)
    param_space = get_parameter_space(algorithm)
    
    def objective(trial):
        # Sample parameters
        params = {}
        for param_name, param_range in param_space.items():
            if isinstance(param_range, tuple) and len(param_range) == 2:
                if isinstance(param_range[0], float):
                    params[param_name] = trial.suggest_float(param_name, param_range[0], param_range[1])
                elif isinstance(param_range[0], int):
                    params[param_name] = trial.suggest_int(param_name, param_range[0], param_range[1])
            elif isinstance(param_range, list):
                params[param_name] = trial.suggest_categorical(param_name, param_range)
        
        # Evaluate parameters
        metrics = evaluator.evaluate_parameters(algorithm, params, scenarios)
        return metrics['composite_objective']
    
    # Create study and optimize
    study = optuna.create_study(direction='minimize')
    study.optimize(objective, n_trials=n_trials)
    
    logger.info(f"Best {algorithm} objective: {study.best_value:.6f}")
    logger.info(f"Best {algorithm} parameters: {study.best_params}")
    
    return {
        'algorithm': algorithm,
        'best_objective': study.best_value,
        'best_parameters': study.best_params,
        'study': study
    }

def save_results(results: Dict, output_file: str):
    """Save optimization results to file."""
    # Convert optuna study to serializable format
    if 'study' in results:
        study = results.pop('study')
        results['trials'] = []
        for trial in study.trials:
            trial_data = {
                'number': trial.number,
                'value': trial.value,
                'params': trial.params,
                'state': str(trial.state)
            }
            results['trials'].append(trial_data)
    
    with open(output_file, 'w') as f:
        json.dump(results, f, indent=2)

def main():
    """Main function."""
    logger = setup_logging()
    logger.info("Starting Simplified Parameter Tuning")
    
    # Find C++ executable
    project_root = Path(__file__).parent.parent.parent
    cpp_executable = project_root / "build" / "apps" / "ParameterTuning" / "EnhancedParameterEvaluator"
    
    if not cpp_executable.exists():
        logger.error(f"C++ executable not found: {cpp_executable}")
        logger.error("Please build the project first:")
        logger.error("  cd PathPlanner_US_wip/build && make EnhancedParameterEvaluator")
        return 1
    
    logger.info(f"Using C++ evaluator: {cpp_executable}")
    
    # Create evaluator and scenarios
    evaluator = SimplifiedEvaluator(str(cpp_executable))
    scenarios = create_scenarios()
    
    if not scenarios:
        logger.error("No valid scenarios could be created")
        return 1
    
    logger.info(f"Created {len(scenarios)} test scenarios")
    
    # Create output directory
    output_dir = Path("simplified_tuning_results")
    output_dir.mkdir(exist_ok=True)
    
    # Optimize STOMP parameters
    logger.info("Optimizing STOMP parameters...")
    stomp_results = optimize_with_optuna(evaluator, 'STOMP', scenarios, n_trials=50)
    save_results(stomp_results, output_dir / "stomp_results.json")
    
    # Optimize Hauser parameters
    logger.info("Optimizing Hauser parameters...")
    hauser_results = optimize_with_optuna(evaluator, 'Hauser', scenarios, n_trials=50)
    save_results(hauser_results, output_dir / "hauser_results.json")
    
    # Print summary
    logger.info("\n" + "="*50)
    logger.info("OPTIMIZATION SUMMARY")
    logger.info("="*50)
    logger.info(f"STOMP Best Objective: {stomp_results['best_objective']:.6f}")
    logger.info(f"STOMP Best Parameters: {stomp_results['best_parameters']}")
    logger.info(f"Hauser Best Objective: {hauser_results['best_objective']:.6f}")
    logger.info(f"Hauser Best Parameters: {hauser_results['best_parameters']}")
    
    # Determine winner
    if stomp_results['best_objective'] < hauser_results['best_objective']:
        logger.info(f"\n🏆 WINNER: STOMP (objective: {stomp_results['best_objective']:.6f})")
    else:
        logger.info(f"\n🏆 WINNER: Hauser (objective: {hauser_results['best_objective']:.6f})")
    
    logger.info(f"\nResults saved to: {output_dir}/")
    return 0

if __name__ == "__main__":
    exit(main())