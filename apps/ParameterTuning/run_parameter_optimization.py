#!/usr/bin/env python3
"""
Parameter Optimization for Trajectory Planning Algorithms

This script provides a clean interface for optimizing STOMP and Hauser trajectory 
planning parameters using real motion generation libraries and ultrasound scanning scenarios.

Usage:
    python run_parameter_optimization.py [options]

Examples:
    # Quick optimization (20 trials each)
    python run_parameter_optimization.py --quick
    
    # Full optimization (100 trials each)
    python run_parameter_optimization.py --full
    
    # Optimize only STOMP
    python run_parameter_optimization.py --algorithm STOMP --trials 50
    
    # Custom optimization with specific settings
    python run_parameter_optimization.py --algorithm both --trials 75 --scenarios all
"""

import os
import sys
import json
import yaml
import logging
import argparse
import tempfile
import subprocess
from pathlib import Path
from typing import Dict, List, Optional
import time

# Add current directory to path for imports
sys.path.append(str(Path(__file__).parent))

def setup_logging(log_file: str = "parameter_optimization.log") -> logging.Logger:
    """Setup logging configuration."""
    logging.basicConfig(
        level=logging.INFO,
        format='%(asctime)s - %(levelname)s - %(message)s',
        handlers=[
            logging.FileHandler(log_file),
            logging.StreamHandler(sys.stdout)
        ]
    )
    return logging.getLogger(__name__)

def check_dependencies() -> bool:
    """Check if required dependencies are available."""
    logger = logging.getLogger(__name__)
    
    required_packages = [
        'numpy', 'pandas', 'matplotlib', 'seaborn', 
        'optuna', 'yaml', 'scipy'
    ]
    
    missing_packages = []
    for package in required_packages:
        try:
            __import__(package)
        except ImportError:
            missing_packages.append(package)
    
    if missing_packages:
        logger.error(f"Missing required packages: {missing_packages}")
        logger.error("Please install them using: pip install " + " ".join(missing_packages))
        return False
    
    return True

def find_cpp_evaluator() -> Optional[Path]:
    """Find the compiled C++ parameter evaluator."""
    script_dir = Path(__file__).parent
    project_root = script_dir.parent.parent
    
    # Look for the executable in build directory
    potential_paths = [
        project_root / "build" / "apps" / "ParameterTuning" / "EnhancedParameterEvaluator",
        project_root / "build" / "EnhancedParameterEvaluator",
        script_dir / "EnhancedParameterEvaluator"
    ]
    
    for path in potential_paths:
        if path.exists() and path.is_file():
            return path
    
    return None

def verify_scenario_data() -> bool:
    """Verify that required scenario data files exist."""
    script_dir = Path(__file__).parent
    project_root = script_dir.parent.parent
    scenario_dir = project_root / "res" / "scenario_1"
    
    required_files = [
        "obstacles.xml",
        "panda_US.urdf", 
        "scan_poses.csv"
    ]
    
    for file_name in required_files:
        file_path = scenario_dir / file_name
        if not file_path.exists():
            return False
    
    return True

def load_scenario_poses() -> List[Dict]:
    """Load poses from scenario_1/scan_poses.csv."""
    try:
        import pandas as pd
    except ImportError:
        logging.error("pandas not available for loading poses")
        return []
    
    poses_file = Path(__file__).parent.parent.parent / "res" / "scenario_1" / "scan_poses.csv"
    
    if not poses_file.exists():
        logging.warning(f"Poses file not found: {poses_file}")
        return []
    
    try:
        # Read CSV: x,y,z,qx,qy,qz,qw,contact,distance,index
        df = pd.read_csv(poses_file, header=None)
        poses = []
        
        for _, row in df.iterrows():
            pose_data = {
                'position': [float(row[0]), float(row[1]), float(row[2])],
                'orientation': [float(row[6]), float(row[3]), float(row[4]), float(row[5])],  # w,x,y,z
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

def create_test_scenarios(scenario_type: str = "standard") -> List[Dict]:
    """Create test scenarios based on type."""
    scenario_poses = load_scenario_poses()
    start_config = [-0.785398, 0.196349, -0.196349, -1.047197, 0.0, 1.570796, 0.785398]
    
    scenarios = []
    
    if not scenario_poses:
        logging.warning("No poses available, creating minimal scenario")
        return [{
            'name': 'minimal_test',
            'description': 'Minimal configuration test',
            'difficulty': 1,
            'start_config': start_config,
            'environment': '../../res/scenario_1/obstacles.xml',
            'urdf': '../../res/scenario_1/panda_US.urdf'
        }]
    
    if scenario_type in ["quick", "standard"]:
        # Simple scenario: 3 poses
        scenarios.append({
            'name': 'scenario_1_simple',
            'description': 'Simple ultrasound scanning (3 poses)',
            'difficulty': 1,
            'start_config': start_config,
            'target_poses': scenario_poses[:3],
            'environment': '../../res/scenario_1/obstacles.xml',
            'urdf': '../../res/scenario_1/panda_US.urdf'
        })
    
    if scenario_type in ["standard", "full", "all"]:
        # Medium scenario: 8 poses
        scenarios.append({
            'name': 'scenario_1_medium',
            'description': 'Medium complexity scanning (8 poses)',
            'difficulty': 2,
            'start_config': start_config,
            'target_poses': scenario_poses[:8],
            'environment': '../../res/scenario_1/obstacles.xml',
            'urdf': '../../res/scenario_1/panda_US.urdf'
        })
    
    if scenario_type in ["full", "all"]:
        # Complete scenario: all poses
        scenarios.append({
            'name': 'scenario_1_complete',
            'description': 'Complete scanning sequence (all poses)',
            'difficulty': 3,
            'start_config': start_config,
            'target_poses': scenario_poses,
            'environment': '../../res/scenario_1/obstacles.xml',
            'urdf': '../../res/scenario_1/panda_US.urdf'
        })
    
    return scenarios

class ParameterOptimizer:
    """Main parameter optimization class."""
    
    def __init__(self, cpp_executable: Path, output_dir: Path):
        self.cpp_executable = cpp_executable
        self.output_dir = output_dir
        self.temp_dir = Path(tempfile.mkdtemp())
        self.logger = logging.getLogger(__name__)
        
        # Validate paths
        scenario_1_dir = Path(__file__).parent.parent.parent / "res" / "scenario_1"
        self.obstacles_file = scenario_1_dir / "obstacles.xml"
        self.urdf_file = scenario_1_dir / "panda_US.urdf"
        self.poses_file = scenario_1_dir / "scan_poses.csv"
    
    def get_parameter_space(self, algorithm: str) -> Dict:
        """Get parameter space for the specified algorithm."""
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
    
    def evaluate_parameters(self, algorithm: str, parameters: Dict, scenarios: List[Dict]) -> Dict:
        """Evaluate parameters using the C++ evaluator."""
        config_file = self.temp_dir / f"config_{algorithm}_{int(time.time() * 1000000)}.yaml"
        
        # Create configuration
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
        
        # Process scenarios
        for scenario in scenarios:
            scenario_data = {
                'name': scenario['name'],
                'description': scenario['description'],
                'difficulty': scenario['difficulty'],
                'start_config': scenario['start_config'],
                'environment': scenario['environment'],
                'urdf': scenario['urdf']
            }
            
            if 'target_poses' in scenario:
                scenario_data['target_poses'] = scenario['target_poses']
            
            config_data['scenarios'].append(scenario_data)
        
        try:
            # Write configuration
            with open(config_file, 'w') as f:
                yaml.dump(config_data, f, default_flow_style=False)
            
            # Run evaluator
            result = subprocess.run([
                str(self.cpp_executable),
                '--config', str(config_file),
                '--output-format', 'json'
            ], capture_output=True, text=True, timeout=300)
            
            if result.returncode != 0:
                self.logger.error(f"C++ evaluator failed: {result.stderr}")
                return self._get_failure_metrics()
            
            # Parse results
            try:
                metrics = json.loads(result.stdout)
                return self._process_metrics(metrics)
            except json.JSONDecodeError as e:
                self.logger.error(f"Failed to parse JSON: {e}")
                return self._get_failure_metrics()
                
        except subprocess.TimeoutExpired:
            self.logger.warning("Evaluation timeout")
            return self._get_failure_metrics()
        except Exception as e:
            self.logger.error(f"Evaluation error: {e}")
            return self._get_failure_metrics()
        finally:
            if config_file.exists():
                config_file.unlink()
    
    def _process_metrics(self, raw_metrics: Dict) -> Dict:
        """Process raw metrics into composite objective."""
        processed = {
            'success_rate': raw_metrics.get('success_rate', 0.0),
            'avg_planning_time_ms': raw_metrics.get('avg_planning_time_ms', 10000.0),
            'avg_path_length': raw_metrics.get('avg_path_length', float('inf')),
            'avg_smoothness_score': raw_metrics.get('avg_smoothness_score', 0.0),
            'avg_safety_score': raw_metrics.get('avg_safety_score', 0.0),
        }
        
        # Composite objective (lower is better)
        weights = {
            'time_weight': 0.3,
            'success_weight': 0.25,
            'path_weight': 0.2,
            'safety_weight': 0.15,
            'smoothness_weight': 0.1
        }
        
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
        """Return failure metrics."""
        return {
            'success_rate': 0.0,
            'avg_planning_time_ms': 30000.0,
            'avg_path_length': float('inf'),
            'avg_smoothness_score': 0.0,
            'avg_safety_score': 0.0,
            'composite_objective': 10.0
        }
    
    def optimize_algorithm(self, algorithm: str, scenarios: List[Dict], n_trials: int = 50) -> Dict:
        """Optimize parameters for a specific algorithm."""
        try:
            import optuna
            optuna.logging.set_verbosity(optuna.logging.WARNING)
        except ImportError:
            self.logger.error("Optuna not available for optimization")
            return {}
        
        self.logger.info(f"Starting {algorithm} optimization with {n_trials} trials...")
        param_space = self.get_parameter_space(algorithm)
        
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
            
            # Evaluate
            metrics = self.evaluate_parameters(algorithm, params, scenarios)
            return metrics['composite_objective']
        
        # Optimize
        study = optuna.create_study(direction='minimize')
        study.optimize(objective, n_trials=n_trials)
        
        self.logger.info(f"{algorithm} optimization complete!")
        self.logger.info(f"Best objective: {study.best_value:.6f}")
        
        # Save results
        results = {
            'algorithm': algorithm,
            'best_objective': study.best_value,
            'best_parameters': study.best_params,
            'trials': []
        }
        
        for trial in study.trials:
            trial_data = {
                'number': trial.number,
                'value': trial.value,
                'params': trial.params,
                'state': str(trial.state)
            }
            results['trials'].append(trial_data)
        
        # Save to file
        results_file = self.output_dir / f"{algorithm.lower()}_results.json"
        with open(results_file, 'w') as f:
            json.dump(results, f, indent=2)
        
        self.logger.info(f"Results saved to: {results_file}")
        return results

def main():
    """Main function."""
    parser = argparse.ArgumentParser(
        description='Parameter Optimization for Trajectory Planning',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__
    )
    
    parser.add_argument('--algorithm', choices=['STOMP', 'Hauser', 'both'], 
                       default='both', help='Algorithm to optimize')
    parser.add_argument('--trials', type=int, default=50, 
                       help='Number of optimization trials per algorithm')
    parser.add_argument('--scenarios', choices=['quick', 'standard', 'full', 'all'], 
                       default='standard', help='Test scenarios to use')
    parser.add_argument('--output-dir', default='results/optimization_results', 
                       help='Output directory for results')
    parser.add_argument('--quick', action='store_true', 
                       help='Quick optimization (20 trials, simple scenarios)')
    parser.add_argument('--full', action='store_true', 
                       help='Full optimization (100 trials, all scenarios)')
    parser.add_argument('--log-file', default='parameter_optimization.log', 
                       help='Log file path')
    
    args = parser.parse_args()
    
    # Handle quick/full presets
    if args.quick:
        args.trials = 20
        args.scenarios = 'quick'
    elif args.full:
        args.trials = 100
        args.scenarios = 'all'
    
    # Setup logging
    logger = setup_logging(args.log_file)
    logger.info("Starting Parameter Optimization System")
    
    # Check dependencies
    if not check_dependencies():
        return 1
    
    # Find C++ evaluator
    cpp_executable = find_cpp_evaluator()
    if not cpp_executable:
        logger.error("C++ evaluator not found!")
        logger.error("Please build the project first:")
        logger.error("  cd PathPlanner_US_wip/build")
        logger.error("  make EnhancedParameterEvaluator")
        return 1
    
    logger.info(f"Found C++ evaluator: {cpp_executable}")
    
    # Verify scenario data
    if not verify_scenario_data():
        logger.error("Required scenario data files not found!")
        logger.error("Please ensure res/scenario_1/ contains:")
        logger.error("  - obstacles.xml")
        logger.error("  - panda_US.urdf")
        logger.error("  - scan_poses.csv")
        return 1
    
    # Create output directory
    output_dir = Path(args.output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)
    
    # Create scenarios
    scenarios = create_test_scenarios(args.scenarios)
    logger.info(f"Created {len(scenarios)} test scenarios ({args.scenarios} mode)")
    
    # Initialize optimizer
    optimizer = ParameterOptimizer(cpp_executable, output_dir)
    
    # Run optimization
    algorithms = [args.algorithm] if args.algorithm != 'both' else ['STOMP', 'Hauser']
    results = {}
    
    for algorithm in algorithms:
        try:
            result = optimizer.optimize_algorithm(algorithm, scenarios, args.trials)
            results[algorithm] = result
        except Exception as e:
            logger.error(f"Optimization failed for {algorithm}: {e}")
            results[algorithm] = None
    
    # Print summary
    logger.info("\n" + "="*60)
    logger.info("OPTIMIZATION COMPLETE!")
    logger.info("="*60)
    
    best_algorithms = []
    for algorithm, result in results.items():
        if result:
            objective = result.get('best_objective', float('inf'))
            logger.info(f"{algorithm:>8}: {objective:.6f} (best objective)")
            best_algorithms.append((algorithm, objective))
    
    if len(best_algorithms) >= 2:
        best_algorithms.sort(key=lambda x: x[1])
        winner = best_algorithms[0][0]
        improvement = abs((best_algorithms[1][1] - best_algorithms[0][1]) / best_algorithms[1][1]) * 100
        logger.info(f"\n🏆 Winner: {winner}")
        logger.info(f"📊 Performance improvement: {improvement:.4f}%")
    
    logger.info(f"\n📁 Results saved to: {output_dir}")
    logger.info("📋 Run visualization script to generate plots:")
    logger.info("    python create_comprehensive_plots.py")
    
    return 0

if __name__ == "__main__":
    exit(main())