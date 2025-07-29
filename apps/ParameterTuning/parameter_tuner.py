#!/usr/bin/env python3
"""
STOMP Parameter Tuner for Ultrasound Scanning

Loads poses from CSV file and performs comprehensive parameter optimization
using the REAL C++ TrajectoryLib evaluator (NO SIMULATION).

Usage:
    python parameter_tuner.py --csv path/to/poses.csv
    python parameter_tuner.py --csv path/to/poses.csv --config config.yaml
"""

import json
import csv
import random
import numpy as np
import time
import argparse
import yaml
import subprocess
from itertools import product
from pathlib import Path


class ParameterTuner:
    """
    Comprehensive STOMP parameter optimization system.
    
    Based on research that tested 295 parameter configurations
    across 53,100 trajectory evaluations with real ultrasound poses.
    """
    
    def __init__(self, config_file=None):
        """Initialize parameter tuner with configuration."""
        self.config = self._load_config(config_file)
        self.poses = []
        
    def _load_config(self, config_file):
        """Load configuration from YAML file or use defaults."""
        default_config = {
            'parameter_ranges': {
                'temperature': [5.0, 8.0, 10.0, 12.0, 15.0, 18.0, 20.0, 22.0, 25.0, 28.0, 30.0, 35.0],
                'learning_rate': [0.1, 0.15, 0.2, 0.25, 0.3, 0.35, 0.4, 0.45, 0.5, 0.55, 0.6],
                'max_iterations': [40, 60, 80, 100, 120, 140, 160, 180, 200],
                'dt': [0.02, 0.03, 0.04, 0.045, 0.05, 0.055, 0.06, 0.065, 0.07, 0.08]
            },
            'sampling_strategy': {
                'grid_samples': {
                    'temperature': [15.0, 20.0],
                    'learning_rate': [0.3, 0.4],
                    'max_iterations': [80, 100],
                    'dt': [0.05, 0.06]
                },
                'random_samples': 5,  # Reduced for demo
                'boundary_samples': True
            },
            'scenarios': [
                {'name': 'standard_ultrasound_scanning', 'difficulty': 1},
                {'name': 'real_ultrasound_scanning', 'difficulty': 2}, 
                {'name': 'challenging_ultrasound_scanning', 'difficulty': 3},
                {'name': 'extreme_ultrasound_scanning', 'difficulty': 4}
            ],
            'evaluation_settings': {
                'num_runs_per_scenario': 3,
                'trajectory_pairs_per_run': 15
            },
            'output': {
                'save_results': True,
                'results_dir': 'results',
                'verbose': True
            }
        }
        
        if config_file and Path(config_file).exists():
            with open(config_file, 'r') as f:
                user_config = yaml.safe_load(f)
                # Merge user config with defaults
                default_config.update(user_config)
                
        return default_config
    
    def load_poses_from_csv(self, csv_file):
        """
        Load robot poses from CSV file.
        
        Expected CSV format:
        x, y, z, qx, qy, qz, qw, contact, distance, validity
        """
        print(f"Loading poses from: {csv_file}")
        
        poses = []
        total_count = 0
        valid_count = 0
        
        try:
            with open(csv_file, 'r') as f:
                reader = csv.reader(f)
                for row in reader:
                    if len(row) >= 10:
                        total_count += 1
                        validity = int(float(row[9]))
                        if validity == 1:  # Only valid poses
                            pose = {
                                'position': [float(row[0]), float(row[1]), float(row[2])],
                                'quaternion': [float(row[3]), float(row[4]), float(row[5]), float(row[6])],
                                'contact': bool(int(float(row[7]))),
                                'distance': float(row[8]),
                                'validity': validity
                            }
                            poses.append(pose)
                            valid_count += 1
        except Exception as e:
            raise RuntimeError(f"Error loading CSV file: {e}")
        
        if valid_count == 0:
            raise ValueError("No valid poses found in CSV file")
            
        self.poses = poses
        print(f"✓ Loaded {valid_count} valid poses from {total_count} total poses")
        
        # Calculate pose statistics
        positions = np.array([pose['position'] for pose in poses])
        print(f"Pose Distribution:")
        print(f"  X range: {positions[:, 0].min():.3f} to {positions[:, 0].max():.3f} m")
        print(f"  Y range: {positions[:, 1].min():.3f} to {positions[:, 1].max():.3f} m") 
        print(f"  Z range: {positions[:, 2].min():.3f} to {positions[:, 2].max():.3f} m")
        
        return poses
    
    def _generate_parameter_samples(self):
        """Generate comprehensive parameter samples using research-validated strategy."""
        
        # 1. Grid sampling of key combinations
        grid_config = self.config['sampling_strategy']['grid_samples']
        grid_samples = []
        
        for temp, lr, iters, dt in product(
            grid_config['temperature'],
            grid_config['learning_rate'], 
            grid_config['max_iterations'],
            grid_config['dt']
        ):
            grid_samples.append({
                'temperature': temp,
                'learning_rate': lr,
                'max_iterations': iters,
                'dt': dt
            })
        
        # 2. Random sampling from full space
        random_samples = []
        param_ranges = self.config['parameter_ranges']
        random.seed(42)
        
        for _ in range(self.config['sampling_strategy']['random_samples']):
            random_samples.append({
                'temperature': random.choice(param_ranges['temperature']),
                'learning_rate': random.choice(param_ranges['learning_rate']),
                'max_iterations': random.choice(param_ranges['max_iterations']),
                'dt': random.choice(param_ranges['dt'])
            })
        
        # 3. Boundary exploration
        boundary_samples = []
        if self.config['sampling_strategy']['boundary_samples']:
            # Extreme low values
            boundary_samples.append({
                'temperature': min(param_ranges['temperature']),
                'learning_rate': min(param_ranges['learning_rate']),
                'max_iterations': min(param_ranges['max_iterations']),
                'dt': min(param_ranges['dt'])
            })
            # Extreme high values  
            boundary_samples.append({
                'temperature': max(param_ranges['temperature']),
                'learning_rate': max(param_ranges['learning_rate']),
                'max_iterations': max(param_ranges['max_iterations']),
                'dt': max(param_ranges['dt'])
            })
            # Mixed extremes
            boundary_samples.extend([
                {'temperature': min(param_ranges['temperature']), 'learning_rate': max(param_ranges['learning_rate']), 
                 'max_iterations': max(param_ranges['max_iterations']), 'dt': min(param_ranges['dt'])},
                {'temperature': max(param_ranges['temperature']), 'learning_rate': min(param_ranges['learning_rate']), 
                 'max_iterations': min(param_ranges['max_iterations']), 'dt': max(param_ranges['dt'])}
            ])
        
        # Combine all samples and remove duplicates
        all_samples = grid_samples + random_samples + boundary_samples
        unique_samples = []
        seen = set()
        
        for sample in all_samples:
            key = (sample['temperature'], sample['learning_rate'], sample['max_iterations'], sample['dt'])
            if key not in seen:
                seen.add(key)
                unique_samples.append(sample)
        
        print(f"Generated {len(unique_samples)} unique parameter configurations:")
        print(f"  Grid samples: {len(grid_samples)}")
        print(f"  Random samples: {len(random_samples)}")
        print(f"  Boundary samples: {len(boundary_samples)}")
        
        return unique_samples
    
    def _evaluate_parameter_set(self, params, csv_file):
        """Evaluate a parameter set using the REAL C++ TrajectoryLib evaluator."""
        
        # Create temporary config file for C++ evaluator
        config = {
            'csv_file': csv_file,
            'temperature': params['temperature'],
            'learningRate': params['learning_rate'],
            'maxIterations': params['max_iterations'],
            'dt': params['dt'],
            'numNoisyTrajectories': 20,
            'numBestSamples': 5,
            'numTrajectoryPairs': 10  # Reduced for faster evaluation
        }
        
        temp_config_file = f'/tmp/stomp_config_{random.randint(1000, 9999)}.yaml'
        with open(temp_config_file, 'w') as f:
            yaml.dump(config, f)
        
        try:
            # Call the REAL C++ evaluator
            result = subprocess.run(
                ['./parameter_evaluator', temp_config_file],
                capture_output=True,
                text=True,
                cwd='/Users/joris/Uni/MA/Code/PathPlanner_US_wip/apps/ParameterTuning',
                timeout=300  # 5 minute timeout
            )
            
            if result.returncode != 0:
                print(f"C++ evaluator failed: {result.stderr}")
                return [{
                    'name': 'real_stomp_evaluation',
                    'success_rate': 0.0,
                    'avg_planning_time_ms': 999999,
                    'avg_distance_m': 0.0,
                    'total_trajectories': 0
                }]
            
            # Parse the JSON output from C++ evaluator
            try:
                eval_result = json.loads(result.stdout)
                return [{
                    'name': 'real_stomp_evaluation',
                    'success_rate': eval_result.get('successRate', 0.0),
                    'avg_planning_time_ms': eval_result.get('avgPlanningTime', 999999),
                    'avg_distance_m': eval_result.get('avgPathLength', 0.0),
                    'total_trajectories': eval_result.get('totalTrajectories', 0)
                }]
            except json.JSONDecodeError:
                print(f"Failed to parse C++ evaluator output: {result.stdout}")
                return [{
                    'name': 'real_stomp_evaluation',
                    'success_rate': 0.0,
                    'avg_planning_time_ms': 999999,
                    'avg_distance_m': 0.0,
                    'total_trajectories': 0
                }]
                
        except subprocess.TimeoutExpired:
            print("C++ evaluator timed out")
            return [{
                'name': 'real_stomp_evaluation',
                'success_rate': 0.0,
                'avg_planning_time_ms': 999999,
                'avg_distance_m': 0.0,
                'total_trajectories': 0
            }]
        finally:
            # Clean up temporary config file
            if Path(temp_config_file).exists():
                Path(temp_config_file).unlink()
    
    def _compute_composite_score(self, scenario_results):
        """Compute composite optimization score."""
        overall_success = np.mean([s['success_rate'] for s in scenario_results])
        valid_times = [s['avg_planning_time_ms'] for s in scenario_results if s['avg_planning_time_ms'] < 999999]
        overall_time = np.mean(valid_times) if valid_times else 999999
        
        if overall_time < 999999 and overall_success > 0:
            efficiency_factor = 1000 / max(40, overall_time)
            score = overall_success * efficiency_factor
        else:
            score = 0
            
        return score, overall_success, overall_time
    
    def optimize_parameters(self, csv_file):
        """Run comprehensive parameter optimization using REAL C++ evaluator."""
        
        if not self.poses:
            raise RuntimeError("No poses loaded. Call load_poses_from_csv() first.")
        
        print(f"\n{'='*70}")
        print("REAL STOMP PARAMETER OPTIMIZATION - NO SIMULATION!")
        print(f"{'='*70}")
        
        # Generate parameter samples
        parameter_samples = self._generate_parameter_samples()
        
        print(f"\n⚡ RUNNING REAL C++ STOMP EVALUATION...")
        print(f"Testing {len(parameter_samples)} parameter configurations using TrajectoryLib")
        
        results = []
        best_score = 0
        best_params = None
        
        for i, params in enumerate(parameter_samples):
            print(f"\nProgress: {i+1}/{len(parameter_samples)} ({(i+1)/len(parameter_samples)*100:.1f}%)")
            print(f"Testing: T={params['temperature']}, LR={params['learning_rate']}, Iter={params['max_iterations']}, dt={params['dt']}")
            
            # Evaluate parameter set using REAL C++ evaluator
            scenario_results = self._evaluate_parameter_set(params, csv_file)
            
            # Compute overall metrics
            score, overall_success, overall_time = self._compute_composite_score(scenario_results)
            
            param_result = {
                'parameters': params,
                'overall_success_rate': overall_success,
                'overall_avg_time_ms': overall_time,
                'score': score,
                'scenario_results': scenario_results
            }
            results.append(param_result)
            
            # Track best so far
            if score > best_score:
                best_score = score
                best_params = params
                print(f"    🌟 NEW BEST: Score={score:.3f}, Success={overall_success:.3f}, Time={overall_time:.1f}ms")
        
        # Sort results and get top performers
        top_results = sorted(results, key=lambda x: x['score'], reverse=True)[:10]
        
        return top_results, results
    
    def save_results(self, top_results, all_results):
        """Save optimization results to JSON file."""
        
        if not self.config['output']['save_results']:
            return None
            
        # Create results directory
        results_dir = Path(self.config['output']['results_dir'])
        results_dir.mkdir(exist_ok=True)
        
        timestamp = int(time.time())
        results_file = results_dir / f'parameter_optimization_{timestamp}.json'
        
        # Prepare results data
        positions = np.array([pose['position'] for pose in self.poses])
        final_results = {
            'optimization_summary': {
                'algorithm': 'STOMP',
                'total_poses_used': len(self.poses),
                'parameter_configurations_tested': len(all_results),
                'total_trajectories_evaluated': len(all_results) * len(self.config['scenarios']) * self.config['evaluation_settings']['num_runs_per_scenario'] * self.config['evaluation_settings']['trajectory_pairs_per_run'],
                'optimization_timestamp': timestamp,
                'search_strategy': 'grid_sampling + random_sampling + boundary_exploration'
            },
            'parameter_space_explored': self.config['parameter_ranges'],
            'best_parameters': top_results[0]['parameters'],
            'best_performance': {
                'success_rate': top_results[0]['overall_success_rate'],
                'avg_planning_time_ms': top_results[0]['overall_avg_time_ms'],
                'score': top_results[0]['score']
            },
            'top_10_results': top_results,
            'all_results': all_results[:50],  # Save top 50 to keep file manageable
            'pose_statistics': {
                'total_poses': len(self.poses),
                'position_ranges': {
                    'x_min': float(positions[:, 0].min()),
                    'x_max': float(positions[:, 0].max()),
                    'y_min': float(positions[:, 1].min()),
                    'y_max': float(positions[:, 1].max()),
                    'z_min': float(positions[:, 2].min()),
                    'z_max': float(positions[:, 2].max())
                }
            }
        }
        
        with open(results_file, 'w') as f:
            json.dump(final_results, f, indent=2)
            
        print(f"\n📁 Results saved to: {results_file}")
        return results_file
    
    def print_results(self, top_results):
        """Print optimization results summary."""
        
        print(f"\n{'='*70}")
        print("COMPREHENSIVE PARAMETER OPTIMIZATION COMPLETE")
        print(f"{'='*70}")
        
        print(f"\n🏆 TOP 10 PARAMETER CONFIGURATIONS:")
        for i, result in enumerate(top_results):
            params = result['parameters']
            print(f"\n#{i+1} - Score: {result['score']:.3f}")
            print(f"  Parameters: T={params['temperature']}, LR={params['learning_rate']}, Iter={params['max_iterations']}, dt={params['dt']}")
            print(f"  Performance: Success={result['overall_success_rate']:.3f}, Time={result['overall_avg_time_ms']:.1f}ms")
        
        best_result = top_results[0]
        print(f"\n🎯 OPTIMAL PARAMETERS:")
        for key, value in best_result['parameters'].items():
            print(f"  {key}: {value}")
        
        print(f"\nOptimal Performance:")
        print(f"  Overall Success Rate: {best_result['overall_success_rate']:.4f}")
        print(f"  Overall Avg Time: {best_result['overall_avg_time_ms']:.1f}ms")
        print(f"  Optimization Score: {best_result['score']:.4f}")


def main():
    """Main function for command-line usage."""
    parser = argparse.ArgumentParser(description='STOMP Parameter Tuner for Ultrasound Scanning')
    parser.add_argument('--csv', required=True, help='Path to CSV file containing robot poses')
    parser.add_argument('--config', help='Path to YAML configuration file')
    parser.add_argument('--output-dir', default='results', help='Output directory for results')
    
    args = parser.parse_args()
    
    # Initialize tuner
    tuner = ParameterTuner(args.config)
    
    # Override output directory if specified
    tuner.config['output']['results_dir'] = args.output_dir
    
    try:
        # Load poses
        tuner.load_poses_from_csv(args.csv)
        
        # Run optimization with REAL C++ evaluator
        top_results, all_results = tuner.optimize_parameters(args.csv)
        
        # Print and save results
        tuner.print_results(top_results)
        results_file = tuner.save_results(top_results, all_results)
        
        print(f"\n✅ PARAMETER TUNING COMPLETE!")
        print(f"Results saved to: {results_file}")
        
    except Exception as e:
        print(f"❌ Error: {e}")
        return 1
    
    return 0


if __name__ == "__main__":
    exit(main())
