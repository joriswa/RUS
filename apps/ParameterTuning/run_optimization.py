#!/usr/bin/env python3
"""
STOMP Parameter Optimization with Correct YAML Format
Uses the hard cases data for optimization
"""

import json
import subprocess
import yaml
import time
import random
from pathlib import Path
import logging

# Setup logging
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(message)s')
logger = logging.getLogger(__name__)

def create_config(params):
    """Create YAML config file with correct format for parameter_evaluator"""
    config = {
        'csv_file': '/Users/joris/Uni/MA/Code/PathPlanner_US_wip/apps/ParameterTuning/Data/Ext_Ant_Scan/scan_pose_list_ee.csv',
        'obstacles_file': '/Users/joris/Uni/MA/Code/PathPlanner_US_wip/apps/ParameterTuning/Data/Ext_Ant_Scan/obstacles.xml',
        'urdf_file': '/Users/joris/Uni/MA/robot_definition/panda_US.urdf',
        'output_file': f'optimization_results_{int(time.time())}.json',
        'stomp': {
            'N': params['N'],
            'temperature': params['temperature'],
            'learning_rate': params['learning_rate'],
            'max_iterations': params['max_iterations'],
            'num_noisy_trajectories': params['num_noisy_trajectories'],
            'num_best_samples': params['num_best_samples'],
            'obstacle_cost_weight': params['obstacle_cost_weight'],
            'constraint_cost_weight': params['constraint_cost_weight'],
            'joint_std_devs': params['joint_std_devs']
        },
        'evaluation': {
            'trajectory_pairs': params['trajectory_pairs']
        }
    }
    return config

def evaluate_parameters(params):
    """Evaluate STOMP parameters using the C++ parameter_evaluator"""
    config = create_config(params)
    
    # Create temporary config file
    config_file = f'/tmp/stomp_opt_config_{int(time.time() * 1000000) % 1000000}.yaml'
    with open(config_file, 'w') as f:
        yaml.dump(config, f, default_flow_style=False)
    
    try:
        # Run parameter evaluator
        evaluator_path = '/Users/joris/Uni/MA/Code/PathPlanner_US_wip/apps/ParameterTuning/parameter_evaluator'
        result = subprocess.run([evaluator_path, config_file], 
                              capture_output=True, text=True, timeout=300)
        
        if result.returncode != 0:
            logger.error(f"Evaluation failed: {result.stderr}")
            return None
        
        # Extract objective value
        for line in result.stdout.split('\n'):
            if line.startswith('OBJECTIVE_VALUE:'):
                score = float(line.split(':')[1].strip())
                
                # Extract additional metrics for logging
                success_rate = 0.0
                avg_time = 0.0
                for output_line in result.stdout.split('\n'):
                    if 'Success Rate:' in output_line:
                        success_rate = float(output_line.split(':')[1].strip().replace('%', ''))
                    elif 'Avg Planning Time:' in output_line:
                        time_str = output_line.split(':')[1].strip().replace('ms', '')
                        avg_time = float(time_str)
                
                return {
                    'score': score,
                    'success_rate': success_rate,
                    'avg_time': avg_time
                }
        
        logger.error("Could not find OBJECTIVE_VALUE in output")
        return None
        
    except subprocess.TimeoutExpired:
        logger.error("Evaluation timed out")
        return None
    except Exception as e:
        logger.error(f"Evaluation error: {e}")
        return None
    finally:
        # Cleanup
        if Path(config_file).exists():
            Path(config_file).unlink()

def random_search_optimization(n_trials=20):
    """Run random search optimization"""
    logger.info(f"Starting random search optimization with {n_trials} trials")
    logger.info("Using hard cases from scan_pose_list_ee.csv")
    
    best_score = float('inf')
    best_params = None
    results = []
    
    for trial in range(n_trials):
        # Generate random parameters
        params = {
            'N': random.choice([50, 60, 70, 80]),
            'temperature': random.uniform(5.0, 25.0),
            'learning_rate': random.uniform(0.2, 0.8),
            'max_iterations': random.choice([40, 50, 60]),
            'num_noisy_trajectories': random.choice([8, 12, 16, 20]),
            'num_best_samples': random.choice([4, 6, 8]),
            'obstacle_cost_weight': random.uniform(1.0, 4.0),
            'constraint_cost_weight': random.uniform(1.0, 3.0),
            'joint_std_devs': [random.uniform(0.02, 0.12) for _ in range(7)],
            'trajectory_pairs': 10  # Test with 10 pairs for good evaluation
        }
        
        logger.info(f"Trial {trial+1}/{n_trials}: T={params['temperature']:.2f}, "
                   f"LR={params['learning_rate']:.2f}, N={params['N']}, "
                   f"Iter={params['max_iterations']}")
        
        result = evaluate_parameters(params)
        
        if result is not None:
            score = result['score']
            success_rate = result['success_rate']
            avg_time = result['avg_time']
            
            logger.info(f"  Score: {score:.3f}, Success: {success_rate:.1f}%, Time: {avg_time:.1f}ms")
            
            if score < best_score:
                best_score = score
                best_params = params.copy()
                logger.info(f"  🎯 NEW BEST! Score: {score:.3f}")
            
            results.append({
                'trial': trial + 1,
                'params': params,
                'score': score,
                'success_rate': success_rate,
                'avg_time': avg_time
            })
        else:
            logger.warning(f"  ❌ Trial {trial+1} failed")
    
    return best_params, best_score, results

def grid_search_optimization():
    """Run grid search on key parameters"""
    logger.info("Starting grid search optimization")
    
    # Grid search space
    temperature_values = [8.0, 12.0, 16.0, 20.0]
    learning_rate_values = [0.3, 0.5, 0.7]
    iterations_values = [40, 50]
    
    best_score = float('inf')
    best_params = None
    results = []
    
    total_combinations = len(temperature_values) * len(learning_rate_values) * len(iterations_values)
    trial = 0
    
    for temp in temperature_values:
        for lr in learning_rate_values:
            for iterations in iterations_values:
                trial += 1
                
                params = {
                    'N': 60,  # Fixed
                    'temperature': temp,
                    'learning_rate': lr,
                    'max_iterations': iterations,
                    'num_noisy_trajectories': 12,  # Fixed
                    'num_best_samples': 6,  # Fixed
                    'obstacle_cost_weight': 2.5,  # Fixed
                    'constraint_cost_weight': 1.5,  # Fixed
                    'joint_std_devs': [0.06, 0.02, 0.05, 0.08, 0.04, 0.07, 0.09],  # Fixed
                    'trajectory_pairs': 8
                }
                
                logger.info(f"Grid trial {trial}/{total_combinations}: T={temp}, LR={lr}, Iter={iterations}")
                
                result = evaluate_parameters(params)
                
                if result is not None:
                    score = result['score']
                    success_rate = result['success_rate']
                    avg_time = result['avg_time']
                    
                    logger.info(f"  Score: {score:.3f}, Success: {success_rate:.1f}%, Time: {avg_time:.1f}ms")
                    
                    if score < best_score:
                        best_score = score
                        best_params = params.copy()
                        logger.info(f"  🎯 NEW BEST! Score: {score:.3f}")
                    
                    results.append({
                        'trial': trial,
                        'params': params,
                        'score': score,
                        'success_rate': success_rate,
                        'avg_time': avg_time
                    })
                else:
                    logger.warning(f"  ❌ Grid trial {trial} failed")
    
    return best_params, best_score, results

def save_results(best_params, best_score, results, method_name):
    """Save optimization results"""
    output_data = {
        'method': method_name,
        'best_score': best_score,
        'best_params': best_params,
        'all_results': results,
        'timestamp': time.time()
    }
    
    output_file = f'optimization_results_{method_name}_{int(time.time())}.json'
    with open(output_file, 'w') as f:
        json.dump(output_data, f, indent=2)
    
    logger.info(f"Results saved to: {output_file}")
    
    # Also save best config in YAML format
    if best_params:
        best_config = create_config(best_params)
        best_config['evaluation']['trajectory_pairs'] = 20  # More pairs for final validation
        
        yaml_file = f'best_config_{method_name}_{int(time.time())}.yaml'
        with open(yaml_file, 'w') as f:
            f.write(f"# Best STOMP parameters found by {method_name}\n")
            f.write(f"# Best score: {best_score:.4f}\n\n")
            yaml.dump(best_config, f, default_flow_style=False)
        
        logger.info(f"Best config saved to: {yaml_file}")

def main():
    print("🚀 STOMP PARAMETER OPTIMIZATION WITH HARD CASES")
    print("=" * 60)
    print("Using 127 challenging poses from scan_pose_list_ee.csv")
    print("=" * 60)
    
    # Method 1: Grid Search (systematic)
    print("\n🔍 Method 1: Grid Search Optimization")
    print("-" * 40)
    best_params_grid, best_score_grid, results_grid = grid_search_optimization()
    save_results(best_params_grid, best_score_grid, results_grid, 'grid_search')
    
    # Method 2: Random Search (exploration)
    print("\n🎲 Method 2: Random Search Optimization")
    print("-" * 40)
    best_params_random, best_score_random, results_random = random_search_optimization(n_trials=15)
    save_results(best_params_random, best_score_random, results_random, 'random_search')
    
    # Final comparison
    print("\n🏆 OPTIMIZATION COMPLETE!")
    print("=" * 40)
    print(f"Grid Search Best Score: {best_score_grid:.4f}")
    print(f"Random Search Best Score: {best_score_random:.4f}")
    
    if best_score_grid < best_score_random:
        print("🥇 Grid Search found the best parameters!")
        final_best = best_params_grid
        final_score = best_score_grid
    else:
        print("🥇 Random Search found the best parameters!")
        final_best = best_params_random
        final_score = best_score_random
    
    print(f"\nFinal Best Score: {final_score:.4f}")
    print("Final Best Parameters:")
    for key, value in final_best.items():
        if key == 'joint_std_devs':
            print(f"  {key}: [{', '.join([f'{v:.3f}' for v in value])}]")
        elif isinstance(value, float):
            print(f"  {key}: {value:.3f}")
        else:
            print(f"  {key}: {value}")

if __name__ == "__main__":
    main()
