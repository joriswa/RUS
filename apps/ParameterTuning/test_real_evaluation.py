#!/usr/bin/env python3
"""
Test real parameter evaluation with actual values
"""

import json
import subprocess
import yaml
from pathlib import Path
import numpy as np

def test_real_evaluation():
    """Test the C++ evaluator with real parameter values"""
    
    # Real parameter configuration
    real_params = {
        'algorithm': 'STOMP',
        'parameters': {
            'STOMP': {
                'num_noisy_trajectories': 50,
                'num_best_samples': 20,
                'max_iterations': 100,
                'learning_rate': 0.3,
                'temperature': 15.0,
                'dt': 0.05,
                'velocity_limit': 1.5,
                'acceleration_limit': 0.4
            }
        },
        'scenario': {
            'urdf_file': '../../res/scenario_1/panda_US.urdf',
            'environment_file': '../../res/scenario_1/obstacles.xml',
            'poses_file': '../../res/scenario_1/scan_poses.csv',
            'scenario_id': 'real_scenario_1'
        }
    }
    
    # Save config
    config_file = Path("real_test_config.yaml")
    with open(config_file, 'w') as f:
        yaml.dump(real_params, f)
    
    print("🔬 Testing real parameter evaluation...")
    print(f"Parameters: {real_params['parameters']['STOMP']}")
    
    try:
        # Run C++ evaluator
        cmd = ['./EnhancedParameterEvaluator', '--config', str(config_file), '--output-format', 'json']
        print(f"Running: {' '.join(cmd)}")
        
        result = subprocess.run(cmd, capture_output=True, text=True, timeout=60)
        
        if result.returncode == 0:
            output_data = json.loads(result.stdout)
            
            print("\n✅ REAL EVALUATION RESULTS:")
            print(f"Planning time: {output_data.get('planning_time', 'N/A')} seconds")
            print(f"Trajectory cost: {output_data.get('trajectory_cost', 'N/A')}")
            print(f"Success: {output_data.get('success', 'N/A')}")
            print(f"Energy consumption: {output_data.get('energy_consumption', 'N/A')}")
            print(f"Collision margin: {output_data.get('collision_margin', 'N/A')}")
            
            # Test multiple parameter sets
            print("\n🎯 Testing parameter variations...")
            test_multiple_params()
            
        else:
            print(f"❌ Evaluation failed: {result.stderr}")
            
    except subprocess.TimeoutExpired:
        print("❌ Evaluation timed out")
    except Exception as e:
        print(f"❌ Error: {e}")
    finally:
        if config_file.exists():
            config_file.unlink()

def test_multiple_params():
    """Test different parameter configurations"""
    
    # Parameter variations to test
    param_variations = [
        {
            'name': 'Fast_LowQuality',
            'num_noisy_trajectories': 20,
            'max_iterations': 50,
            'learning_rate': 0.5,
            'temperature': 25.0
        },
        {
            'name': 'Slow_HighQuality', 
            'num_noisy_trajectories': 80,
            'max_iterations': 200,
            'learning_rate': 0.2,
            'temperature': 10.0
        },
        {
            'name': 'Balanced',
            'num_noisy_trajectories': 50,
            'max_iterations': 100,
            'learning_rate': 0.3,
            'temperature': 15.0
        }
    ]
    
    results = []
    
    for variation in param_variations:
        print(f"\nTesting {variation['name']}...")
        
        # Base parameters
        params = {
            'num_noisy_trajectories': variation['num_noisy_trajectories'],
            'num_best_samples': 15,
            'max_iterations': variation['max_iterations'],
            'learning_rate': variation['learning_rate'],
            'temperature': variation['temperature'],
            'dt': 0.05,
            'velocity_limit': 1.5,
            'acceleration_limit': 0.4
        }
        
        # Evaluate
        result = evaluate_parameters(params)
        if result:
            result['config_name'] = variation['name']
            results.append(result)
            
            print(f"  Time: {result.get('planning_time', 'N/A'):.2f}s")
            print(f"  Cost: {result.get('trajectory_cost', 'N/A'):.3f}")
            print(f"  Success: {result.get('success', 'N/A')}")
    
    if results:
        print("\n📊 PARAMETER COMPARISON:")
        print("Config\t\tTime(s)\tCost\tSuccess")
        print("-" * 45)
        for r in results:
            print(f"{r['config_name']:<15}\t{r.get('planning_time', 0):.2f}\t{r.get('trajectory_cost', 0):.3f}\t{r.get('success', False)}")
        
        # Find best trade-offs
        successful = [r for r in results if r.get('success', False)]
        if successful:
            fastest = min(successful, key=lambda x: x.get('planning_time', float('inf')))
            best_quality = min(successful, key=lambda x: x.get('trajectory_cost', float('inf')))
            
            print(f"\n🏆 REAL OPTIMIZATION INSIGHTS:")
            print(f"Fastest: {fastest['config_name']} ({fastest.get('planning_time', 0):.2f}s)")
            print(f"Best quality: {best_quality['config_name']} (cost: {best_quality.get('trajectory_cost', 0):.3f})")

def evaluate_parameters(params):
    """Evaluate a single parameter configuration"""
    
    config = {
        'algorithm': 'STOMP',
        'parameters': {'STOMP': params},
        'scenario': {
            'urdf_file': '../../res/scenario_1/panda_US.urdf',
            'environment_file': '../../res/scenario_1/obstacles.xml', 
            'poses_file': '../../res/scenario_1/scan_poses.csv',
            'scenario_id': 'real_test'
        }
    }
    
    config_file = Path("temp_test_config.yaml")
    try:
        with open(config_file, 'w') as f:
            yaml.dump(config, f)
            
        result = subprocess.run(['./EnhancedParameterEvaluator', '--config', str(config_file), '--output-format', 'json'], 
                              capture_output=True, text=True, timeout=30)
        
        if result.returncode == 0:
            return json.loads(result.stdout)
        else:
            print(f"    ❌ Evaluation failed: {result.stderr[:100]}")
            return None
            
    except Exception as e:
        print(f"    ❌ Error: {e}")
        return None
    finally:
        if config_file.exists():
            config_file.unlink()

if __name__ == '__main__':
    test_real_evaluation()
