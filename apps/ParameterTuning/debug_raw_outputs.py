#!/usr/bin/env python3
"""
Debug Raw C++ Evaluator Outputs

This script runs simple tests with the C++ evaluator to examine raw outputs
and understand why parameters aren't having meaningful effects.
"""

import json
import yaml
import subprocess
import tempfile
from pathlib import Path
import time

def create_test_config(algorithm: str, parameters: dict) -> str:
    """Create a test configuration file."""
    scenario_1_dir = Path(__file__).parent.parent.parent / "res" / "scenario_1"
    
    # Simple single pose test
    simple_scenario = {
        'name': 'debug_test',
        'description': 'Debug test scenario',
        'difficulty': 1,
        'start_config': [-0.785398, 0.196349, -0.196349, -1.047197, 0.0, 1.570796, 0.785398],
        'target_poses': [{
            'position': [0.55, -0.32, 0.45],
            'orientation': [0.32, 0.051, 0.317, 0.891],
            'contact': False,
            'distance': 0.06,
            'index': 0
        }],
        'environment': '../../res/scenario_1/obstacles.xml',
        'urdf': '../../res/scenario_1/panda_US.urdf'
    }
    
    config_data = {
        'algorithm': algorithm,
        'parameters': parameters,
        'scenarios': [simple_scenario],
        'evaluation_settings': {
            'num_runs_per_scenario': 3,
            'timeout_seconds': 30,
            'output_trajectories': False
        },
        'resources': {
            'obstacles_file': str(scenario_1_dir / "obstacles.xml"),
            'urdf_file': str(scenario_1_dir / "panda_US.urdf"),
            'poses_file': str(scenario_1_dir / "scan_poses.csv")
        }
    }
    
    temp_file = tempfile.NamedTemporaryFile(mode='w', suffix='.yaml', delete=False)
    with open(temp_file.name, 'w') as f:
        yaml.dump(config_data, f, default_flow_style=False)
    
    return temp_file.name

def run_cpp_evaluator(config_file: str) -> dict:
    """Run the C++ evaluator and capture all outputs."""
    cpp_executable = Path("../../build/apps/ParameterTuning/EnhancedParameterEvaluator")
    
    try:
        result = subprocess.run([
            str(cpp_executable),
            '--config', config_file,
            '--output-format', 'json'
        ], capture_output=True, text=True, timeout=60)
        
        return {
            'returncode': result.returncode,
            'stdout': result.stdout,
            'stderr': result.stderr,
            'config_file': config_file
        }
    
    except Exception as e:
        return {
            'error': str(e),
            'config_file': config_file
        }

def main():
    print("=" * 60)
    print("DEBUG: Raw C++ Evaluator Outputs")
    print("=" * 60)
    
    # Test 1: STOMP with minimal parameters
    print("\n1. Testing STOMP with minimal exploration...")
    stomp_minimal = {
        'exploration_constant': 0.001,
        'num_noisy_trajectories': 5,
        'num_best_samples': 3,
        'max_iterations': 50,
        'learning_rate': 0.1,
        'temperature': 5.0,
        'dt': 0.01,
        'adaptive_sampling': False,
        'early_termination': False
    }
    
    config_file = create_test_config('STOMP', stomp_minimal)
    result = run_cpp_evaluator(config_file)
    
    print(f"Return code: {result['returncode']}")
    print(f"STDERR: {result['stderr']}")
    print(f"STDOUT:\n{result['stdout']}")
    
    if result['returncode'] == 0:
        try:
            metrics = json.loads(result['stdout'])
            print(f"Parsed metrics: {json.dumps(metrics, indent=2)}")
        except:
            print("Failed to parse JSON output")
    
    Path(config_file).unlink()
    
    # Test 2: STOMP with maximal parameters
    print("\n" + "="*60)
    print("2. Testing STOMP with maximal exploration...")
    stomp_maximal = {
        'exploration_constant': 0.5,
        'num_noisy_trajectories': 100,
        'num_best_samples': 20,
        'max_iterations': 1000,
        'learning_rate': 0.7,
        'temperature': 50.0,
        'dt': 0.2,
        'adaptive_sampling': True,
        'early_termination': True
    }
    
    config_file = create_test_config('STOMP', stomp_maximal)
    result = run_cpp_evaluator(config_file)
    
    print(f"Return code: {result['returncode']}")
    print(f"STDERR: {result['stderr']}")
    print(f"STDOUT:\n{result['stdout']}")
    
    if result['returncode'] == 0:
        try:
            metrics = json.loads(result['stdout'])
            print(f"Parsed metrics: {json.dumps(metrics, indent=2)}")
        except:
            print("Failed to parse JSON output")
    
    Path(config_file).unlink()
    
    # Test 3: Hauser with minimal parameters
    print("\n" + "="*60)
    print("3. Testing Hauser with minimal deviation...")
    hauser_minimal = {
        'max_deviation': 0.1,
        'time_step': 0.01,
        'max_iterations': 100,
        'tolerance': 1e-6,
        'acceleration_limit': 0.5,
        'velocity_limit': 0.5,
        'interpolation_dt': 0.01
    }
    
    config_file = create_test_config('Hauser', hauser_minimal)
    result = run_cpp_evaluator(config_file)
    
    print(f"Return code: {result['returncode']}")
    print(f"STDERR: {result['stderr']}")
    print(f"STDOUT:\n{result['stdout']}")
    
    if result['returncode'] == 0:
        try:
            metrics = json.loads(result['stdout'])
            print(f"Parsed metrics: {json.dumps(metrics, indent=2)}")
        except:
            print("Failed to parse JSON output")
    
    Path(config_file).unlink()
    
    # Test 4: Hauser with maximal parameters
    print("\n" + "="*60)
    print("4. Testing Hauser with maximal deviation...")
    hauser_maximal = {
        'max_deviation': 2.0,
        'time_step': 0.5,
        'max_iterations': 2000,
        'tolerance': 1e-3,
        'acceleration_limit': 5.0,
        'velocity_limit': 3.0,
        'interpolation_dt': 0.1
    }
    
    config_file = create_test_config('Hauser', hauser_maximal)
    result = run_cpp_evaluator(config_file)
    
    print(f"Return code: {result['returncode']}")
    print(f"STDERR: {result['stderr']}")
    print(f"STDOUT:\n{result['stdout']}")
    
    if result['returncode'] == 0:
        try:
            metrics = json.loads(result['stdout'])
            print(f"Parsed metrics: {json.dumps(metrics, indent=2)}")
        except:
            print("Failed to parse JSON output")
    
    Path(config_file).unlink()
    
    print("\n" + "="*60)
    print("DEBUG COMPLETE")
    print("="*60)
    print("Analysis:")
    print("1. Compare the raw metrics between minimal/maximal parameter sets")
    print("2. Check if success_rate, planning_time_ms, path_length actually vary")
    print("3. Look for patterns that explain the lack of parameter sensitivity")
    print("4. Examine stderr for any algorithm-specific messages")

if __name__ == "__main__":
    main()