#!/usr/bin/env python3
"""
Test script to verify YAML configuration generation for C++ evaluator
"""

import yaml
import tempfile
from pathlib import Path

def create_test_config():
    """Create a simple test configuration to verify YAML format."""
    
    # Simple test scenario
    test_scenario = {
        'name': 'test_scenario',
        'description': 'Simple test scenario',
        'difficulty': 1,
        'start_config': [-0.785398, 0.196349, -0.196349, -1.047197, 0.0, 1.570796, 0.785398],
        'environment': '../../res/scenario_1/obstacles.xml',
        'urdf': '../../res/scenario_1/panda_US.urdf',
        'target_poses': [
            {
                'position': [0.55, -0.32, 0.45],
                'orientation': [0.32, 0.051, 0.317, 0.891],  # w,x,y,z format
                'contact': False,
                'distance': 0.06,
                'index': 0
            },
            {
                'position': [0.54, -0.32, 0.45],
                'orientation': [0.072, 0.223, -0.363, -0.902],  # w,x,y,z format
                'contact': False,
                'distance': 0.06,
                'index': 1
            }
        ]
    }
    
    config_data = {
        'algorithm': 'STOMP',
        'parameters': {
            'exploration_constant': 0.1,
            'num_noisy_trajectories': 50,
            'num_best_samples': 10,
            'max_iterations': 100,
            'learning_rate': 0.3,
            'temperature': 15.0,
            'dt': 0.1,
            'adaptive_sampling': True,
            'early_termination': True
        },
        'scenarios': [test_scenario],
        'evaluation_settings': {
            'num_runs_per_scenario': 3,
            'timeout_seconds': 30,
            'output_trajectories': False
        },
        'resources': {
            'obstacles_file': '../../res/scenario_1/obstacles.xml',
            'urdf_file': '../../res/scenario_1/panda_US.urdf',
            'poses_file': '../../res/scenario_1/scan_poses.csv'
        }
    }
    
    return config_data

def test_yaml_generation():
    """Test YAML generation and parsing."""
    print("Testing YAML configuration generation...")
    
    # Create test config
    config_data = create_test_config()
    
    # Generate YAML
    temp_file = tempfile.NamedTemporaryFile(mode='w', suffix='.yaml', delete=False)
    try:
        with open(temp_file.name, 'w') as f:
            yaml.dump(config_data, f, default_flow_style=False)
        
        print(f"YAML file generated: {temp_file.name}")
        
        # Read back and verify
        with open(temp_file.name, 'r') as f:
            loaded_config = yaml.safe_load(f)
        
        print("YAML loaded successfully!")
        
        # Print the YAML content
        print("\nGenerated YAML content:")
        print("=" * 50)
        with open(temp_file.name, 'r') as f:
            print(f.read())
        print("=" * 50)
        
        # Verify key elements
        assert loaded_config['algorithm'] == 'STOMP'
        assert len(loaded_config['scenarios']) == 1
        assert len(loaded_config['scenarios'][0]['target_poses']) == 2
        assert len(loaded_config['scenarios'][0]['target_poses'][0]['orientation']) == 4
        
        print("All validation checks passed!")
        
        return temp_file.name
        
    except Exception as e:
        print(f"Error: {e}")
        return None

def test_cpp_evaluator(yaml_file):
    """Test the C++ evaluator with the generated YAML."""
    import subprocess
    
    # Find the C++ evaluator
    project_root = Path(__file__).parent.parent.parent
    cpp_executable = project_root / "build" / "apps" / "ParameterTuning" / "EnhancedParameterEvaluator"
    
    if not cpp_executable.exists():
        print(f"C++ evaluator not found: {cpp_executable}")
        return False
    
    print(f"\nTesting C++ evaluator with: {yaml_file}")
    
    try:
        result = subprocess.run([
            str(cpp_executable),
            '--config', yaml_file,
            '--output-format', 'json'
        ], capture_output=True, text=True, timeout=60)
        
        print(f"Return code: {result.returncode}")
        print(f"STDOUT:\n{result.stdout}")
        if result.stderr:
            print(f"STDERR:\n{result.stderr}")
        
        return result.returncode == 0
        
    except subprocess.TimeoutExpired:
        print("C++ evaluator timed out")
        return False
    except Exception as e:
        print(f"Error running C++ evaluator: {e}")
        return False

def main():
    """Main test function."""
    print("YAML Configuration Test")
    print("=" * 30)
    
    # Test YAML generation
    yaml_file = test_yaml_generation()
    
    if yaml_file:
        # Test C++ evaluator
        success = test_cpp_evaluator(yaml_file)
        
        if success:
            print("\n✅ All tests passed!")
        else:
            print("\n❌ C++ evaluator test failed")
            
        # Cleanup
        Path(yaml_file).unlink()
    else:
        print("\n❌ YAML generation test failed")

if __name__ == "__main__":
    main()