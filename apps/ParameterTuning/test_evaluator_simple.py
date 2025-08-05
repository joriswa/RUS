#!/usr/bin/env python3
"""
Clean evaluation module for STOMP parameter testing.
Simplified and methodologically sound approach.
"""

from pathlib import Path
import subprocess
import json
import os
import time
import re
import tempfile
from typing import Dict, Any, Optional, Tuple


def create_config_file(stomp_params: Dict[str, Any], output_filename: str, trajectory_pairs: int = 3) -> Tuple[str, str]:
    """Create a YAML config file with STOMP parameters."""
    base_path = Path.cwd()
    
    # Ensure output directories exist
    (base_path / "output" / "configs").mkdir(parents=True, exist_ok=True)
    (base_path / "output" / "results").mkdir(parents=True, exist_ok=True)
    
    # Add joint_std_devs if not provided
    joint_std_devs = stomp_params.get('joint_std_devs', [0.1] * 7)
    
    # Use the correct URDF path that working configs use
    urdf_path = "/Users/joris/Uni/MA/robot_definition/panda_US.urdf"
    
    # Place output file in results directory
    results_output_file = f"output/results/{output_filename}"
    
    yaml_content = f"""csv_file: "{base_path}/Data/Ext_Ant_Scan/scan_pose_list_ee.csv"
obstacles_file: "{base_path}/Data/Ext_Ant_Scan/obstacles.xml"
urdf_file: "{urdf_path}"
output_file: "{results_output_file}"

stomp:
  temperature: {stomp_params['temperature']}
  learning_rate: {stomp_params['learning_rate']}
  max_iterations: {stomp_params['max_iterations']}
  dt: {stomp_params['dt']}
  num_noisy_trajectories: {stomp_params['num_noisy_trajectories']}
  num_best_samples: {stomp_params['num_best_samples']}
  obstacle_cost_weight: {stomp_params['obstacle_cost_weight']}
  constraint_cost_weight: {stomp_params['constraint_cost_weight']}
  control_cost_weight: {stomp_params['control_cost_weight']}
  joint_std_devs: {joint_std_devs}

evaluation:
  trajectory_pairs: {trajectory_pairs}
"""
    
    # Place config file in configs directory
    config_file = f"output/configs/temp_config_{int(time.time())}.yaml"
    with open(config_file, 'w') as f:
        f.write(yaml_content)
    
    return config_file, results_output_file


def run_evaluator(config_file: str, timeout_seconds: int = 180) -> Tuple[bool, str, str, int]:
    """Run the parameter evaluator with the given config file."""
    try:
        # Try primary build path first
        evaluator_path = "../../build/apps/ParameterTuning/parameter_evaluator"
        if not os.path.exists(evaluator_path):
            # Try alternative Qt build path
            evaluator_path = "../../build/Qt_6_6_0_for_macOS-Release/apps/ParameterTuning/parameter_evaluator"
        
        result = subprocess.run(
            [evaluator_path, config_file],
            capture_output=True,
            text=True,
            timeout=timeout_seconds
        )
        
        return True, result.stdout, result.stderr, result.returncode
            
    except subprocess.TimeoutExpired:
        return False, "", f"Evaluator timed out after {timeout_seconds} seconds", -1
    except Exception as e:
        return False, "", f"Error running evaluator: {e}", -1


def parse_results(stdout: str, json_file: str) -> Optional[Dict[str, float]]:
    """Parse stdout and JSON output."""
    results = {
        'success_rate': 0.0,
        'avg_planning_time_ms': 30000.0,  # Default to timeout value
        'avg_path_length': 100.0,         # Default to large penalty value
        'avg_clearance': 0.001,           # Default to minimal clearance
        'objective_value': 100.0          # Default to large penalty value
    }
    
    # Parse JSON results if file exists
    if os.path.exists(json_file):
        try:
            with open(json_file, 'r') as f:
                json_data = json.load(f)
                results['success_rate'] = json_data.get('success_rate', 0.0)
                results['avg_planning_time_ms'] = json_data.get('avg_planning_time_ms', 30000.0)
                results['avg_path_length'] = json_data.get('avg_path_length', 100.0)
                results['avg_clearance'] = json_data.get('avg_clearance', 0.001)
                # Use composite_score as objective_value if available
                results['objective_value'] = json_data.get('composite_score', json_data.get('objective_value', 100.0))
                return results
        except Exception as e:
            print(f"Warning: Could not parse JSON file: {e}")
    
    # Try to parse from stdout as fallback
    try:
        # Look for key metrics in stdout
        success_match = re.search(r'Success Rate:\s*([\d.]+)', stdout, re.IGNORECASE)
        if success_match:
            results['success_rate'] = float(success_match.group(1)) / 100.0  # Convert percentage to fraction
        
        time_match = re.search(r'Avg Planning Time:\s*([\d.]+)', stdout, re.IGNORECASE)
        if time_match:
            results['avg_planning_time_ms'] = float(time_match.group(1))
        
        length_match = re.search(r'Avg Path Length:\s*([\d.]+)', stdout, re.IGNORECASE)
        if length_match:
            results['avg_path_length'] = float(length_match.group(1))
        
        clearance_match = re.search(r'Avg Clearance:\s*([\d.]+)', stdout, re.IGNORECASE)
        if clearance_match:
            results['avg_clearance'] = float(clearance_match.group(1))
        
        # Look for OBJECTIVE_VALUE in stdout
        objective_match = re.search(r'OBJECTIVE_VALUE:\s*([\d.]+)', stdout, re.IGNORECASE)
        if objective_match:
            results['objective_value'] = float(objective_match.group(1))
            
    except Exception as e:
        print(f"Error parsing stdout: {e}")
    
    return results

def evaluate_parameters(stomp_params):
    """Evaluate STOMP parameters and return success rate."""
    # Create config file with organized output
    config_file, output_file = create_config_file(stomp_params)
    
    print(f"Created config: {config_file}")
    print(f"Results will be in: {output_file}")
    
    try:
        # Run evaluator
        success, stdout, stderr, returncode = run_evaluator(config_file)
        
        if not success or returncode != 0:
            print(f"Evaluator failed: {stderr}")
            return 0.0
        
        # Parse results
        results = parse_results(stdout, output_file)
        if results is None:
            return 0.0
        
        # Extract success rate
        success_rate = results.get('success_rate', 0.0)
        print(f"Success rate: {success_rate:.2%}")
        
        return success_rate
        
    finally:
        # Clean up config file after use
        if os.path.exists(config_file):
            os.remove(config_file)
            print(f"Cleaned up config file: {config_file}")
        
        # Clean up results file to avoid clutter
        if os.path.exists(output_file):
            os.remove(output_file)
            print(f"Cleaned up results file: {output_file}")

def test_evaluator():
    """Test the evaluator with sample parameters."""
    print("=== PARAMETER EVALUATOR TEST ===")
    
    # Change to ParameterTuning directory where parameter_evaluator executable is located
    script_dir = Path(__file__).parent
    os.chdir(script_dir)
    print(f"Working directory: {Path.cwd()}")
    
    # Parameters matching the working test_ext_ant_config.yaml
    test_params = {
        'temperature': 0.5,
        'learning_rate': 1.0,
        'max_iterations': 300,
        'N': 60,
        'num_noisy_trajectories': 6,
        'num_best_samples': 6,
        'obstacle_cost_weight': 20.0,
        'constraint_cost_weight': 1.0,
        'control_cost_weight': 0.00000001  # Very small value like working config
    }
    
    print("Test parameters:")
    for key, value in test_params.items():
        print(f"  {key}: {value}")
    print()
    
    config_file = None
    output_file = f"temp_results_{int(time.time())}.json"
    
    try:
        # Create config
        config_file, results_file = create_config_file(test_params, output_file, trajectory_pairs=5)
        print(f"✅ Created config: {config_file}")
        print(f"✅ Results will be in: {results_file}")
        
        # Run evaluator
        print("🚀 Running evaluator...")
        success, stdout, stderr, returncode = run_evaluator(config_file, timeout_seconds=120)
        
        if not success:
            print(f"❌ Failed: {stderr}")
            return
        
        if returncode != 0:
            print(f"❌ Error (code {returncode}): {stderr}")
            return
        
        # Parse results
        results = parse_results(stdout, results_file)
        
        print("✅ Results:")
        print(f"  Objective Value: {results['objective_value']}")
        print(f"  Success Rate: {results['success_rate']:.3f}")
        print(f"  Avg Planning Time: {results['avg_planning_time_ms']:.1f}ms")
        print(f"  Avg Path Length: {results['avg_path_length']:.3f}")
        print(f"  Avg Clearance: {results['avg_clearance']:.3f}m")
        
    finally:
        # Clean up
        if config_file and os.path.exists(config_file):
            os.remove(config_file)
            print(f"Cleaned up config file: {config_file}")
        if results_file and os.path.exists(results_file):
            os.remove(results_file)
            print(f"Cleaned up results file: {results_file}")

if __name__ == "__main__":
    test_evaluator()
