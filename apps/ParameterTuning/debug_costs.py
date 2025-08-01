#!/usr/bin/env python3
"""
Quick Cost Debug Script

Test current STOMP implementation to identify if cost explosion is still happening
"""

import subprocess
import yaml
import tempfile
import os
import json

def create_debug_config():
    """Create a config designed to trigger cost issues if they exist"""
    config = {
        "temperature": 10.0,
        "learning_rate": 0.5,
        "max_iterations": 10,  # Just a few iterations to see costs
        "N": 30,  # Smaller for faster testing
        "obstacle_cost_weight": 5.0,  # Higher weights to trigger issues
        "constraint_cost_weight": 10.0,  # Very high to see if costs explode
        "joint_std_devs": [0.2, 0.2, 0.2, 0.2, 0.2, 0.2, 0.2],  # Large noise
        "num_noisy_trajectories": 4,
        "num_best_samples": 2,
        "enable_early_stopping": False,  # Don't stop early, see all costs
        "early_stopping_patience": 100,
        "max_compute_time_ms": 10000.0,  # 10 second timeout
        "disable_internal_parallelization": False
    }
    return config

def run_cost_debug():
    config = create_debug_config()
    
    with tempfile.NamedTemporaryFile(mode='w', suffix='.yaml', delete=False) as f:
        yaml.dump(config, f)
        config_file = f.name
    
    try:
        print("Running cost debug test...")
        print("Config:", config)
        print("\n=== COST DEBUG OUTPUT ===")
        
        result = subprocess.run(
            ['./parameter_evaluator', config_file],
            capture_output=True,
            text=True,
            timeout=60  # 1 minute timeout
        )
        
        print("STDOUT:")
        print(result.stdout)
        print("\nSTDERR:")
        print(result.stderr)
        print(f"\nReturn code: {result.returncode}")
        
        # Look for cost values in output
        lines = result.stdout.split('\n')
        for line in lines:
            if 'cost' in line.lower() or 'Cost' in line:
                print(f"COST LINE: {line}")
        
        # Try to parse final results
        try:
            final_result = lines[-2] if lines[-1] == '' else lines[-1]
            results = json.loads(final_result)
            print(f"\n=== FINAL RESULTS ===")
            print(f"Success rate: {results.get('success_rate', 'N/A')}")
            print(f"Avg planning time: {results.get('avg_planning_time_ms', 'N/A')}ms")
            print(f"Composite score: {results.get('composite_score', 'N/A')}")
        except:
            print("Could not parse final JSON results")
            
    except subprocess.TimeoutExpired:
        print("DEBUG TEST TIMED OUT - possible cost explosion or infinite loop!")
    except Exception as e:
        print(f"Debug test error: {e}")
    finally:
        os.unlink(config_file)

if __name__ == "__main__":
    run_cost_debug()
