#!/usr/bin/env python3
"""
Simple STOMP Parameter Tuner - Real C++ Evaluator Integration
"""

import json
import subprocess
import time
from pathlib import Path


def create_config_file(params, csv_file):
    """Create YAML config file for C++ evaluator"""
    config_content = f"""csv_file: {csv_file}
temperature: {params['temperature']}
learningRate: {params['learning_rate']}
maxIterations: {params['max_iterations']}
dt: {params['dt']}
numNoisyTrajectories: 20
numBestSamples: 5
numTrajectoryPairs: 5
"""
    
    config_file = f'/tmp/stomp_config_{int(time.time() * 1000000) % 1000000}.yaml'
    with open(config_file, 'w') as f:
        f.write(config_content)
    
    return config_file


def evaluate_parameters(params, csv_file):
    """Evaluate parameters using real C++ evaluator"""
    config_file = create_config_file(params, csv_file)
    
    try:
        print(f"  Testing: T={params['temperature']}, LR={params['learning_rate']}, Iter={params['max_iterations']}, dt={params['dt']}")
        
        # Run the real C++ evaluator
        result = subprocess.run(
            ['./parameter_evaluator', config_file],
            capture_output=True,
            text=True,
            cwd='/Users/joris/Uni/MA/Code/PathPlanner_US_wip/apps/ParameterTuning',
            timeout=120
        )
        
        if result.returncode != 0:
            print(f"    ❌ Failed: {result.stderr.strip()}")
            return 0.0, 999999
        
        # Parse JSON output
        try:
            eval_result = json.loads(result.stdout)
            success_rate = eval_result.get('successRate', 0.0)
            avg_time = eval_result.get('avgPlanningTime', 999999)
            
            print(f"    ✅ Success: {success_rate:.3f}, Time: {avg_time:.1f}ms")
            return success_rate, avg_time
            
        except json.JSONDecodeError:
            print(f"    ❌ Failed to parse output: {result.stdout[:100]}")
            return 0.0, 999999
            
    except subprocess.TimeoutExpired:
        print("    ❌ Timeout")
        return 0.0, 999999
    finally:
        if Path(config_file).exists():
            Path(config_file).unlink()


def main():
    print("🚀 REAL STOMP PARAMETER OPTIMIZATION")
    print("=" * 50)
    
    csv_file = "demo_poses.csv"
    
    # Test parameter configurations
    parameter_sets = [
        {'temperature': 15.0, 'learning_rate': 0.3, 'max_iterations': 80, 'dt': 0.05},
        {'temperature': 20.0, 'learning_rate': 0.3, 'max_iterations': 80, 'dt': 0.05},
        {'temperature': 15.0, 'learning_rate': 0.4, 'max_iterations': 80, 'dt': 0.05},
        {'temperature': 15.0, 'learning_rate': 0.3, 'max_iterations': 100, 'dt': 0.05},
        {'temperature': 15.0, 'learning_rate': 0.3, 'max_iterations': 80, 'dt': 0.06},
    ]
    
    results = []
    
    print(f"\nTesting {len(parameter_sets)} parameter configurations...")
    
    for i, params in enumerate(parameter_sets):
        print(f"\n[{i+1}/{len(parameter_sets)}]")
        success_rate, avg_time = evaluate_parameters(params, csv_file)
        
        # Compute composite score
        if success_rate > 0 and avg_time < 999999:
            score = success_rate * (1000 / max(40, avg_time))
        else:
            score = 0
            
        results.append({
            'parameters': params,
            'success_rate': success_rate,
            'avg_time': avg_time,
            'score': score
        })
    
    # Sort by score
    results.sort(key=lambda x: x['score'], reverse=True)
    
    print(f"\n🏆 RESULTS:")
    print("=" * 50)
    
    for i, result in enumerate(results):
        params = result['parameters']
        print(f"\n#{i+1} - Score: {result['score']:.3f}")
        print(f"  Success Rate: {result['success_rate']:.3f}")
        print(f"  Avg Time: {result['avg_time']:.1f}ms")
        print(f"  T={params['temperature']}, LR={params['learning_rate']}, Iter={params['max_iterations']}, dt={params['dt']}")
    
    if results and results[0]['score'] > 0:
        best = results[0]
        print(f"\n🎯 BEST PARAMETERS:")
        for key, value in best['parameters'].items():
            print(f"  {key}: {value}")
    
    print(f"\n✅ Parameter optimization complete!")


if __name__ == "__main__":
    main()
