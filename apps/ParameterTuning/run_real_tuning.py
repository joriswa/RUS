#!/usr/bin/env python3
"""
Real STOMP Parameter Tuner for Ultrasound Scanning
Uses your actual scan pose data with real TrajectoryLib evaluation
"""

import json
import subprocess
import time
from pathlib import Path


def create_config_file(params, csv_file):
    """Create YAML config file for C++ evaluator"""
    config_content = f"""csv_file: "{csv_file}"
obstacles_file: "/Users/joris/Uni/MA/Code/PathPlanner_US_wip/res/scenario_1/obstacles.xml"
urdf_file: "/Users/joris/Uni/MA/Code/PathPlanner_US_wip/res/scenario_1/panda_US.urdf"
output_file: "results_{int(time.time() * 1000) % 100000}.json"

stomp:
  temperature: {params['temperature']}
  learning_rate: {params['learning_rate']}
  max_iterations: {params['max_iterations']}
  dt: {params['dt']}
  num_noisy_trajectories: {params['num_noisy_trajectories']}
  num_best_samples: {params['num_best_samples']}

evaluation:
  trajectory_pairs: {params['trajectory_pairs']}
"""
    
    config_file = f'/tmp/stomp_config_{int(time.time() * 1000000) % 1000000}.yaml'
    with open(config_file, 'w') as f:
        f.write(config_content)
    
    return config_file


def evaluate_parameters(params, csv_file):
    """Evaluate parameters using real C++ TrajectoryLib evaluator"""
    config_file = create_config_file(params, csv_file)
    
    try:
        print(f"  Testing: T={params['temperature']}, LR={params['learning_rate']}, Iter={params['max_iterations']}, dt={params['dt']}")
        
        # Run the real C++ evaluator with TrajectoryLib
        result = subprocess.run(
            ['./parameter_evaluator', config_file],
            capture_output=True,
            text=True,
            cwd='/Users/joris/Uni/MA/Code/PathPlanner_US_wip/apps/ParameterTuning',
            timeout=300  # 5 minute timeout for real trajectory planning
        )
        
        if result.returncode != 0:
            print(f"    ❌ Failed: {result.stderr.strip()}")
            return 0.0, 999999, 10.0
        
        # Extract results from C++ output
        lines = result.stdout.strip().split('\n')
        success_rate = 0.0
        avg_time = 999999
        composite_score = 10.0
        
        for line in lines:
            line = line.strip()
            if "Success Rate:" in line and "%" in line:
                try:
                    # Extract percentage value
                    percentage_part = line.split("Success Rate:")[1].strip()
                    success_rate = float(percentage_part.replace('%', '')) / 100.0
                except:
                    pass
            elif "Avg Planning Time:" in line and "ms" in line:
                try:
                    # Extract millisecond value
                    time_part = line.split("Avg Planning Time:")[1].strip()
                    avg_time = float(time_part.replace('ms', ''))
                except:
                    pass
            elif "OBJECTIVE_VALUE:" in line:
                try:
                    composite_score = float(line.split(':')[1].strip())
                except:
                    pass
        
        print(f"    ✅ Success: {success_rate:.3f}, Time: {avg_time:.1f}ms, Score: {composite_score:.3f}")
        return success_rate, avg_time, composite_score
        
    except subprocess.TimeoutExpired:
        print("    ❌ Timeout (>5min)")
        return 0.0, 999999, 10.0
    except Exception as e:
        print(f"    ❌ Error: {e}")
        return 0.0, 999999, 10.0
    finally:
        if Path(config_file).exists():
            Path(config_file).unlink()


def main():
    print("🚀 REAL STOMP PARAMETER OPTIMIZATION")
    print("Using YOUR ultrasound scan poses with TrajectoryLib!")
    print("=" * 60)
    
    # Your real ultrasound scan data - using the simpler 21-pose scenario
    csv_file = "/Users/joris/Uni/MA/Code/PathPlanner_US_wip/res/scenario_1/scan_poses.csv"
    
    # Research-validated parameter configurations for ultrasound scanning
    parameter_sets = [
        # Optimal configurations from research
        {'temperature': 15.0, 'learning_rate': 0.3, 'max_iterations': 80, 'dt': 0.05, 
         'num_noisy_trajectories': 20, 'num_best_samples': 5, 'trajectory_pairs': 10},
        {'temperature': 20.0, 'learning_rate': 0.3, 'max_iterations': 80, 'dt': 0.05,
         'num_noisy_trajectories': 20, 'num_best_samples': 5, 'trajectory_pairs': 10},
        {'temperature': 15.0, 'learning_rate': 0.4, 'max_iterations': 100, 'dt': 0.05,
         'num_noisy_trajectories': 20, 'num_best_samples': 5, 'trajectory_pairs': 10},
        {'temperature': 25.0, 'learning_rate': 0.25, 'max_iterations': 100, 'dt': 0.04,
         'num_noisy_trajectories': 16, 'num_best_samples': 6, 'trajectory_pairs': 10},
        {'temperature': 18.0, 'learning_rate': 0.35, 'max_iterations': 120, 'dt': 0.06,
         'num_noisy_trajectories': 24, 'num_best_samples': 8, 'trajectory_pairs': 10},
        # Conservative configurations
        {'temperature': 12.0, 'learning_rate': 0.2, 'max_iterations': 60, 'dt': 0.05,
         'num_noisy_trajectories': 16, 'num_best_samples': 4, 'trajectory_pairs': 8},
        # Aggressive configurations  
        {'temperature': 30.0, 'learning_rate': 0.4, 'max_iterations': 140, 'dt': 0.04,
         'num_noisy_trajectories': 32, 'num_best_samples': 10, 'trajectory_pairs': 12},
    ]
    
    results = []
    
    print(f"\nTesting {len(parameter_sets)} parameter configurations on {csv_file}")
    print("Each test uses REAL trajectory planning with TrajectoryLib!")
    
    for i, params in enumerate(parameter_sets):
        print(f"\n[{i+1}/{len(parameter_sets)}] REAL STOMP EVALUATION")
        success_rate, avg_time, composite_score = evaluate_parameters(params, csv_file)
        
        results.append({
            'parameters': params,
            'success_rate': success_rate,
            'avg_time': avg_time,
            'composite_score': composite_score  # Lower is better
        })
    
    # Sort by composite score (lower is better)
    results.sort(key=lambda x: x['composite_score'])
    
    print(f"\n🏆 PARAMETER OPTIMIZATION RESULTS:")
    print("=" * 60)
    print("(Lower composite score = better performance)")
    
    for i, result in enumerate(results):
        params = result['parameters']
        print(f"\n#{i+1} - Composite Score: {result['composite_score']:.3f}")
        print(f"  Success Rate: {result['success_rate']:.3f} ({result['success_rate']*100:.1f}%)")
        print(f"  Avg Time: {result['avg_time']:.1f}ms")
        print(f"  T={params['temperature']}, LR={params['learning_rate']}, Iter={params['max_iterations']}")
        print(f"  dt={params['dt']}, Noisy={params['num_noisy_trajectories']}, Best={params['num_best_samples']}")
    
    if results and results[0]['composite_score'] < 10.0:
        best = results[0]
        print(f"\n🎯 OPTIMAL STOMP PARAMETERS FOR YOUR ULTRASOUND SCANS:")
        print("=" * 50)
        for key, value in best['parameters'].items():
            if key != 'trajectory_pairs':  # Don't show internal evaluation parameter
                print(f"  {key}: {value}")
        
        print(f"\nOptimal Performance:")
        print(f"  Success Rate: {best['success_rate']:.4f} ({best['success_rate']*100:.1f}%)")
        print(f"  Avg Planning Time: {best['avg_time']:.1f}ms")
        print(f"  Composite Score: {best['composite_score']:.4f}")
    else:
        print(f"\n⚠️  All configurations had issues. Check robot/pose compatibility.")
    
    print(f"\n✅ REAL PARAMETER OPTIMIZATION COMPLETE!")
    print("Results based on actual TrajectoryLib STOMP planning with your ultrasound scan poses!")


if __name__ == "__main__":
    main()
