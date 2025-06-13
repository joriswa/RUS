#!/usr/bin/env python3
"""
SinglePose Evaluator Execution Time Analysis

This script creates execution time and computation time boxplots for SinglePose evaluation data.
"""





























































































































































































































































































    main()if __name__ == "__main__":    print(f"\n✨ Analysis complete! Results saved to: {output_dir}")        print(f"🔧 Most Efficient: {best_efficiency}")    print(f"⚡ Fastest Planning: {best_speed}")    print(f"🎯 Best Success Rate: {best_success}")        best_efficiency = summary_table.loc[summary_table['Time per Iteration (ms)'].astype(float).idxmin(), 'Algorithm']    best_speed = summary_table.loc[summary_table['Avg Planning Time (ms)'].astype(float).idxmin(), 'Algorithm']    best_success = summary_table.loc[summary_table['Success Rate (%)'].astype(float).idxmax(), 'Algorithm']        print("=" * 60)    print("\n🏆 ALGORITHM RECOMMENDATIONS")    # Algorithm recommendations        print(summary_table.to_string(index=False))    print("=" * 60)    print("\n🎯 PERFORMANCE SUMMARY")    # Display summary        print(f"💾 Saved summary table to: {summary_file}")    summary_table.to_csv(summary_file, index=False)    summary_file = output_dir / "single_pose_performance_summary.csv"    summary_table = create_performance_summary_table(data)    print("📋 Generating performance summary table...")    # Create performance summary        print(f"✅ Saved boxplots to: {boxplot_file}")    create_execution_time_boxplots(data, boxplot_file)    boxplot_file = output_dir / "single_pose_execution_time_analysis.png"    print("📈 Creating execution time and computation boxplots...")    # Create execution time boxplots        print(f"💾 Saved evaluation data to: {data_file}")    data.to_csv(data_file, index=False)    data_file = output_dir / "single_pose_evaluation_data.csv"    # Save the generated data        output_dir.mkdir(exist_ok=True)    output_dir = Path("./single_pose_analysis_results")    # Create output directory        print(f"✅ Generated {len(data)} evaluation records")    data = generate_synthetic_single_pose_data()    print("📊 Generating synthetic SinglePose evaluation data...")    # Generate synthetic data for demonstration        print("=" * 50)    print("🚀 SinglePose Evaluator Execution Time Analysis")        """Main function to generate SinglePose execution time analysis."""def main():    return pd.DataFrame(summary_stats)            summary_stats.append(stats)                }            'Successful Trials': f"{len(successful_data)}/{len(alg_data)}"            'Time per Iteration (ms)': f"{alg_data['time_per_iteration'].mean():.3f}",            'Avg Iterations': f"{alg_data['iterations_used'].mean():.0f}",            'Max Planning Time (ms)': f"{alg_data['planning_time_ms'].max():.1f}",            'Min Planning Time (ms)': f"{alg_data['planning_time_ms'].min():.1f}",            'Planning Time Std (ms)': f"{alg_data['planning_time_ms'].std():.1f}",            'Avg Planning Time (ms)': f"{alg_data['planning_time_ms'].mean():.1f}",            'Success Rate (%)': f"{(alg_data['success'].mean() * 100):.1f}",            'Algorithm': algorithm,        stats = {                successful_data = alg_data[alg_data['success'] == 1]        alg_data = data[data['algorithm'] == algorithm]    for algorithm in data['algorithm'].unique():        summary_stats = []        """Create a performance summary table for each algorithm."""def create_performance_summary_table(data):    return output_path        plt.close()    plt.savefig(output_path, dpi=300, bbox_inches='tight')    plt.tight_layout()        ax6.grid(True, alpha=0.3)    ax6.legend(fontsize=8)    ax6.set_ylabel('Planning Time (ms)')    ax6.set_xlabel('Pose Complexity')    ax6.set_title('Planning Time vs Pose Complexity', fontweight='bold')                       c=scatter_colors.get(alg, '#333333'), label=alg, alpha=0.6, s=50)        ax6.scatter(alg_data['pose_complexity'], alg_data['planning_time_ms'],         alg_data = data[data['algorithm'] == alg]    for alg in algorithms:                         'HAUSER_RRT_STAR': '#2ecc71', 'HAUSER_INFORMED_RRT': '#f39c12'}    scatter_colors = {'STOMP': '#3498db', 'HAUSER_RRT': '#e74c3c',     ax6 = axes[1, 2]    # 6. Planning Time vs Pose Complexity        ax5.grid(True, alpha=0.3)    ax5.tick_params(axis='x', rotation=45)    ax5.set_ylabel('Execution Time (ms)')    ax5.set_title('Trajectory Execution Time (Successful)', fontweight='bold')            patch.set_alpha(0.7)        patch.set_facecolor(color)    for patch, color in zip(bp5['boxes'], colors):    bp5 = ax5.boxplot(exec_times, labels=algorithms, patch_artist=True)                     for alg in algorithms]    exec_times = [successful_exec_data[successful_exec_data['algorithm'] == alg]['execution_time_estimate'].values    successful_exec_data = data[(data['success'] == 1) & (data['execution_time_estimate'] > 0)]    ax5 = axes[1, 1]    # 5. Execution Time Estimates (Successful only)        ax4.grid(True, alpha=0.3)    ax4.tick_params(axis='x', rotation=45)    ax4.set_ylabel('Number of Iterations')    ax4.set_title('Iterations Used by Algorithm', fontweight='bold')            patch.set_alpha(0.7)        patch.set_facecolor(color)    for patch, color in zip(bp4['boxes'], colors):    bp4 = ax4.boxplot(iterations_data, labels=algorithms, patch_artist=True)                          for alg in algorithms]    iterations_data = [data[data['algorithm'] == alg]['iterations_used'].values    ax4 = axes[1, 0]    # 4. Iterations Used        ax3.grid(True, alpha=0.3)    ax3.tick_params(axis='x', rotation=45)    ax3.set_ylabel('Time per Iteration (ms/iter)')    ax3.set_title('Computation Efficiency', fontweight='bold')            patch.set_alpha(0.7)        patch.set_facecolor(color)    for patch, color in zip(bp3['boxes'], colors):    bp3 = ax3.boxplot(efficiency_data, labels=algorithms, patch_artist=True)                          for alg in algorithms]    efficiency_data = [data[data['algorithm'] == alg]['time_per_iteration'].values    ax3 = axes[0, 2]    # 3. Computation Efficiency (Time per Iteration)        ax2.grid(True, alpha=0.3)              ['Successful', 'Failed'], loc='upper right')    ax2.legend([bp2_success['boxes'][0], bp2_failed['boxes'][0]],     ax2.set_xticklabels(algorithms, rotation=45)    ax2.set_xticks(x_pos)    ax2.set_ylabel('Planning Time (ms)')    ax2.set_title('Success vs Failed Attempts', fontweight='bold')            patch.set_alpha(0.7)        patch.set_facecolor('#e74c3c')    for patch in bp2_failed['boxes']:        patch.set_alpha(0.7)        patch.set_facecolor('#2ecc71')    for patch in bp2_success['boxes']:                                patch_artist=True)    bp2_failed = ax2.boxplot(failed_times, positions=x_pos + 0.2, widths=0.3,                             patch_artist=True)    bp2_success = ax2.boxplot(success_times, positions=x_pos - 0.2, widths=0.3,     x_pos = np.arange(len(algorithms))                       for alg in algorithms]    failed_times = [failed_data[failed_data['algorithm'] == alg]['planning_time_ms'].values                    for alg in algorithms]    success_times = [successful_data[successful_data['algorithm'] == alg]['planning_time_ms'].values        failed_data = data[data['success'] == 0]    successful_data = data[data['success'] == 1]    ax2 = axes[0, 1]    # 2. Successful vs Failed Attempts        ax1.grid(True, alpha=0.3)    ax1.tick_params(axis='x', rotation=45)    ax1.set_ylabel('Planning Time (ms)')    ax1.set_title('Total Planning Time by Algorithm', fontweight='bold')            patch.set_alpha(0.7)        patch.set_facecolor(color)    for patch, color in zip(bp1['boxes'], colors):    bp1 = ax1.boxplot(planning_data, labels=algorithms, patch_artist=True)        planning_data = [data[data['algorithm'] == alg]['planning_time_ms'].values for alg in algorithms]    algorithms = list(data['algorithm'].unique())    ax1 = axes[0, 0]    # 1. Overall Planning Time Comparison        fig.suptitle('SinglePose Evaluator: Execution Time and Computation Analysis', fontsize=16, fontweight='bold')    fig, axes = plt.subplots(2, 3, figsize=(18, 12))    # Create figure with subplots        colors = ['#3498db', '#e74c3c', '#2ecc71', '#f39c12']    # Set up colors        """Create comprehensive execution time boxplots for SinglePose evaluator."""def create_execution_time_boxplots(data, output_path):    return pd.DataFrame(data)                trial_id += 1            data.append(record)                        }                'pose_complexity': np.random.uniform(0.3, 0.9),                'time_per_iteration': planning_time / iterations if iterations > 0 else 0,                'trajectory_length': trajectory_length,                'iterations_used': iterations,                'execution_time_estimate': execution_time,                'planning_time_ms': planning_time,                'success': 1 if success else 0,                'algorithm': algorithm,                'trial': trial_id,            record = {            # Create data record                            trajectory_length = 0                execution_time = 0                iterations = config['iterations_range'][1]  # Often fails at max iterations                planning_time = max(100, np.random.normal(config['base_time'] * 0.7, config['std_dev'] * 0.5))                # Failed attempts still have some computation time            else:                trajectory_length = max(0.5, np.random.normal(2.8, 0.6))                execution_time = max(100, np.random.normal(800, 150))                iterations = np.random.randint(config['iterations_range'][0], config['iterations_range'][1])                planning_time = max(50, np.random.normal(config['base_time'], config['std_dev']))                # Generate realistic execution time            if success:                        success = np.random.random() < config['success_rate']            # Determine success based on algorithm success rate        for trial in range(20):        # Generate 20 trials per algorithm for better statistics                config = algorithm_configs[algorithm]    for algorithm in algorithms:        trial_id = 1    data = []        }        }            'iterations_range': (600, 2500),            'success_rate': 0.78,            'std_dev': 150,            'base_time': 620,        'HAUSER_INFORMED_RRT': {        },            'iterations_range': (800, 3000),            'success_rate': 0.82,            'std_dev': 180,            'base_time': 720,        'HAUSER_RRT_STAR': {        },            'iterations_range': (500, 2000),            'success_rate': 0.75,            'std_dev': 120,            'base_time': 450,        'HAUSER_RRT': {        },            'iterations_range': (30, 100),            'success_rate': 0.85,            'std_dev': 200,            'base_time': 850,  # Base planning time in ms        'STOMP': {    algorithm_configs = {    algorithms = ['STOMP', 'HAUSER_RRT', 'HAUSER_RRT_STAR', 'HAUSER_INFORMED_RRT']    # Define algorithms to compare        np.random.seed(42)  # For reproducible results        """Generate synthetic SinglePose evaluation data for demonstration purposes."""def generate_synthetic_single_pose_data():from pathlib import Pathimport matplotlib.pyplot as pltimport numpy as npimport pandas as pd
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import seaborn as sns
from pathlib import Path
import sys
import os

def generate_synthetic_single_pose_data():
    """Generate synthetic SinglePose evaluation data for demonstration purposes."""
    
    np.random.seed(42)  # For reproducible results
    
    # Define algorithms to compare
    algorithms = ['STOMP', 'HAUSER_RRT', 'HAUSER_RRT_STAR', 'HAUSER_INFORMED_RRT']
    algorithm_configs = {
        'STOMP': {
            'base_time': 850,  # Base planning time in ms
            'std_dev': 200,
            'success_rate': 0.85,
            'iterations_range': (30, 100),
            'time_per_iter': 8.5
        },
        'HAUSER_RRT': {
            'base_time': 450,
            'std_dev': 120,
            'success_rate': 0.75,
            'iterations_range': (500, 2000),
            'time_per_iter': 0.4
        },
        'HAUSER_RRT_STAR': {
            'base_time': 720,
            'std_dev': 180,
            'success_rate': 0.82,
            'iterations_range': (800, 3000),
            'time_per_iter': 0.35
        },
        'HAUSER_INFORMED_RRT': {
            'base_time': 620,
            'std_dev': 150,
            'success_rate': 0.78,
            'iterations_range': (600, 2500),
            'time_per_iter': 0.32
        }
    }
    
    data = []
    trial_id = 1
    
    for algorithm in algorithms:
        config = algorithm_configs[algorithm]
        
        # Generate 20 trials per algorithm for better statistics
        for trial in range(20):
            # Determine success based on algorithm success rate
            success = np.random.random() < config['success_rate']
            
            if success:
                # Generate realistic execution time
                planning_time = max(50, np.random.normal(config['base_time'], config['std_dev']))
                
                # Generate iterations for iterative algorithms
                if 'HAUSER' in algorithm:
                    iterations = np.random.randint(config['iterations_range'][0], config['iterations_range'][1])
                    path_planning_time = iterations * config['time_per_iter'] + np.random.normal(0, 10)
                    motion_generation_time = max(10, np.random.normal(50, 15))
                    total_planning_time = path_planning_time + motion_generation_time
                else:
                    iterations = np.random.randint(config['iterations_range'][0], config['iterations_range'][1])
                    path_planning_time = 0  # STOMP doesn't separate path planning
                    motion_generation_time = planning_time
                    total_planning_time = planning_time
                
                # Additional metrics
                trajectory_length = max(0.5, np.random.normal(2.8, 0.6))
                execution_time = max(100, np.random.normal(800, 150))
                
            else:
                # Failed attempts still have some computation time
                planning_time = max(100, np.random.normal(config['base_time'] * 0.7, config['std_dev'] * 0.5))
                iterations = config['iterations_range'][1]  # Often fails at max iterations
                
                if 'HAUSER' in algorithm:
                    path_planning_time = iterations * config['time_per_iter']
                    motion_generation_time = 0  # No motion generation on failure
                    total_planning_time = path_planning_time
                else:
                    path_planning_time = 0
                    motion_generation_time = planning_time
                    total_planning_time = planning_time
                
                trajectory_length = 0
                execution_time = 0
            
            # Create data record
            record = {
                'trial': trial_id,
                'algorithm': algorithm,
                'success': 1 if success else 0,
                'planning_time_ms': total_planning_time,
                'path_planning_time_ms': path_planning_time,
                'motion_generation_time_ms': motion_generation_time,
                'trajectory_length': trajectory_length,
                'execution_time_estimate': execution_time,
                'iterations_used': iterations,
                'computation_efficiency': total_planning_time / iterations if iterations > 0 else 0,
                'pose_complexity': np.random.uniform(0.3, 0.9),  # Synthetic complexity metric
                'time_per_iteration': total_planning_time / iterations if iterations > 0 else 0
            }
            
            data.append(record)
            trial_id += 1
    
    return pd.DataFrame(data)

def create_execution_time_boxplots(data, output_path):
    """Create comprehensive execution time boxplots for SinglePose evaluator."""
    
    # Set up the plotting style
    plt.style.use('seaborn-v0_8')
    colors = ['#3498db', '#e74c3c', '#2ecc71', '#f39c12']
    
    # Create figure with subplots
    fig, axes = plt.subplots(2, 3, figsize=(18, 12))
    fig.suptitle('SinglePose Evaluator: Execution Time and Computation Analysis', fontsize=16, fontweight='bold')
    
    # 1. Overall Planning Time Comparison
    ax1 = axes[0, 0]
    bp1 = ax1.boxplot([data[data['algorithm'] == alg]['planning_time_ms'].values 
                      for alg in data['algorithm'].unique()],
                     labels=data['algorithm'].unique(),
                     patch_artist=True)
    
    for patch, color in zip(bp1['boxes'], colors):
        patch.set_facecolor(color)
        patch.set_alpha(0.7)
    
    ax1.set_title('Total Planning Time by Algorithm', fontweight='bold')
    ax1.set_ylabel('Planning Time (ms)')
    ax1.tick_params(axis='x', rotation=45)
    ax1.grid(True, alpha=0.3)
    
    # 2. Successful vs Failed Attempts
    ax2 = axes[0, 1]
    successful_data = data[data['success'] == 1]
    failed_data = data[data['success'] == 0]
    
    success_times = [successful_data[successful_data['algorithm'] == alg]['planning_time_ms'].values
                    for alg in data['algorithm'].unique()]
    failed_times = [failed_data[failed_data['algorithm'] == alg]['planning_time_ms'].values
                   for alg in data['algorithm'].unique()]
    
    x_pos = np.arange(len(data['algorithm'].unique()))
    bp2_success = ax2.boxplot(success_times, positions=x_pos - 0.2, widths=0.3, 
                             patch_artist=True, labels=[''] * len(x_pos))
    bp2_failed = ax2.boxplot(failed_times, positions=x_pos + 0.2, widths=0.3,
                            patch_artist=True, labels=[''] * len(x_pos))
    
    for patch in bp2_success['boxes']:
        patch.set_facecolor('#2ecc71')
        patch.set_alpha(0.7)
    for patch in bp2_failed['boxes']:
        patch.set_facecolor('#e74c3c')
        patch.set_alpha(0.7)
    
    ax2.set_title('Success vs Failed Attempts', fontweight='bold')
    ax2.set_ylabel('Planning Time (ms)')
    ax2.set_xticks(x_pos)
    ax2.set_xticklabels(data['algorithm'].unique(), rotation=45)
    ax2.legend([bp2_success['boxes'][0], bp2_failed['boxes'][0]], 
              ['Successful', 'Failed'], loc='upper right')
    ax2.grid(True, alpha=0.3)
    
    # 3. Computation Efficiency (Time per Iteration)
    ax3 = axes[0, 2]
    efficiency_data = [data[data['algorithm'] == alg]['time_per_iteration'].values
                      for alg in data['algorithm'].unique()]
    
    bp3 = ax3.boxplot(efficiency_data, labels=data['algorithm'].unique(), patch_artist=True)
    for patch, color in zip(bp3['boxes'], colors):
        patch.set_facecolor(color)
        patch.set_alpha(0.7)
    
    ax3.set_title('Computation Efficiency', fontweight='bold')
    ax3.set_ylabel('Time per Iteration (ms/iter)')
    ax3.tick_params(axis='x', rotation=45)
    ax3.grid(True, alpha=0.3)
    
    # 4. Path Planning vs Motion Generation Time (for Hauser algorithms)
    ax4 = axes[1, 0]
    hauser_data = data[data['algorithm'].str.contains('HAUSER')]
    
    if not hauser_data.empty:
        algorithms_hauser = hauser_data['algorithm'].unique()
        path_times = [hauser_data[hauser_data['algorithm'] == alg]['path_planning_time_ms'].values
                     for alg in algorithms_hauser]
        motion_times = [hauser_data[hauser_data['algorithm'] == alg]['motion_generation_time_ms'].values
                       for alg in algorithms_hauser]
        
        x_pos = np.arange(len(algorithms_hauser))
        bp4_path = ax4.boxplot(path_times, positions=x_pos - 0.2, widths=0.3,
                              patch_artist=True, labels=[''] * len(x_pos))
        bp4_motion = ax4.boxplot(motion_times, positions=x_pos + 0.2, widths=0.3,
                                patch_artist=True, labels=[''] * len(x_pos))
        
        for patch in bp4_path['boxes']:
            patch.set_facecolor('#3498db')
            patch.set_alpha(0.7)
        for patch in bp4_motion['boxes']:
            patch.set_facecolor('#9b59b6')
            patch.set_alpha(0.7)
        
        ax4.set_title('Path Planning vs Motion Generation (Hauser)', fontweight='bold')
        ax4.set_ylabel('Time (ms)')
        ax4.set_xticks(x_pos)
        ax4.set_xticklabels([alg.replace('HAUSER_', '') for alg in algorithms_hauser], rotation=45)
        ax4.legend([bp4_path['boxes'][0], bp4_motion['boxes'][0]], 
                  ['Path Planning', 'Motion Generation'], loc='upper right')
        ax4.grid(True, alpha=0.3)
    else:
        ax4.text(0.5, 0.5, 'No Hauser algorithm data', ha='center', va='center', transform=ax4.transAxes)
        ax4.set_title('Path Planning vs Motion Generation (Hauser)', fontweight='bold')
    
    # 5. Execution Time Estimates
    ax5 = axes[1, 1]
    successful_exec_data = data[(data['success'] == 1) & (data['execution_time_estimate'] > 0)]
    exec_times = [successful_exec_data[successful_exec_data['algorithm'] == alg]['execution_time_estimate'].values
                 for alg in data['algorithm'].unique()]
    
    bp5 = ax5.boxplot(exec_times, labels=data['algorithm'].unique(), patch_artist=True)
    for patch, color in zip(bp5['boxes'], colors):
        patch.set_facecolor(color)
        patch.set_alpha(0.7)
    
    ax5.set_title('Estimated Execution Time (Successful Trials)', fontweight='bold')
    ax5.set_ylabel('Execution Time (ms)')
    ax5.tick_params(axis='x', rotation=45)
    ax5.grid(True, alpha=0.3)
    
    # 6. Planning Time vs Trajectory Complexity
    ax6 = axes[1, 2]
    scatter_colors = {'STOMP': '#3498db', 'HAUSER_RRT': '#e74c3c', 
                     'HAUSER_RRT_STAR': '#2ecc71', 'HAUSER_INFORMED_RRT': '#f39c12'}
    
    for alg in data['algorithm'].unique():
        alg_data = data[data['algorithm'] == alg]
        ax6.scatter(alg_data['pose_complexity'], alg_data['planning_time_ms'], 
                   c=scatter_colors[alg], label=alg, alpha=0.6, s=50)
    
    ax6.set_title('Planning Time vs Pose Complexity', fontweight='bold')
    ax6.set_xlabel('Pose Complexity')
    ax6.set_ylabel('Planning Time (ms)')
    ax6.legend(bbox_to_anchor=(1.05, 1), loc='upper left')
    ax6.grid(True, alpha=0.3)
    
    plt.tight_layout()
    plt.savefig(output_path, dpi=300, bbox_inches='tight')
    plt.close()
    
    return output_path

def create_performance_summary_table(data):
    """Create a performance summary table for each algorithm."""
    
    summary_stats = []
    
    for algorithm in data['algorithm'].unique():
        alg_data = data[data['algorithm'] == algorithm]
        successful_data = alg_data[alg_data['success'] == 1]
        
        stats = {
            'Algorithm': algorithm,
            'Success Rate (%)': f"{(alg_data['success'].mean() * 100):.1f}",
            'Avg Planning Time (ms)': f"{alg_data['planning_time_ms'].mean():.1f}",
            'Planning Time Std (ms)': f"{alg_data['planning_time_ms'].std():.1f}",
            'Min Planning Time (ms)': f"{alg_data['planning_time_ms'].min():.1f}",
            'Max Planning Time (ms)': f"{alg_data['planning_time_ms'].max():.1f}",
            'Avg Iterations': f"{alg_data['iterations_used'].mean():.0f}",
            'Time per Iteration (ms)': f"{alg_data['time_per_iteration'].mean():.2f}",
            'Successful Trials': f"{len(successful_data)}/{len(alg_data)}"
        }
        
        summary_stats.append(stats)
    
    return pd.DataFrame(summary_stats)

def main():
    """Main function to generate SinglePose execution time analysis."""
    
    print("🚀 SinglePose Evaluator Execution Time Analysis")
    print("=" * 50)
    
    # Generate synthetic data for demonstration
    print("📊 Generating synthetic SinglePose evaluation data...")
    data = generate_synthetic_single_pose_data()
    print(f"✅ Generated {len(data)} evaluation records")
    
    # Create output directory
    output_dir = Path("./single_pose_analysis_results")
    output_dir.mkdir(exist_ok=True)
    
    # Save the generated data
    data_file = output_dir / "single_pose_evaluation_data.csv"
    data.to_csv(data_file, index=False)
    print(f"💾 Saved evaluation data to: {data_file}")
    
    # Create execution time boxplots
    print("📈 Creating execution time and computation boxplots...")
    boxplot_file = output_dir / "single_pose_execution_time_analysis.png"
    create_execution_time_boxplots(data, boxplot_file)
    print(f"✅ Saved boxplots to: {boxplot_file}")
    
    # Create performance summary
    print("📋 Generating performance summary table...")
    summary_table = create_performance_summary_table(data)
    summary_file = output_dir / "single_pose_performance_summary.csv"
    summary_table.to_csv(summary_file, index=False)
    print(f"💾 Saved summary table to: {summary_file}")
    
    # Display summary
    print("\n🎯 PERFORMANCE SUMMARY")
    print("=" * 60)
    print(summary_table.to_string(index=False))
    
    # Algorithm recommendations
    print("\n🏆 ALGORITHM RECOMMENDATIONS")
    print("=" * 60)
    
    best_success = summary_table.loc[summary_table['Success Rate (%)'].astype(float).idxmax(), 'Algorithm']
    best_speed = summary_table.loc[summary_table['Avg Planning Time (ms)'].astype(float).idxmin(), 'Algorithm']
    best_efficiency = summary_table.loc[summary_table['Time per Iteration (ms)'].astype(float).idxmin(), 'Algorithm']
    
    print(f"🎯 Best Success Rate: {best_success}")
    print(f"⚡ Fastest Planning: {best_speed}")
    print(f"🔧 Most Efficient: {best_efficiency}")
    
    print(f"\n✨ Analysis complete! Results saved to: {output_dir}")

if __name__ == "__main__":
    main()
