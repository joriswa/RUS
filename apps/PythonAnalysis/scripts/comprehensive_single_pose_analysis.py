#!/usr/bin/env python3
"""
Comprehensive SinglePose Evaluator Analysis

This script creates comprehensive execution time and computation time analysis 
for SinglePose evaluation data, including 6-panel boxplot analysis.
"""

import pandas as pd
import numpy as np
import matplotlib
matplotlib.use('Agg')  # Use non-interactive backend
import matplotlib.pyplot as plt
import seaborn as sns
from pathlib import Path
import sys
import os

def generate_comprehensive_single_pose_data():
    """Generate comprehensive synthetic SinglePose evaluation data."""
    
    np.random.seed(42)  # For reproducible results
    
    # Define algorithms to compare
    algorithms = ['STOMP', 'HAUSER_RRT', 'HAUSER_RRT_STAR', 'HAUSER_INFORMED_RRT']
    
    # Algorithm-specific configurations
    algorithm_configs = {
        'STOMP': {
            'base_time': 850,  # Base planning time in ms
            'std_dev': 200,
            'success_rate': 0.85,
            'iterations_range': (30, 100),
        },
        'HAUSER_RRT': {
            'base_time': 450,
            'std_dev': 120,
            'success_rate': 0.75,
            'iterations_range': (500, 2000),
        },
        'HAUSER_RRT_STAR': {
            'base_time': 720,
            'std_dev': 180,
            'success_rate': 0.82,
            'iterations_range': (800, 3000),
        },
        'HAUSER_INFORMED_RRT': {
            'base_time': 620,
            'std_dev': 150,
            'success_rate': 0.78,
            'iterations_range': (600, 2500),
        }
    }
    
    data = []
    trial_id = 1
    
    # Generate 50 trials per algorithm for better statistics
    for algorithm in algorithms:
        config = algorithm_configs[algorithm]
        
        for trial in range(50):
            # Determine success based on algorithm success rate
            success = np.random.random() < config['success_rate']
            
            if success:
                # Generate realistic planning time
                planning_time = max(50, np.random.normal(config['base_time'], config['std_dev']))
                iterations = np.random.randint(config['iterations_range'][0], config['iterations_range'][1])
                execution_time = max(100, np.random.normal(800, 150))
                trajectory_length = max(0.5, np.random.normal(2.8, 0.6))
            else:
                # Failed attempts still have some computation time
                planning_time = max(100, np.random.normal(config['base_time'] * 0.7, config['std_dev'] * 0.5))
                iterations = config['iterations_range'][1]  # Often fails at max iterations
                execution_time = 0
                trajectory_length = 0
            
            # Create data record
            record = {
                'trial': trial_id,
                'algorithm': algorithm,
                'success': 1 if success else 0,
                'planning_time_ms': planning_time,
                'execution_time_estimate': execution_time,
                'iterations_used': iterations,
                'trajectory_length': trajectory_length,
                'time_per_iteration': planning_time / iterations if iterations > 0 else 0,
                'pose_complexity': np.random.uniform(0.3, 0.9),
            }
            
            data.append(record)
            trial_id += 1
    
    return pd.DataFrame(data)

def create_comprehensive_boxplots(data, output_path):
    """Create comprehensive 6-panel execution time boxplots for SinglePose evaluator."""
    
    # Set up colors for consistent visualization
    colors = ['#3498db', '#e74c3c', '#2ecc71', '#f39c12']
    
    # Create figure with subplots
    fig, axes = plt.subplots(2, 3, figsize=(18, 12))
    fig.suptitle('SinglePose Evaluator: Comprehensive Performance Analysis', fontsize=16, fontweight='bold')
    
    algorithms = list(data['algorithm'].unique())
    
    # 1. Overall Planning Time Comparison
    ax1 = axes[0, 0]
    planning_data = [data[data['algorithm'] == alg]['planning_time_ms'].values for alg in algorithms]
    bp1 = ax1.boxplot(planning_data, labels=algorithms, patch_artist=True)
    
    for patch, color in zip(bp1['boxes'], colors):
        patch.set_facecolor(color)
        patch.set_alpha(0.7)
    
    ax1.set_title('Planning Time by Algorithm', fontweight='bold')
    ax1.set_ylabel('Planning Time (ms)')
    ax1.tick_params(axis='x', rotation=45)
    ax1.grid(True, alpha=0.3)
    
    # 2. Successful vs Failed Attempts
    ax2 = axes[0, 1]
    successful_data = data[data['success'] == 1]
    failed_data = data[data['success'] == 0]
    
    success_times = [successful_data[successful_data['algorithm'] == alg]['planning_time_ms'].values 
                    for alg in algorithms]
    failed_times = [failed_data[failed_data['algorithm'] == alg]['planning_time_ms'].values 
                   for alg in algorithms]
    
    x_pos = np.arange(len(algorithms))
    bp2_success = ax2.boxplot(success_times, positions=x_pos - 0.2, widths=0.3, 
                             patch_artist=True)
    bp2_failed = ax2.boxplot(failed_times, positions=x_pos + 0.2, widths=0.3, 
                            patch_artist=True)
    
    for patch in bp2_success['boxes']:
        patch.set_facecolor('#2ecc71')
        patch.set_alpha(0.7)
    
    for patch in bp2_failed['boxes']:
        patch.set_facecolor('#e74c3c')
        patch.set_alpha(0.7)
    
    ax2.set_title('Success vs Failed Attempts', fontweight='bold')
    ax2.set_ylabel('Planning Time (ms)')
    ax2.set_xticks(x_pos)
    ax2.set_xticklabels(algorithms, rotation=45)
    ax2.legend([bp2_success['boxes'][0], bp2_failed['boxes'][0]], 
              ['Successful', 'Failed'], loc='upper right')
    ax2.grid(True, alpha=0.3)
    
    # 3. Computation Efficiency (Time per Iteration)
    ax3 = axes[0, 2]
    efficiency_data = [data[data['algorithm'] == alg]['time_per_iteration'].values 
                      for alg in algorithms]
    bp3 = ax3.boxplot(efficiency_data, labels=algorithms, patch_artist=True)
    
    for patch, color in zip(bp3['boxes'], colors):
        patch.set_facecolor(color)
        patch.set_alpha(0.7)
    
    ax3.set_title('Computation Efficiency', fontweight='bold')
    ax3.set_ylabel('Time per Iteration (ms/iter)')
    ax3.tick_params(axis='x', rotation=45)
    ax3.grid(True, alpha=0.3)
    
    # 4. Iterations Used
    ax4 = axes[1, 0]
    iterations_data = [data[data['algorithm'] == alg]['iterations_used'].values 
                      for alg in algorithms]
    bp4 = ax4.boxplot(iterations_data, labels=algorithms, patch_artist=True)
    
    for patch, color in zip(bp4['boxes'], colors):
        patch.set_facecolor(color)
        patch.set_alpha(0.7)
    
    ax4.set_title('Iterations Used by Algorithm', fontweight='bold')
    ax4.set_ylabel('Number of Iterations')
    ax4.tick_params(axis='x', rotation=45)
    ax4.grid(True, alpha=0.3)
    
    # 5. Execution Time Estimates (Successful only)
    ax5 = axes[1, 1]
    successful_exec_data = data[(data['success'] == 1) & (data['execution_time_estimate'] > 0)]
    exec_times = [successful_exec_data[successful_exec_data['algorithm'] == alg]['execution_time_estimate'].values 
                 for alg in algorithms]
    bp5 = ax5.boxplot(exec_times, labels=algorithms, patch_artist=True)
    
    for patch, color in zip(bp5['boxes'], colors):
        patch.set_facecolor(color)
        patch.set_alpha(0.7)
    
    ax5.set_title('Trajectory Execution Time (Successful)', fontweight='bold')
    ax5.set_ylabel('Execution Time (ms)')
    ax5.tick_params(axis='x', rotation=45)
    ax5.grid(True, alpha=0.3)
    
    # 6. Planning Time vs Pose Complexity
    ax6 = axes[1, 2]
    scatter_colors = {'STOMP': '#3498db', 'HAUSER_RRT': '#e74c3c', 
                     'HAUSER_RRT_STAR': '#2ecc71', 'HAUSER_INFORMED_RRT': '#f39c12'}
    
    for alg in algorithms:
        alg_data = data[data['algorithm'] == alg]
        ax6.scatter(alg_data['pose_complexity'], alg_data['planning_time_ms'], 
                   c=scatter_colors.get(alg, '#333333'), label=alg, alpha=0.6, s=50)
    
    ax6.set_title('Planning Time vs Pose Complexity', fontweight='bold')
    ax6.set_xlabel('Pose Complexity')
    ax6.set_ylabel('Planning Time (ms)')
    ax6.legend(fontsize=8)
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
            'Time per Iteration (ms)': f"{alg_data['time_per_iteration'].mean():.3f}",
            'Successful Trials': f"{len(successful_data)}/{len(alg_data)}"
        }
        
        summary_stats.append(stats)
    
    return pd.DataFrame(summary_stats)

def main():
    """Main function to generate comprehensive SinglePose execution time analysis."""
    
    print("=" * 60)
    print("🚀 Comprehensive SinglePose Evaluator Analysis")
    print("=" * 60)
    
    # Generate synthetic data for demonstration
    print("📊 Generating comprehensive SinglePose evaluation data...")
    data = generate_comprehensive_single_pose_data()
    print(f"✅ Generated {len(data)} evaluation records")
    
    # Create output directory
    output_dir = Path("results/comprehensive_single_pose_analysis")
    output_dir.mkdir(parents=True, exist_ok=True)
    
    # Save the generated data
    data_file = output_dir / "comprehensive_single_pose_evaluation_data.csv"
    data.to_csv(data_file, index=False)
    print(f"💾 Saved evaluation data to: {data_file}")
    
    # Create comprehensive boxplots
    print("📈 Creating comprehensive execution time and computation boxplots...")
    boxplot_file = output_dir / "comprehensive_single_pose_analysis.png"
    create_comprehensive_boxplots(data, boxplot_file)
    print(f"✅ Saved comprehensive analysis to: {boxplot_file}")
    
    # Create performance summary
    print("📋 Generating performance summary table...")
    summary_table = create_performance_summary_table(data)
    summary_file = output_dir / "comprehensive_performance_summary.csv"
    summary_table.to_csv(summary_file, index=False)
    print(f"💾 Saved summary table to: {summary_file}")
    
    # Display summary
    print("\n" + "=" * 60)
    print("🎯 PERFORMANCE SUMMARY")
    print("=" * 60)
    print(summary_table.to_string(index=False))
    
    # Algorithm recommendations
    print("\n" + "=" * 60)
    print("🏆 ALGORITHM RECOMMENDATIONS")
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
