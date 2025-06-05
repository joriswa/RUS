#!/usr/bin/env python3
"""
Simple Execution Time Plotting Script for ComparisonIK

This script creates basic but informative plots of execution time data from the 
ComparisonIK variance analysis.
"""

import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
from pathlib import Path
import argparse

def create_simple_plots(csv_file, output_dir='.'):
    """Create simple execution time plots"""
    
    # Load data
    try:
        df = pd.read_csv(csv_file)
        print(f"Loaded {len(df)} records from {csv_file}")
    except FileNotFoundError:
        print(f"Error: File {csv_file} not found")
        return
    except Exception as e:
        print(f"Error loading data: {e}")
        return
    
    output_dir = Path(output_dir)
    output_dir.mkdir(exist_ok=True)
    
    # Create a 2x2 subplot layout
    fig, ((ax1, ax2), (ax3, ax4)) = plt.subplots(2, 2, figsize=(14, 10))
    fig.suptitle('SelectGoalPose Execution Time Analysis', fontsize=16, fontweight='bold')
    
    # Plot 1: Simple histogram of all execution times
    ax1.hist(df['solve_time'], bins=20, alpha=0.7, color='skyblue', edgecolor='black')
    ax1.set_xlabel('Execution Time (seconds)')
    ax1.set_ylabel('Frequency')
    ax1.set_title('Distribution of Execution Times')
    ax1.grid(True, alpha=0.3)
    
    # Add mean line
    mean_time = df['solve_time'].mean()
    ax1.axvline(mean_time, color='red', linestyle='--', linewidth=2, 
                label=f'Mean: {mean_time:.4f}s')
    ax1.legend()
    
    # Plot 2: Box plot by pose
    poses = sorted(df['pose_index'].unique())
    pose_data = [df[df['pose_index'] == pose]['solve_time'].values for pose in poses]
    
    box_plot = ax2.boxplot(pose_data, patch_artist=True)
    colors = plt.cm.Set3(np.linspace(0, 1, len(poses)))
    for patch, color in zip(box_plot['boxes'], colors):
        patch.set_facecolor(color)
    
    ax2.set_xlabel('Pose Index')
    ax2.set_ylabel('Execution Time (seconds)')
    ax2.set_title('Execution Time by Pose')
    ax2.set_xticklabels([f'P{pose}' for pose in poses])
    ax2.grid(True, alpha=0.3)
    
    # Plot 3: Trial progression for each pose
    for pose in poses:
        pose_df = df[df['pose_index'] == pose].sort_values('trial_number')
        ax3.plot(pose_df['trial_number'], pose_df['solve_time'], 
                marker='o', alpha=0.7, label=f'Pose {pose}')
    
    ax3.set_xlabel('Trial Number')
    ax3.set_ylabel('Execution Time (seconds)')
    ax3.set_title('Execution Time Across Trials')
    ax3.legend()
    ax3.grid(True, alpha=0.3)
    
    # Plot 4: Mean and standard deviation by pose
    pose_stats = df.groupby('pose_index')['solve_time'].agg(['mean', 'std']).reset_index()
    
    x_pos = range(len(pose_stats))
    ax4.errorbar(x_pos, pose_stats['mean'], yerr=pose_stats['std'], 
                fmt='o-', capsize=5, capthick=2, linewidth=2, markersize=8,
                color='darkblue', alpha=0.8)
    
    ax4.set_xlabel('Pose Index')
    ax4.set_ylabel('Execution Time (seconds)')
    ax4.set_title('Mean Execution Time with Standard Deviation')
    ax4.set_xticks(x_pos)
    ax4.set_xticklabels([f'P{int(pose)}' for pose in pose_stats['pose_index']])
    ax4.grid(True, alpha=0.3)
    
    plt.tight_layout()
    
    # Save the plot
    output_file = output_dir / 'execution_time_analysis.png'
    plt.savefig(output_file, dpi=300, bbox_inches='tight')
    print(f"Plot saved to: {output_file}")
    
    # Show the plot
    plt.show()
    
    # Print basic statistics
    print("\n" + "="*50)
    print("EXECUTION TIME STATISTICS")
    print("="*50)
    print(f"Total trials: {len(df)}")
    print(f"Number of poses: {df['pose_index'].nunique()}")
    print(f"Trials per pose: {len(df[df['pose_index'] == df['pose_index'].iloc[0]])}")
    print(f"Mean execution time: {df['solve_time'].mean():.6f} seconds")
    print(f"Standard deviation: {df['solve_time'].std():.6f} seconds")
    print(f"Min execution time: {df['solve_time'].min():.6f} seconds")
    print(f"Max execution time: {df['solve_time'].max():.6f} seconds")
    print(f"Success rate: {df['success'].mean() * 100:.1f}%")
    
    print("\nPer-pose statistics:")
    for pose in sorted(df['pose_index'].unique()):
        pose_data = df[df['pose_index'] == pose]['solve_time']
        print(f"  Pose {pose}: mean={pose_data.mean():.6f}s, std={pose_data.std():.6f}s")

def main():
    parser = argparse.ArgumentParser(description='Create simple execution time plots')
    parser.add_argument('--csv', default='ik_variance_analysis_results.csv',
                       help='Path to variance analysis CSV file')
    parser.add_argument('--output-dir', default='.',
                       help='Output directory for plots')
    
    args = parser.parse_args()
    create_simple_plots(args.csv, args.output_dir)

if __name__ == "__main__":
    main()
