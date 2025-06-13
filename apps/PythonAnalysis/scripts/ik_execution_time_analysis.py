#!/usr/bin/env python3
"""
Execution Time and Computation Time Analysis for IK Comparison

This script analyzes execution times and computation performance from IK comparison data,
focusing on the middle 10 poses for detailed analysis.
"""

import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
import seaborn as sns
from pathlib import Path
import sys

def load_and_analyze_ik_data(csv_file):
    """Load IK comparison data and perform execution time analysis."""
    
    try:
        data = pd.read_csv(csv_file)
        print(f"✅ Loaded {len(data)} records from {csv_file}")
        print(f"📊 Columns: {list(data.columns)}")
        print(f"🔍 Methods: {data['method'].unique()}")
        print(f"📍 Poses: {data['pose_id'].nunique()} unique poses")
        
        return data
        
    except Exception as e:
        print(f"❌ Error loading data: {e}")
        return None

def select_middle_poses(data, n_poses=10):
    """Select the middle n poses from the dataset."""
    
    unique_poses = sorted(data['pose_id'].unique())
    total_poses = len(unique_poses)
    
    # Calculate middle range
    start_idx = (total_poses - n_poses) // 2
    end_idx = start_idx + n_poses
    
    selected_poses = unique_poses[start_idx:end_idx]
    
    print(f"📍 Total poses available: {total_poses}")
    print(f"🎯 Selected middle {n_poses} poses: {selected_poses}")
    
    # Filter data to only include selected poses
    filtered_data = data[data['pose_id'].isin(selected_poses)]
    print(f"📊 Filtered data: {len(filtered_data)} records")
    
    return filtered_data, selected_poses

def create_execution_time_analysis(data, title_suffix=""):
    """Create comprehensive execution time analysis visualization."""
    
    # Create figure with subplots
    fig, axes = plt.subplots(2, 3, figsize=(20, 12))
    fig.suptitle(f'IK Methods: Execution Time and Computation Analysis{title_suffix}', 
                 fontsize=16, fontweight='bold')
    
    # Color palette for methods
    methods = data['method'].unique()
    colors = sns.color_palette("husl", len(methods))
    color_map = dict(zip(methods, colors))
    
    # 1. Execution Time Boxplot by Method
    ax1 = axes[0, 0]
    
    # Filter out infinite or very large values for better visualization
    plot_data = data[data['time_ms'] < 1000].copy()  # Remove outliers > 1 second
    
    if not plot_data.empty:
        box_plot = ax1.boxplot([plot_data[plot_data['method'] == method]['time_ms'].values 
                               for method in methods], 
                              labels=methods, patch_artist=True)
        
        # Color the boxes
        for patch, color in zip(box_plot['boxes'], colors):
            patch.set_facecolor(color)
            patch.set_alpha(0.7)
        
        ax1.set_ylabel('Execution Time (ms)')
        ax1.set_title('Execution Time Distribution by Method')
        ax1.tick_params(axis='x', rotation=45)
        ax1.grid(True, alpha=0.3)
    
    # 2. Success Rate vs Execution Time
    ax2 = axes[0, 1]
    
    successful_data = data[data['success'] == 1]
    failed_data = data[data['success'] == 0]
    
    for method in methods:
        method_success = successful_data[successful_data['method'] == method]
        method_failed = failed_data[failed_data['method'] == method]
        
        if not method_success.empty:
            ax2.scatter(method_success['time_ms'], [method]*len(method_success), 
                       color=color_map[method], alpha=0.7, s=60, marker='o', 
                       label=f'{method} (Success)')
        
        if not method_failed.empty:
            ax2.scatter(method_failed['time_ms'], [method]*len(method_failed), 
                       color=color_map[method], alpha=0.3, s=30, marker='x', 
                       label=f'{method} (Failed)')
    
    ax2.set_xlabel('Execution Time (ms)')
    ax2.set_ylabel('Method')
    ax2.set_title('Success/Failure vs Execution Time')
    ax2.legend(bbox_to_anchor=(1.05, 1), loc='upper left')
    ax2.grid(True, alpha=0.3)
    
    # 3. Iterations vs Time (for successful methods)
    ax3 = axes[0, 2]
    
    if 'iterations' in data.columns:
        success_with_iterations = successful_data[successful_data['iterations'] > 0]
        
        for method in methods:
            method_data = success_with_iterations[success_with_iterations['method'] == method]
            if not method_data.empty:
                ax3.scatter(method_data['iterations'], method_data['time_ms'], 
                           color=color_map[method], alpha=0.7, s=60, label=method)
        
        ax3.set_xlabel('Iterations')
        ax3.set_ylabel('Execution Time (ms)')
        ax3.set_title('Iterations vs Execution Time (Successful)')
        ax3.legend()
        ax3.grid(True, alpha=0.3)
    
    # 4. Computation Efficiency (Time per Iteration)
    ax4 = axes[1, 0]
    
    if 'iterations' in data.columns:
        # Calculate time per iteration for successful methods with iterations > 0
        efficient_data = successful_data[successful_data['iterations'] > 0].copy()
        efficient_data['time_per_iteration'] = efficient_data['time_ms'] / efficient_data['iterations']
        
        if not efficient_data.empty:
            efficiency_by_method = []
            method_labels = []
            
            for method in methods:
                method_efficiency = efficient_data[efficient_data['method'] == method]['time_per_iteration']
                if not method_efficiency.empty:
                    efficiency_by_method.append(method_efficiency.values)
                    method_labels.append(method)
            
            if efficiency_by_method:
                box_plot = ax4.boxplot(efficiency_by_method, labels=method_labels, patch_artist=True)
                
                # Color the boxes
                method_colors = [color_map[method] for method in method_labels]
                for patch, color in zip(box_plot['boxes'], method_colors):
                    patch.set_facecolor(color)
                    patch.set_alpha(0.7)
                
                ax4.set_ylabel('Time per Iteration (ms/iter)')
                ax4.set_title('Computation Efficiency by Method')
                ax4.tick_params(axis='x', rotation=45)
                ax4.grid(True, alpha=0.3)
    
    # 5. Pose-wise Execution Time Comparison
    ax5 = axes[1, 1]
    
    # Create heatmap of execution times across poses and methods
    pivot_data = data.pivot_table(values='time_ms', index='method', columns='pose_id', aggfunc='mean')
    
    if not pivot_data.empty:
        # Replace inf values with NaN for better visualization
        pivot_data = pivot_data.replace([np.inf, -np.inf], np.nan)
        
        im = ax5.imshow(pivot_data.values, cmap='viridis', aspect='auto')
        ax5.set_xticks(range(len(pivot_data.columns)))
        ax5.set_xticklabels(pivot_data.columns)
        ax5.set_yticks(range(len(pivot_data.index)))
        ax5.set_yticklabels(pivot_data.index)
        ax5.set_xlabel('Pose ID')
        ax5.set_ylabel('Method')
        ax5.set_title('Execution Time Heatmap (ms)')
        
        # Add colorbar
        cbar = plt.colorbar(im, ax=ax5)
        cbar.set_label('Execution Time (ms)')
    
    # 6. Performance Summary Statistics
    ax6 = axes[1, 2]
    ax6.axis('off')
    
    # Create summary statistics table
    summary_data = []
    
    for method in methods:
        method_data = data[data['method'] == method]
        success_rate = method_data['success'].mean() * 100
        avg_time = method_data['time_ms'].mean()
        std_time = method_data['time_ms'].std()
        min_time = method_data['time_ms'].min()
        max_time = method_data['time_ms'].max()
        
        summary_data.append([
            method,
            f"{success_rate:.1f}%",
            f"{avg_time:.1f}",
            f"{std_time:.1f}",
            f"{min_time:.1f}",
            f"{max_time:.1f}"
        ])
    
    if summary_data:
        table = ax6.table(cellText=summary_data,
                         colLabels=['Method', 'Success Rate', 'Avg Time (ms)', 
                                   'Std Dev (ms)', 'Min (ms)', 'Max (ms)'],
                         cellLoc='center',
                         loc='center',
                         bbox=[0, 0, 1, 1])
        table.auto_set_font_size(False)
        table.set_fontsize(10)
        table.scale(1, 2)
        ax6.set_title('Performance Summary Statistics', pad=20)
    
    plt.tight_layout()
    return fig

def create_detailed_boxplot_analysis(data, title_suffix=""):
    """Create detailed boxplot analysis for execution and computation metrics."""
    
    # Create figure with subplots for detailed analysis
    fig, axes = plt.subplots(2, 2, figsize=(16, 12))
    fig.suptitle(f'Detailed Execution Time and Computation Analysis{title_suffix}', 
                 fontsize=16, fontweight='bold')
    
    methods = data['method'].unique()
    colors = sns.color_palette("husl", len(methods))
    
    # 1. Execution Time Boxplot (All Data)
    ax1 = axes[0, 0]
    sns.boxplot(data=data, x='method', y='time_ms', ax=ax1, palette=colors)
    ax1.set_title('Execution Time Distribution (All Attempts)')
    ax1.set_ylabel('Execution Time (ms)')
    ax1.tick_params(axis='x', rotation=45)
    ax1.grid(True, alpha=0.3)
    
    # 2. Execution Time Boxplot (Successful Only)
    ax2 = axes[0, 1]
    successful_data = data[data['success'] == 1]
    if not successful_data.empty:
        sns.boxplot(data=successful_data, x='method', y='time_ms', ax=ax2, palette=colors)
        ax2.set_title('Execution Time Distribution (Successful Only)')
        ax2.set_ylabel('Execution Time (ms)')
        ax2.tick_params(axis='x', rotation=45)
        ax2.grid(True, alpha=0.3)
    
    # 3. Iterations Boxplot (if available)
    ax3 = axes[1, 0]
    if 'iterations' in data.columns:
        iteration_data = data[data['iterations'] > 0]  # Filter out -1 values
        if not iteration_data.empty:
            sns.boxplot(data=iteration_data, x='method', y='iterations', ax=ax3, palette=colors)
            ax3.set_title('Iterations Distribution (Iterative Methods)')
            ax3.set_ylabel('Number of Iterations')
            ax3.tick_params(axis='x', rotation=45)
            ax3.grid(True, alpha=0.3)
    
    # 4. Time per Iteration Boxplot (Computation Efficiency)
    ax4 = axes[1, 1]
    if 'iterations' in data.columns:
        efficient_data = successful_data[successful_data['iterations'] > 0].copy()
        if not efficient_data.empty:
            efficient_data['time_per_iteration'] = efficient_data['time_ms'] / efficient_data['iterations']
            sns.boxplot(data=efficient_data, x='method', y='time_per_iteration', ax=ax4, palette=colors)
            ax4.set_title('Computation Efficiency (Time per Iteration)')
            ax4.set_ylabel('Time per Iteration (ms/iter)')
            ax4.tick_params(axis='x', rotation=45)
            ax4.grid(True, alpha=0.3)
    
    plt.tight_layout()
    return fig

def generate_summary_report(data, selected_poses):
    """Generate a comprehensive summary report."""
    
    print("\n" + "="*80)
    print("EXECUTION TIME AND COMPUTATION ANALYSIS REPORT")
    print("="*80)
    
    print(f"\n📊 DATASET OVERVIEW")
    print(f"   - Total records analyzed: {len(data)}")
    print(f"   - Selected poses: {selected_poses}")
    print(f"   - Methods compared: {', '.join(data['method'].unique())}")
    print(f"   - Overall success rate: {data['success'].mean()*100:.1f}%")
    
    print(f"\n⏱️  EXECUTION TIME ANALYSIS")
    methods = data['method'].unique()
    
    for method in methods:
        method_data = data[data['method'] == method]
        successful_method = method_data[method_data['success'] == 1]
        
        print(f"\n   {method}:")
        print(f"     - Success rate: {method_data['success'].mean()*100:.1f}%")
        print(f"     - Average time (all): {method_data['time_ms'].mean():.1f} ms")
        print(f"     - Std deviation: {method_data['time_ms'].std():.1f} ms")
        
        if not successful_method.empty:
            print(f"     - Average time (successful): {successful_method['time_ms'].mean():.1f} ms")
            print(f"     - Min time: {successful_method['time_ms'].min():.1f} ms")
            print(f"     - Max time: {successful_method['time_ms'].max():.1f} ms")
        
        if 'iterations' in method_data.columns:
            iter_data = method_data[method_data['iterations'] > 0]
            if not iter_data.empty:
                print(f"     - Average iterations: {iter_data['iterations'].mean():.1f}")
                if not successful_method.empty:
                    success_iter = successful_method[successful_method['iterations'] > 0]
                    if not success_iter.empty:
                        time_per_iter = (success_iter['time_ms'] / success_iter['iterations']).mean()
                        print(f"     - Time per iteration: {time_per_iter:.2f} ms/iter")
    
    print(f"\n🎯 POSE-SPECIFIC ANALYSIS")
    for pose_id in selected_poses:
        pose_data = data[data['pose_id'] == pose_id]
        successful_pose = pose_data[pose_data['success'] == 1]
        
        print(f"\n   Pose {pose_id}:")
        print(f"     - Success rate: {pose_data['success'].mean()*100:.1f}%")
        print(f"     - Successful methods: {len(successful_pose)} out of {len(pose_data)}")
        
        if not successful_pose.empty:
            fastest_method = successful_pose.loc[successful_pose['time_ms'].idxmin()]
            print(f"     - Fastest successful method: {fastest_method['method']} ({fastest_method['time_ms']:.1f} ms)")

def main():
    """Main function for execution time analysis."""
    
    if len(sys.argv) != 2:
        print("Usage: python execution_time_analysis.py <csv_file>")
        print("Example: python execution_time_analysis.py four_method_comparison_results.csv")
        return 1
    
    csv_file = sys.argv[1]
    
    if not Path(csv_file).exists():
        print(f"❌ File not found: {csv_file}")
        return 1
    
    # Load data
    data = load_and_analyze_ik_data(csv_file)
    if data is None:
        return 1
    
    # Select middle 10 poses
    filtered_data, selected_poses = select_middle_poses(data, n_poses=10)
    
    # Create visualizations
    print("\n🔄 Generating execution time analysis...")
    
    # Full dataset analysis
    full_analysis_fig = create_execution_time_analysis(data, " (All Poses)")
    full_analysis_fig.savefig('execution_time_analysis_full.png', dpi=300, bbox_inches='tight')
    print("✅ Saved: execution_time_analysis_full.png")
    
    # Middle 10 poses analysis
    middle_analysis_fig = create_execution_time_analysis(filtered_data, " (Middle 10 Poses)")
    middle_analysis_fig.savefig('execution_time_analysis_middle10.png', dpi=300, bbox_inches='tight')
    print("✅ Saved: execution_time_analysis_middle10.png")
    
    # Detailed boxplot analysis
    boxplot_fig = create_detailed_boxplot_analysis(filtered_data, " (Middle 10 Poses)")
    boxplot_fig.savefig('execution_time_boxplots_detailed.png', dpi=300, bbox_inches='tight')
    print("✅ Saved: execution_time_boxplots_detailed.png")
    
    # Generate summary report
    generate_summary_report(filtered_data, selected_poses)
    
    # Show plots if display is available
    try:
        plt.show()
    except:
        print("ℹ️  Display not available - plots saved as PNG files")
    
    print(f"\n🎉 Analysis complete! Generated visualizations for middle {len(selected_poses)} poses")
    return 0

if __name__ == "__main__":
    exit(main())
