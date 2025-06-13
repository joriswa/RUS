#!/usr/bin/env python3
"""
Quick Trajectory Analysis Dashboard

A simplified script for interactive viewing of trajectory planning metrics.
"""

import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
from pathlib import Path
import sys

def create_quick_dashboard(csv_file):
    """Create a quick interactive dashboard for trajectory analysis."""
    
    # Load data
    try:
        data = pd.read_csv(csv_file)
        success_data = data[data['success'] == 1]
        
        if len(success_data) == 0:
            print("No successful trials found!")
            return
            
        print(f"Loaded {len(success_data)} successful trials")
        
    except Exception as e:
        print(f"Error loading data: {e}")
        return
    
    # Create figure with subplots
    fig, axes = plt.subplots(2, 3, figsize=(18, 12))
    fig.suptitle('Trajectory Planning Analysis Dashboard', fontsize=16, fontweight='bold')
    
    # 1. Success Rate
    ax = axes[0, 0]
    success_rate = data['success'].mean() * 100
    ax.pie([success_rate, 100 - success_rate], labels=['Success', 'Failure'], 
           colors=['green', 'red'], autopct='%1.1f%%', startangle=90)
    ax.set_title(f'Success Rate: {success_rate:.1f}%')
    
    # 2. Planning Time
    ax = axes[0, 1]
    if 'planning_time_ms' in success_data.columns:
        planning_times = success_data['planning_time_ms'] / 1000
        ax.hist(planning_times, bins=10, alpha=0.7, color='skyblue', edgecolor='black')
        ax.axvline(planning_times.mean(), color='red', linestyle='--', 
                   label=f'Mean: {planning_times.mean():.1f}s')
        ax.set_xlabel('Planning Time (s)')
        ax.set_ylabel('Frequency')
        ax.set_title('Planning Time Distribution')
        ax.legend()
    
    # 3. Safety Assessment (Clearance)
    ax = axes[0, 2]
    if 'min_clearance' in success_data.columns:
        clearances = success_data['min_clearance']
        ax.hist(clearances, bins=10, alpha=0.7, color='lightgreen', edgecolor='black')
        ax.axvline(0.05, color='red', linestyle='--', label='Critical (5cm)')
        ax.axvline(0.15, color='orange', linestyle='--', label='Safe (15cm)')
        ax.set_xlabel('Minimum Clearance (m)')
        ax.set_ylabel('Frequency')
        ax.set_title('Clearance Distribution')
        ax.legend()
    
    # 4. Manipulability
    ax = axes[1, 0]
    if 'min_manipulability' in success_data.columns:
        manip = success_data['min_manipulability']
        ax.boxplot(manip, patch_artist=True, 
                   boxprops=dict(facecolor='lightblue', alpha=0.7))
        ax.set_ylabel('Manipulability Index')
        ax.set_title('Manipulability Distribution')
        ax.text(1.1, manip.median(), f'Median: {manip.median():.3f}',
                verticalalignment='center')
    
    # 5. Time-to-Collision Safety
    ax = axes[1, 1]
    if 'min_ttc' in success_data.columns:
        ttc = success_data['min_ttc']
        ax.hist(ttc, bins=10, alpha=0.7, color='coral', edgecolor='black')
        ax.axvline(1.0, color='red', linestyle='--', label='Critical (1s)')
        ax.axvline(3.0, color='green', linestyle='--', label='Safe (3s)')
        ax.set_xlabel('Minimum Time-to-Collision (s)')
        ax.set_ylabel('Frequency')
        ax.set_title('TTC Distribution')
        ax.legend()
    
    # 6. Performance Summary Table
    ax = axes[1, 2]
    ax.axis('off')
    
    # Create summary data
    summary_data = []
    metrics = {
        'Planning Time (s)': 'planning_time_ms',
        'Trajectory Length': 'trajectory_length',
        'Min Clearance (m)': 'min_clearance',
        'Min TTC (s)': 'min_ttc',
        'Min Manipulability': 'min_manipulability',
        'Execution Time (s)': 'execution_time'
    }
    
    for display_name, metric in metrics.items():
        if metric in success_data.columns:
            values = success_data[metric]
            if metric == 'planning_time_ms':
                values = values / 1000  # Convert to seconds
            
            summary_data.append([
                display_name,
                f"{values.mean():.3f}",
                f"{values.std():.3f}",
                f"{values.min():.3f}",
                f"{values.max():.3f}"
            ])
    
    if summary_data:
        table = ax.table(cellText=summary_data,
                        colLabels=['Metric', 'Mean', 'Std', 'Min', 'Max'],
                        cellLoc='center',
                        loc='center',
                        bbox=[0, 0, 1, 1])
        table.auto_set_font_size(False)
        table.set_fontsize(9)
        table.scale(1, 2)
        ax.set_title('Performance Summary')
    
    plt.tight_layout()
    plt.show()

def main():
    """Main function."""
    if len(sys.argv) != 2:
        print("Usage: python quick_dashboard.py <csv_file>")
        print("Example: python quick_dashboard.py ../data/four_method_comparison_results.csv")
        print("Example: python quick_dashboard.py ../results/single_pose_analysis_results/single_pose_evaluation_data.csv")
        return 1
    
    csv_file = sys.argv[1]
    if not Path(csv_file).exists():
        print(f"File not found: {csv_file}")
        return 1
    
    create_quick_dashboard(csv_file)
    return 0

if __name__ == "__main__":
    exit(main())
