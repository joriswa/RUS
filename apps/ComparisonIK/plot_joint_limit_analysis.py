#!/usr/bin/env python3
"""
Joint Limit Analysis for ComparisonIK
=====================================

Analyzes joint limit proximity and safety margins for IK solutions.
Now enabled with joint angle data from the updated TwoMethodComparison program.
"""

import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import seaborn as sns
from pathlib import Path

def load_data():
    """Load the comparison data with joint angles."""
    data_file = Path(__file__).parent / "two_method_comparison_results.csv"
    if not data_file.exists():
        raise FileNotFoundError(f"Data file not found: {data_file}")
    
    return pd.read_csv(data_file)

def calculate_joint_limit_distances(df):
    """
    Calculate distances to joint limits for collision-free solutions only.
    Uses the official Franka Panda joint limits from franka_ik_He.h
    """
    
    # Official Franka Panda joint limits (from franka_ik_He.h)
    q_min = np.array([-2.8973, -1.7628, -2.8973, -3.0718, -2.8973, -0.0175, -2.8973])
    q_max = np.array([2.8973, 1.7628, 2.8973, -0.0698, 2.8973, 3.7525, 2.8973])
    
    # Filter collision-free solutions only (successful AND collision-free)
    collision_free_df = df[(df['success'] == 1) & (df['collision_free'] == 1)].copy()
    
    if collision_free_df.empty:
        print("❌ No collision-free solutions found in the data!")
        return None
    
    total_successful = len(df[df['success'] == 1])
    print(f"✅ Found {len(collision_free_df)} collision-free solutions for analysis")
    print(f"   (out of {total_successful} successful solutions total)")
    
    # Calculate distances to limits for each joint
    joint_distances = {}
    for i in range(1, 8):  # q1 to q7
        joint_col = f'q{i}'
        if joint_col in collision_free_df.columns:
            angles = collision_free_df[joint_col].values
            
            # Distance to lower and upper limits
            dist_to_lower = angles - q_min[i-1]
            dist_to_upper = q_max[i-1] - angles
            
            # Minimum distance to either limit
            min_distances = np.minimum(dist_to_lower, dist_to_upper)
            joint_distances[f'q{i}_distance'] = min_distances
    
    # Add joint distance columns to dataframe
    for col, distances in joint_distances.items():
        collision_free_df[col] = distances
    
    # Calculate overall minimum distance to any joint limit
    distance_cols = [f'q{i}_distance' for i in range(1, 8)]
    collision_free_df['min_distance_to_limits'] = collision_free_df[distance_cols].min(axis=1)
    
    return collision_free_df

def create_joint_limit_analysis_plots(df):
    """Create the two focused joint limit analysis plots."""
    
    print("\n" + "="*70)
    print("CREATING JOINT LIMIT ANALYSIS PLOTS")
    print("="*70)
    
    # Create the plots
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(16, 6))
    colors = ['#1f77b4', '#ff7f0e']  # Default blue/orange for SA vs NR
    
    # Plot 1: Minimum Distance to Joint Limits
    ax1.set_title('Minimum Distance to Joint Limits\n(Collision-Free Solutions Only)', fontsize=14, fontweight='bold')
    
    # Prepare data for boxplot
    sa_data = df[df['method'] == 'SA-Optimized']['min_distance_to_limits']
    nr_data = df[df['method'] == 'Newton-Raphson']['min_distance_to_limits']
    
    box_data = [sa_data, nr_data]
    tick_labels = ['SA-Optimized', 'Newton-Raphson']
    
    bp = ax1.boxplot(box_data, tick_labels=tick_labels, patch_artist=True, widths=0.6)
    
    # Set box colors
    for patch, color in zip(bp['boxes'], colors):
        patch.set_facecolor(color)
        patch.set_alpha(0.7)
    
    # Set median lines to black
    for median in bp['medians']:
        median.set_color('black')
    
    ax1.set_ylabel('Distance to Nearest Joint Limit (rad)', fontsize=12)
    ax1.set_xlabel('IK Method', fontsize=12)
    ax1.grid(True, alpha=0.3)
    
    # Add safety threshold lines
    ax1.axhline(y=0.1, color='red', linestyle='--', alpha=0.7, label='Critical (0.1 rad)')
    ax1.axhline(y=0.2, color='orange', linestyle='--', alpha=0.7, label='Warning (0.2 rad)')
    ax1.axhline(y=0.5, color='green', linestyle='--', alpha=0.7, label='Safe (0.5 rad)')
    ax1.legend(fontsize=10)
    
    # Plot 2: Distance to Limits by Joint
    ax2.set_title('Distance to Joint Limits by Joint\n(Collision-Free Solutions Only)', fontsize=14, fontweight='bold')
    
    # Prepare data for grouped boxplot
    joint_data = []
    joint_labels = []
    joint_methods = []
    
    for i in range(1, 8):
        joint_col = f'q{i}_distance'
        if joint_col in df.columns:
            for method in ['SA-Optimized', 'Newton-Raphson']:
                method_data = df[df['method'] == method][joint_col].dropna()
                if not method_data.empty:
                    joint_data.extend(method_data.values)
                    joint_labels.extend([f'q{i}'] * len(method_data))
                    joint_methods.extend([method] * len(method_data))
    
    if joint_data:
        plot_df = pd.DataFrame({
            'distance': joint_data,
            'joint': joint_labels,
            'method': joint_methods
        })
        
        # Create color palette for the two methods
        palette_dict = {'SA-Optimized': colors[0], 'Newton-Raphson': colors[1]}
        
        bp = sns.boxplot(data=plot_df, x='joint', y='distance', hue='method', 
                        palette=palette_dict, ax=ax2)
        
        # Set median lines to black for better visibility
        for patch in ax2.artists:
            patch.set_alpha(0.7)
        
        # Set median lines to black
        for line in ax2.lines:
            if line.get_linestyle() == '-':  # median lines
                line.set_color('black')
        
        ax2.set_ylabel('Distance to Joint Limit (rad)', fontsize=12)
        ax2.set_xlabel('Joint', fontsize=12)
        ax2.grid(True, alpha=0.3)
        
        # Add safety threshold lines
        ax2.axhline(y=0.1, color='red', linestyle='--', alpha=0.7)
        ax2.axhline(y=0.2, color='orange', linestyle='--', alpha=0.7)
        ax2.axhline(y=0.5, color='green', linestyle='--', alpha=0.7)
        
        handles, labels = ax2.get_legend_handles_labels()
        ax2.legend(handles, labels, title='Method', fontsize=10)
    else:
        ax2.text(0.5, 0.5, 'No joint data available', 
                horizontalalignment='center', verticalalignment='center',
                transform=ax2.transAxes, fontsize=14)
    
    plt.tight_layout()
    
    # Save plots
    output_dir = Path(__file__).parent / "plots"
    output_dir.mkdir(exist_ok=True)
    
    plt.savefig(output_dir / "joint_limit_analysis.png", dpi=300, bbox_inches='tight')
    plt.savefig(output_dir / "joint_limit_analysis.pdf", bbox_inches='tight')
    
    plt.show()
    
    return True

def analyze_joint_limit_statistics(df):
    """Analyze and print joint limit safety statistics for collision-free solutions."""
    
    print("\n" + "="*70)
    print("JOINT LIMIT SAFETY ANALYSIS (COLLISION-FREE SOLUTIONS)")
    print("="*70)
    
    # Group by method
    for method in ['SA-Optimized', 'Newton-Raphson']:
        method_data = df[df['method'] == method]
        if method_data.empty:
            continue
            
        print(f"\n🔧 {method}:")
        print(f"  • Collision-free solutions: {len(method_data)}")
        
        if 'min_distance_to_limits' in method_data.columns:
            min_dist = method_data['min_distance_to_limits']
            
            print(f"  • Minimum safety margin: {min_dist.min():.4f} rad ({np.degrees(min_dist.min()):.1f}°)")
            print(f"  • Average safety margin: {min_dist.mean():.4f} rad ({np.degrees(min_dist.mean()):.1f}°)")
            print(f"  • Maximum safety margin: {min_dist.max():.4f} rad ({np.degrees(min_dist.max()):.1f}°)")
            
            # Safety zone classification
            critical = (min_dist < 0.1).sum()
            warning = ((min_dist >= 0.1) & (min_dist < 0.2)).sum()
            safe = (min_dist >= 0.2).sum()
            
            total = len(min_dist)
            print(f"  • Critical zone (<0.1 rad): {critical}/{total} ({100*critical/total:.1f}%)")
            print(f"  • Warning zone (0.1-0.2 rad): {warning}/{total} ({100*warning/total:.1f}%)")
            print(f"  • Safe zone (≥0.2 rad): {safe}/{total} ({100*safe/total:.1f}%)")
            
            # Most constrained joint
            joint_risks = []
            for i in range(1, 8):
                joint_col = f'q{i}_distance'
                if joint_col in method_data.columns:
                    avg_distance = method_data[joint_col].mean()
                    joint_risks.append((f'q{i}', avg_distance))
            
            if joint_risks:
                joint_risks.sort(key=lambda x: x[1])
                print(f"  • Most constrained joint: {joint_risks[0][0]} (avg: {joint_risks[0][1]:.4f} rad)")
                print(f"  • Least constrained joint: {joint_risks[-1][0]} (avg: {joint_risks[-1][1]:.4f} rad)")

def main():
    """Main execution function."""
    try:
        # Load data
        print("Loading comparison data with joint angles...")
        df = load_data()
        
        print(f"Loaded {len(df)} data points")
        print(f"Methods: {', '.join(df['method'].unique())}")
        print(f"Success rate: {(df['success'].sum() / len(df) * 100):.1f}%")
        
        # Check if joint angle data is available
        joint_cols = [f'q{i}' for i in range(1, 8)]
        missing_cols = [col for col in joint_cols if col not in df.columns]
        
        if missing_cols:
            print(f"❌ Missing joint angle columns: {missing_cols}")
            print("Please ensure the C++ program has been updated to output joint angles.")
            return
        
        # Calculate joint limit distances
        collision_free_df = calculate_joint_limit_distances(df)
        if collision_free_df is None:
            return
        
        # Create plots
        create_joint_limit_analysis_plots(collision_free_df)
        
        # Analyze statistics
        analyze_joint_limit_statistics(collision_free_df)
        
        print("\n" + "="*70)
        print("JOINT LIMIT ANALYSIS - COMPLETE")
        print("="*70)
        print("✅ Real joint angle data analyzed successfully") 
        print("✅ Joint limit safety plots generated (collision-free solutions)")
        print("✅ Safety margin statistics calculated")
        print("📋 Results saved to plots/ directory")
        
    except Exception as e:
        print(f"Error in joint limit analysis: {e}")
        raise

if __name__ == "__main__":
    main()