#!/usr/bin/env python3
"""
Execution Time Analysis for IK Methods
Analyzes computational efficiency across different IK methods.
This measures how quickly methods can solve inverse kinematics problems.

Created: December 2024
Author: PathPlanner Analysis Suite
"""

import pandas as pd
import matplotlib.pyplot as plt
import seaborn as sns
import numpy as np
import os
from datetime import datetime
import warnings

# Suppress future warnings
warnings.filterwarnings('ignore', category=FutureWarning)

def set_median_lines_black():
    """Helper function to set median lines to black color in boxplots."""
    for line in plt.gca().lines:
        if line.get_linestyle() == '-':  # median lines
            line.set_color('black')

def plot_execution_time_analysis():
    """Generate execution time boxplots for different success categories - Original Format."""
    
    print("🚀 Execution Time Analysis for IK Methods - Original Format")
    print("=" * 65)
    
    # Load data
    try:
        df = pd.read_csv('three_method_comparison_results.csv')
        print(f"📊 Total records loaded: {len(df)}")
        print(f"🔢 Unique poses: {df['pose_id'].nunique()}")
        print(f"🎯 Methods: {list(df['method'].unique())}")
        
    except FileNotFoundError:
        print("❌ Error: Could not find 'three_method_comparison_results.csv'")
        print("   Please run the C++ comparison script first to generate the data.")
        return
    except Exception as e:
        print(f"❌ Error loading data: {e}")
        return
    
    # Set consistent styling
    plt.style.use('default')
    
    # Create three separate plots
    fig, axes = plt.subplots(1, 3, figsize=(18, 6))
    fig.suptitle('IK Method Execution Time Comparison', fontsize=16, fontweight='bold')
    
    # Define consistent colors (default matplotlib colors)
    colors = ['#1f77b4', '#ff7f0e']  # Blue, Orange (default matplotlib)
    method_colors = {'Newton-Raphson': colors[0], 'SA-Optimized': colors[1]}
    
    # Function to set median lines to black
    def set_median_lines_black(ax):
        for line in ax.lines:
            if line.get_linestyle() == '-':  # median lines
                line.set_color('black')
    
    # 1. All Attempts
    ax1 = axes[0]
    df_all = df.copy()
    sns.boxplot(data=df_all, x='method', y='time_ms', hue='method', ax=ax1, palette=method_colors, legend=False)
    set_median_lines_black(ax1)
    ax1.set_title('All Attempts', fontsize=14, fontweight='bold')
    ax1.set_ylabel('Execution Time (ms)')
    ax1.set_xlabel('Method')
    
    set_median_lines_black(ax1)
    
    # Add sample sizes and statistics
    for i, method in enumerate(df_all['method'].unique()):
        method_data = df_all[df_all['method'] == method]['time_ms']
        n_samples = len(method_data)
        median_val = method_data.median()
        mean_val = method_data.mean()
        
        ax1.text(i, ax1.get_ylim()[1] * 0.85, 
                f'n={n_samples}\nμ={mean_val:.2f}ms\nM={median_val:.2f}ms', 
                ha='center', va='top', fontsize=10,
                bbox=dict(boxstyle="round,pad=0.3", facecolor="lightgray", alpha=0.7))
    
    # 2. Successful Attempts Only
    ax2 = axes[1]
    df_success = df[df['success'] == 1].copy()
    if len(df_success) > 0:
        sns.boxplot(data=df_success, x='method', y='time_ms', hue='method', ax=ax2, palette=method_colors, legend=False)
        
        set_median_lines_black(ax2)
        
        # Add sample sizes and statistics
        for i, method in enumerate(df_success['method'].unique()):
            method_data = df_success[df_success['method'] == method]['time_ms']
            n_samples = len(method_data)
            if n_samples > 0:
                median_val = method_data.median()
                mean_val = method_data.mean()
                
                ax2.text(i, ax2.get_ylim()[1] * 0.85, 
                        f'n={n_samples}\nμ={mean_val:.2f}ms\nM={median_val:.2f}ms', 
                        ha='center', va='top', fontsize=10,
                        bbox=dict(boxstyle="round,pad=0.3", facecolor="lightgreen", alpha=0.7))
    else:
        ax2.text(0.5, 0.5, 'No successful\ndata available', 
                ha='center', va='center', transform=ax2.transAxes, fontsize=12)
    
    ax2.set_title('Successful Attempts Only', fontsize=14, fontweight='bold')
    ax2.set_ylabel('Execution Time (ms)')
    ax2.set_xlabel('Method')
    
    # 3. Collision-Free Attempts Only
    ax3 = axes[2]
    df_collision_free = df[df['collision_free'] == 1].copy()
    
    if len(df_collision_free) > 0:
        sns.boxplot(data=df_collision_free, x='method', y='time_ms', hue='method', ax=ax3, palette=method_colors, legend=False)
        
        set_median_lines_black(ax3)
        
        # Add sample sizes and statistics
        for i, method in enumerate(df_collision_free['method'].unique()):
            method_data = df_collision_free[df_collision_free['method'] == method]['time_ms']
            n_samples = len(method_data)
            if n_samples > 0:
                median_val = method_data.median()
                mean_val = method_data.mean()
                
                ax3.text(i, ax3.get_ylim()[1] * 0.85, 
                        f'n={n_samples}\nμ={mean_val:.2f}ms\nM={median_val:.2f}ms', 
                        ha='center', va='top', fontsize=10,
                        bbox=dict(boxstyle="round,pad=0.3", facecolor="lightblue", alpha=0.7))
    else:
        ax3.text(0.5, 0.5, 'No collision-free\ndata available', 
                ha='center', va='center', transform=ax3.transAxes, fontsize=12)
    
    ax3.set_title('Collision-Free Attempts Only', fontsize=14, fontweight='bold')
    ax3.set_ylabel('Execution Time (ms)')
    ax3.set_xlabel('Method')
    
    plt.tight_layout()
    
    # Save plots
    os.makedirs('plots', exist_ok=True)
    plt.savefig('plots/execution_time_analysis.png', dpi=300, bbox_inches='tight')
    plt.savefig('plots/execution_time_analysis.pdf', bbox_inches='tight')
    plt.close()
    
    # Print summary statistics
    print(f"\n📈 EXECUTION TIME SUMMARY")
    print("-" * 40)
    
    categories = [
        ("All Attempts", df),
        ("Successful Only", df[df['success'] == 1]),
        ("Collision-Free Only", df[df['collision_free'] == 1])
    ]
    
    for cat_name, cat_data in categories:
        print(f"\n{cat_name}:")
        for method in ['Newton-Raphson', 'SA-Optimized']:
            method_data = cat_data[cat_data['method'] == method]['time_ms']
            if len(method_data) > 0:
                print(f"  {method}: n={len(method_data)}, "
                      f"μ={method_data.mean():.3f}ms, "
                      f"σ={method_data.std():.3f}ms, "
                      f"median={method_data.median():.3f}ms")
            else:
                print(f"  {method}: No data available")
    
    print(f"\n✅ Generated: plots/execution_time_analysis.png")
    print(f"✅ Generated: plots/execution_time_analysis.pdf")

if __name__ == "__main__":
    plot_execution_time_analysis()
