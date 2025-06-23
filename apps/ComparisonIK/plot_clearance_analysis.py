#!/usr/bin/env python3
"""
Clearance Analysis: Boxplots for SA-Optimized vs Newton-Raphson
Generates separate plots for:
1. Minimum clearance (successful attempts)
2. Average clearance (successful attempts)
"""

import pandas as pd
import matplotlib.pyplot as plt
import seaborn as sns
import numpy as np
import os
from datetime import datetime

def plot_clearance_analysis():
    """Generate clearance analysis plots."""
    
    # Function to set median lines to black
    def set_median_lines_black(ax):
        for line in ax.lines:
            if line.get_linestyle() == '-':  # median lines
                line.set_color('black')
    
    print("🛡️ CLEARANCE ANALYSIS")
    print("="*50)
    
    # Load data
    df = pd.read_csv('two_method_comparison_results.csv')
    print(f"📊 Loaded {len(df)} data points")
    
    # Filter for successful attempts only (clearance only meaningful for successful IK)
    df_success = df[df['success'] == 1].copy()
    print(f"📊 Successful attempts: {len(df_success)}")
    
    # Clean up infinite values for plotting
    df_success['min_clearance_plot'] = df_success['min_clearance'].replace([np.inf, -np.inf], np.nan)
    df_success['avg_clearance_plot'] = df_success['avg_clearance'].replace([np.inf, -np.inf], np.nan)
    
    # Remove rows with NaN clearance values
    df_min_clear = df_success.dropna(subset=['min_clearance_plot'])
    df_avg_clear = df_success.dropna(subset=['avg_clearance_plot'])
    
    print(f"📊 With valid min clearance: {len(df_min_clear)}")
    print(f"📊 With valid avg clearance: {len(df_avg_clear)}")
    
    # Set consistent styling
    plt.style.use('default')
    
    # Create single plot for average clearance only
    fig, ax = plt.subplots(1, 1, figsize=(8, 6))
    fig.suptitle('IK Method Average Clearance Comparison (Successful Attempts)', fontsize=16, fontweight='bold')
    
    # Define consistent colors (default matplotlib colors)
    colors = ['#1f77b4', '#ff7f0e']  # Blue, Orange (default matplotlib)
    method_colors = {'Newton-Raphson': colors[0], 'SA-Optimized': colors[1]}
    
    # Average Clearance Plot
    if len(df_avg_clear) > 0:
        sns.boxplot(data=df_avg_clear, x='method', y='avg_clearance_plot', hue='method', ax=ax, palette=method_colors, legend=False)
        set_median_lines_black(ax)
        ax.set_title('Average Clearance', fontsize=14, fontweight='bold')
        ax.set_ylabel('Average Clearance (m)')
        ax.set_xlabel('Method')
        
        # Add safety thresholds
        safety_thresholds = [0.05, 0.10, 0.20]
        threshold_colors = ['red', 'orange', 'green']
        threshold_labels = ['Critical (5cm)', 'Caution (10cm)', 'Safe (20cm)']
        
        for threshold, color, label in zip(safety_thresholds, threshold_colors, threshold_labels):
            ax.axhline(y=threshold, color=color, linestyle='--', alpha=0.7)
        
        # Add sample sizes and statistics
        for i, method in enumerate(df_avg_clear['method'].unique()):
            method_data = df_avg_clear[df_avg_clear['method'] == method]['avg_clearance_plot']
            n_samples = len(method_data)
            if n_samples > 0:
                median_val = method_data.median()
                mean_val = method_data.mean()
                
                ax.text(i, ax.get_ylim()[1] * 0.85, 
                        f'n={n_samples}\nμ={mean_val:.3f}m\nM={median_val:.3f}m', 
                        ha='center', va='top', fontsize=10,
                        bbox=dict(boxstyle="round,pad=0.3", facecolor="lightcyan", alpha=0.7))
        
        # Add legend for thresholds
        from matplotlib.lines import Line2D
        threshold_lines = [Line2D([0], [0], color=color, linestyle='--', alpha=0.7) 
                          for color in threshold_colors]
        ax.legend(threshold_lines, threshold_labels, loc='upper right', fontsize=9)
        
        # Set median lines to black
        set_median_lines_black(ax)
        
    else:
        ax.text(0.5, 0.5, 'No valid average\nclearance data available', 
                ha='center', va='center', transform=ax.transAxes, fontsize=12)
        ax.set_title('Average Clearance', fontsize=14, fontweight='bold')
    
    plt.tight_layout()
    
    # Save plots
    os.makedirs('plots', exist_ok=True)
    plt.savefig('plots/clearance_analysis.png', dpi=300, bbox_inches='tight')
    plt.savefig('plots/clearance_analysis.pdf', bbox_inches='tight')
    plt.close()
    
    # Print summary statistics
    print(f"\n📈 CLEARANCE SUMMARY")
    print("-" * 40)
    
    if len(df_min_clear) > 0:
        print(f"\nMinimum Clearance:")
        for method in df_min_clear['method'].unique():
            method_data = df_min_clear[df_min_clear['method'] == method]['min_clearance_plot']
            if len(method_data) > 0:
                # Safety compliance
                critical_safe = (method_data >= 0.05).mean() * 100
                caution_safe = (method_data >= 0.10).mean() * 100
                safe = (method_data >= 0.20).mean() * 100
                
                print(f"  {method}: n={len(method_data)}")
                print(f"    μ={method_data.mean():.4f}m, σ={method_data.std():.4f}m")
                print(f"    median={method_data.median():.4f}m")
                print(f"    Safety: {critical_safe:.1f}% > 5cm, {caution_safe:.1f}% > 10cm, {safe:.1f}% > 20cm")
    
    if len(df_avg_clear) > 0:
        print(f"\nAverage Clearance:")
        for method in df_avg_clear['method'].unique():
            method_data = df_avg_clear[df_avg_clear['method'] == method]['avg_clearance_plot']
            if len(method_data) > 0:
                # Safety compliance
                critical_safe = (method_data >= 0.05).mean() * 100
                caution_safe = (method_data >= 0.10).mean() * 100
                safe = (method_data >= 0.20).mean() * 100
                
                print(f"  {method}: n={len(method_data)}")
                print(f"    μ={method_data.mean():.4f}m, σ={method_data.std():.4f}m")
                print(f"    median={method_data.median():.4f}m")
                print(f"    Safety: {critical_safe:.1f}% > 5cm, {caution_safe:.1f}% > 10cm, {safe:.1f}% > 20cm")
    
    print(f"\n✅ Generated: plots/clearance_analysis.png")
    print(f"✅ Generated: plots/clearance_analysis.pdf")

if __name__ == "__main__":
    plot_clearance_analysis()
