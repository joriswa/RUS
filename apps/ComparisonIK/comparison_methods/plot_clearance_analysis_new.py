#!/usr/bin/env python3
"""
Clearance Analysis for IK Methods
Analyzes safety clearance for robot collision avoidance across different methods.
This measures how safely methods position the robot away from obstacles.

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

# Suppress warnings for clean output
warnings.filterwarnings('ignore', category=FutureWarning)
warnings.filterwarnings('ignore', category=UserWarning)

def set_median_lines_black(ax):
    """Helper function to set median lines to black color in boxplots."""
    for line in ax.lines:
        if line.get_linestyle() == '-':  # median lines
            line.set_color('black')

def plot_clearance_analysis():
    """Generate comprehensive clearance analysis plots."""
    
    print("🛡️ CLEARANCE ANALYSIS FOR IK METHODS")
    print("=" * 50)
    
    # Load and validate data
    try:
        df = pd.read_csv('three_method_comparison_results.csv')
        print(f"📊 Total records loaded: {len(df)}")
        print(f"🎯 Unique poses: {df['pose_id'].nunique()}")
        print(f"📈 Methods analyzed: {', '.join(df['method'].unique())}")
        
    except FileNotFoundError:
        print("❌ Error: Could not find 'three_method_comparison_results.csv'")
        print("   Please run the C++ comparison script first to generate the data.")
        return
    except Exception as e:
        print(f"❌ Error loading data: {e}")
        return
    
    # Filter for successful attempts only (clearance only meaningful for successful IK)
    df_successful = df[df['success'] == 1].copy()
    
    if len(df_successful) == 0:
        print("❌ No successful IK solutions found for clearance analysis")
        return
    
    print(f"✅ Analyzing {len(df_successful)} successful solutions")
    
    # Set consistent styling
    plt.style.use('default')
    
    # Create two separate plots
    fig, axes = plt.subplots(1, 2, figsize=(14, 6))
    fig.suptitle('Robot Clearance Analysis Comparison', fontsize=16, fontweight='bold')
    
    # Define consistent colors (default matplotlib colors)
    colors = ['#1f77b4', '#ff7f0e', '#2ca02c']  # Blue, Orange, Green (default matplotlib)
    method_colors = {'Newton-Raphson': colors[0], 'SA-Optimized': colors[1], 'Grid-Search': colors[2]}
    
    # 1. Overall Clearance Distribution
    ax1 = axes[0]
    
    # Prepare data for boxplot
    clearance_data = []
    clearance_labels = []
    
    for method in df_successful['method'].unique():
        method_data = df_successful[df_successful['method'] == method]['min_clearance'].values
        clearance_data.append(method_data)
        clearance_labels.append(method)
    
    bp1 = ax1.boxplot(clearance_data, labels=clearance_labels, patch_artist=True)
    
    # Set colors for boxes
    for patch, method in zip(bp1['boxes'], clearance_labels):
        patch.set_facecolor(method_colors.get(method, '#999999'))
        patch.set_alpha(0.8)
    
    # Set median lines to black
    set_median_lines_black(ax1)
    
    ax1.set_title('Minimum Clearance Distribution', fontsize=14, fontweight='bold')
    ax1.set_ylabel('Clearance Distance (m)', fontsize=12)
    ax1.set_xlabel('IK Method', fontsize=12)
    ax1.grid(axis='y', alpha=0.3)
    
    # Add horizontal reference lines for safety thresholds
    ax1.axhline(y=0.05, color='red', linestyle='--', alpha=0.7, label='Critical (5cm)')
    ax1.axhline(y=0.10, color='orange', linestyle='--', alpha=0.7, label='Caution (10cm)')
    ax1.axhline(y=0.15, color='green', linestyle='--', alpha=0.7, label='Safe (15cm)')
    ax1.legend(loc='upper right')
    
    # 2. Safety Category Analysis
    ax2 = axes[1]
    
    # Define safety categories
    def categorize_clearance(clearance):
        if clearance < 0.05:
            return 'Critical (<5cm)'
        elif clearance < 0.10:
            return 'Caution (5-10cm)'
        elif clearance < 0.15:
            return 'Safe (10-15cm)'
        else:
            return 'Very Safe (≥15cm)'
    
    df_successful['safety_category'] = df_successful['min_clearance'].apply(categorize_clearance)
    
    # Count safety categories by method
    safety_counts = df_successful.groupby(['method', 'safety_category']).size().unstack(fill_value=0)
    safety_percentages = safety_counts.div(safety_counts.sum(axis=1), axis=0) * 100
    
    # Plot stacked bar chart
    category_colors = {'Critical (<5cm)': '#d62728', 'Caution (5-10cm)': '#ff7f0e', 
                      'Safe (10-15cm)': '#2ca02c', 'Very Safe (≥15cm)': '#1f77b4'}
    
    safety_percentages.plot(kind='bar', stacked=True, ax=ax2, 
                           color=[category_colors.get(cat, '#999999') for cat in safety_percentages.columns],
                           alpha=0.8, edgecolor='black')
    
    ax2.set_title('Safety Category Distribution', fontsize=14, fontweight='bold')
    ax2.set_ylabel('Percentage (%)', fontsize=12)
    ax2.set_xlabel('IK Method', fontsize=12)
    ax2.set_xticklabels(safety_percentages.index, rotation=45, ha='right')
    ax2.legend(title='Safety Categories', bbox_to_anchor=(1.05, 1), loc='upper left')
    ax2.grid(axis='y', alpha=0.3)
    
    plt.tight_layout()
    
    # Save plots
    os.makedirs('plots', exist_ok=True)
    plt.savefig('plots/clearance_analysis.png', dpi=300, bbox_inches='tight')
    plt.savefig('plots/clearance_analysis.pdf', bbox_inches='tight')
    plt.close()
    
    # Print summary statistics
    print(f"\n📊 CLEARANCE ANALYSIS RESULTS")
    print("=" * 50)
    
    print(f"\nClearance Statistics by Method:")
    for method in df_successful['method'].unique():
        method_data = df_successful[df_successful['method'] == method]['min_clearance']
        print(f"  {method}:")
        print(f"    Mean: {method_data.mean():.3f}m ± {method_data.std():.3f}m")
        print(f"    Median: {method_data.median():.3f}m")
        print(f"    Range: {method_data.min():.3f}m - {method_data.max():.3f}m")
    
    print(f"\nSafety Category Distribution:")
    for method in safety_percentages.index:
        print(f"  {method}:")
        for category in safety_percentages.columns:
            if category in safety_percentages.loc[method]:
                percentage = safety_percentages.loc[method, category]
                count = safety_counts.loc[method, category] if category in safety_counts.columns else 0
                print(f"    {category}: {percentage:.1f}% ({count} solutions)")
    
    print(f"\n✅ Analysis complete!")
    print(f"📊 Generated: plots/clearance_analysis.png")
    print(f"📄 Generated: plots/clearance_analysis.pdf")

if __name__ == "__main__":
    plot_clearance_analysis()
