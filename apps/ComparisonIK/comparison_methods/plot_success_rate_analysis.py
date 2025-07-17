#!/usr/bin/env python3
"""
Success Rate Analysis for IK Methods
Analyzes solution success rates across different IK methods.
This measures how reliably methods can solve inverse kinematics problems.

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

def load_and_filter_data(filename):
    """Load CSV data and filter for analysis."""
    df = pd.read_csv(filename)
    
    print(f"📊 Total records loaded: {len(df)}")
    print(f"🔢 Unique poses: {df['pose_id'].nunique()}")
    print(f"🎯 Methods: {list(df['method'].unique())}")
    
    return df

def plot_success_rate_analysis():
    """Generate success rate analysis for different categories - Original Format."""
    
    print("📊 Success Rate Analysis for IK Methods - Original Format")
    print("=" * 65)
    
    # Load data
    try:
        df = load_and_filter_data('three_method_comparison_results.csv')
        
    except FileNotFoundError:
        print("❌ Error: Could not find 'three_method_comparison_results.csv'")
        print("   Please run the C++ comparison script first to generate the data.")
        return
    except Exception as e:
        print(f"❌ Error loading data: {e}")
        return
    
    # Set consistent styling
    plt.style.use('default')
     # Create three separate plots for comprehensive analysis
    fig, axes = plt.subplots(1, 3, figsize=(18, 6))
    fig.suptitle('IK Method Success Rate Comparison', fontsize=16, fontweight='bold')
    
    # Define consistent colors (default matplotlib colors)
    colors = ['#1f77b4', '#ff7f0e', '#2ca02c']  # Blue, Orange, Green (default matplotlib)
    method_colors = {'Newton-Raphson': colors[0], 'SA-Optimized': colors[1], 'Grid-Search': colors[2]}
    
    # Function to set median lines to black
    def set_median_lines_black(ax):
        for line in ax.lines:
            if line.get_linestyle() == '-':  # median lines
                line.set_color('black')
    
    # 1. Overall Success Rate (All Attempts)
    ax1 = axes[0]
    
    # Calculate overall success rates
    overall_stats = df.groupby('method').agg({
        'success': ['count', 'sum', 'mean']
    }).round(4)
    overall_stats.columns = ['total_attempts', 'successful_attempts', 'success_rate']
    overall_stats['success_percentage'] = overall_stats['success_rate'] * 100
    overall_stats = overall_stats.reset_index()
    
    bars1 = ax1.bar(overall_stats['method'], overall_stats['success_percentage'], 
                    color=[method_colors.get(method, '#999999') for method in overall_stats['method']],
                    alpha=0.8, edgecolor='black')
    
    ax1.set_title('Overall Success Rate', fontsize=14, fontweight='bold')
    ax1.set_ylabel('Success Rate (%)', fontsize=12)
    ax1.set_xlabel('IK Method', fontsize=12)
    ax1.set_ylim(0, 105)
    ax1.grid(axis='y', alpha=0.3)
    
    # Add value labels on bars
    for i, bar in enumerate(bars1):
        height = bar.get_height()
        total = overall_stats.iloc[i]['total_attempts']
        successful = overall_stats.iloc[i]['successful_attempts']
        ax1.text(bar.get_x() + bar.get_width()/2., height + 1,
                f'{height:.1f}%\n({successful}/{total})',
                ha='center', va='bottom', fontsize=10, fontweight='bold')
    
    # 2. Collision-Free Success Rate
    ax2 = axes[1]
    
    # Calculate collision-free success rates
    cf_stats = df.groupby('method').agg({
        'collision_free': ['count', 'sum', 'mean']
    }).round(4)
    cf_stats.columns = ['total_attempts', 'collision_free_attempts', 'cf_rate']
    cf_stats['cf_percentage'] = cf_stats['cf_rate'] * 100
    cf_stats = cf_stats.reset_index()
    
    bars2 = ax2.bar(cf_stats['method'], cf_stats['cf_percentage'], 
                    color=[method_colors.get(method, '#999999') for method in cf_stats['method']],
                    alpha=0.8, edgecolor='black')
    
    ax2.set_title('Collision-Free Rate', fontsize=14, fontweight='bold')
    ax2.set_ylabel('Collision-Free Rate (%)', fontsize=12)
    ax2.set_xlabel('IK Method', fontsize=12)
    ax2.set_ylim(0, 105)
    ax2.grid(axis='y', alpha=0.3)
    
    # Add value labels on bars
    for i, bar in enumerate(bars2):
        height = bar.get_height()
        total = cf_stats.iloc[i]['total_attempts']
        cf_attempts = cf_stats.iloc[i]['collision_free_attempts']
        ax2.text(bar.get_x() + bar.get_width()/2., height + 1,
                f'{height:.1f}%\n({cf_attempts}/{total})',
                ha='center', va='bottom', fontsize=10, fontweight='bold')
    
    # 3. Success Rate Distribution (Box Plot)
    ax3 = axes[2]
    
    # Create success percentage data for box plots
    box_data = []
    box_labels = []
    for method in df['method'].unique():
        method_data = df[df['method'] == method]
        # Calculate success rate for each run
        success_rates = method_data.groupby(['run_id'])['success'].mean() * 100
        box_data.append(success_rates.values)
        box_labels.append(method)
    
    bp = ax3.boxplot(box_data, labels=box_labels, patch_artist=True)
    
    # Set colors for boxes
    for patch, method in zip(bp['boxes'], box_labels):
        patch.set_facecolor(method_colors.get(method, '#999999'))
        patch.set_alpha(0.8)
    
    # Set median lines to black
    set_median_lines_black(ax3)
    
    ax3.set_title('Success Rate Distribution', fontsize=14, fontweight='bold')
    ax3.set_ylabel('Success Rate (%)', fontsize=12)
    ax3.set_xlabel('IK Method', fontsize=12)
    ax3.set_ylim(0, 105)
    ax3.grid(axis='y', alpha=0.3)
    
    plt.tight_layout()
    
    # Save plots
    os.makedirs('plots', exist_ok=True)
    plt.savefig('plots/success_rate_analysis.png', dpi=300, bbox_inches='tight')
    plt.savefig('plots/success_rate_analysis.pdf', bbox_inches='tight')
    plt.close()
    
    # Print summary statistics
    print(f"\n📈 SUCCESS RATE ANALYSIS RESULTS")
    print("=" * 50)
    
    print(f"\nOverall Success Rates:")
    for _, row in overall_stats.iterrows():
        print(f"  {row['method']}: {row['success_percentage']:.1f}% "
              f"({row['successful_attempts']:.0f}/{row['total_attempts']:.0f})")
    
    print(f"\nOverall Collision-Free Rates:")
    for _, row in cf_stats.iterrows():
        print(f"  {row['method']}: {row['cf_percentage']:.1f}% "
              f"({row['collision_free_attempts']:.0f}/{row['total_attempts']:.0f})")
    
    # Statistical summary for distribution
    print(f"\nSuccess Rate Distribution Statistics:")
    for i, method in enumerate(box_labels):
        data = box_data[i]
        print(f"  {method}:")
        print(f"    Mean: {np.mean(data):.1f}% ± {np.std(data):.1f}%")
        print(f"    Median: {np.median(data):.1f}%")
        print(f"    Range: {np.min(data):.1f}% - {np.max(data):.1f}%")
    
    print(f"\n✅ Analysis complete!")
    print(f"📊 Generated: plots/success_rate_analysis.png")
    print(f"📄 Generated: plots/success_rate_analysis.pdf")

if __name__ == "__main__":
    plot_success_rate_analysis()
