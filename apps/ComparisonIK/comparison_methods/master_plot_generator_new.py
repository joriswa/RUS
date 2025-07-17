#!/usr/bin/env python3
"""
Master Plot Generator for IK Methods
Comprehensive plotting script for IK method comparison analysis.
Generates all comparison plots with consistent styling.

Created: December 2024
Author: PathPlanner Analysis Suite
"""

import pandas as pd
import matplotlib.pyplot as plt
import seaborn as sns
import numpy as np
import warnings
from pathlib import Path

# Suppress warnings for clean output
warnings.filterwarnings('ignore', category=FutureWarning)
warnings.filterwarnings('ignore', category=UserWarning)

def set_median_lines_black(ax):
    """Helper function to set median lines to black color in boxplots."""
    for line in ax.lines:
        if line.get_linestyle() == '-':  # median lines
            line.set_color('black')

# Define consistent colors (default matplotlib colors)
colors = ['#1f77b4', '#ff7f0e', '#2ca02c']  # Blue, Orange, Green (default matplotlib)
method_colors = {'Newton-Raphson': colors[0], 'SA-Optimized': colors[1], 'Grid-Search': colors[2]}

# Set consistent style
plt.style.use('default')

def load_comparison_data(csv_file='three_method_comparison_results.csv'):
    """Load and validate comparison data."""
    try:
        df = pd.read_csv(csv_file)
        print(f"📊 Total records loaded: {len(df)}")
        print(f"🎯 Unique poses: {df['pose_id'].nunique()}")
        print(f"📈 Methods analyzed: {', '.join(df['method'].unique())}")
        return df
    except FileNotFoundError:
        print("❌ Error: Could not find 'three_method_comparison_results.csv'")
        print("   Please run the C++ comparison script first to generate the data.")
        return None
    except Exception as e:
        print(f"❌ Error loading data: {e}")
        return None

def create_comprehensive_analysis():
    """Generate comprehensive analysis plots for all IK methods."""
    
    print("📊 MASTER PLOT GENERATOR FOR IK METHODS")
    print("=" * 50)
    print("Generating comprehensive comparison plots...")
    
    # Load and validate data
    df = load_comparison_data()
    if df is None:
        return
    
    # Create output directory
    Path('plots').mkdir(exist_ok=True)
    
    # 1. Success Rate Comparison
    print("\n📈 Creating success rate analysis...")
    create_success_rate_plot(df)
    
    # 2. Execution Time Analysis
    print("⏱️ Creating execution time analysis...")
    create_execution_time_plot(df)
    
    # 3. Clearance Analysis
    print("🛡️ Creating clearance analysis...")
    create_clearance_plot(df)
    
    print(f"\n✅ All plots generated successfully!")
    print(f"📁 Check the 'plots/' directory for results")

def create_success_rate_plot(df):
    """Create success rate comparison plot."""
    
    fig, axes = plt.subplots(1, 2, figsize=(14, 6))
    fig.suptitle('IK Method Success Rate Analysis', fontsize=16, fontweight='bold')
    
    # Overall success rates
    success_rates = df.groupby('method')['success'].mean() * 100
    
    bars = axes[0].bar(success_rates.index, success_rates.values,
                       color=[method_colors.get(method, '#999999') for method in success_rates.index],
                       alpha=0.8, edgecolor='black')
    
    axes[0].set_title('Overall Success Rate', fontsize=14, fontweight='bold')
    axes[0].set_ylabel('Success Rate (%)', fontsize=12)
    axes[0].set_xlabel('IK Method', fontsize=12)
    axes[0].grid(axis='y', alpha=0.3)
    
    # Add value labels
    for bar, value in zip(bars, success_rates.values):
        axes[0].text(bar.get_x() + bar.get_width()/2., value + 1,
                    f'{value:.1f}%', ha='center', va='bottom', fontweight='bold')
    
    # Collision-free rates
    cf_rates = df.groupby('method')['collision_free'].mean() * 100
    
    bars2 = axes[1].bar(cf_rates.index, cf_rates.values,
                        color=[method_colors.get(method, '#999999') for method in cf_rates.index],
                        alpha=0.8, edgecolor='black')
    
    axes[1].set_title('Collision-Free Rate', fontsize=14, fontweight='bold')
    axes[1].set_ylabel('Collision-Free Rate (%)', fontsize=12)
    axes[1].set_xlabel('IK Method', fontsize=12)
    axes[1].grid(axis='y', alpha=0.3)
    
    # Add value labels
    for bar, value in zip(bars2, cf_rates.values):
        axes[1].text(bar.get_x() + bar.get_width()/2., value + 1,
                    f'{value:.1f}%', ha='center', va='bottom', fontweight='bold')
    
    plt.tight_layout()
    plt.savefig('plots/master_success_rates.png', dpi=300, bbox_inches='tight')
    plt.savefig('plots/master_success_rates.pdf', bbox_inches='tight')
    plt.close()

def create_execution_time_plot(df):
    """Create execution time comparison plot."""
    
    # Filter successful attempts only
    successful_df = df[df['success'] == 1]
    
    if len(successful_df) == 0:
        print("❌ No successful solutions for execution time analysis")
        return
    
    fig, ax = plt.subplots(1, 1, figsize=(10, 6))
    
    # Prepare data for boxplot
    exec_data = []
    labels = []
    for method in successful_df['method'].unique():
        method_data = successful_df[successful_df['method'] == method]['execution_time_ms']
        exec_data.append(method_data)
        labels.append(method)
    
    bp = ax.boxplot(exec_data, labels=labels, patch_artist=True)
    
    # Set colors
    for patch, method in zip(bp['boxes'], labels):
        patch.set_facecolor(method_colors.get(method, '#999999'))
        patch.set_alpha(0.8)
    
    # Set median lines to black
    set_median_lines_black(ax)
    
    ax.set_title('Execution Time Comparison', fontsize=16, fontweight='bold')
    ax.set_ylabel('Execution Time (ms)', fontsize=12)
    ax.set_xlabel('IK Method', fontsize=12)
    ax.grid(axis='y', alpha=0.3)
    
    plt.tight_layout()
    plt.savefig('plots/master_execution_times.png', dpi=300, bbox_inches='tight')
    plt.savefig('plots/master_execution_times.pdf', bbox_inches='tight')
    plt.close()

def create_clearance_plot(df):
    """Create clearance comparison plot."""
    
    # Filter successful and collision-free attempts
    clearance_df = df[(df['success'] == 1) & (df['collision_free'] == 1)]
    
    if len(clearance_df) == 0:
        print("❌ No collision-free solutions for clearance analysis")
        return
    
    fig, ax = plt.subplots(1, 1, figsize=(10, 6))
    
    # Prepare data for boxplot
    clearance_data = []
    labels = []
    for method in clearance_df['method'].unique():
        method_data = clearance_df[clearance_df['method'] == method]['min_clearance']
        clearance_data.append(method_data)
        labels.append(method)
    
    bp = ax.boxplot(clearance_data, labels=labels, patch_artist=True)
    
    # Set colors
    for patch, method in zip(bp['boxes'], labels):
        patch.set_facecolor(method_colors.get(method, '#999999'))
        patch.set_alpha(0.8)
    
    # Set median lines to black
    set_median_lines_black(ax)
    
    ax.set_title('Minimum Clearance Comparison', fontsize=16, fontweight='bold')
    ax.set_ylabel('Minimum Clearance (m)', fontsize=12)
    ax.set_xlabel('IK Method', fontsize=12)
    ax.grid(axis='y', alpha=0.3)
    
    # Add safety threshold lines
    ax.axhline(y=0.05, color='red', linestyle='--', alpha=0.7, label='Critical (5cm)')
    ax.axhline(y=0.10, color='orange', linestyle='--', alpha=0.7, label='Caution (10cm)')
    ax.axhline(y=0.15, color='green', linestyle='--', alpha=0.7, label='Safe (15cm)')
    ax.legend(loc='upper right')
    
    plt.tight_layout()
    plt.savefig('plots/master_clearance.png', dpi=300, bbox_inches='tight')
    plt.savefig('plots/master_clearance.pdf', bbox_inches='tight')
    plt.close()

if __name__ == "__main__":
    create_comprehensive_analysis()
