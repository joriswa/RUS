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

# Method name mapping for cleaner display
METHOD_DISPLAY_NAMES = {
    'Newton-Raphson': 'Newton-Raphson',
    'SA-Optimized': 'SA-Optimized',
    'Grid-Search': 'Grid-Search',
}

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

def create_success_rate_plot(df, output_dir='plots'):
    """Create success rate comparison plot"""
    Path(output_dir).mkdir(exist_ok=True)
    
    # Calculate success rates by method
    success_rates = df.groupby('method')['success'].mean() * 100
    
    # Map to display names
    display_names = [METHOD_DISPLAY_NAMES.get(method, method) for method in success_rates.index]
    colors = [COLORS.get(method, '#999999') for method in success_rates.index]
    
    fig, ax = plt.subplots(figsize=(10, 6))
    bars = ax.bar(display_names, success_rates.values, 
                  color=colors, alpha=0.8, edgecolor='white', linewidth=2)
    
    # Styling
    ax.set_ylabel('Success Rate (%)', fontsize=12, color=COLORS['text'])
    ax.set_title('IK Method Success Rate Comparison', fontsize=14, color=COLORS['text'], pad=20)
    ax.set_ylim(0, 105)
    ax.grid(True, alpha=0.3)
    ax.set_facecolor(COLORS['background'])
    
    # Add value labels on bars
    for bar in bars:
        height = bar.get_height()
        ax.text(bar.get_x() + bar.get_width()/2., height + 1,
                f'{height:.1f}%', ha='center', va='bottom', 
                fontsize=11, color=COLORS['text'])
    
    plt.tight_layout()
    plt.savefig(f'{output_dir}/success_rates_comparison.png', dpi=300, bbox_inches='tight')
    plt.close()
    print(f"Saved success rate plot to {output_dir}/success_rates_comparison.png")

def create_execution_time_boxplot(df, output_dir='plots'):
    """Create execution time boxplot"""
    Path(output_dir).mkdir(exist_ok=True)
    
    # Filter successful solutions only
    df_success = df[df['success'] == True].copy()
    
    fig, ax = plt.subplots(figsize=(10, 6))
    
    # Create boxplot with custom colors
    methods = df_success['method'].unique()
    box_data = [df_success[df_success['method'] == method]['time_ms'].values for method in methods]
    display_names = [METHOD_DISPLAY_NAMES.get(method, method) for method in methods]
    
    bp = ax.boxplot(box_data, tick_labels=display_names, patch_artist=True)
    
    # Apply grey colors
    for patch, method in zip(bp['boxes'], methods):
        patch.set_facecolor(COLORS.get(method, '#999999'))
        patch.set_alpha(0.8)
    
    # Styling
    ax.set_ylabel('Execution Time (ms)', fontsize=12, color=COLORS['text'])
    ax.set_title('IK Method Execution Time Comparison (Successful Solutions)', 
                 fontsize=14, color=COLORS['text'], pad=20)
    ax.grid(True, alpha=0.3)
    ax.set_facecolor(COLORS['background'])
    
    plt.xticks(rotation=45)
    plt.tight_layout()
    plt.savefig(f'{output_dir}/execution_time_boxplot.png', dpi=300, bbox_inches='tight')
    plt.close()
    print(f"Saved execution time plot to {output_dir}/execution_time_boxplot.png")

def create_clearance_comparison(df, output_dir='plots'):
    """Create clearance comparison plot"""
    Path(output_dir).mkdir(exist_ok=True)
    
    # Filter collision-free solutions
    df_safe = df[(df['success'] == True) & (df['collision_free'] == True)].copy()
    
    if df_safe.empty:
        print("No collision-free solutions found for clearance analysis")
        return
    
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(15, 6))
    
    # Min clearance boxplot
    methods = df_safe['method'].unique()
    min_clearance_data = [df_safe[df_safe['method'] == method]['min_clearance'].values for method in methods]
    display_names = [METHOD_DISPLAY_NAMES.get(method, method) for method in methods]
    
    bp1 = ax1.boxplot(min_clearance_data, tick_labels=display_names, patch_artist=True)
    for patch, method in zip(bp1['boxes'], methods):
        patch.set_facecolor(COLORS.get(method, '#999999'))
        patch.set_alpha(0.8)
    
    ax1.set_ylabel('Minimum Clearance (m)', fontsize=12, color=COLORS['text'])
    ax1.set_title('Minimum Clearance Comparison', fontsize=12, color=COLORS['text'])
    ax1.grid(True, alpha=0.3)
    ax1.set_facecolor(COLORS['background'])
    
    # Average clearance boxplot
    avg_clearance_data = [df_safe[df_safe['method'] == method]['avg_clearance'].values for method in methods]
    
    bp2 = ax2.boxplot(avg_clearance_data, tick_labels=display_names, patch_artist=True)
    for patch, method in zip(bp2['boxes'], methods):
        patch.set_facecolor(COLORS.get(method, '#999999'))
        patch.set_alpha(0.8)
    
    ax2.set_ylabel('Average Clearance (m)', fontsize=12, color=COLORS['text'])
    ax2.set_title('Average Clearance Comparison', fontsize=12, color=COLORS['text'])
    ax2.grid(True, alpha=0.3)
    ax2.set_facecolor(COLORS['background'])
    
    plt.tight_layout()
    plt.savefig(f'{output_dir}/clearance_comparison.png', dpi=300, bbox_inches='tight')
    plt.close()
    print(f"Saved clearance comparison to {output_dir}/clearance_comparison.png")

def create_summary_dashboard(df, output_dir='plots'):
    """Create comprehensive summary dashboard"""
    Path(output_dir).mkdir(exist_ok=True)
    
    fig = plt.figure(figsize=(16, 12))
    gs = fig.add_gridspec(3, 2, hspace=0.3, wspace=0.3)
    
    # 1. Success rates
    ax1 = fig.add_subplot(gs[0, 0])
    success_rates = df.groupby('method')['success'].mean() * 100
    display_names_1 = [METHOD_DISPLAY_NAMES.get(method, method) for method in success_rates.index]
    colors_1 = [COLORS.get(method, '#999999') for method in success_rates.index]
    bars = ax1.bar(display_names_1, success_rates.values, color=colors_1, alpha=0.8)
    ax1.set_ylabel('Success Rate (%)')
    ax1.set_title('Success Rate by Method')
    ax1.set_facecolor(COLORS['background'])
    
    # 2. Collision-free rates
    ax2 = fig.add_subplot(gs[0, 1])
    collision_free_rates = df[df['success'] == True].groupby('method')['collision_free'].mean() * 100
    display_names_2 = [METHOD_DISPLAY_NAMES.get(method, method) for method in collision_free_rates.index]
    colors_2 = [COLORS.get(method, '#999999') for method in collision_free_rates.index]
    bars = ax2.bar(display_names_2, collision_free_rates.values, color=colors_2, alpha=0.8)
    ax2.set_ylabel('Collision-Free Rate (%)')
    ax2.set_title('Collision-Free Rate (Successful Solutions)')
    ax2.set_facecolor(COLORS['background'])
    
    # 3. Execution time
    ax3 = fig.add_subplot(gs[1, :])
    df_success = df[df['success'] == True]
    methods = df_success['method'].unique()
    time_data = [df_success[df_success['method'] == method]['time_ms'].values for method in methods]
    display_names_3 = [METHOD_DISPLAY_NAMES.get(method, method) for method in methods]
    bp = ax3.boxplot(time_data, tick_labels=display_names_3, patch_artist=True)
    for patch, method in zip(bp['boxes'], methods):
        patch.set_facecolor(COLORS.get(method, '#999999'))
        patch.set_alpha(0.8)
    ax3.set_ylabel('Execution Time (ms)')
    ax3.set_title('Execution Time Distribution')
    ax3.set_facecolor(COLORS['background'])
    
    # 4. Method statistics table
    ax4 = fig.add_subplot(gs[2, :])
    ax4.axis('off')
    
    # Create summary statistics
    stats = []
    for method in df['method'].unique():
        method_data = df[df['method'] == method]
        success_data = method_data[method_data['success'] == True]
        display_name = METHOD_DISPLAY_NAMES.get(method, method)
        
        stats.append([
            display_name,
            f"{method_data['success'].mean() * 100:.1f}%",
            f"{success_data['time_ms'].mean():.1f}ms" if not success_data.empty else "N/A",
            f"{success_data['collision_free'].mean() * 100:.1f}%" if not success_data.empty else "N/A"
        ])
    
    table = ax4.table(cellText=stats,
                     colLabels=['Method', 'Success Rate', 'Avg Time', 'Collision-Free Rate'],
                     cellLoc='center',
                     loc='center')
    table.auto_set_font_size(False)
    table.set_fontsize(11)
    table.scale(1, 2)
    
    plt.suptitle('IK Methods Comparison Dashboard', fontsize=16, color=COLORS['text'], y=0.98)
    plt.savefig(f'{output_dir}/comparison_dashboard.png', dpi=300, bbox_inches='tight')
    plt.close()
    print(f"Saved comparison dashboard to {output_dir}/comparison_dashboard.png")

def main():
    """Main plotting function"""
    print("Starting IK method comparison plotting...")
    
    # Load data
    df = load_comparison_data()
    if df is None:
        return
    
    # Create output directory
    output_dir = 'comparison_methods/plots'
    Path(output_dir).mkdir(parents=True, exist_ok=True)
    
    # Generate all plots
    create_success_rate_plot(df, output_dir)
    create_execution_time_boxplot(df, output_dir)
    create_clearance_comparison(df, output_dir)
    create_summary_dashboard(df, output_dir)
    
    print(f"\nAll plots generated in {output_dir}/")
    print("Plots use consistent grey styling with AAAAA, CCCCC color scheme as requested.")

if __name__ == "__main__":
    main()
