#!/usr/bin/env python3
"""
Comprehensive IK Method Comparison Analysis

This script generates detailed boxplot analyses for:
1. Execution Time Analysis (successful vs all vs collision-free)
2. Success Rates (all vs collision-free)
3. Clearance Analysis (minimum and average clearance)
4. Solution Configuration Variance Analysis

Uses consistent styling with default matplotlib colors.
"""

import pandas as pd
import matplotlib
matplotlib.use('Agg')  # Use non-interactive backend
import matplotlib.pyplot as plt
import numpy as np
import os
from datetime import datetime
import seaborn as sns

def load_and_prepare_data():
    """Load comparison data and prepare it for analysis."""
    df = pd.read_csv('two_method_comparison_results.csv')
    
    # Replace inf values with NaN for proper statistical analysis
    df['min_clearance'] = df['min_clearance'].replace([np.inf, -np.inf], np.nan)
    df['avg_clearance'] = df['avg_clearance'].replace([np.inf, -np.inf], np.nan)
    df['pos_error'] = df['pos_error'].replace([np.inf, -np.inf], np.nan)
    df['ori_error'] = df['ori_error'].replace([np.inf, -np.inf], np.nan)
    
    print(f"📊 Loaded {len(df)} data points")
    print(f"Methods: {df['method'].unique()}")
    print(f"Poses analyzed: {df['pose_id'].nunique()}")
    
    return df

def create_execution_time_analysis(df, ax):
    """Create execution time boxplots for different categories."""
    
    # Prepare data for different categories
    categories = []
    times = []
    methods = []
    
    for method in df['method'].unique():
        method_data = df[df['method'] == method]
        
        # All attempts
        for time_val in method_data['time_ms']:
            categories.append('All Attempts')
            times.append(time_val)
            methods.append(method)
        
        # Successful only
        successful_data = method_data[method_data['success'] == 1]
        for time_val in successful_data['time_ms']:
            categories.append('Successful')
            times.append(time_val)
            methods.append(method)
        
        # Collision-free only
        collision_free_data = method_data[method_data['collision_free'] == 1]
        for time_val in collision_free_data['time_ms']:
            categories.append('Collision-Free')
            times.append(time_val)
            methods.append(method)
    
    plot_df = pd.DataFrame({
        'Category': categories,
        'Time (ms)': times,
        'Method': methods
    })
    
    # Create boxplot with default matplotlib colors
    box_plot = ax.boxplot([
        plot_df[(plot_df['Method'] == 'Newton-Raphson') & (plot_df['Category'] == 'All Attempts')]['Time (ms)'],
        plot_df[(plot_df['Method'] == 'SA-Optimized') & (plot_df['Category'] == 'All Attempts')]['Time (ms)'],
        plot_df[(plot_df['Method'] == 'Newton-Raphson') & (plot_df['Category'] == 'Successful')]['Time (ms)'],
        plot_df[(plot_df['Method'] == 'SA-Optimized') & (plot_df['Category'] == 'Successful')]['Time (ms)'],
        plot_df[(plot_df['Method'] == 'Newton-Raphson') & (plot_df['Category'] == 'Collision-Free')]['Time (ms)'],
        plot_df[(plot_df['Method'] == 'SA-Optimized') & (plot_df['Category'] == 'Collision-Free')]['Time (ms)']
    ], patch_artist=True, labels=[
        'NR\nAll', 'SA\nAll', 'NR\nSuccess', 'SA\nSuccess', 'NR\nCol-Free', 'SA\nCol-Free'
    ])
    
    # Color the boxes with default matplotlib colors
    colors = ['C0', 'C1', 'C0', 'C1', 'C0', 'C1']  # Default matplotlib color cycle
    for patch, color in zip(box_plot['boxes'], colors):
        patch.set_facecolor(color)
        patch.set_alpha(0.7)
    
    ax.set_title('Execution Time Analysis', fontsize=14, fontweight='bold')
    ax.set_ylabel('Time (ms)')
    ax.grid(True, alpha=0.3)
    
    # Add statistics
    stats_text = []
    for i, (data, label) in enumerate(zip([
        plot_df[(plot_df['Method'] == 'Newton-Raphson') & (plot_df['Category'] == 'All Attempts')]['Time (ms)'],
        plot_df[(plot_df['Method'] == 'SA-Optimized') & (plot_df['Category'] == 'All Attempts')]['Time (ms)'],
        plot_df[(plot_df['Method'] == 'Newton-Raphson') & (plot_df['Category'] == 'Successful')]['Time (ms)'],
        plot_df[(plot_df['Method'] == 'SA-Optimized') & (plot_df['Category'] == 'Successful')]['Time (ms)'],
        plot_df[(plot_df['Method'] == 'Newton-Raphson') & (plot_df['Category'] == 'Collision-Free')]['Time (ms)'],
        plot_df[(plot_df['Method'] == 'SA-Optimized') & (plot_df['Category'] == 'Collision-Free')]['Time (ms)']
    ], ['NR All', 'SA All', 'NR Success', 'SA Success', 'NR Col-Free', 'SA Col-Free'])):
        if len(data) > 0:
            stats_text.append(f'{label}: n={len(data)}, μ={data.mean():.1f}ms')
    
    ax.text(0.02, 0.98, '\n'.join(stats_text), transform=ax.transAxes, 
            verticalalignment='top', fontsize=9, 
            bbox=dict(boxstyle="round,pad=0.3", facecolor="lightyellow", alpha=0.8))

def create_success_rate_analysis(df, ax):
    """Create success rate comparison."""
    
    methods = df['method'].unique()
    categories = ['All Attempts', 'Among Collision-Free']
    
    success_rates = []
    error_bars = []
    
    for method in methods:
        method_data = df[df['method'] == method]
        
        # Overall success rate
        overall_success = method_data['success'].mean()
        overall_n = len(method_data)
        overall_se = np.sqrt(overall_success * (1 - overall_success) / overall_n)
        
        # Success rate among collision-free
        collision_free_data = method_data[method_data['collision_free'] == 1]
        if len(collision_free_data) > 0:
            cf_success = collision_free_data['success'].mean()
            cf_n = len(collision_free_data)
            cf_se = np.sqrt(cf_success * (1 - cf_success) / cf_n)
        else:
            cf_success = 0
            cf_se = 0
        
        success_rates.extend([overall_success * 100, cf_success * 100])
        error_bars.extend([overall_se * 100, cf_se * 100])
    
    x = np.arange(len(categories))
    width = 0.35
    
    # Use default matplotlib colors
    bars1 = ax.bar(x - width/2, success_rates[::2], width, 
                   yerr=error_bars[::2], capsize=5, 
                   label='Newton-Raphson', color='C0', alpha=0.7)
    bars2 = ax.bar(x + width/2, success_rates[1::2], width, 
                   yerr=error_bars[1::2], capsize=5, 
                   label='SA-Optimized', color='C1', alpha=0.7)
    
    ax.set_title('Success Rates', fontsize=14, fontweight='bold')
    ax.set_ylabel('Success Rate (%)')
    ax.set_xlabel('Category')
    ax.set_xticks(x)
    ax.set_xticklabels(categories)
    ax.legend()
    ax.grid(True, alpha=0.3)
    ax.set_ylim(0, 100)
    
    # Add percentage labels on bars
    for bars in [bars1, bars2]:
        for bar in bars:
            height = bar.get_height()
            ax.text(bar.get_x() + bar.get_width()/2., height + 1,
                   f'{height:.1f}%', ha='center', va='bottom', fontweight='bold')

def create_clearance_analysis(df, ax):
    """Create clearance boxplots."""
    
    # Filter for valid clearance data (successful and collision-free)
    valid_data = df[(df['success'] == 1) & (df['collision_free'] == 1)].copy()
    valid_data = valid_data.dropna(subset=['min_clearance', 'avg_clearance'])
    
    if len(valid_data) == 0:
        ax.text(0.5, 0.5, 'No valid clearance data available', 
                ha='center', va='center', transform=ax.transAxes, fontsize=12)
        ax.set_title('Clearance Analysis', fontsize=14, fontweight='bold')
        return
    
    # Prepare data for boxplot
    clearance_data = []
    clearance_types = []
    methods = []
    
    for method in valid_data['method'].unique():
        method_data = valid_data[valid_data['method'] == method]
        
        # Minimum clearance
        for val in method_data['min_clearance'].dropna():
            clearance_data.append(val)
            clearance_types.append('Minimum')
            methods.append(method)
        
        # Average clearance
        for val in method_data['avg_clearance'].dropna():
            clearance_data.append(val)
            clearance_types.append('Average')
            methods.append(method)
    
    plot_df = pd.DataFrame({
        'Clearance (m)': clearance_data,
        'Type': clearance_types,
        'Method': methods
    })
    
    # Create boxplot
    box_plot = ax.boxplot([
        plot_df[(plot_df['Method'] == 'Newton-Raphson') & (plot_df['Type'] == 'Minimum')]['Clearance (m)'],
        plot_df[(plot_df['Method'] == 'SA-Optimized') & (plot_df['Type'] == 'Minimum')]['Clearance (m)'],
        plot_df[(plot_df['Method'] == 'Newton-Raphson') & (plot_df['Type'] == 'Average')]['Clearance (m)'],
        plot_df[(plot_df['Method'] == 'SA-Optimized') & (plot_df['Type'] == 'Average')]['Clearance (m)']
    ], patch_artist=True, labels=[
        'NR\nMin', 'SA\nMin', 'NR\nAvg', 'SA\nAvg'
    ])
    
    # Color the boxes
    colors = ['C0', 'C1', 'C0', 'C1']
    for patch, color in zip(box_plot['boxes'], colors):
        patch.set_facecolor(color)
        patch.set_alpha(0.7)
    
    ax.set_title('Clearance Analysis', fontsize=14, fontweight='bold')
    ax.set_ylabel('Clearance (m)')
    ax.grid(True, alpha=0.3)
    
    # Add safety thresholds
    ax.axhline(y=0.05, color='red', linestyle='--', alpha=0.7, label='5cm Critical')
    ax.axhline(y=0.10, color='orange', linestyle='--', alpha=0.7, label='10cm Safety')
    ax.axhline(y=0.20, color='green', linestyle='--', alpha=0.7, label='20cm Optimal')
    ax.legend(loc='upper right')

def create_solution_variance_analysis(df, ax):
    """Create solution configuration variance analysis."""
    
    # We'll analyze the variance in solution quality metrics
    successful_data = df[df['success'] == 1].copy()
    
    if len(successful_data) == 0:
        ax.text(0.5, 0.5, 'No successful solutions for variance analysis', 
                ha='center', va='center', transform=ax.transAxes, fontsize=12)
        ax.set_title('Solution Configuration Variance', fontsize=14, fontweight='bold')
        return
    
    # Calculate coefficient of variation for different metrics
    metrics = []
    cv_values = []
    methods = []
    
    for method in successful_data['method'].unique():
        method_data = successful_data[successful_data['method'] == method]
        
        # Position error variance
        pos_errors = method_data['pos_error'].dropna()
        if len(pos_errors) > 1 and pos_errors.mean() > 0:
            cv_pos = pos_errors.std() / pos_errors.mean()
            metrics.append('Position Error CV')
            cv_values.append(cv_pos)
            methods.append(method)
        
        # Orientation error variance
        ori_errors = method_data['ori_error'].dropna()
        if len(ori_errors) > 1 and ori_errors.mean() > 0:
            cv_ori = ori_errors.std() / ori_errors.mean()
            metrics.append('Orientation Error CV')
            cv_values.append(cv_ori)
            methods.append(method)
        
        # Execution time variance
        times = method_data['time_ms'].dropna()
        if len(times) > 1:
            cv_time = times.std() / times.mean()
            metrics.append('Time CV')
            cv_values.append(cv_time)
            methods.append(method)
        
        # Clearance variance (if available)
        clearances = method_data['min_clearance'].dropna()
        clearances = clearances[clearances != np.inf]
        if len(clearances) > 1 and clearances.mean() > 0:
            cv_clearance = clearances.std() / clearances.mean()
            metrics.append('Clearance CV')
            cv_values.append(cv_clearance)
            methods.append(method)
    
    if len(cv_values) == 0:
        ax.text(0.5, 0.5, 'Insufficient data for variance analysis', 
                ha='center', va='center', transform=ax.transAxes, fontsize=12)
        ax.set_title('Solution Configuration Variance', fontsize=14, fontweight='bold')
        return
    
    plot_df = pd.DataFrame({
        'Metric': metrics,
        'CV': cv_values,
        'Method': methods
    })
    
    # Group by metric and method for plotting
    unique_metrics = plot_df['Metric'].unique()
    x = np.arange(len(unique_metrics))
    width = 0.35
    
    nr_cvs = []
    sa_cvs = []
    
    for metric in unique_metrics:
        metric_data = plot_df[plot_df['Metric'] == metric]
        nr_data = metric_data[metric_data['Method'] == 'Newton-Raphson']['CV']
        sa_data = metric_data[metric_data['Method'] == 'SA-Optimized']['CV']
        
        nr_cvs.append(nr_data.mean() if len(nr_data) > 0 else 0)
        sa_cvs.append(sa_data.mean() if len(sa_data) > 0 else 0)
    
    bars1 = ax.bar(x - width/2, nr_cvs, width, label='Newton-Raphson', color='C0', alpha=0.7)
    bars2 = ax.bar(x + width/2, sa_cvs, width, label='SA-Optimized', color='C1', alpha=0.7)
    
    ax.set_title('Solution Configuration Variance\n(Coefficient of Variation)', fontsize=14, fontweight='bold')
    ax.set_ylabel('Coefficient of Variation')
    ax.set_xlabel('Metric')
    ax.set_xticks(x)
    ax.set_xticklabels(unique_metrics, rotation=45, ha='right')
    ax.legend()
    ax.grid(True, alpha=0.3)
    
    # Add value labels on bars
    for bars in [bars1, bars2]:
        for bar in bars:
            height = bar.get_height()
            if height > 0:
                ax.text(bar.get_x() + bar.get_width()/2., height + height*0.01,
                       f'{height:.2f}', ha='center', va='bottom', fontweight='bold', fontsize=9)

def generate_comprehensive_analysis():
    """Generate the comprehensive comparison analysis."""
    
    print("🤖 COMPREHENSIVE IK METHOD COMPARISON ANALYSIS")
    print("="*60)
    print(f"Generated on: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}")
    print("="*60)
    
    try:
        # Load and prepare data
        print("📊 Loading data...")
        df = load_and_prepare_data()
        print("✅ Data loaded successfully")
        
        # Set up the plot with consistent styling
        print("🎨 Setting up plots...")
        plt.style.use('default')  # Use default matplotlib style
        fig, ((ax1, ax2), (ax3, ax4)) = plt.subplots(2, 2, figsize=(16, 12))
        fig.suptitle('Comprehensive IK Method Comparison Analysis', 
                     fontsize=18, fontweight='bold', y=0.96)
        print("✅ Plot setup complete")
        
        # Generate all analyses
        print("📈 Creating execution time analysis...")
        create_execution_time_analysis(df, ax1)
        print("✅ Execution time analysis complete")
        
        print("📊 Creating success rate analysis...")
        create_success_rate_analysis(df, ax2)
        print("✅ Success rate analysis complete")
        
        print("🛡️ Creating clearance analysis...")
        create_clearance_analysis(df, ax3)
        print("✅ Clearance analysis complete")
        
        print("📐 Creating solution variance analysis...")
        create_solution_variance_analysis(df, ax4)
        print("✅ Solution variance analysis complete")
        
        print("💾 Saving plots...")
        plt.tight_layout()
        plt.subplots_adjust(top=0.92)
        
        # Save plots
        os.makedirs('plots', exist_ok=True)
        plt.savefig('plots/comprehensive_ik_comparison_analysis.png', dpi=150, bbox_inches='tight')
        plt.savefig('plots/comprehensive_ik_comparison_analysis.pdf', bbox_inches='tight')
        plt.close()
        print("✅ Plots saved successfully")
        
    except Exception as e:
        print(f"❌ Error during analysis: {e}")
        import traceback
        traceback.print_exc()
        raise
    
    # Generate summary statistics
    print(f"\n📈 ANALYSIS SUMMARY")
    print("="*60)
    
    for method in df['method'].unique():
        method_data = df[df['method'] == method]
        successful = method_data[method_data['success'] == 1]
        collision_free = method_data[method_data['collision_free'] == 1]
        
        print(f"\n🔧 {method.upper()}")
        print(f"   Total attempts: {len(method_data)}")
        print(f"   Successful: {len(successful)} ({len(successful)/len(method_data)*100:.1f}%)")
        print(f"   Collision-free: {len(collision_free)} ({len(collision_free)/len(method_data)*100:.1f}%)")
        
        if len(successful) > 0:
            print(f"   Avg time (successful): {successful['time_ms'].mean():.2f} ± {successful['time_ms'].std():.2f} ms")
            
            pos_errors = successful['pos_error'].dropna()
            if len(pos_errors) > 0:
                print(f"   Avg pos error: {pos_errors.mean():.2e} ± {pos_errors.std():.2e} m")
            
            clearances = successful['min_clearance'].dropna()
            clearances = clearances[clearances != np.inf]
            if len(clearances) > 0:
                print(f"   Avg min clearance: {clearances.mean():.3f} ± {clearances.std():.3f} m")
    
    print(f"\n✅ ANALYSIS COMPLETE!")
    print(f"📊 Generated: plots/comprehensive_ik_comparison_analysis.png")
    print(f"📊 Generated: plots/comprehensive_ik_comparison_analysis.pdf")
    print("="*60)

if __name__ == "__main__":
    print("Starting comprehensive IK method comparison analysis...", flush=True)
    try:
        generate_comprehensive_analysis()
        print("✅ Comprehensive analysis finished successfully!", flush=True)
    except Exception as e:
        print(f"❌ Error: {e}", flush=True)
        import traceback
        traceback.print_exc()
        exit(1)
