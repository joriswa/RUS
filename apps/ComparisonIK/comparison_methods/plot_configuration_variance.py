#!/usr/bin/env python3
"""
Configuration Variance Analysis for IK Methods
Analyzes how much joint angle solutions vary for the same pose across multiple runs.
This measures solution consistency - whether methods find similar or different configurations.

Created: December 2024
Author: PathPlanner Analysis Suite
"""

import pandas as pd
import numpy as np
import matplotlib
matplotlib.use('Agg')  # Use non-interactive backend for server environments
import matplotlib.pyplot as plt
import seaborn as sns
import os
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
    """Load CSV data and filter for successful results with joint angles."""
    try:
        df = pd.read_csv(filename)
        print(f"📊 Total records loaded: {len(df)}")
        print(f"🎯 Unique poses: {df['pose_id'].nunique()}")
        print(f"📈 Methods analyzed: {', '.join(df['method'].unique())}")
        
    except FileNotFoundError:
        print("❌ Error: Could not find 'three_method_comparison_results.csv'")
        print("   Please run the C++ comparison script first to generate the data.")
        return None, None
    except Exception as e:
        print(f"❌ Error loading data: {e}")
        return None, None
    
    # Filter for successful results only
    successful_df = df[df['success'] == True].copy()
    print(f"✅ Successful results: {len(successful_df)} ({len(successful_df)/len(df)*100:.1f}%)")
    
    # Check for joint angle data
    joint_cols = ['q1', 'q2', 'q3', 'q4', 'q5', 'q6', 'q7']
    if all(col in successful_df.columns for col in joint_cols):
        successful_df = successful_df.dropna(subset=joint_cols)
        print(f"📐 Results with valid joint angles: {len(successful_df)}")
    else:
        print("⚠️ Warning: No joint angle columns found in data")
        return df, None
    
    return df, successful_df

def calculate_variance_metrics(successful_df):
    """Calculate configuration variance metrics for each pose and method."""
    joint_cols = ['q1', 'q2', 'q3', 'q4', 'q5', 'q6', 'q7']
    variance_results = []
    
    for pose_id in sorted(successful_df['pose_id'].unique()):
        for method in successful_df['method'].unique():
            pose_method_data = successful_df[
                (successful_df['pose_id'] == pose_id) & 
                (successful_df['method'] == method)
            ]
            
            if len(pose_method_data) < 2:  # Need at least 2 samples for variance
                continue
            
            # Extract joint angles matrix
            joint_matrix = pose_method_data[joint_cols].values
            n_solutions = len(joint_matrix)
            
            # Calculate variance metrics
            joint_variances = np.var(joint_matrix, axis=0, ddof=1)
            total_variance = np.sum(joint_variances)
            
            # Configuration distance metrics
            mean_config = np.mean(joint_matrix, axis=0)
            config_distances = [np.linalg.norm(config - mean_config) for config in joint_matrix]
            mean_distance_from_center = np.mean(config_distances)
            
            # Pairwise distances between configurations
            pairwise_distances = []
            for i in range(n_solutions):
                for j in range(i+1, n_solutions):
                    distance = np.linalg.norm(joint_matrix[i] - joint_matrix[j])
                    pairwise_distances.append(distance)
            
            mean_pairwise_distance = np.mean(pairwise_distances) if pairwise_distances else 0
            
            variance_results.append({
                'pose_id': pose_id,
                'method': method,
                'n_solutions': n_solutions,
                'total_variance': total_variance,
                'mean_pairwise_distance': mean_pairwise_distance,
                'mean_distance_from_center': mean_distance_from_center,
                'success_rate': n_solutions / 100.0  # Assuming 100 runs per pose
            })
    
    return pd.DataFrame(variance_results)

def create_variance_plots(variance_df, output_dir='plots'):
    """Create configuration variance visualization plots."""
    os.makedirs(output_dir, exist_ok=True)
    
    plt.style.use('default')
    
    # Define consistent colors (same as other analysis plots)
    colors = ['#1f77b4', '#ff7f0e']  # Blue, Orange (default matplotlib)
    method_colors = {'Newton-Raphson': colors[0], 'SA-Optimized': colors[1]}
    
    # Create figure with top plot and per-joint analysis
    fig = plt.figure(figsize=(18, 12))
    
    # Function to set median lines to black (consistent with other plots)
    def set_median_lines_black(ax):
        for line in ax.lines:
            if line.get_linestyle() == '-':  # median lines
                line.set_color('black')
    
    # Top plot: Overall variance comparison (spanning full width)
    ax_top = plt.subplot(2, 1, 1)
    sns.boxplot(data=variance_df, x='method', y='total_variance', hue='method', ax=ax_top, 
               palette=method_colors, legend=False)
    set_median_lines_black(ax_top)
    
    ax_top.set_title('Configuration Variance by Method\n(Lower = More Consistent)', 
                    fontweight='bold', fontsize=16)
    ax_top.set_ylabel('Total Variance (rad²)', fontsize=12)
    ax_top.set_xlabel('Method', fontsize=12)
    ax_top.grid(True, alpha=0.3)
    
    # Add summary statistics as text
    for i, method in enumerate(variance_df['method'].unique()):
        method_data = variance_df[variance_df['method'] == method]
        mean_var = method_data['total_variance'].mean()
        n_poses = len(method_data)
        ax_top.text(i, ax_top.get_ylim()[1]*0.9, f'n={n_poses}\nμ = {mean_var:.3f}', 
                   ha='center', fontweight='bold', 
                   bbox=dict(boxstyle='round', facecolor='lightgreen', alpha=0.8))
    
    # Calculate per-joint variance data
    df_full = pd.read_csv('two_method_comparison_results.csv')
    successful_df = df_full[df_full['success'] == True].copy()
    joint_cols = ['q1', 'q2', 'q3', 'q4', 'q5', 'q6', 'q7']
    successful_df = successful_df.dropna(subset=joint_cols)
    
    # Calculate per-joint standard deviations for each pose-method combination
    joint_variance_data = []
    for pose_id in sorted(successful_df['pose_id'].unique()):
        for method in successful_df['method'].unique():
            pose_method_data = successful_df[
                (successful_df['pose_id'] == pose_id) & 
                (successful_df['method'] == method)
            ]
            
            if len(pose_method_data) < 2:
                continue
            
            # Calculate standard deviation for each joint
            joint_matrix = pose_method_data[joint_cols].values
            joint_std_devs = np.std(joint_matrix, axis=0, ddof=1)
            
            for joint_idx, std_dev in enumerate(joint_std_devs):
                joint_variance_data.append({
                    'pose_id': pose_id,
                    'method': method,
                    'joint': f'Joint {joint_idx + 1}',
                    'joint_index': joint_idx + 1,
                    'std_dev': std_dev
                })
    
    joint_variance_df = pd.DataFrame(joint_variance_data)
    
    # Bottom plots: Per-joint variance analysis (7 subplots)
    for joint_idx in range(7):
        ax = plt.subplot(2, 7, 8 + joint_idx)  # Second row, 7 columns
        joint_name = f'Joint {joint_idx + 1}'
        joint_data = joint_variance_df[joint_variance_df['joint'] == joint_name]
        
        if len(joint_data) > 0:
            sns.boxplot(data=joint_data, x='method', y='std_dev', hue='method', ax=ax, 
                       palette=method_colors, legend=False)
            set_median_lines_black(ax)
            ax.set_title(f'{joint_name}', fontweight='bold', fontsize=11)
            ax.set_ylabel('Std Dev (rad)' if joint_idx == 0 else '', fontsize=10)
            ax.set_xlabel('')
            ax.grid(True, alpha=0.3)
            ax.tick_params(axis='x', rotation=45, labelsize=9)
            
            # Add mean values as text
            for i, method in enumerate(joint_data['method'].unique()):
                method_data = joint_data[joint_data['method'] == method]['std_dev']
                if len(method_data) > 0:
                    mean_val = method_data.mean()
                    ax.text(i, ax.get_ylim()[1]*0.85, f'μ={mean_val:.3f}', 
                           ha='center', fontsize=8, 
                           bbox=dict(boxstyle='round', facecolor='lightblue', alpha=0.7))
        else:
            ax.text(0.5, 0.5, 'No Data', ha='center', va='center', transform=ax.transAxes)
            ax.set_title(f'{joint_name} - No Data', fontsize=11)
    
    plt.tight_layout()
    
    # Save plots
    png_path = os.path.join(output_dir, 'configuration_variance_analysis.png')
    pdf_path = os.path.join(output_dir, 'configuration_variance_analysis.pdf')
    plt.savefig(png_path, dpi=300, bbox_inches='tight')
    plt.savefig(pdf_path, bbox_inches='tight')
    plt.close()
    
    print(f"✅ Generated: {png_path}")
    print(f"✅ Generated: {pdf_path}")
    
    return png_path, pdf_path

def print_analysis_summary(variance_df):
    """Print comprehensive variance analysis summary."""
    print("\n" + "="*70)
    print("🔍 CONFIGURATION VARIANCE ANALYSIS SUMMARY")
    print("="*70)
    
    methods = variance_df['method'].unique()
    
    print(f"\n📊 Dataset Overview:")
    print(f"   • Poses analyzed: {len(variance_df['pose_id'].unique())}")
    print(f"   • Methods compared: {', '.join(methods)}")
    
    print(f"\n🏆 Method Consistency Comparison:")
    for method in methods:
        method_data = variance_df[variance_df['method'] == method]
        print(f"\n   {method}:")
        print(f"     - Average variance: {method_data['total_variance'].mean():.4f} rad²")
        print(f"     - Std deviation: {method_data['total_variance'].std():.4f} rad²")
        print(f"     - Average success rate: {method_data['success_rate'].mean():.1%}")
        print(f"     - Mean pairwise distance: {method_data['mean_pairwise_distance'].mean():.4f} rad")
    
    # Method ranking
    print(f"\n📈 Consistency Ranking (Lower variance = More consistent):")
    method_ranking = variance_df.groupby('method')['total_variance'].mean().sort_values()
    for i, (method, avg_var) in enumerate(method_ranking.items()):
        print(f"   {i+1}. {method}: {avg_var:.4f} rad² average variance")
    
    # Most challenging poses
    print(f"\n⚠️ Most Challenging Poses (Top 5):")
    pose_max_var = variance_df.groupby('pose_id')['total_variance'].max().sort_values(ascending=False).head(5)
    for i, (pose_id, max_var) in enumerate(pose_max_var.items()):
        print(f"   {i+1}. Pose {pose_id}: max variance {max_var:.4f} rad²")

def plot_configuration_variance():
    """Main analysis function for configuration variance."""
    
    print("🔍 CONFIGURATION VARIANCE ANALYSIS FOR IK METHODS")
    print("=" * 50)
    print("Analyzing solution consistency across multiple runs")
    
    # Load and validate data
    full_df, successful_df = load_and_filter_data('three_method_comparison_results.csv')
    
    if full_df is None:
        return
        
    if successful_df is None or len(successful_df) == 0:
        print("❌ No successful results with joint angles found.")
        return
    
    # Calculate variance metrics
    print("\n📊 Calculating variance metrics...")
    variance_df = calculate_variance_metrics(successful_df)
    
    if len(variance_df) == 0:
        print("❌ No variance data could be calculated.")
        return
    
    print(f"✅ Calculated variance for {len(variance_df)} pose-method combinations")
    
    # Create visualizations
    print("\n📈 Creating visualizations...")
    create_variance_plots(variance_df)
    
    # Print summary
    print_analysis_summary(variance_df)
    
    # Save variance data for further use
    variance_df.to_csv('configuration_variance_data.csv', index=False)
    print(f"📄 Saved: configuration_variance_data.csv")
    
    print(f"\n✅ Analysis complete!")
    print(f"📊 Generated: plots/configuration_variance.png")
    print(f"📄 Generated: plots/configuration_variance.pdf")

if __name__ == "__main__":
    plot_configuration_variance()
