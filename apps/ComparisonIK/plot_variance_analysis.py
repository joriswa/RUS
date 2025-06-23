#!/usr/bin/env python3
"""
Configuration Variance Analysis for IK Methods
Analyzes how much joint angle solutions vary for the same pose across 100 runs per method.
This measures solution consistency - do methods find similar or very different configurations?
"""

import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import seaborn as sns
from scipy import stats
import warnings

# Suppress future warnings
warnings.filterwarnings('ignore', category=FutureWarning)

def set_median_lines_black():
    """Helper function to set median lines to black color in boxplots."""
    ax = plt.gca()
    for patch in ax.artists:
        # The median line is the 5th line (index 4) for each box
        r, g, b, alpha = patch.get_facecolor()
        patch.set_facecolor((r, g, b, alpha))
    
    # Set median line properties
    for line in ax.lines:
        if line.get_linestyle() == '-':  # Median lines are solid
            line.set_color('black')

def load_and_filter_data(filename):
    """Load CSV data and filter for successful results with joint angles."""
    df = pd.read_csv(filename)
    
    print(f"📊 Total records loaded: {len(df)}")
    print(f"🔢 Unique poses: {df['pose_id'].nunique()}")
    print(f"🔢 Unique runs per pose: {df['run_id'].nunique()}")
    print(f"🎯 Methods: {list(df['method'].unique())}")
    
    # Filter for successful results only (for joint angle analysis)
    successful_df = df[df['success'] == True].copy()
    print(f"✅ Successful results: {len(successful_df)} ({len(successful_df)/len(df)*100:.1f}%)")
    
    # Check if we have joint angle data
    joint_cols = ['q1', 'q2', 'q3', 'q4', 'q5', 'q6', 'q7']
    if all(col in successful_df.columns for col in joint_cols):
        # Remove rows with NaN joint angles
        successful_df = successful_df.dropna(subset=joint_cols)
        print(f"📐 Results with valid joint angles: {len(successful_df)}")
    else:
        print("⚠️ Warning: No joint angle columns found in data")
        return df, None
    
    return df, successful_df

def calculate_joint_angle_variance(successful_df):
    """Calculate variance metrics for each joint across all methods and poses."""
    joint_cols = ['q1', 'q2', 'q3', 'q4', 'q5', 'q6', 'q7']
    variance_data = []
    
    for pose_id in sorted(successful_df['pose_id'].unique()):
        for method in successful_df['method'].unique():
            pose_method_data = successful_df[
                (successful_df['pose_id'] == pose_id) & 
                (successful_df['method'] == method)
            ]
            
            if len(pose_method_data) < 2:  # Need at least 2 samples for variance
                continue
                
            for joint_idx, joint_col in enumerate(joint_cols):
                joint_angles = pose_method_data[joint_col].values
                
                variance_data.append({
                    'pose_id': pose_id,
                    'method': method,
                    'joint': f'Joint {joint_idx + 1}',
                    'joint_index': joint_idx + 1,
                    'variance': np.var(joint_angles, ddof=1),
                    'std_dev': np.std(joint_angles, ddof=1),
                    'range': np.max(joint_angles) - np.min(joint_angles),
                    'mean_angle': np.mean(joint_angles),
                    'n_samples': len(joint_angles),
                    'coefficient_variation': np.std(joint_angles, ddof=1) / np.abs(np.mean(joint_angles)) if np.mean(joint_angles) != 0 else np.inf
                })
    
    return pd.DataFrame(variance_data)

def calculate_pose_consistency_metrics(successful_df):
    """Calculate overall consistency metrics per pose and method."""
    joint_cols = ['q1', 'q2', 'q3', 'q4', 'q5', 'q6', 'q7']
    consistency_data = []
    
    for pose_id in sorted(successful_df['pose_id'].unique()):
        for method in successful_df['method'].unique():
            pose_method_data = successful_df[
                (successful_df['pose_id'] == pose_id) & 
                (successful_df['method'] == method)
            ]
            
            if len(pose_method_data) < 2:
                continue
            
            # Calculate total configuration variance (sum of all joint variances)
            total_variance = 0
            max_joint_variance = 0
            joint_variances = []
            
            for joint_col in joint_cols:
                joint_var = np.var(pose_method_data[joint_col].values, ddof=1)
                joint_variances.append(joint_var)
                total_variance += joint_var
                max_joint_variance = max(max_joint_variance, joint_var)
            
            # Calculate Euclidean distance variance between configurations
            configurations = pose_method_data[joint_cols].values
            if len(configurations) > 1:
                config_distances = []
                mean_config = np.mean(configurations, axis=0)
                for config in configurations:
                    distance = np.linalg.norm(config - mean_config)
                    config_distances.append(distance)
                config_variance = np.var(config_distances, ddof=1)
            else:
                config_variance = 0
            
            consistency_data.append({
                'pose_id': pose_id,
                'method': method,
                'total_joint_variance': total_variance,
                'max_joint_variance': max_joint_variance,
                'mean_joint_variance': np.mean(joint_variances),
                'configuration_distance_variance': config_variance,
                'n_samples': len(pose_method_data),
                'success_rate': len(pose_method_data) / 100.0  # Assuming 100 runs per pose
            })
    
    return pd.DataFrame(consistency_data)

def create_variance_analysis_plots(variance_df, consistency_df):
    """Create comprehensive variance analysis plots."""
    # Set up the plot style
    plt.style.use('default')
    sns.set_palette("husl")
    
    # Create figure with subplots
    fig = plt.figure(figsize=(20, 24))
    
    # 1. Joint-specific variance comparison (top row)
    ax1 = plt.subplot(4, 2, 1)
    joint_variance_summary = variance_df.groupby(['joint', 'method'])['variance'].mean().reset_index()
    sns.barplot(data=joint_variance_summary, x='joint', y='variance', hue='method', ax=ax1)
    ax1.set_title('Average Joint Angle Variance by Joint and Method', fontsize=14, fontweight='bold')
    ax1.set_ylabel('Variance (rad²)')
    ax1.legend(title='Method')
    ax1.grid(True, alpha=0.3)
    
    # 2. Total configuration variance per pose
    ax2 = plt.subplot(4, 2, 2)
    sns.boxplot(data=consistency_df, x='method', y='total_joint_variance', ax=ax2, hue='method', legend=False)
    set_median_lines_black()
    ax2.set_title('Total Configuration Variance Distribution', fontsize=14, fontweight='bold')
    ax2.set_ylabel('Total Joint Variance (rad²)')
    ax2.set_xlabel('Method')
    ax2.grid(True, alpha=0.3)
    
    # 3. Pose-by-pose variance comparison (heatmap)
    ax3 = plt.subplot(4, 2, 3)
    variance_pivot = consistency_df.pivot(index='pose_id', columns='method', values='total_joint_variance')
    sns.heatmap(variance_pivot, annot=True, fmt='.4f', cmap='YlOrRd', ax=ax3)
    ax3.set_title('Configuration Variance Heatmap by Pose', fontsize=14, fontweight='bold')
    ax3.set_ylabel('Pose ID')
    
    # 4. Standard deviation comparison by joint
    ax4 = plt.subplot(4, 2, 4)
    sns.boxplot(data=variance_df, x='joint', y='std_dev', hue='method', ax=ax4, legend=False)
    set_median_lines_black()
    ax4.set_title('Joint Angle Standard Deviation by Joint', fontsize=14, fontweight='bold')
    ax4.set_ylabel('Standard Deviation (rad)')
    ax4.tick_params(axis='x', rotation=45)
    ax4.grid(True, alpha=0.3)
    
    # 5. Most variable poses identification
    ax5 = plt.subplot(4, 2, 5)
    pose_max_variance = consistency_df.groupby('pose_id')['total_joint_variance'].max().reset_index()
    pose_max_variance = pose_max_variance.sort_values('total_joint_variance', ascending=False).head(10)
    sns.barplot(data=pose_max_variance, x='pose_id', y='total_joint_variance', ax=ax5)
    ax5.set_title('Top 10 Most Variable Poses (Max Variance)', fontsize=14, fontweight='bold')
    ax5.set_ylabel('Max Total Variance (rad²)')
    ax5.set_xlabel('Pose ID')
    
    # 6. Configuration distance variance
    ax6 = plt.subplot(4, 2, 6)
    sns.scatterplot(data=consistency_df, x='success_rate', y='configuration_distance_variance', 
                   hue='method', s=100, alpha=0.7, ax=ax6)
    ax6.set_title('Configuration Variance vs Success Rate', fontsize=14, fontweight='bold')
    ax6.set_xlabel('Success Rate')
    ax6.set_ylabel('Configuration Distance Variance')
    ax6.grid(True, alpha=0.3)
    
    # 7. Joint variance correlation matrix
    ax7 = plt.subplot(4, 2, 7)
    # Create correlation matrix of joint variances
    joint_variance_wide = variance_df.pivot_table(
        index=['pose_id', 'method'], 
        columns='joint_index', 
        values='variance'
    ).fillna(0)
    correlation_matrix = joint_variance_wide.corr()
    sns.heatmap(correlation_matrix, annot=True, fmt='.2f', cmap='coolwarm', center=0, ax=ax7)
    ax7.set_title('Joint Variance Correlation Matrix', fontsize=14, fontweight='bold')
    ax7.set_xlabel('Joint Index')
    ax7.set_ylabel('Joint Index')
    
    # 8. Method comparison summary
    ax8 = plt.subplot(4, 2, 8)
    method_summary = consistency_df.groupby('method').agg({
        'total_joint_variance': ['mean', 'std', 'max'],
        'success_rate': 'mean'
    }).round(4)
    
    # Create a summary table
    ax8.axis('tight')
    ax8.axis('off')
    table_data = []
    for method in consistency_df['method'].unique():
        method_data = consistency_df[consistency_df['method'] == method]
        table_data.append([
            method,
            f"{method_data['total_joint_variance'].mean():.4f}",
            f"{method_data['total_joint_variance'].std():.4f}",
            f"{method_data['total_joint_variance'].max():.4f}",
            f"{method_data['success_rate'].mean():.3f}"
        ])
    
    table = ax8.table(cellText=table_data,
                     colLabels=['Method', 'Mean Variance', 'Std Variance', 'Max Variance', 'Avg Success Rate'],
                     cellLoc='center',
                     loc='center')
    table.auto_set_font_size(False)
    table.set_fontsize(10)
    table.scale(1.2, 1.5)
    ax8.set_title('Method Comparison Summary', fontsize=14, fontweight='bold')
    
    plt.tight_layout()
    plt.savefig('configuration_variance_analysis.png', dpi=300, bbox_inches='tight')
    print("📊 Saved: configuration_variance_analysis.png")
    
    return fig

def create_detailed_joint_analysis(variance_df):
    """Create detailed per-joint variance analysis."""
    fig, axes = plt.subplots(2, 4, figsize=(20, 10))
    axes = axes.flatten()
    
    joint_names = ['Joint 1', 'Joint 2', 'Joint 3', 'Joint 4', 'Joint 5', 'Joint 6', 'Joint 7']
    
    for i, joint_name in enumerate(joint_names):
        ax = axes[i]
        joint_data = variance_df[variance_df['joint'] == joint_name]
        
        if len(joint_data) > 0:
            sns.boxplot(data=joint_data, x='method', y='std_dev', ax=ax, hue='method', legend=False)
            set_median_lines_black()
            ax.set_title(f'{joint_name} Standard Deviation', fontweight='bold')
            ax.set_ylabel('Std Dev (rad)')
            ax.set_xlabel('')
            ax.grid(True, alpha=0.3)
            ax.tick_params(axis='x', rotation=45)
        else:
            ax.text(0.5, 0.5, 'No Data', ha='center', va='center', transform=ax.transAxes)
            ax.set_title(f'{joint_name} - No Data')
    
    # Remove the 8th subplot (extra)
    axes[7].remove()
    
    plt.suptitle('Per-Joint Variance Analysis Across All Poses', fontsize=16, fontweight='bold')
    plt.tight_layout()
    plt.savefig('detailed_joint_variance_analysis.png', dpi=300, bbox_inches='tight')
    print("📊 Saved: detailed_joint_variance_analysis.png")
    
    return fig

def print_statistical_summary(variance_df, consistency_df):
    """Print comprehensive statistical summary."""
    print("\n" + "="*80)
    print("🔍 CONFIGURATION VARIANCE ANALYSIS SUMMARY")
    print("="*80)
    
    # Overall statistics
    total_poses = len(consistency_df['pose_id'].unique())
    methods = list(consistency_df['method'].unique())
    
    print(f"\n📊 Dataset Overview:")
    print(f"   • Total poses analyzed: {total_poses}")
    print(f"   • Methods compared: {', '.join(methods)}")
    print(f"   • Total variance measurements: {len(variance_df)}")
    
    # Method comparison
    print(f"\n🏆 Method Variance Comparison:")
    for method in methods:
        method_data = consistency_df[consistency_df['method'] == method]
        mean_var = method_data['total_joint_variance'].mean()
        std_var = method_data['total_joint_variance'].std()
        max_var = method_data['total_joint_variance'].max()
        min_var = method_data['total_joint_variance'].min()
        success_rate = method_data['success_rate'].mean()
        
        print(f"   {method}:")
        print(f"     - Mean total variance: {mean_var:.6f} rad²")
        print(f"     - Std dev of variance: {std_var:.6f} rad²")
        print(f"     - Range: {min_var:.6f} - {max_var:.6f} rad²")
        print(f"     - Average success rate: {success_rate:.1%}")
    
    # Most/least consistent poses
    print(f"\n🎯 Pose Consistency Analysis:")
    
    # Find most and least variable poses for each method
    for method in methods:
        method_data = consistency_df[consistency_df['method'] == method]
        if len(method_data) > 0:
            most_variable = method_data.loc[method_data['total_joint_variance'].idxmax()]
            least_variable = method_data.loc[method_data['total_joint_variance'].idxmin()]
            
            print(f"   {method}:")
            print(f"     - Most variable pose: {most_variable['pose_id']} (variance: {most_variable['total_joint_variance']:.6f})")
            print(f"     - Least variable pose: {least_variable['pose_id']} (variance: {least_variable['total_joint_variance']:.6f})")
    
    # Joint-specific analysis
    print(f"\n🔧 Joint-Specific Variance:")
    joint_summary = variance_df.groupby(['joint', 'method'])['variance'].agg(['mean', 'std', 'max']).round(6)
    
    for joint in sorted(variance_df['joint'].unique()):
        print(f"   {joint}:")
        for method in methods:
            if (joint, method) in joint_summary.index:
                stats = joint_summary.loc[(joint, method)]
                print(f"     - {method}: mean={stats['mean']:.6f}, std={stats['std']:.6f}, max={stats['max']:.6f}")
    
    # Statistical significance testing
    print(f"\n📈 Statistical Significance:")
    if len(methods) == 2:
        method1, method2 = methods
        data1 = consistency_df[consistency_df['method'] == method1]['total_joint_variance']
        data2 = consistency_df[consistency_df['method'] == method2]['total_joint_variance']
        
        if len(data1) > 1 and len(data2) > 1:
            # Perform Mann-Whitney U test (non-parametric)
            statistic, p_value = stats.mannwhitneyu(data1, data2, alternative='two-sided')
            print(f"   Mann-Whitney U test ({method1} vs {method2}):")
            print(f"     - Test statistic: {statistic:.2f}")
            print(f"     - P-value: {p_value:.6f}")
            print(f"     - Significance: {'Yes' if p_value < 0.05 else 'No'} (α = 0.05)")
            
            # Effect size (Cohen's d equivalent for Mann-Whitney)
            effect_size = (np.mean(data1) - np.mean(data2)) / np.sqrt((np.var(data1) + np.var(data2)) / 2)
            print(f"     - Effect size: {effect_size:.3f}")

def main():
    """Main analysis function."""
    print("🔍 Configuration Variance Analysis for IK Methods")
    print("=" * 60)
    
    # Load data
    try:
        full_df, successful_df = load_and_filter_data('two_method_comparison_results.csv')
        
        if successful_df is None or len(successful_df) == 0:
            print("❌ No successful results with joint angles found. Cannot perform variance analysis.")
            return
            
    except FileNotFoundError:
        print("❌ Error: Could not find 'two_method_comparison_results.csv'")
        print("   Please run the C++ comparison script first to generate the data.")
        return
    except Exception as e:
        print(f"❌ Error loading data: {e}")
        return
    
    # Calculate variance metrics
    print("\n📊 Calculating variance metrics...")
    variance_df = calculate_joint_angle_variance(successful_df)
    consistency_df = calculate_pose_consistency_metrics(successful_df)
    
    if len(variance_df) == 0:
        print("❌ No variance data could be calculated. Check that poses have multiple successful runs.")
        return
    
    print(f"✅ Calculated variance for {len(variance_df)} joint-pose-method combinations")
    print(f"✅ Calculated consistency for {len(consistency_df)} pose-method combinations")
    
    # Create plots
    print("\n📈 Creating variance analysis plots...")
    create_variance_analysis_plots(variance_df, consistency_df)
    create_detailed_joint_analysis(variance_df)
    
    # Print statistical summary
    print_statistical_summary(variance_df, consistency_df)
    
    print(f"\n✅ Variance analysis complete!")
    print(f"📊 Generated plots:")
    print(f"   • configuration_variance_analysis.png")
    print(f"   • detailed_joint_variance_analysis.png")

if __name__ == "__main__":
    main()
