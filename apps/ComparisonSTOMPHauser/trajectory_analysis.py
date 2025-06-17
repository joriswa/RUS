#!/usr/bin/env python3
"""
STOMP vs Hauser Trajectory Planning Analysis
============================================

This script provides comprehensive analysis of real trajectory metrics comparing
STOMP stochastic optimization and Hauser parabolic ramp trajectory planning.

Key Features:
- Real kinematic metrics analysis (velocity, acceleration, jerk)
- Statistical significance testing
- Publication-quality visualizations
- Comprehensive performance comparison
- Safety and smoothness analysis

Usage:
    python3 trajectory_analysis.py [input_csv_file]

Default input: stdout from stomp_hauser_comparison.cpp
"""

import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import seaborn as sns
from pathlib import Path
import sys
import warnings
from scipy import stats
from scipy.stats import mannwhitneyu, ttest_ind
import matplotlib.patches as mpatches
warnings.filterwarnings('ignore')

# Set publication-quality style
plt.rcParams.update({
    'figure.figsize': (12, 8),
    'font.size': 12,
    'axes.titlesize': 14,
    'axes.labelsize': 12,
    'xtick.labelsize': 10,
    'ytick.labelsize': 10,
    'legend.fontsize': 11,
    'figure.titlesize': 16,
    'lines.linewidth': 2,
    'lines.markersize': 8
})

class TrajectoryAnalyzer:
    """Comprehensive trajectory analysis with statistical testing."""
    
    def __init__(self, csv_file):
        """Initialize analyzer with trajectory data."""
        self.df = self.load_and_validate_data(csv_file)
        self.successful_df = self.df[self.df['success'] == 1].copy()
        self.stomp_data = self.successful_df[self.successful_df['algorithm'] == 'STOMP'].copy()
        self.hauser_data = self.successful_df[self.successful_df['algorithm'] == 'Hauser'].copy()
        
    def load_and_validate_data(self, csv_file):
        """Load and validate the trajectory comparison data."""
        try:
            # Try reading from file first, then from stdin if file doesn't exist
            if Path(csv_file).exists():
                df = pd.read_csv(csv_file)
                print(f"✅ Loaded {len(df)} results from {csv_file}")
            else:
                print(f"⚠️  File {csv_file} not found, reading from stdin...")
                df = pd.read_csv(sys.stdin)
                print(f"✅ Loaded {len(df)} results from stdin")

            # Validate required columns for real metrics
            required_cols = [
                'algorithm', 'success', 'planning_time_ms', 'path_length_rad',
                'max_jerk', 'avg_jerk', 'rms_jerk', 'min_clearance', 'avg_clearance',
                'smoothness_score', 'safety_score', 'max_velocity', 'max_acceleration',
                'velocity_variance', 'acceleration_variance', 'num_points'
            ]
            
            missing_cols = [col for col in required_cols if col not in df.columns]
            if missing_cols:
                raise ValueError(f"Missing required columns: {missing_cols}")

            # Clean data
            df = df.replace([np.inf, -np.inf], np.nan)
            
            print(f"Data validation successful: {len(df)} total runs")
            print(f"Algorithms found: {df['algorithm'].unique()}")
            
            return df

        except Exception as e:
            print(f"❌ Error loading data: {e}")
            sys.exit(1)

    def generate_comprehensive_summary(self):
        """Generate detailed statistical summary with significance testing."""
        print("\n" + "="*80)
        print("COMPREHENSIVE TRAJECTORY PLANNING ANALYSIS - REAL METRICS")
        print("="*80)

        total_runs = len(self.df)
        successful_runs = len(self.successful_df)
        
        print(f"\nOverall Statistics:")
        print(f"  Total Runs: {total_runs}")
        print(f"  Successful Runs: {successful_runs} ({100*successful_runs/total_runs:.1f}%)")
        
        for algorithm in ['STOMP', 'Hauser']:
            alg_data = self.df[self.df['algorithm'] == algorithm]
            alg_successful = alg_data[alg_data['success'] == 1]
            
            if len(alg_data) == 0:
                continue

            print(f"\n{algorithm} ALGORITHM PERFORMANCE:")
            print("-" * 60)

            # Success metrics
            success_rate = alg_data['success'].mean() * 100
            print(f"Success Rate: {success_rate:.1f}% ({alg_data['success'].sum()}/{len(alg_data)})")

            if len(alg_successful) == 0:
                print("No successful runs for detailed analysis")
                continue

            # Planning performance
            planning_times = alg_successful['planning_time_ms']
            print(f"Planning Time: {planning_times.mean():.1f} ± {planning_times.std():.1f} ms")
            print(f"  Range: [{planning_times.min():.1f}, {planning_times.max():.1f}] ms")

            # Path quality metrics
            path_lengths = alg_successful['path_length_rad']
            print(f"Path Length: {path_lengths.mean():.3f} ± {path_lengths.std():.3f} rad")
            
            durations = alg_successful.get('trajectory_duration_s', pd.Series([0]))
            if not durations.empty and durations.notna().any():
                print(f"Trajectory Duration: {durations.mean():.2f} ± {durations.std():.2f} s")

            # Kinematic metrics (the real differentiators)
            print(f"\nKinematic Analysis:")
            velocities = alg_successful['max_velocity']
            accelerations = alg_successful['max_acceleration']
            max_jerks = alg_successful['max_jerk']
            rms_jerks = alg_successful['rms_jerk']
            
            print(f"  Max Velocity: {velocities.mean():.3f} ± {velocities.std():.3f} rad/s")
            print(f"  Max Acceleration: {accelerations.mean():.3f} ± {accelerations.std():.3f} rad/s²")
            print(f"  Max Jerk: {max_jerks.mean():.3f} ± {max_jerks.std():.3f} rad/s³")
            print(f"  RMS Jerk: {rms_jerks.mean():.3f} ± {rms_jerks.std():.3f} rad/s³")

            # Smoothness and consistency
            smoothness = alg_successful['smoothness_score']
            vel_var = alg_successful['velocity_variance']
            acc_var = alg_successful['acceleration_variance']
            
            print(f"\nSmoothness Analysis:")
            print(f"  Smoothness Score: {smoothness.mean():.4f} ± {smoothness.std():.4f}")
            print(f"  Velocity Variance: {vel_var.mean():.6f} ± {vel_var.std():.6f}")
            print(f"  Acceleration Variance: {acc_var.mean():.6f} ± {acc_var.std():.6f}")

            # Safety metrics
            min_clearances = alg_successful['min_clearance']
            avg_clearances = alg_successful['avg_clearance']
            collision_free_rate = alg_successful['collision_free'].mean() * 100
            safety_scores = alg_successful['safety_score']
            
            print(f"\nSafety Analysis:")
            print(f"  Min Clearance: {min_clearances.mean():.4f} ± {min_clearances.std():.4f} m")
            print(f"  Avg Clearance: {avg_clearances.mean():.4f} ± {avg_clearances.std():.4f} m")
            print(f"  Collision-Free Rate: {collision_free_rate:.1f}%")
            print(f"  Safety Score: {safety_scores.mean():.4f} ± {safety_scores.std():.4f}")

        # Statistical comparison
        self.perform_statistical_tests()

    def perform_statistical_tests(self):
        """Perform statistical significance tests between algorithms."""
        print(f"\n" + "="*60)
        print("STATISTICAL SIGNIFICANCE ANALYSIS")
        print("="*60)
        
        if len(self.stomp_data) == 0 or len(self.hauser_data) == 0:
            print("❌ Insufficient data for statistical comparison")
            return

        metrics_to_test = [
            ('planning_time_ms', 'Planning Time', 'ms'),
            ('rms_jerk', 'RMS Jerk', 'rad/s³'),
            ('smoothness_score', 'Smoothness Score', ''),
            ('min_clearance', 'Minimum Clearance', 'm'),
            ('path_length_rad', 'Path Length', 'rad'),
            ('max_velocity', 'Maximum Velocity', 'rad/s'),
            ('safety_score', 'Safety Score', '')
        ]

        for metric, name, unit in metrics_to_test:
            if metric not in self.stomp_data.columns or metric not in self.hauser_data.columns:
                continue
                
            stomp_values = self.stomp_data[metric].dropna()
            hauser_values = self.hauser_data[metric].dropna()
            
            if len(stomp_values) < 3 or len(hauser_values) < 3:
                continue
                
            # Perform both parametric and non-parametric tests
            try:
                # T-test (assumes normality)
                t_stat, t_p_value = ttest_ind(stomp_values, hauser_values)
                
                # Mann-Whitney U test (non-parametric)
                u_stat, u_p_value = mannwhitneyu(stomp_values, hauser_values, alternative='two-sided')
                
                # Effect size (Cohen's d)
                pooled_std = np.sqrt(((len(stomp_values) - 1) * stomp_values.std()**2 +
                                    (len(hauser_values) - 1) * hauser_values.std()**2) /
                                   (len(stomp_values) + len(hauser_values) - 2))
                cohens_d = (stomp_values.mean() - hauser_values.mean()) / pooled_std
                
                print(f"\n{name} ({unit}):")
                print(f"  STOMP: {stomp_values.mean():.4f} ± {stomp_values.std():.4f} (n={len(stomp_values)})")
                print(f"  Hauser: {hauser_values.mean():.4f} ± {hauser_values.std():.4f} (n={len(hauser_values)})")
                print(f"  T-test p-value: {t_p_value:.4f} {'***' if t_p_value < 0.001 else '**' if t_p_value < 0.01 else '*' if t_p_value < 0.05 else 'ns'}")
                print(f"  Mann-Whitney p-value: {u_p_value:.4f} {'***' if u_p_value < 0.001 else '**' if u_p_value < 0.01 else '*' if u_p_value < 0.05 else 'ns'}")
                print(f"  Effect size (Cohen's d): {cohens_d:.3f} {'(large)' if abs(cohens_d) > 0.8 else '(medium)' if abs(cohens_d) > 0.5 else '(small)'}")
                
            except Exception as e:
                print(f"  Statistical test failed: {e}")

    def create_performance_dashboard(self):
        """Create comprehensive performance comparison dashboard."""
        fig = plt.figure(figsize=(20, 16))
        
        # Define color palette
        colors = {'STOMP': '#1f77b4', 'Hauser': '#ff7f0e'}
        
        # 1. Success Rate and Planning Time
        ax1 = plt.subplot(3, 4, 1)
        success_rates = self.df.groupby('algorithm')['success'].mean() * 100
        bars = ax1.bar(success_rates.index, success_rates.values, 
                      color=[colors[alg] for alg in success_rates.index], alpha=0.8)
        ax1.set_title('Success Rate Comparison', fontweight='bold')
        ax1.set_ylabel('Success Rate (%)')
        ax1.set_ylim(0, 100)
        for bar, rate in zip(bars, success_rates.values):
            ax1.text(bar.get_x() + bar.get_width()/2, bar.get_height() + 2,
                    f'{rate:.1f}%', ha='center', va='bottom', fontweight='bold')

        ax2 = plt.subplot(3, 4, 2)
        if len(self.successful_df) > 0:
            sns.boxplot(data=self.successful_df, x='algorithm', y='planning_time_ms', 
                       palette=colors, ax=ax2)
            ax2.set_title('Planning Time Distribution', fontweight='bold')
            ax2.set_ylabel('Planning Time (ms)')

        # 2. Smoothness Analysis (Key differentiator)
        ax3 = plt.subplot(3, 4, 3)
        if len(self.successful_df) > 0:
            sns.boxplot(data=self.successful_df, x='algorithm', y='rms_jerk', 
                       palette=colors, ax=ax3)
            ax3.set_title('RMS Jerk Comparison\n(Lower = Smoother)', fontweight='bold')
            ax3.set_ylabel('RMS Jerk (rad/s³)')

        ax4 = plt.subplot(3, 4, 4)
        if len(self.successful_df) > 0:
            sns.boxplot(data=self.successful_df, x='algorithm', y='smoothness_score', 
                       palette=colors, ax=ax4)
            ax4.set_title('Smoothness Score\n(Higher = Better)', fontweight='bold')
            ax4.set_ylabel('Smoothness Score')

        # 3. Safety Metrics
        ax5 = plt.subplot(3, 4, 5)
        if len(self.successful_df) > 0:
            sns.boxplot(data=self.successful_df, x='algorithm', y='min_clearance', 
                       palette=colors, ax=ax5)
            ax5.set_title('Minimum Clearance', fontweight='bold')
            ax5.set_ylabel('Min Clearance (m)')

        ax6 = plt.subplot(3, 4, 6)
        if len(self.successful_df) > 0:
            sns.boxplot(data=self.successful_df, x='algorithm', y='safety_score', 
                       palette=colors, ax=ax6)
            ax6.set_title('Safety Score', fontweight='bold')
            ax6.set_ylabel('Safety Score')

        # 4. Path Quality
        ax7 = plt.subplot(3, 4, 7)
        if len(self.successful_df) > 0:
            sns.boxplot(data=self.successful_df, x='algorithm', y='path_length_rad', 
                       palette=colors, ax=ax7)
            ax7.set_title('Path Length', fontweight='bold')
            ax7.set_ylabel('Path Length (rad)')

        ax8 = plt.subplot(3, 4, 8)
        if len(self.successful_df) > 0:
            sns.boxplot(data=self.successful_df, x='algorithm', y='max_velocity', 
                       palette=colors, ax=ax8)
            ax8.set_title('Maximum Velocity', fontweight='bold')
            ax8.set_ylabel('Max Velocity (rad/s)')

        # 5. Variance Analysis (Consistency)
        ax9 = plt.subplot(3, 4, 9)
        if len(self.successful_df) > 0:
            sns.boxplot(data=self.successful_df, x='algorithm', y='velocity_variance', 
                       palette=colors, ax=ax9)
            ax9.set_title('Velocity Variance\n(Lower = More Consistent)', fontweight='bold')
            ax9.set_ylabel('Velocity Variance')

        ax10 = plt.subplot(3, 4, 10)
        if len(self.successful_df) > 0:
            sns.boxplot(data=self.successful_df, x='algorithm', y='acceleration_variance', 
                       palette=colors, ax=ax10)
            ax10.set_title('Acceleration Variance\n(Lower = More Consistent)', fontweight='bold')
            ax10.set_ylabel('Acceleration Variance')

        # 6. Correlation Analysis
        ax11 = plt.subplot(3, 4, 11)
        if len(self.successful_df) > 0:
            scatter_data = self.successful_df.copy()
            for alg in scatter_data['algorithm'].unique():
                alg_data = scatter_data[scatter_data['algorithm'] == alg]
                ax11.scatter(alg_data['rms_jerk'], alg_data['planning_time_ms'], 
                           c=colors[alg], label=alg, alpha=0.7, s=50)
            ax11.set_xlabel('RMS Jerk (rad/s³)')
            ax11.set_ylabel('Planning Time (ms)')
            ax11.set_title('Smoothness vs Speed Trade-off', fontweight='bold')
            ax11.legend()

        # 7. Algorithm Characteristics Summary
        ax12 = plt.subplot(3, 4, 12)
        ax12.axis('off')
        
        summary_text = "Algorithm Characteristics:\n\n"
        
        if len(self.stomp_data) > 0:
            stomp_avg_jerk = self.stomp_data['rms_jerk'].mean()
            stomp_avg_time = self.stomp_data['planning_time_ms'].mean()
            stomp_success = len(self.stomp_data) / len(self.df[self.df['algorithm'] == 'STOMP']) * 100
            summary_text += f"STOMP (Stochastic):\n"
            summary_text += f"• RMS Jerk: {stomp_avg_jerk:.3f}\n"
            summary_text += f"• Avg Time: {stomp_avg_time:.1f}ms\n"
            summary_text += f"• Success: {stomp_success:.1f}%\n\n"
        
        if len(self.hauser_data) > 0:
            hauser_avg_jerk = self.hauser_data['rms_jerk'].mean()
            hauser_avg_time = self.hauser_data['planning_time_ms'].mean()
            hauser_success = len(self.hauser_data) / len(self.df[self.df['algorithm'] == 'Hauser']) * 100
            summary_text += f"Hauser (Parabolic Ramps):\n"
            summary_text += f"• RMS Jerk: {hauser_avg_jerk:.3f}\n"
            summary_text += f"• Avg Time: {hauser_avg_time:.1f}ms\n"
            summary_text += f"• Success: {hauser_success:.1f}%\n\n"
            
            if len(self.stomp_data) > 0:
                if hauser_avg_jerk < stomp_avg_jerk:
                    summary_text += "✓ Hauser produces smoother trajectories\n"
                if hauser_avg_time > stomp_avg_time:
                    summary_text += "⚠ Hauser takes longer to plan\n"
        
        ax12.text(0.05, 0.95, summary_text, transform=ax12.transAxes, fontsize=11,
                 verticalalignment='top', fontfamily='monospace',
                 bbox=dict(boxstyle='round', facecolor='lightgray', alpha=0.8))

        plt.suptitle('STOMP vs Hauser Trajectory Planning - Comprehensive Analysis\nBased on Real Kinematic Metrics', 
                    fontsize=18, fontweight='bold', y=0.98)
        plt.tight_layout()
        return fig

    def create_detailed_kinematic_analysis(self):
        """Create detailed kinematic analysis plots."""
        fig, axes = plt.subplots(2, 3, figsize=(18, 12))
        colors = {'STOMP': '#1f77b4', 'Hauser': '#ff7f0e'}
        
        if len(self.successful_df) == 0:
            plt.suptitle('No successful trajectories for kinematic analysis')
            return fig

        # Velocity Analysis
        sns.violinplot(data=self.successful_df, x='algorithm', y='max_velocity', 
                      palette=colors, ax=axes[0,0])
        axes[0,0].set_title('Maximum Velocity Distribution')
        axes[0,0].set_ylabel('Max Velocity (rad/s)')

        # Acceleration Analysis
        sns.violinplot(data=self.successful_df, x='algorithm', y='max_acceleration', 
                      palette=colors, ax=axes[0,1])
        axes[0,1].set_title('Maximum Acceleration Distribution')
        axes[0,1].set_ylabel('Max Acceleration (rad/s²)')

        # Jerk Analysis (Most Important)
        sns.violinplot(data=self.successful_df, x='algorithm', y='max_jerk', 
                      palette=colors, ax=axes[0,2])
        axes[0,2].set_title('Maximum Jerk Distribution')
        axes[0,2].set_ylabel('Max Jerk (rad/s³)')

        # RMS Jerk (Smoothness Indicator)
        sns.violinplot(data=self.successful_df, x='algorithm', y='rms_jerk', 
                      palette=colors, ax=axes[1,0])
        axes[1,0].set_title('RMS Jerk Distribution\n(Key Smoothness Metric)')
        axes[1,0].set_ylabel('RMS Jerk (rad/s³)')

        # Consistency Analysis
        consistency_data = self.successful_df.copy()
        consistency_data['consistency_metric'] = 1.0 / (1.0 + consistency_data['velocity_variance'] + 
                                                       consistency_data['acceleration_variance'])
        sns.violinplot(data=consistency_data, x='algorithm', y='consistency_metric', 
                      palette=colors, ax=axes[1,1])
        axes[1,1].set_title('Motion Consistency\n(Higher = More Consistent)')
        axes[1,1].set_ylabel('Consistency Score')

        # Energy Efficiency
        if 'energy_estimate' in self.successful_df.columns:
            sns.violinplot(data=self.successful_df, x='algorithm', y='energy_estimate', 
                          palette=colors, ax=axes[1,2])
            axes[1,2].set_title('Energy Estimate\n(Lower = More Efficient)')
            axes[1,2].set_ylabel('Energy Estimate')
        else:
            axes[1,2].text(0.5, 0.5, 'Energy data\nnot available', 
                          ha='center', va='center', transform=axes[1,2].transAxes)

        plt.suptitle('Detailed Kinematic Analysis - STOMP vs Hauser', fontsize=16, fontweight='bold')
        plt.tight_layout()
        return fig

    def save_all_plots(self, output_dir='plots'):
        """Generate and save all analysis plots."""
        output_path = Path(output_dir)
        output_path.mkdir(exist_ok=True)
        
        print(f"\nGenerating analysis plots...")
        
        # Main dashboard
        print("  Creating performance dashboard...")
        dashboard_fig = self.create_performance_dashboard()
        dashboard_fig.savefig(output_path / 'comprehensive_performance_dashboard.png', 
                            dpi=300, bbox_inches='tight')
        dashboard_fig.savefig(output_path / 'comprehensive_performance_dashboard.pdf', 
                            bbox_inches='tight')
        
        # Kinematic analysis
        print("  Creating kinematic analysis...")
        kinematic_fig = self.create_detailed_kinematic_analysis()
        kinematic_fig.savefig(output_path / 'detailed_kinematic_analysis.png', 
                            dpi=300, bbox_inches='tight')
        kinematic_fig.savefig(output_path / 'detailed_kinematic_analysis.pdf', 
                            bbox_inches='tight')
        
        plt.close('all')
        print(f"✅ All plots saved to {output_path}/")

def main():
    """Main analysis function."""
    # Default input file
    input_file = 'stomp_hauser_improved_results.csv'
    
    if len(sys.argv) > 1:
        input_file = sys.argv[1]
    
    print("="*80)
    print("IMPROVED STOMP vs HAUSER TRAJECTORY ANALYSIS")
    print("Real Metrics with Statistical Significance Testing")
    print("="*80)
    
    try:
        # Initialize analyzer
        analyzer = TrajectoryAnalyzer(input_file)
        
        # Generate comprehensive analysis
        analyzer.generate_comprehensive_summary()
        
        # Create and save visualizations
        analyzer.save_all_plots()
        
        print("\n" + "="*80)
        print("ANALYSIS COMPLETE!")
        print("="*80)
        print("Key Findings:")
        
        if len(analyzer.stomp_data) > 0 and len(analyzer.hauser_data) > 0:
            stomp_jerk = analyzer.stomp_data['rms_jerk'].mean()
            hauser_jerk = analyzer.hauser_data['rms_jerk'].mean()
            
            print(f"• RMS Jerk: STOMP={stomp_jerk:.3f}, Hauser={hauser_jerk:.3f}")
            if hauser_jerk < stomp_jerk:
                print("  → Hauser produces smoother trajectories")
            else:
                print("  → STOMP produces smoother trajectories")
                
            stomp_time = analyzer.stomp_data['planning_time_ms'].mean()
            hauser_time = analyzer.hauser_data['planning_time_ms'].mean()
            
            print(f"• Planning Time: STOMP={stomp_time:.1f}ms, Hauser={hauser_time:.1f}ms")
            if stomp_time < hauser_time:
                print("  → STOMP is faster")
            else:
                print("  → Hauser is faster")
        
        print(f"\nFiles generated:")
        print(f"  • comprehensive_performance_dashboard.png/pdf")
        print(f"  • detailed_kinematic_analysis.png/pdf")
        print(f"\nRun with real trajectory data for meaningful results!")
        
    except Exception as e:
        print(f"❌ Analysis failed: {e}")
        import traceback
        traceback.print_exc()
        sys.exit(1)

if __name__ == "__main__":
    main()