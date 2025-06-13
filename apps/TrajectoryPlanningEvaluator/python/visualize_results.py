#!/usr/bin/env python3
"""
Trajectory Planning Evaluation Visualizer

Generates comprehensive boxplot PDFs for trajectory planning algorithm evaluation results.
Each metric gets its own PDF with detailed statistical analysis.
"""

import pandas as pd
import numpy as np
import matplotlib
matplotlib.use('Agg')  # Use non-interactive backend for PDF generation
import matplotlib.pyplot as plt
import seaborn as sns
from matplotlib.backends.backend_pdf import PdfPages
from pathlib import Path
import sys
import argparse
from datetime import datetime
import warnings
warnings.filterwarnings('ignore')

class TrajectoryPlanningVisualizer:
    """Comprehensive visualization system for trajectory planning evaluation results."""
    
    def __init__(self, data_file: str, output_dir: str = "results"):
        """Initialize visualizer with data file."""
        self.data_file = Path(data_file)
        self.output_dir = Path(output_dir)
        self.output_dir.mkdir(exist_ok=True)
        
        # Algorithm mapping for better labels (define before loading data)
        self.algorithm_names = {
            0: 'STOMP',
            1: 'Hauser RRT',
            2: 'Hauser RRT*', 
            3: 'Hauser Informed RRT*',
            4: 'Hauser Bi-RRT'
        }
        
        # Load and preprocess data
        self.data = self.load_and_preprocess_data()
        
        # Color scheme for consistent visualization
        self.colors = {
            'STOMP': '#1f77b4',
            'Hauser RRT': '#ff7f0e',
            'Hauser RRT*': '#2ca02c',
            'Hauser Informed RRT*': '#d62728',
            'Hauser Bi-RRT': '#9467bd'
        }
        
        print(f"📊 Loaded {len(self.data)} evaluation results")
        print(f"📋 Algorithms: {list(self.data['algorithm_name'].unique())}")
        print(f"📁 Output directory: {self.output_dir}")
    
    def load_and_preprocess_data(self) -> pd.DataFrame:
        """Load CSV data and add computed columns."""
        try:
            df = pd.read_csv(self.data_file)
            
            # Add algorithm names
            df['algorithm_name'] = df['algorithm'].map(self.algorithm_names)
            
            # Add success rate for groupings
            df['success_rate'] = df.groupby(['algorithm', 'config'])['success'].transform('mean')
            
            # Add efficiency metrics
            df['time_per_iteration'] = df['planning_time_ms'] / df['iterations_used']
            df['time_per_iteration'] = df['time_per_iteration'].replace([np.inf, -np.inf], np.nan)
            
            # Add configuration complexity score
            df['config_complexity'] = df.groupby('config').cumcount() + 1
            
            return df
            
        except Exception as e:
            print(f"❌ Error loading data: {e}")
            sys.exit(1)
    
    def create_metric_boxplot_pdf(self, metric: str, title: str, ylabel: str, 
                                 filename: str, successful_only: bool = False):
        """Create comprehensive boxplot PDF for a specific metric."""
        
        print(f"📈 Generating {title} boxplot PDF...")
        
        # Filter data if needed
        plot_data = self.data.copy()
        if successful_only:
            plot_data = plot_data[plot_data['success'] == 1]
            if len(plot_data) == 0:
                print(f"⚠️  No successful trials for {metric}, skipping...")
                return
        
        # Remove invalid data
        plot_data = plot_data.dropna(subset=[metric])
        plot_data = plot_data[plot_data[metric] > 0]
        
        if len(plot_data) == 0:
            print(f"⚠️  No valid data for {metric}, skipping...")
            return
        
        pdf_path = self.output_dir / f"{filename}.pdf"
        
        with PdfPages(pdf_path) as pdf:
            # Page 1: Overall algorithm comparison
            self._create_algorithm_comparison_page(pdf, plot_data, metric, title, ylabel)
            
            # Page 2: Configuration parameter analysis
            self._create_configuration_analysis_page(pdf, plot_data, metric, title, ylabel)
            
            # Page 3: Success rate correlation
            self._create_success_correlation_page(pdf, plot_data, metric, title, ylabel)
            
            # Page 4: Statistical summary
            self._create_statistical_summary_page(pdf, plot_data, metric, title)
        
        print(f"✅ Saved {title} analysis to: {pdf_path}")
    
    def _create_algorithm_comparison_page(self, pdf, data, metric, title, ylabel):
        """Create algorithm comparison boxplot page."""
        fig, axes = plt.subplots(2, 2, figsize=(16, 12))
        fig.suptitle(f'{title} - Algorithm Comparison Analysis', fontsize=16, fontweight='bold')
        
        # Main boxplot
        ax1 = axes[0, 0]
        algorithms = sorted(data['algorithm_name'].unique())
        metric_data = [data[data['algorithm_name'] == alg][metric].values for alg in algorithms]
        
        bp = ax1.boxplot(metric_data, labels=algorithms, patch_artist=True, showmeans=True)
        for patch, alg in zip(bp['boxes'], algorithms):
            patch.set_facecolor(self.colors.get(alg, '#cccccc'))
            patch.set_alpha(0.7)
        
        ax1.set_title('Overall Algorithm Performance', fontweight='bold')
        ax1.set_ylabel(ylabel)
        ax1.tick_params(axis='x', rotation=45)
        ax1.grid(True, alpha=0.3)
        
        # Violin plot for distribution shape
        ax2 = axes[0, 1]
        data_pivot = data.pivot_table(values=metric, index=['trial_id', 'pose_id'], 
                                     columns='algorithm_name', aggfunc='first')
        data_pivot.boxplot(ax=ax2, rot=45)
        ax2.set_title('Distribution Shapes', fontweight='bold')
        ax2.set_ylabel(ylabel)
        ax2.grid(True, alpha=0.3)
        
        # Success rate vs metric scatter
        ax3 = axes[1, 0]
        for alg in algorithms:
            alg_data = data[data['algorithm_name'] == alg]
            grouped = alg_data.groupby('config').agg({
                metric: 'mean',
                'success_rate': 'first'
            }).reset_index()
            
            ax3.scatter(grouped['success_rate'], grouped[metric], 
                       label=alg, color=self.colors.get(alg, '#cccccc'), alpha=0.7, s=60)
        
        ax3.set_xlabel('Success Rate')
        ax3.set_ylabel(ylabel)
        ax3.set_title('Success Rate vs Performance', fontweight='bold')
        ax3.legend()
        ax3.grid(True, alpha=0.3)
        
        # Algorithm ranking
        ax4 = axes[1, 1]
        summary_stats = data.groupby('algorithm_name')[metric].agg(['mean', 'median', 'std']).round(2)
        summary_stats = summary_stats.sort_values('median')
        
        bars = ax4.bar(range(len(summary_stats)), summary_stats['median'], 
                      color=[self.colors.get(alg, '#cccccc') for alg in summary_stats.index],
                      alpha=0.7)
        ax4.set_xticks(range(len(summary_stats)))
        ax4.set_xticklabels(summary_stats.index, rotation=45)
        ax4.set_ylabel(ylabel)
        ax4.set_title('Median Performance Ranking', fontweight='bold')
        ax4.grid(True, alpha=0.3, axis='y')
        
        # Add value labels on bars
        for bar, value in zip(bars, summary_stats['median']):
            height = bar.get_height()
            ax4.text(bar.get_x() + bar.get_width()/2., height,
                    f'{value:.1f}', ha='center', va='bottom', fontweight='bold')
        
        plt.tight_layout()
        pdf.savefig(fig, bbox_inches='tight')
        plt.close(fig)
    
    def _create_configuration_analysis_page(self, pdf, data, metric, title, ylabel):
        """Create configuration parameter analysis page."""
        fig, axes = plt.subplots(2, 2, figsize=(16, 12))
        fig.suptitle(f'{title} - Configuration Parameter Analysis', fontsize=16, fontweight='bold')
        
        # STOMP configurations
        ax1 = axes[0, 0]
        stomp_data = data[data['algorithm_name'] == 'STOMP']
        if len(stomp_data) > 0:
            stomp_configs = stomp_data['config'].unique()[:10]  # Limit to first 10 for readability
            stomp_subset = stomp_data[stomp_data['config'].isin(stomp_configs)]
            
            config_data = [stomp_subset[stomp_subset['config'] == config][metric].values 
                          for config in stomp_configs]
            
            bp = ax1.boxplot(config_data, labels=[c[:15] + '...' if len(c) > 15 else c for c in stomp_configs], 
                            patch_artist=True)
            for patch in bp['boxes']:
                patch.set_facecolor(self.colors['STOMP'])
                patch.set_alpha(0.7)
            
            ax1.set_title('STOMP Configuration Variants', fontweight='bold')
            ax1.set_ylabel(ylabel)
            ax1.tick_params(axis='x', rotation=90)
            ax1.grid(True, alpha=0.3)
        else:
            ax1.text(0.5, 0.5, 'No STOMP data', ha='center', va='center', transform=ax1.transAxes)
            ax1.set_title('STOMP Configuration Variants', fontweight='bold')
        
        # Hauser configurations
        ax2 = axes[0, 1]
        hauser_data = data[data['algorithm_name'].str.contains('Hauser', na=False)]
        if len(hauser_data) > 0:
            hauser_configs = hauser_data['config'].unique()[:10]  # Limit for readability
            hauser_subset = hauser_data[hauser_data['config'].isin(hauser_configs)]
            
            config_data = [hauser_subset[hauser_subset['config'] == config][metric].values 
                          for config in hauser_configs]
            
            bp = ax2.boxplot(config_data, labels=[c[:15] + '...' if len(c) > 15 else c for c in hauser_configs], 
                            patch_artist=True)
            for patch in bp['boxes']:
                patch.set_facecolor('#ff7f0e')
                patch.set_alpha(0.7)
            
            ax2.set_title('Hauser Configuration Variants', fontweight='bold')
            ax2.set_ylabel(ylabel)
            ax2.tick_params(axis='x', rotation=90)
            ax2.grid(True, alpha=0.3)
        else:
            ax2.text(0.5, 0.5, 'No Hauser data', ha='center', va='center', transform=ax2.transAxes)
            ax2.set_title('Hauser Configuration Variants', fontweight='bold')
        
        # Parameter correlation heatmap
        ax3 = axes[1, 0]
        # Create correlation matrix for numeric columns
        numeric_cols = data.select_dtypes(include=[np.number]).columns
        correlation_cols = [col for col in numeric_cols if col in [metric, 'planning_time_ms', 'success_rate', 'iterations_used']]
        
        if len(correlation_cols) > 1:
            corr_data = data[correlation_cols].corr()
            im = ax3.imshow(corr_data, cmap='coolwarm', aspect='auto', vmin=-1, vmax=1)
            ax3.set_xticks(range(len(correlation_cols)))
            ax3.set_yticks(range(len(correlation_cols)))
            ax3.set_xticklabels(correlation_cols, rotation=45)
            ax3.set_yticklabels(correlation_cols)
            
            # Add correlation values
            for i in range(len(correlation_cols)):
                for j in range(len(correlation_cols)):
                    ax3.text(j, i, f'{corr_data.iloc[i, j]:.2f}', 
                            ha='center', va='center', fontweight='bold')
            
            plt.colorbar(im, ax=ax3)
        
        ax3.set_title('Parameter Correlation Matrix', fontweight='bold')
        
        # Performance variance by pose
        ax4 = axes[1, 1]
        pose_variance = data.groupby('pose_id')[metric].agg(['mean', 'std']).reset_index()
        
        ax4.errorbar(pose_variance['pose_id'], pose_variance['mean'], 
                    yerr=pose_variance['std'], fmt='o-', capsize=5, capthick=2)
        ax4.set_xlabel('Pose ID')
        ax4.set_ylabel(ylabel)
        ax4.set_title('Performance Variance by Test Pose', fontweight='bold')
        ax4.grid(True, alpha=0.3)
        
        plt.tight_layout()
        pdf.savefig(fig, bbox_inches='tight')
        plt.close(fig)
    
    def _create_success_correlation_page(self, pdf, data, metric, title, ylabel):
        """Create success rate correlation analysis page."""
        fig, axes = plt.subplots(2, 2, figsize=(16, 12))
        fig.suptitle(f'{title} - Success Rate Correlation Analysis', fontsize=16, fontweight='bold')
        
        # Success vs Failed performance comparison
        ax1 = axes[0, 0]
        success_data = data[data['success'] == 1][metric].values
        failed_data = data[data['success'] == 0][metric].values
        
        if len(success_data) > 0 and len(failed_data) > 0:
            bp = ax1.boxplot([success_data, failed_data], 
                           labels=['Successful', 'Failed'], patch_artist=True)
            bp['boxes'][0].set_facecolor('#2ca02c')
            bp['boxes'][1].set_facecolor('#d62728')
            for patch in bp['boxes']:
                patch.set_alpha(0.7)
        
        ax1.set_title('Successful vs Failed Trials', fontweight='bold')
        ax1.set_ylabel(ylabel)
        ax1.grid(True, alpha=0.3)
        
        # Algorithm success rates
        ax2 = axes[0, 1]
        success_rates = data.groupby('algorithm_name')['success'].mean()
        bars = ax2.bar(range(len(success_rates)), success_rates.values * 100,
                      color=[self.colors.get(alg, '#cccccc') for alg in success_rates.index],
                      alpha=0.7)
        
        ax2.set_xticks(range(len(success_rates)))
        ax2.set_xticklabels(success_rates.index, rotation=45)
        ax2.set_ylabel('Success Rate (%)')
        ax2.set_title('Success Rate by Algorithm', fontweight='bold')
        ax2.set_ylim(0, 100)
        ax2.grid(True, alpha=0.3, axis='y')
        
        # Add percentage labels
        for bar, rate in zip(bars, success_rates.values):
            height = bar.get_height()
            ax2.text(bar.get_x() + bar.get_width()/2., height + 1,
                    f'{rate*100:.1f}%', ha='center', va='bottom', fontweight='bold')
        
        # Performance vs success rate scatter (by configuration)
        ax3 = axes[1, 0]
        config_stats = data.groupby(['algorithm_name', 'config']).agg({
            metric: 'mean',
            'success': 'mean'
        }).reset_index()
        
        for alg in data['algorithm_name'].unique():
            alg_data = config_stats[config_stats['algorithm_name'] == alg]
            ax3.scatter(alg_data['success'], alg_data[metric], 
                       label=alg, color=self.colors.get(alg, '#cccccc'), alpha=0.7, s=60)
        
        ax3.set_xlabel('Success Rate')
        ax3.set_ylabel(ylabel)
        ax3.set_title('Performance vs Success Rate by Configuration', fontweight='bold')
        ax3.legend()
        ax3.grid(True, alpha=0.3)
        
        # Trial consistency analysis
        ax4 = axes[1, 1]
        trial_consistency = data.groupby(['algorithm_name', 'pose_id'])[metric].std().reset_index()
        
        for alg in data['algorithm_name'].unique():
            alg_data = trial_consistency[trial_consistency['algorithm_name'] == alg]
            ax4.scatter(alg_data['pose_id'], alg_data[metric], 
                       label=alg, color=self.colors.get(alg, '#cccccc'), alpha=0.7, s=60)
        
        ax4.set_xlabel('Pose ID')
        ax4.set_ylabel(f'{ylabel} Standard Deviation')
        ax4.set_title('Trial Consistency by Pose', fontweight='bold')
        ax4.legend()
        ax4.grid(True, alpha=0.3)
        
        plt.tight_layout()
        pdf.savefig(fig, bbox_inches='tight')
        plt.close(fig)
    
    def _create_statistical_summary_page(self, pdf, data, metric, title):
        """Create statistical summary page with tables and key insights."""
        fig = plt.figure(figsize=(16, 12))
        fig.suptitle(f'{title} - Statistical Summary & Insights', fontsize=16, fontweight='bold')
        
        # Create text summary
        summary_text = self._generate_statistical_summary(data, metric)
        
        ax = fig.add_subplot(111)
        ax.text(0.05, 0.95, summary_text, transform=ax.transAxes, fontsize=12,
               verticalalignment='top', fontfamily='monospace',
               bbox=dict(boxstyle="round,pad=0.5", facecolor="lightgray", alpha=0.8))
        ax.axis('off')
        
        pdf.savefig(fig, bbox_inches='tight')
        plt.close(fig)
    
    def _generate_statistical_summary(self, data, metric):
        """Generate comprehensive statistical summary text."""
        summary = f"STATISTICAL SUMMARY: {metric.upper()}\n"
        summary += "=" * 60 + "\n\n"
        
        # Overall statistics
        summary += "OVERALL STATISTICS:\n"
        summary += f"Total Evaluations: {len(data):,}\n"
        summary += f"Successful Trials: {data['success'].sum():,} ({data['success'].mean()*100:.1f}%)\n"
        summary += f"Mean {metric}: {data[metric].mean():.3f}\n"
        summary += f"Median {metric}: {data[metric].median():.3f}\n"
        summary += f"Std Dev {metric}: {data[metric].std():.3f}\n\n"
        
        # Algorithm comparison
        summary += "ALGORITHM PERFORMANCE:\n"
        alg_stats = data.groupby('algorithm_name')[metric].agg(['count', 'mean', 'median', 'std']).round(3)
        alg_success = data.groupby('algorithm_name')['success'].mean().round(3)
        
        for alg in alg_stats.index:
            summary += f"{alg}:\n"
            summary += f"  Trials: {alg_stats.loc[alg, 'count']}\n"
            summary += f"  Mean: {alg_stats.loc[alg, 'mean']:.3f}\n"
            summary += f"  Median: {alg_stats.loc[alg, 'median']:.3f}\n"
            summary += f"  Success Rate: {alg_success[alg]*100:.1f}%\n\n"
        
        # Best performers
        summary += "BEST PERFORMERS:\n"
        best_mean = alg_stats['mean'].idxmin()
        best_median = alg_stats['median'].idxmin()
        best_success = alg_success.idxmax()
        
        summary += f"Lowest Mean {metric}: {best_mean}\n"
        summary += f"Lowest Median {metric}: {best_median}\n"
        summary += f"Highest Success Rate: {best_success}\n\n"
        
        # Recommendations
        summary += "RECOMMENDATIONS:\n"
        if best_mean == best_median:
            summary += f"• {best_mean} shows consistently best performance\n"
        else:
            summary += f"• {best_mean} has best average performance\n"
            summary += f"• {best_median} has best typical performance\n"
        
        summary += f"• {best_success} has highest reliability\n"
        
        # Configuration insights
        config_variance = data.groupby('algorithm_name')[metric].std()
        most_consistent = config_variance.idxmin()
        summary += f"• {most_consistent} shows most consistent performance\n"
        
        return summary
    
    def generate_all_metric_pdfs(self):
        """Generate comprehensive PDF reports for all metrics."""
        print("🚀 Generating comprehensive trajectory planning analysis PDFs...")
        print("=" * 60)
        
        # Define metrics to analyze
        metrics_config = [
            {
                'metric': 'planning_time_ms',
                'title': 'Planning Time Analysis',
                'ylabel': 'Planning Time (ms)',
                'filename': 'planning_time_analysis',
                'successful_only': False
            },
            {
                'metric': 'execution_time_ms',
                'title': 'Execution Time Analysis', 
                'ylabel': 'Execution Time (ms)',
                'filename': 'execution_time_analysis',
                'successful_only': True
            },
            {
                'metric': 'iterations_used',
                'title': 'Iterations Required Analysis',
                'ylabel': 'Number of Iterations',
                'filename': 'iterations_analysis',
                'successful_only': False
            },
            {
                'metric': 'time_per_iteration',
                'title': 'Computation Efficiency Analysis',
                'ylabel': 'Time per Iteration (ms)',
                'filename': 'efficiency_analysis',
                'successful_only': False
            },
            {
                'metric': 'trajectory_length',
                'title': 'Trajectory Length Analysis',
                'ylabel': 'Trajectory Length',
                'filename': 'trajectory_length_analysis',
                'successful_only': True
            },
            {
                'metric': 'path_quality',
                'title': 'Path Quality Analysis',
                'ylabel': 'Path Quality Score',
                'filename': 'path_quality_analysis',
                'successful_only': True
            }
        ]
        
        # Generate PDF for each metric
        for config in metrics_config:
            self.create_metric_boxplot_pdf(**config)
        
        print("\n" + "=" * 60)
        print("✅ All trajectory planning analysis PDFs generated!")
        print(f"📁 Check the {self.output_dir} directory for PDF files")
        print("📊 Each PDF contains 4 pages of detailed analysis:")
        print("   • Page 1: Algorithm comparison")
        print("   • Page 2: Configuration analysis") 
        print("   • Page 3: Success correlation")
        print("   • Page 4: Statistical summary")
    
    def create_algorithm_specific_pdfs(self):
        """Create comprehensive PDFs for each algorithm type."""
        
        # Create subdirectory for algorithm-specific analyses
        algorithm_dir = self.output_dir / "algorithm_analysis"
        algorithm_dir.mkdir(exist_ok=True)
        
        print("\n📋 Creating algorithm-specific comprehensive analyses...")
        
        for algorithm_name in self.data['algorithm_name'].unique():
            if pd.isna(algorithm_name):
                continue
                
            print(f"📈 Generating {algorithm_name} comprehensive analysis...")
            
            # Filter data for this algorithm
            alg_data = self.data[self.data['algorithm_name'] == algorithm_name].copy()
            
            if len(alg_data) == 0:
                print(f"⚠️  No data for {algorithm_name}, skipping...")
                continue
            
            filename = algorithm_dir / f"{algorithm_name.replace(' ', '_').lower()}_comprehensive_analysis.pdf"
            
            with PdfPages(filename) as pdf:
                # Page 1: Overview Statistics
                fig, ((ax1, ax2), (ax3, ax4)) = plt.subplots(2, 2, figsize=(16, 12))
                fig.suptitle(f'{algorithm_name} - Comprehensive Analysis Overview', fontsize=16, fontweight='bold')
                
                # Success rate by configuration
                success_by_config = alg_data.groupby('config')['success'].mean().sort_values(ascending=False)
                ax1.bar(range(len(success_by_config)), success_by_config.values, 
                       color=self.colors.get(algorithm_name, '#1f77b4'), alpha=0.7)
                ax1.set_title('Success Rate by Configuration')
                ax1.set_ylabel('Success Rate')
                ax1.set_xlabel('Configuration')
                ax1.set_xticks(range(len(success_by_config)))
                ax1.set_xticklabels([f"Config {i+1}" for i in range(len(success_by_config))], rotation=45)
                ax1.grid(True, alpha=0.3)
                
                # Planning time distribution
                successful_data = alg_data[alg_data['success'] == 1]
                if len(successful_data) > 0:
                    ax2.hist(successful_data['planning_time_ms'], bins=30, 
                            color=self.colors.get(algorithm_name, '#1f77b4'), alpha=0.7)
                    ax2.set_title('Planning Time Distribution (Successful Trials)')
                    ax2.set_xlabel('Planning Time (ms)')
                    ax2.set_ylabel('Frequency')
                    ax2.grid(True, alpha=0.3)
                
                # Iterations vs Planning Time scatter
                if len(successful_data) > 0:
                    ax3.scatter(successful_data['iterations_used'], successful_data['planning_time_ms'], 
                               alpha=0.6, color=self.colors.get(algorithm_name, '#1f77b4'))
                    ax3.set_xlabel('Iterations Used')
                    ax3.set_ylabel('Planning Time (ms)')
                    ax3.set_title('Planning Time vs Iterations')
                    ax3.grid(True, alpha=0.3)
                
                # Configuration performance summary
                config_metrics = alg_data.groupby('config').agg({
                    'success': 'mean',
                    'planning_time_ms': 'mean',
                    'trajectory_length': lambda x: x[x > 0].mean() if len(x[x > 0]) > 0 else 0
                }).reset_index()
                
                if len(config_metrics) > 0:
                    x_pos = range(len(config_metrics))
                    ax4.bar(x_pos, config_metrics['planning_time_ms'], 
                           color=self.colors.get(algorithm_name, '#1f77b4'), alpha=0.7)
                    ax4.set_title('Average Planning Time by Configuration')
                    ax4.set_xlabel('Configuration')
                    ax4.set_ylabel('Average Planning Time (ms)')
                    ax4.set_xticks(x_pos)
                    ax4.set_xticklabels([f"Config {i+1}" for i in x_pos], rotation=45)
                    ax4.grid(True, alpha=0.3)
                
                plt.tight_layout()
                pdf.savefig(fig, bbox_inches='tight')
                plt.close()
                
                # Pages 2-7: Detailed metric analysis for this algorithm
                metrics = {
                    'planning_time_ms': {'title': 'Planning Time', 'ylabel': 'Time (ms)', 'successful_only': False},
                    'execution_time_ms': {'title': 'Execution Time', 'ylabel': 'Time (ms)', 'successful_only': True},
                    'iterations_used': {'title': 'Iterations Used', 'ylabel': 'Iterations', 'successful_only': False},
                    'trajectory_length': {'title': 'Trajectory Length', 'ylabel': 'Length', 'successful_only': True},
                    'path_quality': {'title': 'Path Quality', 'ylabel': 'Quality Score', 'successful_only': True},
                    'joint_smoothness': {'title': 'Joint Smoothness', 'ylabel': 'Smoothness Score', 'successful_only': True}
                }
                
                for metric, info in metrics.items():
                    self._create_algorithm_metric_page(pdf, alg_data, algorithm_name, metric, info)
                
            print(f"✅ Saved {algorithm_name} analysis to: {filename}")
    
    def _create_algorithm_metric_page(self, pdf, alg_data, algorithm_name, metric, info):
        """Create a detailed page for a specific metric within an algorithm."""
        
        # Filter data based on success requirement
        plot_data = alg_data.copy()
        if info['successful_only']:
            plot_data = plot_data[plot_data['success'] == 1]
        
        # Remove invalid data
        plot_data = plot_data.dropna(subset=[metric])
        if metric in ['trajectory_length', 'path_quality', 'joint_smoothness']:
            plot_data = plot_data[plot_data[metric] > 0]
        
        if len(plot_data) == 0:
            return
        
        fig, ((ax1, ax2), (ax3, ax4)) = plt.subplots(2, 2, figsize=(16, 12))
        fig.suptitle(f'{algorithm_name} - {info["title"]} Analysis', fontsize=16, fontweight='bold')
        
        # 1. Box plot by configuration
        configs = plot_data['config'].unique()
        config_data = [plot_data[plot_data['config'] == config][metric].values for config in configs]
        
        if len(config_data) > 0:
            bp = ax1.boxplot(config_data, patch_artist=True)
            for patch in bp['boxes']:
                patch.set_facecolor(self.colors.get(algorithm_name, '#1f77b4'))
                patch.set_alpha(0.7)
            ax1.set_title(f'{info["title"]} by Configuration')
            ax1.set_xlabel('Configuration')
            ax1.set_ylabel(info['ylabel'])
            ax1.set_xticklabels([f"Config {i+1}" for i in range(len(configs))], rotation=45)
            ax1.grid(True, alpha=0.3)
        
        # 2. Distribution histogram
        ax2.hist(plot_data[metric], bins=25, color=self.colors.get(algorithm_name, '#1f77b4'), 
                alpha=0.7, edgecolor='black')
        ax2.set_title(f'{info["title"]} Distribution')
        ax2.set_xlabel(info['ylabel'])
        ax2.set_ylabel('Frequency')
        ax2.grid(True, alpha=0.3)
        
        # Add statistics
        mean_val = plot_data[metric].mean()
        median_val = plot_data[metric].median()
        ax2.axvline(mean_val, color='red', linestyle='--', label=f'Mean: {mean_val:.2f}')
        ax2.axvline(median_val, color='green', linestyle='--', label=f'Median: {median_val:.2f}')
        ax2.legend()
        
        # 3. Performance by trial (showing consistency)
        trial_means = plot_data.groupby('trial_id')[metric].mean()
        ax3.plot(trial_means.index, trial_means.values, 'o-', 
                color=self.colors.get(algorithm_name, '#1f77b4'), alpha=0.7)
        ax3.set_title(f'{info["title"]} Consistency Across Trials')
        ax3.set_xlabel('Trial ID')
        ax3.set_ylabel(f'Average {info["ylabel"]}')
        ax3.grid(True, alpha=0.3)
        
        # 4. Statistical summary table
        ax4.axis('off')
        
        # Calculate statistics by configuration
        config_stats = plot_data.groupby('config')[metric].agg(['count', 'mean', 'std', 'min', 'max']).round(3)
        
        # Create table data
        table_data = []
        for i, (config, stats) in enumerate(config_stats.iterrows()):
            row = [f"Config {i+1}"] + [f"{val:.3f}" if not pd.isna(val) else "N/A" for val in stats.values]
            table_data.append(row)
        
        if table_data:
            table = ax4.table(cellText=table_data,
                            colLabels=['Config', 'Count', 'Mean', 'Std', 'Min', 'Max'],
                            cellLoc='center',
                            loc='center')
            table.auto_set_font_size(False)
            table.set_fontsize(9)
            table.scale(1.2, 1.5)
            ax4.set_title(f'{info["title"]} Statistical Summary by Configuration', pad=20)
        
        plt.tight_layout()
        pdf.savefig(fig, bbox_inches='tight')
        plt.close()

    def create_metric_specific_pdfs(self):
        """Create detailed PDFs for each metric comparing all algorithms."""
        
        # Create subdirectory for metric-specific analyses
        metric_dir = self.output_dir / "metric_analysis"
        metric_dir.mkdir(exist_ok=True)
        
        print("\n📊 Creating metric-specific cross-algorithm analyses...")
        
        metrics = {
            'planning_time_ms': {'title': 'Planning Time', 'ylabel': 'Time (ms)', 'successful_only': False},
            'execution_time_ms': {'title': 'Execution Time', 'ylabel': 'Time (ms)', 'successful_only': True},
            'iterations_used': {'title': 'Iterations Used', 'ylabel': 'Iterations', 'successful_only': False},
            'trajectory_length': {'title': 'Trajectory Length', 'ylabel': 'Length', 'successful_only': True},
            'path_quality': {'title': 'Path Quality', 'ylabel': 'Quality Score', 'successful_only': True},
            'joint_smoothness': {'title': 'Joint Smoothness', 'ylabel': 'Smoothness Score', 'successful_only': True}
        }
        
        for metric, info in metrics.items():
            print(f"📈 Generating cross-algorithm {info['title']} analysis...")
            
            filename = metric_dir / f"{metric}_cross_algorithm_analysis.pdf"
            
            with PdfPages(filename) as pdf:
                # Filter data
                plot_data = self.data.copy()
                if info['successful_only']:
                    plot_data = plot_data[plot_data['success'] == 1]
                
                plot_data = plot_data.dropna(subset=[metric])
                if metric in ['trajectory_length', 'path_quality', 'joint_smoothness']:
                    plot_data = plot_data[plot_data[metric] > 0]
                
                if len(plot_data) == 0:
                    continue
                
                # Page 1: Algorithm comparison
                fig, ((ax1, ax2), (ax3, ax4)) = plt.subplots(2, 2, figsize=(16, 12))
                fig.suptitle(f'{info["title"]} - Cross-Algorithm Analysis', fontsize=16, fontweight='bold')
                
                # 1. Box plot comparison
                algorithms = plot_data['algorithm_name'].unique()
                alg_data = [plot_data[plot_data['algorithm_name'] == alg][metric].values for alg in algorithms]
                
                bp = ax1.boxplot(alg_data, patch_artist=True)
                for i, patch in enumerate(bp['boxes']):
                    alg_name = algorithms[i]
                    patch.set_facecolor(self.colors.get(alg_name, '#1f77b4'))
                    patch.set_alpha(0.7)
                
                ax1.set_title(f'{info["title"]} by Algorithm')
                ax1.set_xlabel('Algorithm')
                ax1.set_ylabel(info['ylabel'])
                ax1.set_xticklabels(algorithms, rotation=45)
                ax1.grid(True, alpha=0.3)
                
                # 2. Mean performance comparison
                alg_means = plot_data.groupby('algorithm_name')[metric].mean().sort_values(ascending=False)
                bars = ax2.bar(range(len(alg_means)), alg_means.values, 
                              color=[self.colors.get(alg, '#1f77b4') for alg in alg_means.index],
                              alpha=0.7)
                ax2.set_title(f'Average {info["title"]} by Algorithm')
                ax2.set_xlabel('Algorithm')
                ax2.set_ylabel(f'Average {info["ylabel"]}')
                ax2.set_xticks(range(len(alg_means)))
                ax2.set_xticklabels(alg_means.index, rotation=45)
                ax2.grid(True, alpha=0.3)
                
                # Add value labels on bars
                for i, bar in enumerate(bars):
                    height = bar.get_height()
                    ax2.text(bar.get_x() + bar.get_width()/2., height,
                            f'{height:.2f}', ha='center', va='bottom')
                
                # 3. Distribution comparison
                for i, alg in enumerate(algorithms):
                    alg_subset = plot_data[plot_data['algorithm_name'] == alg][metric]
                    ax3.hist(alg_subset, bins=20, alpha=0.6, 
                            color=self.colors.get(alg, '#1f77b4'),
                            label=alg, density=True)
                
                ax3.set_title(f'{info["title"]} Distribution Comparison')
                ax3.set_xlabel(info['ylabel'])
                ax3.set_ylabel('Density')
                ax3.legend()
                ax3.grid(True, alpha=0.3)
                
                # 4. Statistical summary
                ax4.axis('off')
                stats_summary = plot_data.groupby('algorithm_name')[metric].agg(['count', 'mean', 'std', 'min', 'max']).round(3)
                
                # Create table
                table_data = []
                for alg in stats_summary.index:
                    row = [alg] + [f"{val:.3f}" for val in stats_summary.loc[alg].values]
                    table_data.append(row)
                
                table = ax4.table(cellText=table_data,
                                colLabels=['Algorithm', 'Count', 'Mean', 'Std', 'Min', 'Max'],
                                cellLoc='center',
                                loc='center')
                table.auto_set_font_size(False)
                table.set_fontsize(10)
                table.scale(1.2, 1.5)
                ax4.set_title(f'{info["title"]} Statistical Summary', pad=20)
                
                plt.tight_layout()
                pdf.savefig(fig, bbox_inches='tight')
                plt.close()
                
            print(f"✅ Saved {info['title']} comparison to: {filename}")

    def create_parameter_analysis_pdfs(self):
        """Create detailed parameter analysis PDFs for each algorithm family."""
        
        # Create subdirectory for parameter analyses
        param_dir = self.output_dir / "parameter_analysis"
        param_dir.mkdir(exist_ok=True)
        
        print("\n🔧 Creating parameter-specific analyses...")
        
        # STOMP parameter analysis
        stomp_data = self.data[self.data['algorithm_name'] == 'STOMP'].copy()
        if len(stomp_data) > 0:
            self._create_stomp_parameter_analysis(stomp_data, param_dir)
        
        # Hauser family parameter analysis
        hauser_data = self.data[self.data['algorithm_name'].str.contains('Hauser', na=False)].copy()
        if len(hauser_data) > 0:
            self._create_hauser_parameter_analysis(hauser_data, param_dir)
    
    def _create_stomp_parameter_analysis(self, stomp_data, param_dir):
        """Create STOMP-specific parameter analysis."""
        
        print("📈 Generating STOMP parameter analysis...")
        
        filename = param_dir / "stomp_parameter_analysis.pdf"
        
        with PdfPages(filename) as pdf:
            # Extract parameter values from config strings
            stomp_data['iterations'] = stomp_data['config'].str.extract(r'iter(\d+)').astype(float)
            stomp_data['noisy_samples'] = stomp_data['config'].str.extract(r'noisy(\d+)').astype(float)
            stomp_data['learning_rate'] = stomp_data['config'].str.extract(r'lr(\d+)').astype(float) / 100.0
            
            # Page 1: Parameter effects on success rate
            fig, ((ax1, ax2), (ax3, ax4)) = plt.subplots(2, 2, figsize=(16, 12))
            fig.suptitle('STOMP Parameter Analysis - Success Rate Effects', fontsize=16, fontweight='bold')
            
            # Success rate vs iterations
            iter_success = stomp_data.groupby('iterations')['success'].mean()
            ax1.plot(iter_success.index, iter_success.values, 'o-', color='#1f77b4', linewidth=2, markersize=8)
            ax1.set_xlabel('Max Iterations')
            ax1.set_ylabel('Success Rate')
            ax1.set_title('Success Rate vs Max Iterations')
            ax1.grid(True, alpha=0.3)
            
            # Success rate vs noisy samples
            noisy_success = stomp_data.groupby('noisy_samples')['success'].mean()
            ax2.plot(noisy_success.index, noisy_success.values, 'o-', color='#ff7f0e', linewidth=2, markersize=8)
            ax2.set_xlabel('Noisy Samples')
            ax2.set_ylabel('Success Rate')
            ax2.set_title('Success Rate vs Noisy Samples')
            ax2.grid(True, alpha=0.3)
            
            # Success rate vs learning rate
            lr_success = stomp_data.groupby('learning_rate')['success'].mean()
            ax3.plot(lr_success.index, lr_success.values, 'o-', color='#2ca02c', linewidth=2, markersize=8)
            ax3.set_xlabel('Learning Rate')
            ax3.set_ylabel('Success Rate')
            ax3.set_title('Success Rate vs Learning Rate')
            ax3.grid(True, alpha=0.3)
            
            # Parameter correlation heatmap
            param_corr = stomp_data[['iterations', 'noisy_samples', 'learning_rate', 'success', 
                                   'planning_time_ms']].corr()
            im = ax4.imshow(param_corr, cmap='coolwarm', aspect='auto', vmin=-1, vmax=1)
            ax4.set_xticks(range(len(param_corr.columns)))
            ax4.set_yticks(range(len(param_corr.columns)))
            ax4.set_xticklabels(param_corr.columns, rotation=45)
            ax4.set_yticklabels(param_corr.columns)
            ax4.set_title('Parameter Correlation Matrix')
            
            # Add correlation values
            for i in range(len(param_corr.columns)):
                for j in range(len(param_corr.columns)):
                    ax4.text(j, i, f'{param_corr.iloc[i, j]:.2f}', 
                            ha='center', va='center', color='black' if abs(param_corr.iloc[i, j]) < 0.5 else 'white')
            
            plt.tight_layout()
            pdf.savefig(fig, bbox_inches='tight')
            plt.close()
            
        print(f"✅ Saved STOMP parameter analysis to: {filename}")
    
    def _create_hauser_parameter_analysis(self, hauser_data, param_dir):
        """Create Hauser family parameter analysis."""
        
        print("📈 Generating Hauser family parameter analysis...")
        
        filename = param_dir / "hauser_parameter_analysis.pdf"
        
        with PdfPages(filename) as pdf:
            # Extract parameter values from config strings
            hauser_data['iterations'] = hauser_data['config'].str.extract(r'iter(\d+)').astype(float)
            hauser_data['step_size'] = hauser_data['config'].str.extract(r'step(\d+)').astype(float) / 100.0
            hauser_data['bias'] = hauser_data['config'].str.extract(r'bias(\d+)').astype(float) / 100.0
            hauser_data['is_informed'] = hauser_data['config'].str.contains('informed')
            hauser_data['is_bidirectional'] = hauser_data['config'].str.contains('bi')
            
            # Page 1: Algorithm variant comparison
            fig, ((ax1, ax2), (ax3, ax4)) = plt.subplots(2, 2, figsize=(16, 12))
            fig.suptitle('Hauser Family Analysis - Algorithm Variants', fontsize=16, fontweight='bold')
            
            # Success rate by variant
            variant_success = hauser_data.groupby('algorithm_name')['success'].mean().sort_values(ascending=False)
            bars = ax1.bar(range(len(variant_success)), variant_success.values,
                          color=[self.colors.get(alg, '#ff7f0e') for alg in variant_success.index],
                          alpha=0.7)
            ax1.set_xlabel('Algorithm Variant')
            ax1.set_ylabel('Success Rate')
            ax1.set_title('Success Rate by Hauser Variant')
            ax1.set_xticks(range(len(variant_success)))
            ax1.set_xticklabels([alg.replace('Hauser ', '') for alg in variant_success.index], rotation=45)
            ax1.grid(True, alpha=0.3)
            
            # Planning time by variant
            successful_hauser = hauser_data[hauser_data['success'] == 1]
            if len(successful_hauser) > 0:
                variant_time = successful_hauser.groupby('algorithm_name')['planning_time_ms'].mean()
                bars = ax2.bar(range(len(variant_time)), variant_time.values,
                              color=[self.colors.get(alg, '#ff7f0e') for alg in variant_time.index],
                              alpha=0.7)
                ax2.set_xlabel('Algorithm Variant')
                ax2.set_ylabel('Average Planning Time (ms)')
                ax2.set_title('Planning Time by Hauser Variant')
                ax2.set_xticks(range(len(variant_time)))
                ax2.set_xticklabels([alg.replace('Hauser ', '') for alg in variant_time.index], rotation=45)
                ax2.grid(True, alpha=0.3)
            
            plt.tight_layout()
            pdf.savefig(fig, bbox_inches='tight')
            plt.close()
            
        print(f"✅ Saved Hauser parameter analysis to: {filename}")

def main():
    """Main function to generate trajectory planning visualization PDFs."""
    parser = argparse.ArgumentParser(description='Generate trajectory planning evaluation PDFs')
    parser.add_argument('data_file', help='CSV file with evaluation results')
    parser.add_argument('--output-dir', default='results', help='Output directory for PDFs')
    
    args = parser.parse_args()
    
    # Validate input file
    if not Path(args.data_file).exists():
        print(f"❌ Data file not found: {args.data_file}")
        sys.exit(1)
    
    try:
        # Create visualizer and generate all PDFs
        visualizer = TrajectoryPlanningVisualizer(args.data_file, args.output_dir)
        visualizer.generate_all_metric_pdfs()
        visualizer.create_algorithm_specific_pdfs()
        visualizer.create_metric_specific_pdfs()
        visualizer.create_parameter_analysis_pdfs()
        
    except Exception as e:
        print(f"❌ Error generating visualizations: {e}")
        sys.exit(1)

if __name__ == "__main__":
    main()
