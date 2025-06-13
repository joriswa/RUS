#!/usr/bin/env python3
"""
Enhanced Trajectory Planning Evaluation Visualizer

Generates detailed PDFs for each algorithm and metric combination.
Creates separate analysis for:
- Each algorithm type (STOMP, Hauser RRT, Hauser RRT*, etc.)
- Each metric (planning time, execution time, iterations, etc.)
- Parameter variations within each algorithm
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

class EnhancedTrajectoryPlanningVisualizer:
    """Enhanced visualization system for detailed algorithm-specific analysis."""
    
    def __init__(self, data_file: str, output_dir: str = "results"):
        """Initialize visualizer with data file."""
        self.data_file = Path(data_file)
        self.output_dir = Path(output_dir)
        self.output_dir.mkdir(exist_ok=True)
        
        # Create subdirectories for organized output
        self.algorithm_dir = self.output_dir / "algorithm_analysis"
        self.metric_dir = self.output_dir / "metric_analysis"
        self.parameter_dir = self.output_dir / "parameter_analysis"
        
        for dir_path in [self.algorithm_dir, self.metric_dir, self.parameter_dir]:
            dir_path.mkdir(exist_ok=True)
        
        # Algorithm mapping for better labels
        self.algorithm_names = {
            0: 'STOMP',
            1: 'Hauser RRT',
            2: 'Hauser RRT*', 
            3: 'Hauser Informed RRT*',
            4: 'Hauser Bi-RRT'
        }
        
        # Load and preprocess data
        self.data = self.load_and_preprocess_data()
        
        # Define metrics to analyze
        self.metrics = {
            'planning_time_ms': {'title': 'Planning Time', 'ylabel': 'Time (ms)', 'successful_only': False, 'lower_better': True},
            'execution_time_ms': {'title': 'Execution Time', 'ylabel': 'Time (ms)', 'successful_only': True, 'lower_better': True},
            'iterations_used': {'title': 'Iterations Used', 'ylabel': 'Iterations', 'successful_only': False, 'lower_better': True},
            'trajectory_length': {'title': 'Trajectory Length', 'ylabel': 'Length', 'successful_only': True, 'lower_better': True},
            'path_quality': {'title': 'Path Quality', 'ylabel': 'Quality Score', 'successful_only': True, 'lower_better': False},
            'joint_smoothness': {'title': 'Joint Smoothness', 'ylabel': 'Smoothness Score', 'successful_only': True, 'lower_better': False}
        }
        
        # Color schemes for algorithms
        self.algorithm_colors = {
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
            
            return df
            
        except Exception as e:
            print(f"❌ Error loading data: {e}")
            sys.exit(1)
    
    def create_algorithm_specific_pdf(self, algorithm_name: str):
        """Create comprehensive PDF for a specific algorithm across all metrics."""
        
        print(f"📈 Generating {algorithm_name} comprehensive analysis...")
        
        # Filter data for this algorithm
        alg_data = self.data[self.data['algorithm_name'] == algorithm_name].copy()
        
        if len(alg_data) == 0:
            print(f"⚠️  No data for {algorithm_name}, skipping...")
            return
        
        filename = self.algorithm_dir / f"{algorithm_name.replace(' ', '_').lower()}_comprehensive_analysis.pdf"
        
        with PdfPages(filename) as pdf:
            # Page 1: Overview Statistics
            fig, ((ax1, ax2), (ax3, ax4)) = plt.subplots(2, 2, figsize=(16, 12))
            fig.suptitle(f'{algorithm_name} - Comprehensive Analysis Overview', fontsize=16, fontweight='bold')
            
            # Success rate by configuration
            success_by_config = alg_data.groupby('config')['success'].mean().sort_values(ascending=False)
            ax1.bar(range(len(success_by_config)), success_by_config.values, 
                   color=self.algorithm_colors.get(algorithm_name, '#1f77b4'), alpha=0.7)
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
                        color=self.algorithm_colors.get(algorithm_name, '#1f77b4'), alpha=0.7)
                ax2.set_title('Planning Time Distribution (Successful Trials)')
                ax2.set_xlabel('Planning Time (ms)')
                ax2.set_ylabel('Frequency')
                ax2.grid(True, alpha=0.3)
            
            # Iterations vs Planning Time scatter
            if len(successful_data) > 0:
                ax3.scatter(successful_data['iterations_used'], successful_data['planning_time_ms'], 
                           alpha=0.6, color=self.algorithm_colors.get(algorithm_name, '#1f77b4'))
                ax3.set_xlabel('Iterations Used')
                ax3.set_ylabel('Planning Time (ms)')
                ax3.set_title('Planning Time vs Iterations')
                ax3.grid(True, alpha=0.3)
            
            # Configuration performance comparison
            config_metrics = alg_data.groupby('config').agg({
                'success': 'mean',
                'planning_time_ms': 'mean',
                'trajectory_length': lambda x: x[x > 0].mean() if len(x[x > 0]) > 0 else 0
            }).reset_index()
            
            if len(config_metrics) > 0:
                x_pos = range(len(config_metrics))
                ax4.bar(x_pos, config_metrics['planning_time_ms'], 
                       color=self.algorithm_colors.get(algorithm_name, '#1f77b4'), alpha=0.7)
                ax4.set_title('Average Planning Time by Configuration')
                ax4.set_xlabel('Configuration')
                ax4.set_ylabel('Average Planning Time (ms)')
                ax4.set_xticks(x_pos)
                ax4.set_xticklabels([f"Config {i+1}" for i in x_pos], rotation=45)
                ax4.grid(True, alpha=0.3)
            
            plt.tight_layout()
            pdf.savefig(fig, bbox_inches='tight')
            plt.close()
            
            # Page 2-7: Detailed metric analysis
            for metric, info in self.metrics.items():
                self._create_metric_page_for_algorithm(pdf, alg_data, algorithm_name, metric, info)
            
        print(f"✅ Saved {algorithm_name} analysis to: {filename}")
    
    def _create_metric_page_for_algorithm(self, pdf, alg_data, algorithm_name, metric, info):
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
                patch.set_facecolor(self.algorithm_colors.get(algorithm_name, '#1f77b4'))
                patch.set_alpha(0.7)
            ax1.set_title(f'{info["title"]} by Configuration')
            ax1.set_xlabel('Configuration')
            ax1.set_ylabel(info['ylabel'])
            ax1.set_xticklabels([f"Config {i+1}" for i in range(len(configs))], rotation=45)
            ax1.grid(True, alpha=0.3)
        
        # 2. Distribution histogram
        ax2.hist(plot_data[metric], bins=25, color=self.algorithm_colors.get(algorithm_name, '#1f77b4'), 
                alpha=0.7, edgecolor='black')
        ax2.set_title(f'{info["title"]} Distribution')
        ax2.set_xlabel(info['ylabel'])
        ax2.set_ylabel('Frequency')
        ax2.grid(True, alpha=0.3)
        
        # Add statistics
        mean_val = plot_data[metric].mean()
        median_val = plot_data[metric].median()
        std_val = plot_data[metric].std()
        ax2.axvline(mean_val, color='red', linestyle='--', label=f'Mean: {mean_val:.2f}')
        ax2.axvline(median_val, color='green', linestyle='--', label=f'Median: {median_val:.2f}')
        ax2.legend()
        
        # 3. Performance by trial (showing consistency)
        trial_means = plot_data.groupby('trial_id')[metric].mean()
        ax3.plot(trial_means.index, trial_means.values, 'o-', 
                color=self.algorithm_colors.get(algorithm_name, '#1f77b4'), alpha=0.7)
        ax3.set_title(f'{info["title"]} Consistency Across Trials')
        ax3.set_xlabel('Trial ID')
        ax3.set_ylabel(f'Average {info["ylabel"]}')
        ax3.grid(True, alpha=0.3)
        
        # 4. Correlation with success rate
        config_stats = plot_data.groupby('config').agg({
            metric: ['mean', 'std'],
            'success': 'mean'
        }).round(3)
        
        config_stats.columns = ['_'.join(col).strip() for col in config_stats.columns.values]
        config_means = config_stats[f'{metric}_mean'].values
        success_rates = config_stats['success_mean'].values
        
        ax4.scatter(success_rates, config_means, s=100, 
                   color=self.algorithm_colors.get(algorithm_name, '#1f77b4'), alpha=0.7)
        ax4.set_xlabel('Success Rate')
        ax4.set_ylabel(f'Average {info["ylabel"]}')
        ax4.set_title(f'{info["title"]} vs Success Rate')
        ax4.grid(True, alpha=0.3)
        
        # Add correlation coefficient
        if len(success_rates) > 1:
            corr = np.corrcoef(success_rates, config_means)[0, 1]
            ax4.text(0.05, 0.95, f'Correlation: {corr:.3f}', transform=ax4.transAxes, 
                    bbox=dict(boxstyle="round,pad=0.3", facecolor="white", alpha=0.8))
        
        plt.tight_layout()
        pdf.savefig(fig, bbox_inches='tight')
        plt.close()
    
    def create_metric_comparison_pdfs(self):
        """Create PDFs for each metric showing boxplots of all parameter configurations per algorithm."""
        
        for metric, info in self.metrics.items():
            print(f"📈 Generating {info['title']} parameter configuration analysis...")
            
            filename = self.metric_dir / f"{metric}_parameter_configurations_analysis.pdf"
            
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
                
                # Get unique algorithms
                algorithms = sorted(plot_data['algorithm_name'].unique())
                
                # Create a page for each algorithm showing its parameter configurations
                for algorithm in algorithms:
                    alg_data = plot_data[plot_data['algorithm_name'] == algorithm]
                    
                    if len(alg_data) == 0:
                        continue
                    
                    # Get unique configurations for this algorithm
                    configs = sorted(alg_data['config'].unique())
                    
                    if len(configs) <= 1:
                        continue
                    
                    # Create figure with 2x2 layout
                    fig, ((ax1, ax2), (ax3, ax4)) = plt.subplots(2, 2, figsize=(16, 12))
                    fig.suptitle(f'{algorithm}: {info["title"]} Analysis by Parameter Configuration', 
                                fontsize=16, fontweight='bold')
                    
                    # 1. Box plot for all configurations
                    config_data = []
                    config_labels = []
                    for config in configs:
                        config_subset = alg_data[alg_data['config'] == config][metric].values
                        if len(config_subset) > 0:
                            config_data.append(config_subset)
                            config_labels.append(f"Config {config}")
                    
                    if config_data:
                        bp = ax1.boxplot(config_data, patch_artist=True)
                        for patch in bp['boxes']:
                            patch.set_facecolor(self.algorithm_colors.get(algorithm, '#1f77b4'))
                            patch.set_alpha(0.7)
                        
                        ax1.set_title(f'{info["title"]} by Configuration')
                        ax1.set_xlabel('Configuration')
                        ax1.set_ylabel(info['ylabel'])
                        ax1.set_xticklabels(config_labels, rotation=45)
                        ax1.grid(True, alpha=0.3)
                    
                    # 2. Mean values with error bars
                    config_stats = alg_data.groupby('config')[metric].agg(['mean', 'std']).reset_index()
                    if not config_stats.empty:
                        ax2.errorbar(range(len(config_stats)), config_stats['mean'], 
                                   yerr=config_stats['std'], fmt='o-', capsize=5,
                                   color=self.algorithm_colors.get(algorithm, '#1f77b4'))
                        ax2.set_title(f'Mean {info["title"]} with Standard Deviation')
                        ax2.set_xlabel('Configuration')
                        ax2.set_ylabel(info['ylabel'])
                        ax2.set_xticks(range(len(config_stats)))
                        ax2.set_xticklabels([f"Config {c}" for c in config_stats['config']], rotation=45)
                        ax2.grid(True, alpha=0.3)
                    
                    # 3. Distribution histogram for best and worst configs
                    if len(config_stats) >= 2:
                        best_config = config_stats.loc[config_stats['mean'].idxmin(), 'config'] if info.get('lower_better', True) else config_stats.loc[config_stats['mean'].idxmax(), 'config']
                        worst_config = config_stats.loc[config_stats['mean'].idxmax(), 'config'] if info.get('lower_better', True) else config_stats.loc[config_stats['mean'].idxmin(), 'config']
                        
                        best_data = alg_data[alg_data['config'] == best_config][metric]
                        worst_data = alg_data[alg_data['config'] == worst_config][metric]
                        
                        ax3.hist(best_data, bins=15, alpha=0.7, label=f'Best (Config {best_config})', 
                               color='green', density=True)
                        ax3.hist(worst_data, bins=15, alpha=0.7, label=f'Worst (Config {worst_config})', 
                               color='red', density=True)
                        ax3.set_title(f'{info["title"]} Distribution: Best vs Worst Config')
                        ax3.set_xlabel(info['ylabel'])
                        ax3.set_ylabel('Density')
                        ax3.legend()
                        ax3.grid(True, alpha=0.3)
                    
                    # 4. Statistical summary table
                    ax4.axis('off')
                    summary_stats = alg_data.groupby('config')[metric].agg(['count', 'mean', 'std', 'min', 'max']).round(3)
                    
                    # Create table
                    table_data = []
                    for config in summary_stats.index:
                        row = [f"Config {config}"] + [f"{val:.3f}" for val in summary_stats.loc[config].values]
                        table_data.append(row)
                    
                    if table_data:
                        table = ax4.table(cellText=table_data,
                                        colLabels=['Configuration', 'Count', 'Mean', 'Std', 'Min', 'Max'],
                                        cellLoc='center',
                                        loc='center')
                        table.auto_set_font_size(False)
                        table.set_fontsize(9)
                        table.scale(1.2, 1.5)
                        ax4.set_title(f'{info["title"]} Statistical Summary', pad=20)
                    
                    plt.tight_layout()
                    pdf.savefig(fig, bbox_inches='tight')
                    plt.close()
                
                # Create a summary page comparing all algorithms
                fig, ((ax1, ax2), (ax3, ax4)) = plt.subplots(2, 2, figsize=(16, 12))
                fig.suptitle(f'{info["title"]} - All Algorithms Summary', fontsize=16, fontweight='bold')
                
                # 1. Best configuration per algorithm
                alg_best_configs = {}
                for algorithm in algorithms:
                    alg_subset = plot_data[plot_data['algorithm_name'] == algorithm]
                    if len(alg_subset) > 0:
                        config_means = alg_subset.groupby('config')[metric].mean()
                        if info.get('lower_better', True):
                            best_config = config_means.idxmin()
                            best_value = config_means.min()
                        else:
                            best_config = config_means.idxmax()
                            best_value = config_means.max()
                        alg_best_configs[algorithm] = (best_config, best_value)
                
                if alg_best_configs:
                    algs = list(alg_best_configs.keys())
                    values = [alg_best_configs[alg][1] for alg in algs]
                    colors = [self.algorithm_colors.get(alg, '#1f77b4') for alg in algs]
                    
                    bars = ax1.bar(range(len(algs)), values, color=colors, alpha=0.7)
                    ax1.set_title(f'Best {info["title"]} per Algorithm')
                    ax1.set_xlabel('Algorithm')
                    ax1.set_ylabel(info['ylabel'])
                    ax1.set_xticks(range(len(algs)))
                    ax1.set_xticklabels(algs, rotation=45)
                    ax1.grid(True, alpha=0.3)
                    
                    # Add config labels on bars
                    for i, (bar, alg) in enumerate(zip(bars, algs)):
                        height = bar.get_height()
                        config_num = alg_best_configs[alg][0]
                        ax1.text(bar.get_x() + bar.get_width()/2., height,
                                f'Config {config_num}', ha='center', va='bottom', fontsize=8)
                
                # 2. Overall algorithm comparison (all configs)
                alg_data_all = [plot_data[plot_data['algorithm_name'] == alg][metric].values for alg in algorithms]
                bp = ax2.boxplot(alg_data_all, patch_artist=True)
                for i, patch in enumerate(bp['boxes']):
                    alg_name = algorithms[i]
                    patch.set_facecolor(self.algorithm_colors.get(alg_name, '#1f77b4'))
                    patch.set_alpha(0.7)
                
                ax2.set_title(f'{info["title"]} Overall Comparison')
                ax2.set_xlabel('Algorithm')
                ax2.set_ylabel(info['ylabel'])
                ax2.set_xticklabels(algorithms, rotation=45)
                ax2.grid(True, alpha=0.3)
                
                # 3. Configuration count per algorithm
                config_counts = plot_data.groupby('algorithm_name')['config'].nunique()
                ax3.bar(range(len(config_counts)), config_counts.values,
                       color=[self.algorithm_colors.get(alg, '#1f77b4') for alg in config_counts.index],
                       alpha=0.7)
                ax3.set_title('Number of Configurations per Algorithm')
                ax3.set_xlabel('Algorithm')
                ax3.set_ylabel('Number of Configurations')
                ax3.set_xticks(range(len(config_counts)))
                ax3.set_xticklabels(config_counts.index, rotation=45)
                ax3.grid(True, alpha=0.3)
                
                # 4. Algorithm performance variability
                alg_variability = plot_data.groupby('algorithm_name')[metric].std()
                ax4.bar(range(len(alg_variability)), alg_variability.values,
                       color=[self.algorithm_colors.get(alg, '#1f77b4') for alg in alg_variability.index],
                       alpha=0.7)
                ax4.set_title(f'{info["title"]} Variability (Standard Deviation)')
                ax4.set_xlabel('Algorithm')
                ax4.set_ylabel(f'Std Dev of {info["ylabel"]}')
                ax4.set_xticks(range(len(alg_variability)))
                ax4.set_xticklabels(alg_variability.index, rotation=45)
                ax4.grid(True, alpha=0.3)
                
                plt.tight_layout()
                pdf.savefig(fig, bbox_inches='tight')
                plt.close()
                
                plt.tight_layout()
                pdf.savefig(fig, bbox_inches='tight')
                plt.close()
                
            print(f"✅ Saved {info['title']} comparison to: {filename}")
    
    def create_parameter_analysis_pdfs(self):
        """Create detailed parameter analysis for each algorithm family."""
        
        # STOMP parameter analysis
        stomp_data = self.data[self.data['algorithm_name'] == 'STOMP'].copy()
        if len(stomp_data) > 0:
            self._create_stomp_parameter_analysis(stomp_data)
        
        # Hauser family parameter analysis
        hauser_data = self.data[self.data['algorithm_name'].str.contains('Hauser', na=False)].copy()
        if len(hauser_data) > 0:
            self._create_hauser_parameter_analysis(hauser_data)
    
    def _create_stomp_parameter_analysis(self, stomp_data):
        """Create STOMP-specific parameter analysis."""
        
        print("📈 Generating STOMP parameter analysis...")
        
        filename = self.parameter_dir / "stomp_parameter_analysis.pdf"
        
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
            
            # Page 2: Parameter effects on performance metrics
            successful_stomp = stomp_data[stomp_data['success'] == 1]
            if len(successful_stomp) > 0:
                fig, ((ax1, ax2), (ax3, ax4)) = plt.subplots(2, 2, figsize=(16, 12))
                fig.suptitle('STOMP Parameter Analysis - Performance Effects', fontsize=16, fontweight='bold')
                
                # Planning time vs iterations
                iter_time = successful_stomp.groupby('iterations')['planning_time_ms'].mean()
                ax1.plot(iter_time.index, iter_time.values, 'o-', color='#1f77b4', linewidth=2, markersize=8)
                ax1.set_xlabel('Max Iterations')
                ax1.set_ylabel('Average Planning Time (ms)')
                ax1.set_title('Planning Time vs Max Iterations')
                ax1.grid(True, alpha=0.3)
                
                # Path quality vs learning rate
                lr_quality = successful_stomp.groupby('learning_rate')['path_quality'].mean()
                ax2.plot(lr_quality.index, lr_quality.values, 'o-', color='#ff7f0e', linewidth=2, markersize=8)
                ax2.set_xlabel('Learning Rate')
                ax2.set_ylabel('Average Path Quality')
                ax2.set_title('Path Quality vs Learning Rate')
                ax2.grid(True, alpha=0.3)
                
                # Trajectory length vs noisy samples
                noisy_length = successful_stomp.groupby('noisy_samples')['trajectory_length'].mean()
                ax3.plot(noisy_length.index, noisy_length.values, 'o-', color='#2ca02c', linewidth=2, markersize=8)
                ax3.set_xlabel('Noisy Samples')
                ax3.set_ylabel('Average Trajectory Length')
                ax3.set_title('Trajectory Length vs Noisy Samples')
                ax3.grid(True, alpha=0.3)
                
                # Configuration ranking
                config_performance = successful_stomp.groupby('config').agg({
                    'success': 'mean',
                    'planning_time_ms': 'mean',
                    'path_quality': 'mean'
                }).round(3)
                
                config_performance['score'] = (config_performance['success'] * 0.4 + 
                                              (1 - config_performance['planning_time_ms'] / config_performance['planning_time_ms'].max()) * 0.3 +
                                              config_performance['path_quality'] * 0.3)
                
                config_performance = config_performance.sort_values('score', ascending=False)
                
                bars = ax4.bar(range(len(config_performance)), config_performance['score'], 
                              color='#1f77b4', alpha=0.7)
                ax4.set_xlabel('Configuration')
                ax4.set_ylabel('Performance Score')
                ax4.set_title('STOMP Configuration Ranking')
                ax4.set_xticks(range(len(config_performance)))
                ax4.set_xticklabels([f"Config {i+1}" for i in range(len(config_performance))], rotation=45)
                ax4.grid(True, alpha=0.3)
                
                plt.tight_layout()
                pdf.savefig(fig, bbox_inches='tight')
                plt.close()
        
        print(f"✅ Saved STOMP parameter analysis to: {filename}")
    
    def _create_hauser_parameter_analysis(self, hauser_data):
        """Create Hauser family parameter analysis."""
        
        print("📈 Generating Hauser family parameter analysis...")
        
        filename = self.parameter_dir / "hauser_parameter_analysis.pdf"
        
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
                          color=[self.algorithm_colors.get(alg, '#ff7f0e') for alg in variant_success.index],
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
                              color=[self.algorithm_colors.get(alg, '#ff7f0e') for alg in variant_time.index],
                              alpha=0.7)
                ax2.set_xlabel('Algorithm Variant')
                ax2.set_ylabel('Average Planning Time (ms)')
                ax2.set_title('Planning Time by Hauser Variant')
                ax2.set_xticks(range(len(variant_time)))
                ax2.set_xticklabels([alg.replace('Hauser ', '') for alg in variant_time.index], rotation=45)
                ax2.grid(True, alpha=0.3)
            
            # Parameter effects on success
            if len(hauser_data) > 0:
                param_success = hauser_data.groupby(['iterations', 'step_size', 'bias'])['success'].mean().reset_index()
                scatter = ax3.scatter(param_success['step_size'], param_success['bias'], 
                                    c=param_success['success'], s=param_success['iterations']/10,
                                    cmap='RdYlBu', alpha=0.7)
                ax3.set_xlabel('Step Size')
                ax3.set_ylabel('Goal Bias')
                ax3.set_title('Parameter Effects on Success\n(Color: Success Rate, Size: Iterations)')
                plt.colorbar(scatter, ax=ax3, label='Success Rate')
                ax3.grid(True, alpha=0.3)
            
            # Feature comparison (informed vs non-informed, bidirectional vs unidirectional)
            feature_comparison = []
            if len(hauser_data[hauser_data['is_informed']]) > 0:
                informed_success = hauser_data[hauser_data['is_informed']]['success'].mean()
                feature_comparison.append(('Informed', informed_success))
            
            if len(hauser_data[~hauser_data['is_informed']]) > 0:
                non_informed_success = hauser_data[~hauser_data['is_informed']]['success'].mean()
                feature_comparison.append(('Non-Informed', non_informed_success))
            
            if len(hauser_data[hauser_data['is_bidirectional']]) > 0:
                bi_success = hauser_data[hauser_data['is_bidirectional']]['success'].mean()
                feature_comparison.append(('Bidirectional', bi_success))
            
            if len(hauser_data[~hauser_data['is_bidirectional']]) > 0:
                uni_success = hauser_data[~hauser_data['is_bidirectional']]['success'].mean()
                feature_comparison.append(('Unidirectional', uni_success))
            
            if feature_comparison:
                features, success_rates = zip(*feature_comparison)
                ax4.bar(range(len(features)), success_rates, color='#ff7f0e', alpha=0.7)
                ax4.set_xlabel('Feature')
                ax4.set_ylabel('Success Rate')
                ax4.set_title('Feature Comparison')
                ax4.set_xticks(range(len(features)))
                ax4.set_xticklabels(features, rotation=45)
                ax4.grid(True, alpha=0.3)
            
            plt.tight_layout()
            pdf.savefig(fig, bbox_inches='tight')
            plt.close()
        
        print(f"✅ Saved Hauser parameter analysis to: {filename}")
    
    def generate_all_analyses(self):
        """Generate all enhanced analyses."""
        
        print("\n🚀 Generating enhanced trajectory planning analyses...")
        print("=" * 60)
        
        # 1. Algorithm-specific comprehensive analyses
        print("\n📋 Creating algorithm-specific comprehensive analyses...")
        for algorithm in self.data['algorithm_name'].unique():
            if pd.notna(algorithm):
                self.create_algorithm_specific_pdf(algorithm)
        
        # 2. Cross-algorithm metric comparisons
        print("\n📊 Creating cross-algorithm metric comparisons...")
        self.create_metric_comparison_pdfs()
        
        # 3. Parameter analyses
        print("\n🔧 Creating parameter analyses...")
        self.create_parameter_analysis_pdfs()
        
        print("\n" + "=" * 60)
        print("✅ All enhanced analyses generated!")
        print(f"📁 Algorithm-specific analyses: {self.algorithm_dir}")
        print(f"📊 Metric comparisons: {self.metric_dir}")
        print(f"🔧 Parameter analyses: {self.parameter_dir}")

def main():
    parser = argparse.ArgumentParser(description='Enhanced Trajectory Planning Evaluation Visualizer')
    parser.add_argument('data_file', help='CSV file with evaluation results')
    parser.add_argument('--output-dir', '-o', default='results', help='Output directory for PDFs')
    
    args = parser.parse_args()
    
    if not Path(args.data_file).exists():
        print(f"❌ Data file not found: {args.data_file}")
        sys.exit(1)
    
    visualizer = EnhancedTrajectoryPlanningVisualizer(args.data_file, args.output_dir)
    visualizer.generate_all_analyses()

if __name__ == "__main__":
    main()
