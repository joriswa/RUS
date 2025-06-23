#!/usr/bin/env python3
"""
Comprehensive STOMP Optimization Visualization
Generates detailed plots and analysis of the optimization results.
"""

import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import seaborn as sns
from pathlib import Path
import json
from datetime import datetime
import warnings

warnings.filterwarnings('ignore')

# Set up matplotlib for high-quality plots
plt.style.use('default')
sns.set_palette("husl")
plt.rcParams['figure.dpi'] = 300
plt.rcParams['savefig.dpi'] = 300
plt.rcParams['font.size'] = 10
plt.rcParams['axes.titlesize'] = 12
plt.rcParams['axes.labelsize'] = 10
plt.rcParams['xtick.labelsize'] = 9
plt.rcParams['ytick.labelsize'] = 9
plt.rcParams['legend.fontsize'] = 9

class STOMPVisualizationAnalyzer:
    def __init__(self, data_dir="trajectory_optimization/data/results/stomp"):
        self.data_dir = Path(data_dir)
        self.trials_df = None
        self.best_config = None
        self.param_columns = None
        self.performance_columns = None
        
    def load_data(self):
        """Load optimization results data."""
        print("Loading STOMP optimization data...")
        
        # Load trials data
        trials_file = self.data_dir / "trials.csv"
        if trials_file.exists():
            self.trials_df = pd.read_csv(trials_file)
            print(f"Loaded {len(self.trials_df)} trials")
        else:
            raise FileNotFoundError(f"Trials file not found: {trials_file}")
            
        # Load best configuration
        best_config_file = self.data_dir / "best_config.json"
        if best_config_file.exists():
            with open(best_config_file, 'r') as f:
                self.best_config = json.load(f)
            print(f"Best objective value: {self.best_config['best_value']:.4f}")
        else:
            raise FileNotFoundError(f"Best config file not found: {best_config_file}")
            
        # Identify parameter and performance columns
        self.param_columns = [col for col in self.trials_df.columns if col.startswith('params_stomp_')]
        self.performance_columns = [col for col in self.trials_df.columns if col.startswith('user_attrs_')]
        
        # Convert datetime columns
        self.trials_df['datetime_start'] = pd.to_datetime(self.trials_df['datetime_start'])
        self.trials_df['datetime_complete'] = pd.to_datetime(self.trials_df['datetime_complete'])
        
        print(f"Found {len(self.param_columns)} parameters and {len(self.performance_columns)} performance metrics")
        
    def create_optimization_convergence_plot(self):
        """Create optimization convergence analysis plots."""
        fig, axes = plt.subplots(2, 2, figsize=(15, 12))
        fig.suptitle('STOMP Optimization Convergence Analysis', fontsize=16, fontweight='bold')
        
        # 1. Objective value over trials
        ax1 = axes[0, 0]
        ax1.plot(self.trials_df['number'], self.trials_df['value'], alpha=0.6, linewidth=0.8, color='lightblue')
        
        # Calculate running minimum
        running_min = self.trials_df['value'].cummin()
        ax1.plot(self.trials_df['number'], running_min, color='red', linewidth=2, 
                label=f'Running Best (Final: {running_min.iloc[-1]:.4f})')
        
        # Mark best trial
        best_idx = self.trials_df['value'].idxmin()
        best_trial = self.trials_df.loc[best_idx]
        ax1.scatter(best_trial['number'], best_trial['value'], color='red', s=100, 
                   marker='*', zorder=5, label=f'Global Best (Trial {best_trial["number"]})')
        
        ax1.set_xlabel('Trial Number')
        ax1.set_ylabel('Objective Value')
        ax1.set_title('Optimization Convergence')
        ax1.legend()
        ax1.grid(True, alpha=0.3)
        
        # 2. Distribution of objective values
        ax2 = axes[0, 1]
        ax2.hist(self.trials_df['value'], bins=50, alpha=0.7, color='skyblue', edgecolor='black')
        ax2.axvline(self.trials_df['value'].mean(), color='orange', linestyle='--', 
                   label=f'Mean: {self.trials_df["value"].mean():.4f}')
        ax2.axvline(self.trials_df['value'].median(), color='green', linestyle='--', 
                   label=f'Median: {self.trials_df["value"].median():.4f}')
        ax2.axvline(self.best_config['best_value'], color='red', linestyle='-', linewidth=2,
                   label=f'Best: {self.best_config["best_value"]:.4f}')
        ax2.set_xlabel('Objective Value')
        ax2.set_ylabel('Frequency')
        ax2.set_title('Distribution of Objective Values')
        ax2.legend()
        ax2.grid(True, alpha=0.3)
        
        # 3. Optimization progress over time
        ax3 = axes[1, 0]
        trial_duration = (self.trials_df['datetime_complete'] - self.trials_df['datetime_start'].iloc[0]).dt.total_seconds()
        ax3.plot(trial_duration, self.trials_df['value'], alpha=0.6, linewidth=0.8, color='lightblue')
        ax3.plot(trial_duration, running_min, color='red', linewidth=2, label='Running Best')
        ax3.set_xlabel('Time (seconds)')
        ax3.set_ylabel('Objective Value')
        ax3.set_title('Optimization Progress Over Time')
        ax3.legend()
        ax3.grid(True, alpha=0.3)
        
        # 4. Moving average convergence
        ax4 = axes[1, 1]
        window_size = 20
        moving_avg = self.trials_df['value'].rolling(window=window_size, center=True).mean()
        moving_std = self.trials_df['value'].rolling(window=window_size, center=True).std()
        
        ax4.plot(self.trials_df['number'], moving_avg, color='blue', linewidth=2, 
                label=f'Moving Average (window={window_size})')
        ax4.fill_between(self.trials_df['number'], 
                        moving_avg - moving_std, 
                        moving_avg + moving_std,
                        alpha=0.3, color='blue', label='±1 Std Dev')
        ax4.plot(self.trials_df['number'], running_min, color='red', linewidth=2, 
                label='Running Best')
        ax4.set_xlabel('Trial Number')
        ax4.set_ylabel('Objective Value')
        ax4.set_title('Moving Average Convergence')
        ax4.legend()
        ax4.grid(True, alpha=0.3)
        
        plt.tight_layout()
        return fig
    
    def create_parameter_analysis_plot(self):
        """Create comprehensive parameter analysis plots."""
        n_params = len(self.param_columns)
        n_cols = 3
        n_rows = (n_params + n_cols - 1) // n_cols
        
        fig, axes = plt.subplots(n_rows, n_cols, figsize=(18, 4*n_rows))
        fig.suptitle('STOMP Parameter Analysis', fontsize=16, fontweight='bold')
        
        if n_rows == 1:
            axes = axes.reshape(1, -1)
        
        for i, param in enumerate(self.param_columns):
            row = i // n_cols
            col = i % n_cols
            ax = axes[row, col]
            
            param_name = param.replace('params_stomp_', '').replace('_', ' ').title()
            
            # Scatter plot of parameter vs objective value
            scatter = ax.scatter(self.trials_df[param], self.trials_df['value'], 
                               alpha=0.6, s=20, c=self.trials_df['number'], 
                               cmap='viridis')
            
            # Mark best configuration
            best_param_value = self.best_config['best_params'][param.replace('params_', '')]
            ax.scatter(best_param_value, self.best_config['best_value'], 
                      color='red', s=100, marker='*', zorder=5, 
                      label='Best Config')
            
            # Add trend line
            try:
                z = np.polyfit(self.trials_df[param], self.trials_df['value'], 1)
                p = np.poly1d(z)
                x_trend = np.linspace(self.trials_df[param].min(), self.trials_df[param].max(), 100)
                ax.plot(x_trend, p(x_trend), "r--", alpha=0.8, linewidth=1)
            except:
                pass
            
            ax.set_xlabel(param_name)
            ax.set_ylabel('Objective Value')
            ax.set_title(f'{param_name}')
            ax.grid(True, alpha=0.3)
            
            if i == 0:  # Add colorbar only once
                cbar = plt.colorbar(scatter, ax=ax)
                cbar.set_label('Trial Number')
        
        # Hide unused subplots
        for i in range(n_params, n_rows * n_cols):
            row = i // n_cols
            col = i % n_cols
            axes[row, col].axis('off')
            
        plt.tight_layout()
        return fig
    
    def create_correlation_analysis_plot(self):
        """Create parameter correlation and importance analysis."""
        fig, axes = plt.subplots(2, 2, figsize=(16, 12))
        fig.suptitle('STOMP Parameter Correlation & Importance Analysis', fontsize=16, fontweight='bold')
        
        # Extract parameter values for correlation analysis
        param_data = self.trials_df[self.param_columns].copy()
        param_data.columns = [col.replace('params_stomp_', '') for col in param_data.columns]
        
        # 1. Parameter correlation matrix
        ax1 = axes[0, 0]
        correlation_matrix = param_data.corr()
        mask = np.triu(np.ones_like(correlation_matrix, dtype=bool))
        sns.heatmap(correlation_matrix, mask=mask, annot=True, fmt='.2f', 
                   cmap='coolwarm', center=0, ax=ax1, cbar_kws={'shrink': 0.8})
        ax1.set_title('Parameter Correlation Matrix')
        
        # 2. Parameter importance (correlation with objective)
        ax2 = axes[0, 1]
        param_importance = []
        param_names = []
        for param in self.param_columns:
            param_name = param.replace('params_stomp_', '')
            correlation = abs(np.corrcoef(self.trials_df[param], self.trials_df['value'])[0, 1])
            if not np.isnan(correlation):
                param_importance.append(correlation)
                param_names.append(param_name)
        
        # Sort by importance
        sorted_data = sorted(zip(param_importance, param_names), reverse=True)
        param_importance, param_names = zip(*sorted_data)
        
        bars = ax2.barh(range(len(param_names)), param_importance, color='skyblue')
        ax2.set_yticks(range(len(param_names)))
        ax2.set_yticklabels([name.replace('_', ' ').title() for name in param_names])
        ax2.set_xlabel('Absolute Correlation with Objective')
        ax2.set_title('Parameter Importance')
        ax2.grid(True, alpha=0.3, axis='x')
        
        # Add value labels on bars
        for i, (bar, importance) in enumerate(zip(bars, param_importance)):
            ax2.text(importance + 0.01, i, f'{importance:.3f}', 
                    va='center', fontsize=8)
        
        # 3. Performance metrics correlation
        ax3 = axes[1, 0]
        # Filter only numeric performance columns
        numeric_perf_columns = []
        for col in self.performance_columns:
            if self.trials_df[col].dtype in ['float64', 'int64']:
                numeric_perf_columns.append(col)
        
        if len(numeric_perf_columns) > 1:
            perf_data = self.trials_df[numeric_perf_columns].copy()
            perf_data.columns = [col.replace('user_attrs_', '') for col in perf_data.columns]
            perf_correlation = perf_data.corr()
            sns.heatmap(perf_correlation, annot=True, fmt='.2f', 
                       cmap='coolwarm', center=0, ax=ax3, cbar_kws={'shrink': 0.8})
            ax3.set_title('Performance Metrics Correlation')
        else:
            ax3.text(0.5, 0.5, 'Insufficient numeric performance metrics\nfor correlation analysis', 
                    ha='center', va='center', transform=ax3.transAxes)
            ax3.set_title('Performance Metrics Correlation')
        
        # 4. Best vs worst parameter comparison
        ax4 = axes[1, 1]
        # Get top 10% and bottom 10% trials
        n_top = max(1, len(self.trials_df) // 10)
        n_bottom = max(1, len(self.trials_df) // 10)
        
        top_trials = self.trials_df.nsmallest(n_top, 'value')
        bottom_trials = self.trials_df.nlargest(n_bottom, 'value')
        
        # Compare most important parameters
        top_params = param_names[:5]  # Top 5 most important
        
        x_pos = np.arange(len(top_params))
        width = 0.35
        
        top_means = []
        bottom_means = []
        
        for param in top_params:
            full_param = f'params_stomp_{param}'
            top_mean = top_trials[full_param].mean()
            bottom_mean = bottom_trials[full_param].mean()
            top_means.append(top_mean)
            bottom_means.append(bottom_mean)
        
        ax4.bar(x_pos - width/2, top_means, width, label=f'Top {n_top} trials', alpha=0.8)
        ax4.bar(x_pos + width/2, bottom_means, width, label=f'Bottom {n_bottom} trials', alpha=0.8)
        
        ax4.set_xlabel('Parameters')
        ax4.set_ylabel('Average Parameter Value')
        ax4.set_title('Best vs Worst Trials Parameter Comparison')
        ax4.set_xticks(x_pos)
        ax4.set_xticklabels([name.replace('_', ' ').title() for name in top_params], rotation=45)
        ax4.legend()
        ax4.grid(True, alpha=0.3, axis='y')
        
        plt.tight_layout()
        return fig
    
    def create_performance_analysis_plot(self):
        """Create performance metrics analysis plots."""
        n_metrics = len(self.performance_columns)
        if n_metrics == 0:
            print("No performance metrics found for analysis")
            return None
            
        fig, axes = plt.subplots(2, 2, figsize=(15, 12))
        fig.suptitle('STOMP Performance Metrics Analysis', fontsize=16, fontweight='bold')
        
        # 1. Performance vs Objective scatter plots
        ax1 = axes[0, 0]
        for i, metric in enumerate(self.performance_columns[:3]):  # Limit to 3 metrics
            metric_name = metric.replace('user_attrs_', '').replace('_', ' ').title()
            scatter = ax1.scatter(self.trials_df[metric], self.trials_df['value'], 
                                alpha=0.6, s=30, label=metric_name)
        ax1.set_xlabel('Performance Metric Value')
        ax1.set_ylabel('Objective Value')
        ax1.set_title('Performance Metrics vs Objective')
        ax1.legend()
        ax1.grid(True, alpha=0.3)
        
        # 2. Performance distribution
        ax2 = axes[0, 1]
        if 'user_attrs_planning_time' in self.performance_columns:
            planning_times = self.trials_df['user_attrs_planning_time']
            ax2.hist(planning_times, bins=30, alpha=0.7, color='lightgreen', edgecolor='black')
            ax2.axvline(planning_times.mean(), color='red', linestyle='--', 
                       label=f'Mean: {planning_times.mean():.2f}s')
            ax2.axvline(planning_times.median(), color='orange', linestyle='--', 
                       label=f'Median: {planning_times.median():.2f}s')
            ax2.set_xlabel('Planning Time (seconds)')
            ax2.set_ylabel('Frequency')
            ax2.set_title('Planning Time Distribution')
            ax2.legend()
            ax2.grid(True, alpha=0.3)
        else:
            ax2.text(0.5, 0.5, 'Planning time data\nnot available', 
                    ha='center', va='center', transform=ax2.transAxes)
        
        # 3. Success rate vs objective
        ax3 = axes[1, 0]
        if 'user_attrs_success_rate' in self.performance_columns:
            success_rates = self.trials_df['user_attrs_success_rate']
            scatter = ax3.scatter(success_rates, self.trials_df['value'], 
                                alpha=0.6, s=30, c=self.trials_df['number'], cmap='viridis')
            ax3.set_xlabel('Success Rate')
            ax3.set_ylabel('Objective Value')
            ax3.set_title('Success Rate vs Objective Value')
            ax3.grid(True, alpha=0.3)
            plt.colorbar(scatter, ax=ax3, label='Trial Number')
        else:
            ax3.text(0.5, 0.5, 'Success rate data\nnot available', 
                    ha='center', va='center', transform=ax3.transAxes)
        
        # 4. Performance correlation with best parameters
        ax4 = axes[1, 1]
        if 'user_attrs_safety_clearance' in self.performance_columns:
            safety_clearance = self.trials_df['user_attrs_safety_clearance']
            ax4.hist(safety_clearance, bins=30, alpha=0.7, color='salmon', edgecolor='black')
            ax4.axvline(safety_clearance.mean(), color='red', linestyle='--', 
                       label=f'Mean: {safety_clearance.mean():.4f}')
            ax4.set_xlabel('Safety Clearance')
            ax4.set_ylabel('Frequency')
            ax4.set_title('Safety Clearance Distribution')
            ax4.legend()
            ax4.grid(True, alpha=0.3)
        else:
            ax4.text(0.5, 0.5, 'Safety clearance data\nnot available', 
                    ha='center', va='center', transform=ax4.transAxes)
        
        plt.tight_layout()
        return fig
    
    def create_optimization_summary_plot(self):
        """Create a comprehensive summary visualization."""
        fig = plt.figure(figsize=(20, 12))
        gs = fig.add_gridspec(3, 4, hspace=0.3, wspace=0.3)
        
        fig.suptitle('STOMP Optimization Comprehensive Summary', fontsize=18, fontweight='bold')
        
        # 1. Main convergence plot (spans 2 columns)
        ax1 = fig.add_subplot(gs[0, :2])
        ax1.plot(self.trials_df['number'], self.trials_df['value'], alpha=0.4, linewidth=0.8, color='lightblue')
        running_min = self.trials_df['value'].cummin()
        ax1.plot(self.trials_df['number'], running_min, color='red', linewidth=3, label='Running Best')
        
        best_idx = self.trials_df['value'].idxmin()
        best_trial = self.trials_df.loc[best_idx]
        ax1.scatter(best_trial['number'], best_trial['value'], color='red', s=150, 
                   marker='*', zorder=5, label=f'Global Best: {best_trial["value"]:.4f}')
        
        ax1.set_xlabel('Trial Number')
        ax1.set_ylabel('Objective Value')
        ax1.set_title('Optimization Convergence', fontsize=14, fontweight='bold')
        ax1.legend()
        ax1.grid(True, alpha=0.3)
        
        # 2. Statistics summary (spans 2 columns)
        ax2 = fig.add_subplot(gs[0, 2:])
        stats_text = f"""
        OPTIMIZATION STATISTICS
        
        Total Trials: {len(self.trials_df)}
        Best Value: {self.best_config['best_value']:.4f}
        Mean Value: {self.trials_df['value'].mean():.4f}
        Std Dev: {self.trials_df['value'].std():.4f}
        
        Best Trial: #{best_trial['number']}
        Improvement: {((self.trials_df['value'].max() - self.best_config['best_value']) / self.trials_df['value'].max() * 100):.1f}%
        
        Most Important Parameters:
        """
        
        # Add parameter importance
        param_importance = []
        for param in self.param_columns:
            correlation = abs(np.corrcoef(self.trials_df[param], self.trials_df['value'])[0, 1])
            if not np.isnan(correlation):
                param_name = param.replace('params_stomp_', '').replace('_', ' ')
                param_importance.append((param_name, correlation))
        
        param_importance.sort(key=lambda x: x[1], reverse=True)
        for i, (name, importance) in enumerate(param_importance[:5]):
            stats_text += f"\n        {i+1}. {name.title()}: {importance:.3f}"
        
        ax2.text(0.05, 0.95, stats_text, transform=ax2.transAxes, fontsize=11,
                verticalalignment='top', fontfamily='monospace',
                bbox=dict(boxstyle='round', facecolor='lightgray', alpha=0.8))
        ax2.axis('off')
        
        # 3. Parameter distributions (4 most important)
        for i in range(4):
            ax = fig.add_subplot(gs[1, i])
            if i < len(param_importance):
                param_name, _ = param_importance[i]
                full_param = f'params_stomp_{param_name.replace(" ", "_")}'
                
                if full_param in self.trials_df.columns:
                    values = self.trials_df[full_param]
                    ax.hist(values, bins=20, alpha=0.7, color=f'C{i}', edgecolor='black')
                    
                    # Mark best value
                    best_value = self.best_config['best_params'][full_param.replace('params_', '')]
                    ax.axvline(best_value, color='red', linestyle='--', linewidth=2,
                              label=f'Best: {best_value:.3f}')
                    
                    ax.set_xlabel(param_name.replace('_', ' ').title())
                    ax.set_ylabel('Frequency')
                    ax.set_title(f'{param_name.replace("_", " ").title()}')
                    ax.legend(fontsize=8)
                    ax.grid(True, alpha=0.3)
        
        # 4. Performance evolution (spans 4 columns)
        ax5 = fig.add_subplot(gs[2, :])
        
        # Calculate percentiles over time
        window_size = 25
        percentiles = []
        trial_numbers = []
        
        for i in range(window_size, len(self.trials_df), window_size//2):
            window_data = self.trials_df['value'].iloc[max(0, i-window_size):i]
            percentiles.append([
                np.percentile(window_data, 10),
                np.percentile(window_data, 25),
                np.percentile(window_data, 50),
                np.percentile(window_data, 75),
                np.percentile(window_data, 90)
            ])
            trial_numbers.append(i)
        
        percentiles = np.array(percentiles)
        
        ax5.fill_between(trial_numbers, percentiles[:, 0], percentiles[:, 4], 
                        alpha=0.2, color='blue', label='10th-90th percentile')
        ax5.fill_between(trial_numbers, percentiles[:, 1], percentiles[:, 3], 
                        alpha=0.4, color='blue', label='25th-75th percentile')
        ax5.plot(trial_numbers, percentiles[:, 2], color='blue', linewidth=2, label='Median')
        ax5.plot(self.trials_df['number'], running_min, color='red', linewidth=2, label='Running Best')
        
        ax5.set_xlabel('Trial Number')
        ax5.set_ylabel('Objective Value')
        ax5.set_title('Performance Evolution (Rolling Statistics)', fontsize=14, fontweight='bold')
        ax5.legend()
        ax5.grid(True, alpha=0.3)
        
        return fig
    
    def generate_all_plots(self, output_dir="trajectory_optimization/data/visualizations"):
        """Generate all comprehensive plots and save them."""
        output_dir = Path(output_dir)
        output_dir.mkdir(parents=True, exist_ok=True)
        
        print("Generating comprehensive STOMP optimization visualizations...")
        
        # Load data
        self.load_data()
        
        plots = []
        
        # 1. Optimization convergence analysis
        print("Creating convergence analysis...")
        fig1 = self.create_optimization_convergence_plot()
        convergence_path = output_dir / "stomp_convergence_analysis.png"
        fig1.savefig(convergence_path, dpi=300, bbox_inches='tight')
        plots.append(str(convergence_path))
        plt.close(fig1)
        
        # 2. Parameter analysis
        print("Creating parameter analysis...")
        fig2 = self.create_parameter_analysis_plot()
        parameter_path = output_dir / "stomp_parameter_analysis.png"
        fig2.savefig(parameter_path, dpi=300, bbox_inches='tight')
        plots.append(str(parameter_path))
        plt.close(fig2)
        
        # 3. Correlation analysis
        print("Creating correlation analysis...")
        fig3 = self.create_correlation_analysis_plot()
        correlation_path = output_dir / "stomp_correlation_analysis.png"
        fig3.savefig(correlation_path, dpi=300, bbox_inches='tight')
        plots.append(str(correlation_path))
        plt.close(fig3)
        
        # 4. Performance analysis
        print("Creating performance analysis...")
        fig4 = self.create_performance_analysis_plot()
        if fig4:
            performance_path = output_dir / "stomp_performance_analysis.png"
            fig4.savefig(performance_path, dpi=300, bbox_inches='tight')
            plots.append(str(performance_path))
            plt.close(fig4)
        
        # 5. Comprehensive summary
        print("Creating comprehensive summary...")
        fig5 = self.create_optimization_summary_plot()
        summary_path = output_dir / "stomp_comprehensive_summary.png"
        fig5.savefig(summary_path, dpi=300, bbox_inches='tight')
        plots.append(str(summary_path))
        plt.close(fig5)
        
        # Generate summary report
        self.generate_visualization_report(output_dir, plots)
        
        print(f"\nAll visualizations saved to: {output_dir}")
        print(f"Generated {len(plots)} plots:")
        for plot in plots:
            print(f"  - {Path(plot).name}")
            
        return plots
    
    def generate_visualization_report(self, output_dir, plot_paths):
        """Generate a markdown report with all visualizations."""
        report_path = output_dir / "visualization_report.md"
        
        timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        
        with open(report_path, 'w') as f:
            f.write(f"""# STOMP Optimization Visualization Report

**Generated**: {timestamp}
**Total Trials**: {len(self.trials_df)}
**Best Objective Value**: {self.best_config['best_value']:.4f}

## Overview

This report contains comprehensive visualizations of the STOMP optimization results, 
including convergence analysis, parameter relationships, and performance metrics.

## Visualizations

### 1. Convergence Analysis
![Convergence Analysis]({Path(plot_paths[0]).name})

Shows the optimization convergence over trials, including:
- Objective value progression
- Running best value
- Distribution of results
- Time-based analysis

### 2. Parameter Analysis
![Parameter Analysis]({Path(plot_paths[1]).name})

Individual parameter relationships with objective value:
- Scatter plots for each parameter
- Best configuration markers
- Trend lines where applicable

### 3. Correlation Analysis
![Correlation Analysis]({Path(plot_paths[2]).name})

Parameter correlation and importance analysis:
- Parameter correlation matrix
- Parameter importance ranking
- Best vs worst trials comparison

""")
            
            if len(plot_paths) > 3:
                f.write(f"""### 4. Performance Analysis
![Performance Analysis]({Path(plot_paths[3]).name})

Performance metrics analysis including planning time, success rates, and safety measures.

""")
            
            f.write(f"""### 5. Comprehensive Summary
![Comprehensive Summary]({Path(plot_paths[-1]).name})

Complete optimization overview with statistics, parameter distributions, and performance evolution.

## Key Findings

### Best Configuration
""")
            
            for param, value in self.best_config['best_params'].items():
                f.write(f"- **{param.replace('stomp_', '').replace('_', ' ').title()}**: {value}\n")
            
            # Add parameter importance
            param_importance = []
            for param in self.param_columns:
                correlation = abs(np.corrcoef(self.trials_df[param], self.trials_df['value'])[0, 1])
                if not np.isnan(correlation):
                    param_name = param.replace('params_stomp_', '').replace('_', ' ')
                    param_importance.append((param_name, correlation))
            
            param_importance.sort(key=lambda x: x[1], reverse=True)
            
            f.write(f"""
### Parameter Importance (Top 5)
""")
            for i, (name, importance) in enumerate(param_importance[:5], 1):
                f.write(f"{i}. **{name.title()}**: {importance:.4f}\n")
            
            f.write(f"""
### Statistics
- **Mean Objective Value**: {self.trials_df['value'].mean():.4f}
- **Standard Deviation**: {self.trials_df['value'].std():.4f}
- **Best Trial Number**: {self.trials_df.loc[self.trials_df['value'].idxmin(), 'number']}
- **Improvement from First Trial**: {((self.trials_df['value'].iloc[0] - self.best_config['best_value']) / self.trials_df['value'].iloc[0] * 100):.1f}%

## Files Generated
""")
            for plot_path in plot_paths:
                f.write(f"- `{Path(plot_path).name}`\n")
        
        print(f"Visualization report saved to: {report_path}")


if __name__ == "__main__":
    analyzer = STOMPVisualizationAnalyzer()
    plots = analyzer.generate_all_plots()
    print("\nVisualization generation complete!")
