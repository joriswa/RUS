#!/usr/bin/env python3
"""
Advanced Parameter Tuning Analysis and Visualization
====================================================

This script provides comprehensive analysis and visualization capabilities for the
Enhanced Parameter Tuning framework. It processes parameter tuning results and
generates detailed reports, interactive visualizations, and statistical analyses.

Features:
- Multi-dimensional parameter space visualization
- Bayesian optimization convergence analysis
- Parameter importance and sensitivity analysis
- Statistical significance testing
- Interactive dashboards
- Publication-quality plots

Author: PathPlanner Team
Date: December 2024
"""

import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import seaborn as sns
import plotly.graph_objects as go
import plotly.express as px
from plotly.subplots import make_subplots
import scipy.stats as stats
from scipy.optimize import minimize
from sklearn.ensemble import RandomForestRegressor
from sklearn.inspection import permutation_importance
from sklearn.preprocessing import StandardScaler
from sklearn.model_selection import cross_val_score
import warnings
import argparse
import json
import yaml
from pathlib import Path
from typing import Dict, List, Tuple, Optional, Union
import logging
from datetime import datetime

# Setup logging
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')
logger = logging.getLogger(__name__)

# Suppress warnings for cleaner output
warnings.filterwarnings('ignore')

class ParameterTuningAnalyzer:
    """
    Comprehensive analyzer for parameter tuning results from STOMP and Hauser algorithms.
    """
    
    def __init__(self, results_file: str, output_dir: str = "parameter_analysis_output"):
        """
        Initialize the analyzer with results data.
        
        Args:
            results_file: Path to CSV file containing parameter tuning results
            output_dir: Directory to save analysis outputs
        """
        self.results_file = results_file
        self.output_dir = Path(output_dir)
        self.output_dir.mkdir(exist_ok=True)
        
        # Load and preprocess data
        self.data = self._load_and_preprocess_data()
        self.stomp_data = self.data[self.data['algorithm_name'] == 'STOMP'].copy()
        self.hauser_data = self.data[self.data['algorithm_name'].str.contains('RRT|Hauser', na=False)].copy()
        
        # Analysis results storage
        self.analysis_results = {}
        self.best_configs = {}
        
        logger.info(f"Loaded {len(self.data)} parameter configurations")
        logger.info(f"STOMP configurations: {len(self.stomp_data)}")
        logger.info(f"Hauser configurations: {len(self.hauser_data)}")
    
    def _load_and_preprocess_data(self) -> pd.DataFrame:
        """Load and preprocess the parameter tuning results."""
        try:
            data = pd.read_csv(self.results_file)
            
            # Clean and standardize column names
            data.columns = data.columns.str.lower().str.replace(' ', '_')
            
            # Handle missing values
            numeric_columns = data.select_dtypes(include=[np.number]).columns
            data[numeric_columns] = data[numeric_columns].fillna(0)
            
            # Create composite objective scores
            if 'success' in data.columns:
                # Convert success rate to planning time penalty
                data.loc[~data['success'], 'planning_time_ms'] = 30000  # 30 second penalty
            
            # Normalize metrics for comparison
            if 'planning_time_ms' in data.columns and 'path_length_rad' in data.columns:
                data['composite_score'] = self._compute_composite_score(data)
            
            # Add parameter parsing for STOMP and RRT
            self._parse_parameter_strings(data)
            
            return data
            
        except Exception as e:
            logger.error(f"Error loading data: {e}")
            raise
    
    def _compute_composite_score(self, data: pd.DataFrame) -> pd.Series:
        """
        Compute composite objective score combining multiple metrics.
        Lower is better (minimization objective).
        """
        # Normalize metrics to [0, 1] scale
        time_norm = (data['planning_time_ms'] - data['planning_time_ms'].min()) / \
                   (data['planning_time_ms'].max() - data['planning_time_ms'].min() + 1e-6)
        
        path_norm = (data['path_length_rad'] - data['path_length_rad'].min()) / \
                   (data['path_length_rad'].max() - data['path_length_rad'].min() + 1e-6)
        
        # Include smoothness and safety if available
        smoothness_penalty = 0
        safety_penalty = 0
        
        if 'max_jerk' in data.columns:
            jerk_norm = (data['max_jerk'] - data['max_jerk'].min()) / \
                       (data['max_jerk'].max() - data['max_jerk'].min() + 1e-6)
            smoothness_penalty = 0.2 * jerk_norm
        
        if 'min_clearance' in data.columns:
            # Invert clearance (higher clearance is better)
            clearance_norm = 1 - ((data['min_clearance'] - data['min_clearance'].min()) / \
                                 (data['min_clearance'].max() - data['min_clearance'].min() + 1e-6))
            safety_penalty = 0.15 * clearance_norm
        
        # Weighted combination (weights sum to 1.0)
        composite = (0.4 * time_norm + 0.25 * path_norm + 
                    smoothness_penalty + safety_penalty)
        
        return composite
    
    def _parse_parameter_strings(self, data: pd.DataFrame):
        """Parse parameter strings to extract individual parameter values."""
        if 'parameter_set' not in data.columns:
            return
        
        # STOMP parameter parsing
        stomp_mask = data['algorithm_name'] == 'STOMP'
        for idx, row in data[stomp_mask].iterrows():
            param_str = row['parameter_set']
            if pd.isna(param_str):
                continue
                
            # Parse STOMP parameters (format: exp0.1_iter100_lr0.4)
            parts = param_str.split('_')
            for part in parts:
                if part.startswith('exp'):
                    data.at[idx, 'exploration_constant'] = float(part[3:])
                elif part.startswith('iter'):
                    data.at[idx, 'max_iterations'] = int(part[4:])
                elif part.startswith('lr'):
                    data.at[idx, 'learning_rate'] = float(part[2:])
        
        # RRT parameter parsing
        rrt_mask = ~stomp_mask
        for idx, row in data[rrt_mask].iterrows():
            param_str = row['parameter_set']
            if pd.isna(param_str):
                continue
                
            # Parse RRT parameters (format: RRTStar_step0.1_bias0.2_iter1000)
            parts = param_str.split('_')
            if len(parts) > 0:
                data.at[idx, 'rrt_variant'] = parts[0]
            for part in parts[1:]:
                if part.startswith('step'):
                    data.at[idx, 'step_size'] = float(part[4:])
                elif part.startswith('bias'):
                    data.at[idx, 'goal_bias'] = float(part[4:])
                elif part.startswith('iter'):
                    data.at[idx, 'max_iterations'] = int(part[4:])
    
    def generate_comprehensive_report(self):
        """Generate comprehensive analysis report."""
        logger.info("Generating comprehensive parameter tuning report...")
        
        report = {
            'timestamp': datetime.now().isoformat(),
            'data_summary': self._generate_data_summary(),
            'best_configurations': self._find_best_configurations(),
            'statistical_analysis': self._perform_statistical_analysis(),
            'parameter_importance': self._analyze_parameter_importance(),
            'convergence_analysis': self._analyze_convergence(),
            'recommendations': self._generate_recommendations()
        }
        
        # Save report
        report_file = self.output_dir / 'comprehensive_report.json'
        with open(report_file, 'w') as f:
            json.dump(report, f, indent=2, default=str)
        
        # Generate markdown report
        self._generate_markdown_report(report)
        
        logger.info(f"Report saved to {report_file}")
        return report
    
    def _generate_data_summary(self) -> Dict:
        """Generate summary statistics of the data."""
        summary = {
            'total_configurations': len(self.data),
            'stomp_configurations': len(self.stomp_data),
            'hauser_configurations': len(self.hauser_data),
            'algorithms_tested': list(self.data['algorithm_name'].unique()),
            'metrics_available': list(self.data.select_dtypes(include=[np.number]).columns),
            'success_rates': {
                'overall': self.data['success'].mean() if 'success' in self.data.columns else None,
                'stomp': self.stomp_data['success'].mean() if 'success' in self.stomp_data.columns else None,
                'hauser': self.hauser_data['success'].mean() if 'success' in self.hauser_data.columns else None
            }
        }
        
        return summary
    
    def _find_best_configurations(self) -> Dict:
        """Find best parameter configurations for each algorithm."""
        best_configs = {}
        
        if 'composite_score' in self.data.columns:
            # Find best STOMP configuration
            if len(self.stomp_data) > 0:
                best_stomp_idx = self.stomp_data['composite_score'].idxmin()
                best_configs['stomp'] = {
                    'configuration': self.stomp_data.loc[best_stomp_idx].to_dict(),
                    'score': self.stomp_data.loc[best_stomp_idx, 'composite_score']
                }
            
            # Find best Hauser configuration
            if len(self.hauser_data) > 0:
                best_hauser_idx = self.hauser_data['composite_score'].idxmin()
                best_configs['hauser'] = {
                    'configuration': self.hauser_data.loc[best_hauser_idx].to_dict(),
                    'score': self.hauser_data.loc[best_hauser_idx, 'composite_score']
                }
        
        self.best_configs = best_configs
        return best_configs
    
    def _perform_statistical_analysis(self) -> Dict:
        """Perform statistical analysis comparing algorithms and parameters."""
        analysis = {}
        
        if len(self.stomp_data) > 0 and len(self.hauser_data) > 0:
            # Compare planning times
            stomp_times = self.stomp_data['planning_time_ms'].dropna()
            hauser_times = self.hauser_data['planning_time_ms'].dropna()
            
            if len(stomp_times) > 0 and len(hauser_times) > 0:
                t_stat, p_value = stats.ttest_ind(stomp_times, hauser_times)
                analysis['planning_time_comparison'] = {
                    'stomp_mean': float(stomp_times.mean()),
                    'hauser_mean': float(hauser_times.mean()),
                    't_statistic': float(t_stat),
                    'p_value': float(p_value),
                    'significant': p_value < 0.05,
                    'effect_size': float(abs(stomp_times.mean() - hauser_times.mean()) / 
                                        np.sqrt((stomp_times.var() + hauser_times.var()) / 2))
                }
            
            # Compare success rates
            if 'success' in self.data.columns:
                stomp_success = self.stomp_data['success'].mean()
                hauser_success = self.hauser_data['success'].mean()
                
                analysis['success_rate_comparison'] = {
                    'stomp_success_rate': float(stomp_success),
                    'hauser_success_rate': float(hauser_success),
                    'difference': float(stomp_success - hauser_success)
                }
        
        return analysis
    
    def _analyze_parameter_importance(self) -> Dict:
        """Analyze parameter importance using Random Forest."""
        importance_analysis = {}
        
        for algorithm_name, algorithm_data in [('STOMP', self.stomp_data), 
                                              ('Hauser', self.hauser_data)]:
            if len(algorithm_data) < 10:  # Need minimum samples
                continue
                
            # Prepare features and target
            param_columns = []
            if algorithm_name == 'STOMP':
                param_columns = ['exploration_constant', 'max_iterations', 'learning_rate']
            else:
                param_columns = ['step_size', 'goal_bias', 'max_iterations']
            
            # Filter available columns
            available_params = [col for col in param_columns if col in algorithm_data.columns]
            if len(available_params) == 0:
                continue
            
            X = algorithm_data[available_params].dropna()
            if 'composite_score' in algorithm_data.columns:
                y = algorithm_data.loc[X.index, 'composite_score']
            elif 'planning_time_ms' in algorithm_data.columns:
                y = algorithm_data.loc[X.index, 'planning_time_ms']
            else:
                continue
            
            if len(X) < 10:
                continue
            
            # Train Random Forest
            rf = RandomForestRegressor(n_estimators=100, random_state=42)
            rf.fit(X, y)
            
            # Get feature importance
            importance_scores = dict(zip(available_params, rf.feature_importances_))
            
            # Permutation importance for more robust results
            perm_importance = permutation_importance(rf, X, y, n_repeats=10, random_state=42)
            perm_scores = dict(zip(available_params, perm_importance.importances_mean))
            
            importance_analysis[algorithm_name.lower()] = {
                'feature_importance': importance_scores,
                'permutation_importance': perm_scores,
                'model_score': float(rf.score(X, y))
            }
        
        return importance_analysis
    
    def _analyze_convergence(self) -> Dict:
        """Analyze optimization convergence patterns."""
        convergence = {}
        
        # Analyze convergence for each algorithm
        for algorithm_name, algorithm_data in [('STOMP', self.stomp_data), 
                                              ('Hauser', self.hauser_data)]:
            if len(algorithm_data) == 0:
                continue
                
            # Sort by evaluation order (if available) or by score
            if 'evaluation_order' in algorithm_data.columns:
                sorted_data = algorithm_data.sort_values('evaluation_order')
            else:
                sorted_data = algorithm_data.sort_values('composite_score' if 'composite_score' in algorithm_data.columns else 'planning_time_ms')
            
            # Compute running best
            if 'composite_score' in sorted_data.columns:
                scores = sorted_data['composite_score'].values
                running_best = np.minimum.accumulate(scores)
                
                convergence[algorithm_name.lower()] = {
                    'scores': scores.tolist(),
                    'running_best': running_best.tolist(),
                    'final_best': float(running_best[-1]),
                    'convergence_rate': self._compute_convergence_rate(running_best)
                }
        
        return convergence
    
    def _compute_convergence_rate(self, running_best: np.ndarray) -> float:
        """Compute convergence rate as improvement per iteration."""
        if len(running_best) < 2:
            return 0.0
        
        # Compute relative improvements
        improvements = []
        for i in range(1, len(running_best)):
            if running_best[i-1] > 0:
                improvement = (running_best[i-1] - running_best[i]) / running_best[i-1]
                improvements.append(improvement)
        
        return float(np.mean(improvements)) if improvements else 0.0
    
    def _generate_recommendations(self) -> Dict:
        """Generate recommendations based on analysis."""
        recommendations = {}
        
        if 'stomp' in self.best_configs:
            stomp_config = self.best_configs['stomp']['configuration']
            recommendations['stomp'] = {
                'recommended_parameters': {
                    'exploration_constant': stomp_config.get('exploration_constant', 0.1),
                    'max_iterations': int(stomp_config.get('max_iterations', 100)),
                    'learning_rate': stomp_config.get('learning_rate', 0.4)
                },
                'performance_characteristics': 'Optimized for fast planning with good trajectory quality',
                'use_cases': ['Real-time applications', 'Interactive planning', 'Simple environments']
            }
        
        if 'hauser' in self.best_configs:
            hauser_config = self.best_configs['hauser']['configuration']
            recommendations['hauser'] = {
                'recommended_parameters': {
                    'rrt_variant': hauser_config.get('rrt_variant', 'RRTStar'),
                    'step_size': hauser_config.get('step_size', 0.1),
                    'goal_bias': hauser_config.get('goal_bias', 0.2),
                    'max_iterations': int(hauser_config.get('max_iterations', 1000))
                },
                'performance_characteristics': 'Optimized for smooth trajectories and path optimality',
                'use_cases': ['High-precision applications', 'Complex environments', 'Offline planning']
            }
        
        # General recommendations
        recommendations['general'] = {
            'algorithm_selection': self._recommend_algorithm_selection(),
            'parameter_tuning_strategy': self._recommend_tuning_strategy(),
            'validation_approach': 'Validate parameters on representative test scenarios'
        }
        
        return recommendations
    
    def _recommend_algorithm_selection(self) -> str:
        """Recommend algorithm based on analysis results."""
        if 'stomp' in self.best_configs and 'hauser' in self.best_configs:
            stomp_score = self.best_configs['stomp']['score']
            hauser_score = self.best_configs['hauser']['score']
            
            if stomp_score < hauser_score * 0.9:  # STOMP significantly better
                return "STOMP recommended for most applications due to superior overall performance"
            elif hauser_score < stomp_score * 0.9:  # Hauser significantly better
                return "Hauser recommended for applications requiring high trajectory quality"
            else:
                return "Both algorithms perform similarly; choose based on specific requirements"
        
        return "Insufficient data for algorithm recommendation"
    
    def _recommend_tuning_strategy(self) -> str:
        """Recommend parameter tuning strategy."""
        return ("Use Bayesian optimization for efficient parameter search. "
                "Focus on exploration_constant and learning_rate for STOMP, "
                "step_size and max_iterations for Hauser.")
    
    def _generate_markdown_report(self, report: Dict):
        """Generate human-readable markdown report."""
        markdown_content = f"""# Parameter Tuning Analysis Report

Generated: {report['timestamp']}

## Executive Summary

This report analyzes the parameter tuning results for STOMP and Hauser trajectory planning algorithms.

### Data Overview
- **Total Configurations Tested**: {report['data_summary']['total_configurations']}
- **STOMP Configurations**: {report['data_summary']['stomp_configurations']}
- **Hauser Configurations**: {report['data_summary']['hauser_configurations']}

### Success Rates
"""
        
        if report['data_summary']['success_rates']['overall'] is not None:
            markdown_content += f"""
- **Overall**: {report['data_summary']['success_rates']['overall']:.1%}
- **STOMP**: {report['data_summary']['success_rates']['stomp']:.1%}
- **Hauser**: {report['data_summary']['success_rates']['hauser']:.1%}
"""
        
        # Best configurations
        if 'stomp' in report['best_configurations']:
            stomp_best = report['best_configurations']['stomp']
            markdown_content += f"""
## Best STOMP Configuration

- **Score**: {stomp_best['score']:.4f}
- **Exploration Constant**: {stomp_best['configuration'].get('exploration_constant', 'N/A')}
- **Max Iterations**: {stomp_best['configuration'].get('max_iterations', 'N/A')}
- **Learning Rate**: {stomp_best['configuration'].get('learning_rate', 'N/A')}
"""
        
        if 'hauser' in report['best_configurations']:
            hauser_best = report['best_configurations']['hauser']
            markdown_content += f"""
## Best Hauser Configuration

- **Score**: {hauser_best['score']:.4f}
- **RRT Variant**: {hauser_best['configuration'].get('rrt_variant', 'N/A')}
- **Step Size**: {hauser_best['configuration'].get('step_size', 'N/A')}
- **Goal Bias**: {hauser_best['configuration'].get('goal_bias', 'N/A')}
- **Max Iterations**: {hauser_best['configuration'].get('max_iterations', 'N/A')}
"""
        
        # Statistical analysis
        if 'planning_time_comparison' in report['statistical_analysis']:
            time_analysis = report['statistical_analysis']['planning_time_comparison']
            markdown_content += f"""
## Statistical Analysis

### Planning Time Comparison
- **STOMP Mean**: {time_analysis['stomp_mean']:.2f} ms
- **Hauser Mean**: {time_analysis['hauser_mean']:.2f} ms
- **Statistical Significance**: {'Yes' if time_analysis['significant'] else 'No'} (p = {time_analysis['p_value']:.4f})
- **Effect Size**: {time_analysis['effect_size']:.3f}
"""
        
        # Recommendations
        markdown_content += f"""
## Recommendations

### Algorithm Selection
{report['recommendations']['general']['algorithm_selection']}

### Parameter Tuning Strategy
{report['recommendations']['general']['parameter_tuning_strategy']}

### Validation
{report['recommendations']['general']['validation_approach']}
"""
        
        # Save markdown report
        markdown_file = self.output_dir / 'parameter_tuning_report.md'
        with open(markdown_file, 'w') as f:
            f.write(markdown_content)
        
        logger.info(f"Markdown report saved to {markdown_file}")
    
    def create_visualization_suite(self):
        """Create comprehensive visualization suite."""
        logger.info("Creating visualization suite...")
        
        # Set style
        plt.style.use('seaborn-v0_8-whitegrid')
        sns.set_palette("husl")
        
        # Create individual plots
        self._plot_performance_comparison()
        self._plot_parameter_sensitivity()
        self._plot_convergence_analysis()
        self._plot_correlation_matrix()
        self._create_interactive_dashboard()
        
        logger.info("Visualization suite completed")
    
    def _plot_performance_comparison(self):
        """Create performance comparison plots."""
        fig, axes = plt.subplots(2, 2, figsize=(15, 12))
        fig.suptitle('Algorithm Performance Comparison', fontsize=16, fontweight='bold')
        
        # Planning time comparison
        if 'planning_time_ms' in self.data.columns:
            ax = axes[0, 0]
            data_to_plot = []
            labels = []
            
            if len(self.stomp_data) > 0:
                data_to_plot.append(self.stomp_data['planning_time_ms'].dropna())
                labels.append('STOMP')
            
            if len(self.hauser_data) > 0:
                data_to_plot.append(self.hauser_data['planning_time_ms'].dropna())
                labels.append('Hauser')
            
            if data_to_plot:
                ax.boxplot(data_to_plot, labels=labels)
                ax.set_ylabel('Planning Time (ms)')
                ax.set_title('Planning Time Distribution')
                ax.set_yscale('log')
        
        # Success rate comparison
        if 'success' in self.data.columns:
            ax = axes[0, 1]
            success_rates = []
            algorithms = []
            
            if len(self.stomp_data) > 0:
                success_rates.append(self.stomp_data['success'].mean() * 100)
                algorithms.append('STOMP')
            
            if len(self.hauser_data) > 0:
                success_rates.append(self.hauser_data['success'].mean() * 100)
                algorithms.append('Hauser')
            
            if success_rates:
                bars = ax.bar(algorithms, success_rates, color=['skyblue', 'lightcoral'])
                ax.set_ylabel('Success Rate (%)')
                ax.set_title('Algorithm Success Rates')
                ax.set_ylim(0, 100)
                
                # Add value labels on bars
                for bar, rate in zip(bars, success_rates):
                    ax.text(bar.get_x() + bar.get_width()/2, bar.get_height() + 1,
                           f'{rate:.1f}%', ha='center', va='bottom')
        
        # Path length comparison
        if 'path_length_rad' in self.data.columns:
            ax = axes[1, 0]
            data_to_plot = []
            labels = []
            
            if len(self.stomp_data) > 0:
                data_to_plot.append(self.stomp_data['path_length_rad'].dropna())
                labels.append('STOMP')
            
            if len(self.hauser_data) > 0:
                data_to_plot.append(self.hauser_data['path_length_rad'].dropna())
                labels.append('Hauser')
            
            if data_to_plot:
                ax.boxplot(data_to_plot, labels=labels)
                ax.set_ylabel('Path Length (rad)')
                ax.set_title('Path Length Distribution')
        
        # Composite score comparison
        if 'composite_score' in self.data.columns:
            ax = axes[1, 1]
            data_to_plot = []
            labels = []
            
            if len(self.stomp_data) > 0:
                data_to_plot.append(self.stomp_data['composite_score'].dropna())
                labels.append('STOMP')
            
            if len(self.hauser_data) > 0:
                data_to_plot.append(self.hauser_data['composite_score'].dropna())
                labels.append('Hauser')
            
            if data_to_plot:
                ax.boxplot(data_to_plot, labels=labels)
                ax.set_ylabel('Composite Score')
                ax.set_title('Overall Performance Score\n(Lower is Better)')
        
        plt.tight_layout()
        plt.savefig(self.output_dir / 'performance_comparison.png', dpi=300, bbox_inches='tight')
        plt.close()
    
    def _plot_parameter_sensitivity(self):
        """Create parameter sensitivity analysis plots."""
        if len(self.stomp_data) > 0:
            self._plot_stomp_parameter_sensitivity()
        
        if len(self.hauser_data) > 0:
            self._plot_hauser_parameter_sensitivity()
    
    def _plot_stomp_parameter_sensitivity(self):
        """Plot STOMP parameter sensitivity."""
        param_columns = ['exploration_constant', 'max_iterations', 'learning_rate']
        available_params = [col for col in param_columns if col in self.stomp_data.columns]
        
        if len(available_params) == 0:
            return
        
        n_params = len(available_params)
        fig, axes = plt.subplots(1, n_params, figsize=(5 * n_params, 5))
        if n_params == 1:
            axes = [axes]
        
        fig.suptitle('STOMP Parameter Sensitivity Analysis', fontsize=14, fontweight='bold')
        
        target_metric = 'composite_score' if 'composite_score' in self.stomp_data.columns else 'planning_time_ms'
        
        for i, param in enumerate(available_params):
            ax = axes[i]
            
            # Create scatter plot
            x = self.stomp_data[param].dropna()
            y = self.stomp_data.loc[x.index, target_metric]
            
            ax.scatter(x, y, alpha=0.6, s=50)
            ax.set_xlabel(param.replace('_', ' ').title())
            ax.set_ylabel(target_metric.replace('_', ' ').title())
            ax.set_title(f'{param.replace("_", " ").title()} Sensitivity')
            
            # Add trend line
            if len(x) > 2:
                z = np.polyfit(x, y, 1)
                p = np.poly1d(z)
                ax.plot(x, p(x), "r--", alpha=0.8)
                
                # Calculate correlation
                corr = np.corrcoef(x, y)[0, 1]
                ax.text(0.05, 0.95, f'Correlation: {corr:.3f}', 
                       transform=ax.transAxes, bbox=dict(boxstyle="round", facecolor='wheat'))
        
        plt.tight_layout()
        plt.savefig(self.output_dir / 'stomp_parameter_sensitivity.png', dpi=300, bbox_inches='tight')
        plt.close()
    
    def _plot_hauser_parameter_sensitivity(self):
        """Plot Hauser parameter sensitivity."""
        param_columns = ['step_size', 'goal_bias', 'max_iterations']
        available_params = [col for col in param_columns if col in self.hauser_data.columns]
        
        if len(available_params) == 0:
            return
        
        n_params = len(available_params)
        fig, axes = plt.subplots(1, n_params, figsize=(5 * n_params, 5))
        if n_params == 1:
            axes = [axes]
        
        fig.suptitle('Hauser Parameter Sensitivity Analysis', fontsize=14, fontweight='bold')
        
        target_metric = 'composite_score' if 'composite_score' in self.hauser_data.columns else 'planning_time_ms'
        
        for i, param in enumerate(available_params):
            ax = axes[i]
            
            # Create scatter plot
            x = self.hauser_data[param].dropna()
            y = self.hauser_data.loc[x.index, target_metric]
            
            ax.scatter(x, y, alpha=0.6, s=50, color='coral')
            ax.set_xlabel(param.replace('_', ' ').title())
            ax.set_ylabel(target_metric.replace('_', ' ').title())
            ax.set_title(f'{param.replace("_", " ").title()} Sensitivity')
            
            # Add trend line
            if len(x) > 2:
                z = np.polyfit(x, y, 1)
                p = np.poly1d(z)
                ax.plot(x, p(x), "r--", alpha=0.8)
                
                # Calculate correlation
                corr = np.corrcoef(x, y)[0, 1]
                ax.text(0.05, 0.95, f'Correlation: {corr:.3f}', 
                       transform=ax.transAxes, bbox=dict(boxstyle="round", facecolor='wheat'))
        
        plt.tight_layout()
        plt.savefig(self.output_dir / 'hauser_parameter_sensitivity.png', dpi=300, bbox_inches='tight')
        plt.close()
    
    def _plot_convergence_analysis(self):
        """Plot optimization convergence analysis."""
        if 'composite_score' not in self.data.columns:
            return
        
        fig, axes = plt.subplots(1, 2, figsize=(14, 6))
        fig.suptitle('Optimization Convergence Analysis', fontsize=14, fontweight='bold')
        
        # STOMP convergence
        if len(self.stomp_data) > 0:
            ax = axes[0]
            scores = self.stomp_data['composite_score'].values
            running_best = np.minimum.accumulate(scores)
            
            ax.plot(range(1, len(scores) + 1), scores, 'b.', alpha=0.6, label='Evaluations')
            ax.plot(range(1, len(running_best) + 1), running_best, 'r-', linewidth=2, label='Best So Far')
            ax.set_xlabel('Evaluation Number')
            ax.set_ylabel('Objective Score')
            ax.set_title('STOMP Convergence')
            ax.legend()
            ax.grid(True, alpha=0.3)
        
        # Hauser convergence
        if len(self.hauser_data) > 0:
            ax = axes[1]
            scores = self.hauser_data['composite_score'].values
            running_best = np.minimum.accumulate(scores)
            
            ax.plot(range(1, len(scores) + 1), scores, 'b.', alpha=0.6, label='Evaluations')
            ax.plot(range(1, len(running_best) + 1), running_best, 'r-', linewidth=2, label='Best So Far')
            ax.set_xlabel('Evaluation Number')
            ax.set_ylabel('Objective Score')
            ax.set_title('Hauser Convergence')
            ax.legend()
            ax.grid(True, alpha=0.3)
        
        plt.tight_layout()
        plt.savefig(self.output_dir / 'convergence_analysis.png', dpi=300, bbox_inches='tight')
        plt.close()
    
    def _plot_correlation_matrix(self):
        """Plot correlation matrix of parameters and metrics."""
        numeric_columns = self.data.select_dtypes(include=[np.number]).columns
        if len(numeric_columns) < 2:
            return
        
        # Compute correlation matrix
        corr_matrix = self.data[numeric_columns].corr()
        
        # Create heatmap
        plt.figure(figsize=(12, 10))
        mask = np.triu(np.ones_like(corr_matrix, dtype=bool))
        
        sns.heatmap(corr_matrix, mask=mask, annot=True, cmap='coolwarm', center=0,
                   square=True, fmt='.2f', cbar_kws={"shrink": .5})
        
        plt.title('Parameter and Metric Correlation Matrix', fontsize=14, fontweight='bold')
        plt.tight_layout()
        plt.savefig(self.output_dir / 'correlation_matrix.png', dpi=300, bbox_inches='tight')
        plt.close()
    
    def _create_interactive_dashboard(self):
        """Create interactive Plotly dashboard."""
        if 'composite_score' not in self.data.columns:
            return
        
        # Create subplots
        fig = make_subplots(
            rows=2, cols=2,
            subplot_titles=('Performance vs Parameters', 'Algorithm Comparison', 
                           'Convergence Analysis', 'Parameter Importance'),
            specs=[[{"secondary_y": False}, {"secondary_y": False}],
                   [{"secondary_y": False}, {"secondary_y": False}]]
        )
        
        # Performance vs parameters scatter plot
        if len(self.stomp_data) > 0 and 'exploration_constant' in self.stomp_data.columns:
            fig.add_trace(
                go.Scatter(
                    x=self.stomp_data['exploration_constant'],
                    y=self.stomp_data['composite_score'],
                    mode='markers',
                    name='STOMP',
                    marker=dict(color='blue', size=8),
                    hovertemplate='Exploration: %{x}<br>Score: %{y:.4f}<extra></extra>'
                ),
                row=1, col=1
            )
        
        if len(self.hauser_data) > 0 and 'step_size' in self.hauser_data.columns:
            fig.add_trace(
                go.Scatter(
                    x=self.hauser_data['step_size'],
                    y=self.hauser_data['composite_score'],
                    mode='markers',
                    name='Hauser',
                    marker=dict(color='red', size=8),
                    hovertemplate='Step Size: %{x}<br>Score: %{y:.4f}<extra></extra>'
                ),
                row=1, col=1
            )
        
        # Algorithm comparison box plot
        if 'planning_time_ms' in self.data.columns:
            for i, (name, data) in enumerate([('STOMP', self.stomp_data), ('Hauser', self.hauser_data)]):
                if len(data) > 0:
                    fig.add_trace(
                        go.Box(
                            y=data['planning_time_ms'],
                            name=name,
                            boxpoints='outliers'
                        ),
                        row=1, col=2
                    )
        
        # Convergence analysis
        if len(self.stomp_data) > 0:
            running_best = np.minimum.accumulate(self.stomp_data['composite_score'].values)
            fig.add_trace(
                go.Scatter(
                    x=list(range(1, len(running_best) + 1)),
                    y=running_best,
                    mode='lines',
                    name='STOMP Convergence',
                    line=dict(color='blue')
                ),
                row=2, col=1
            )
        
        if len(self.hauser_data) > 0:
            running_best = np.minimum.accumulate(self.hauser_data['composite_score'].values)
            fig.add_trace(
                go.Scatter(
                    x=list(range(1, len(running_best) + 1)),
                    y=running_best,
                    mode='lines',
                    name='Hauser Convergence',
                    line=dict(color='red')
                ),
                row=2, col=1
            )
        
        # Update layout
        fig.update_layout(
            height=800,
            title_text="Parameter Tuning Interactive Dashboard",
            showlegend=True
        )
        
        # Save interactive plot
        fig.write_html(str(self.output_dir / 'interactive_dashboard.html'))
        logger.info("Interactive dashboard saved to interactive_dashboard.html")


def main():
    """Main function to run the parameter tuning analysis."""
    parser = argparse.ArgumentParser(description='Advanced Parameter Tuning Analysis')
    parser.add_argument('results_file', help='Path to parameter tuning results CSV file')
    parser.add_argument('--output-dir', default='parameter_analysis_output', 
                       help='Output directory for analysis results')
    parser.add_argument('--generate-plots', action='store_true', 
                       help='Generate visualization plots')
    parser.add_argument('--interactive', action='store_true', 
                       help='Create interactive dashboard')
    
    args = parser.parse_args()
    
    # Check if results file exists
    if not Path(args.results_file).exists():
        logger.error(f"Results file not found: {args.results_file}")
        return 1
    
    try:
        # Create analyzer
        analyzer = ParameterTuningAnalyzer(args.results_file, args.output_dir)
        
        # Generate comprehensive report
        report = analyzer.generate_comprehensive_report()
        logger.info("Comprehensive report generated successfully")
        
        # Generate visualizations if requested
        if args.generate_plots or args.interactive:
            analyzer.create_visualization_suite()
            logger.info("Visualization suite created successfully")
        
        # Print summary
        print("\n" + "="*60)
        print("PARAMETER TUNING ANALYSIS COMPLETED")
        print("="*60)
        print(f"Results analyzed: {len(analyzer.data)} configurations")
        print(f"Output directory: {analyzer.output_dir}")
        print(f"Reports generated: comprehensive_report.json, parameter_tuning_report.md")
        
        if args.generate_plots:
            print("Plots generated: performance_comparison.png, parameter_sensitivity.png, convergence_analysis.png")
        
        if args.interactive:
            print("Interactive dashboard: interactive_dashboard.html")
        
        print("="*60)
        
        return 0
        
    except Exception as e:
        logger.error(f"Analysis failed: {e}")
        return 1


if __name__ == "__main__":
    exit(main())