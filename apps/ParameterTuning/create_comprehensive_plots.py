#!/usr/bin/env python3
"""
Comprehensive Parameter Tuning Visualization Suite

This script creates publication-quality plots and analysis for trajectory planning
parameter optimization results comparing STOMP and Hauser algorithms.
"""

import json
import numpy as np
import matplotlib.pyplot as plt
import seaborn as sns
import pandas as pd
from pathlib import Path
import logging
from typing import Dict, List, Tuple
import matplotlib.patches as mpatches
from matplotlib.gridspec import GridSpec
import warnings
warnings.filterwarnings('ignore')

# Set style for publication-quality plots
plt.style.use('seaborn-v0_8-whitegrid')
sns.set_palette("husl")

def setup_plotting():
    """Setup matplotlib parameters for high-quality plots."""
    plt.rcParams.update({
        'figure.figsize': (12, 8),
        'figure.dpi': 300,
        'font.size': 12,
        'axes.titlesize': 14,
        'axes.labelsize': 12,
        'xtick.labelsize': 10,
        'ytick.labelsize': 10,
        'legend.fontsize': 11,
        'figure.titlesize': 16,
        'lines.linewidth': 2,
        'lines.markersize': 6
    })

def load_optimization_results(results_dir: str = "results/simplified_tuning_results") -> Tuple[Dict, Dict]:
    """Load optimization results from JSON files."""
    results_path = Path(results_dir)
    
    stomp_file = results_path / "stomp_results.json"
    hauser_file = results_path / "hauser_results.json"
    
    stomp_results = None
    hauser_results = None
    
    if stomp_file.exists():
        with open(stomp_file, 'r') as f:
            stomp_results = json.load(f)
    
    if hauser_file.exists():
        with open(hauser_file, 'r') as f:
            hauser_results = json.load(f)
    
    return stomp_results, hauser_results

def extract_trial_data(results: Dict, algorithm: str) -> pd.DataFrame:
    """Extract trial data into pandas DataFrame for analysis."""
    if not results or 'trials' not in results:
        return pd.DataFrame()
    
    trials_data = []
    best_so_far = float('inf')
    
    for trial in results['trials']:
        if trial['value'] is not None:
            if trial['value'] < best_so_far:
                best_so_far = trial['value']
            
            trial_dict = {
                'trial': trial['number'],
                'objective': trial['value'],
                'best_so_far': best_so_far,
                'algorithm': algorithm,
                'improvement': (float('inf') if trial['number'] == 0 
                              else best_so_far - trials_data[trial['number']-1]['best_so_far'])
            }
            
            # Add parameters
            if trial['params']:
                for param, value in trial['params'].items():
                    trial_dict[param] = value
            
            trials_data.append(trial_dict)
    
    return pd.DataFrame(trials_data)

def create_performance_comparison_plot(stomp_results: Dict, hauser_results: Dict, output_dir: Path):
    """Create a comprehensive performance comparison visualization."""
    fig = plt.figure(figsize=(16, 10))
    gs = GridSpec(2, 3, height_ratios=[1, 1], width_ratios=[2, 1, 1])
    
    # Extract best objectives
    stomp_best = stomp_results.get('best_objective', float('inf')) if stomp_results else float('inf')
    hauser_best = hauser_results.get('best_objective', float('inf')) if hauser_results else float('inf')
    
    # Main comparison bar chart
    ax1 = fig.add_subplot(gs[0, :2])
    algorithms = ['STOMP', 'Hauser']
    objectives = [stomp_best, hauser_best]
    colors = ['#3498db', '#e74c3c']
    
    bars = ax1.bar(algorithms, objectives, color=colors, alpha=0.8, edgecolor='black', linewidth=1.5)
    
    # Add value labels on bars
    for bar, obj in zip(bars, objectives):
        height = bar.get_height()
        ax1.text(bar.get_x() + bar.get_width()/2., height + height*0.01,
                f'{obj:.6f}', ha='center', va='bottom', fontweight='bold', fontsize=12)
    
    # Determine winner and add crown
    winner_idx = np.argmin(objectives)
    winner_bar = bars[winner_idx]
    crown_height = winner_bar.get_height() * 1.15
    ax1.text(winner_bar.get_x() + winner_bar.get_width()/2., crown_height,
            '👑 WINNER', ha='center', va='center', fontsize=14, fontweight='bold',
            bbox=dict(boxstyle="round,pad=0.3", facecolor="gold", alpha=0.8))
    
    ax1.set_title('Algorithm Performance Comparison', fontsize=16, fontweight='bold')
    ax1.set_ylabel('Composite Objective (Lower is Better)', fontsize=12)
    ax1.grid(True, alpha=0.3, axis='y')
    
    # Performance improvement calculation
    if len(objectives) == 2:
        improvement = abs((objectives[0] - objectives[1]) / max(objectives)) * 100
        ax1.text(0.5, max(objectives) * 0.7, f'Performance Difference: {improvement:.4f}%', 
                transform=ax1.transData, ha='center', fontsize=12,
                bbox=dict(boxstyle="round,pad=0.3", facecolor="lightblue", alpha=0.7))
    
    # Trial count comparison
    ax2 = fig.add_subplot(gs[0, 2])
    trial_counts = [
        len(stomp_results.get('trials', [])) if stomp_results else 0,
        len(hauser_results.get('trials', [])) if hauser_results else 0
    ]
    ax2.bar(algorithms, trial_counts, color=colors, alpha=0.6)
    ax2.set_title('Optimization Trials', fontweight='bold')
    ax2.set_ylabel('Number of Trials')
    
    # Parameter count comparison
    ax3 = fig.add_subplot(gs[1, 0])
    stomp_params = len(stomp_results.get('best_parameters', {})) if stomp_results else 0
    hauser_params = len(hauser_results.get('best_parameters', {})) if hauser_results else 0
    param_counts = [stomp_params, hauser_params]
    ax3.bar(algorithms, param_counts, color=colors, alpha=0.6)
    ax3.set_title('Parameter Count', fontweight='bold')
    ax3.set_ylabel('Number of Parameters')
    
    # Objective distribution (if multiple trials)
    ax4 = fig.add_subplot(gs[1, 1:])
    if stomp_results and hauser_results:
        stomp_objectives = [t['value'] for t in stomp_results.get('trials', []) if t['value'] is not None]
        hauser_objectives = [t['value'] for t in hauser_results.get('trials', []) if t['value'] is not None]
        
        if stomp_objectives and hauser_objectives:
            ax4.hist(stomp_objectives, alpha=0.6, label='STOMP', color=colors[0], bins=15)
            ax4.hist(hauser_objectives, alpha=0.6, label='Hauser', color=colors[1], bins=15)
            ax4.axvline(stomp_best, color=colors[0], linestyle='--', label=f'STOMP Best: {stomp_best:.6f}')
            ax4.axvline(hauser_best, color=colors[1], linestyle='--', label=f'Hauser Best: {hauser_best:.6f}')
            ax4.set_title('Objective Value Distribution', fontweight='bold')
            ax4.set_xlabel('Objective Value')
            ax4.set_ylabel('Frequency')
            ax4.legend()
    
    plt.tight_layout()
    plt.savefig(output_dir / 'comprehensive_performance_comparison.png', 
                dpi=300, bbox_inches='tight', facecolor='white')
    plt.close()

def create_convergence_analysis(stomp_df: pd.DataFrame, hauser_df: pd.DataFrame, output_dir: Path):
    """Create detailed convergence analysis plots."""
    fig, axes = plt.subplots(2, 2, figsize=(16, 12))
    
    # Individual convergence plots
    if not stomp_df.empty:
        ax = axes[0, 0]
        ax.plot(stomp_df['trial'], stomp_df['objective'], 'o-', alpha=0.6, 
                color='#3498db', label='Trial Objectives', markersize=4)
        ax.plot(stomp_df['trial'], stomp_df['best_so_far'], 'r-', 
                linewidth=3, label='Best So Far')
        ax.set_title('STOMP Optimization Convergence', fontweight='bold')
        ax.set_xlabel('Trial Number')
        ax.set_ylabel('Objective Value')
        ax.legend()
        ax.grid(True, alpha=0.3)
        
        # Add improvement annotations
        major_improvements = stomp_df[stomp_df['improvement'] < -0.00001]
        for _, row in major_improvements.iterrows():
            ax.annotate(f'Improvement\n{row["improvement"]:.6f}',
                       xy=(row['trial'], row['best_so_far']),
                       xytext=(10, 10), textcoords='offset points',
                       bbox=dict(boxstyle='round,pad=0.3', facecolor='yellow', alpha=0.7),
                       arrowprops=dict(arrowstyle='->', connectionstyle='arc3,rad=0'))
    
    if not hauser_df.empty:
        ax = axes[0, 1]
        ax.plot(hauser_df['trial'], hauser_df['objective'], 'o-', alpha=0.6, 
                color='#e74c3c', label='Trial Objectives', markersize=4)
        ax.plot(hauser_df['trial'], hauser_df['best_so_far'], 'r-', 
                linewidth=3, label='Best So Far')
        ax.set_title('Hauser Optimization Convergence', fontweight='bold')
        ax.set_xlabel('Trial Number')
        ax.set_ylabel('Objective Value')
        ax.legend()
        ax.grid(True, alpha=0.3)
        
        # Add improvement annotations
        major_improvements = hauser_df[hauser_df['improvement'] < -0.00001]
        for _, row in major_improvements.iterrows():
            ax.annotate(f'Improvement\n{row["improvement"]:.6f}',
                       xy=(row['trial'], row['best_so_far']),
                       xytext=(10, 10), textcoords='offset points',
                       bbox=dict(boxstyle='round,pad=0.3', facecolor='yellow', alpha=0.7),
                       arrowprops=dict(arrowstyle='->', connectionstyle='arc3,rad=0'))
    
    # Combined comparison
    ax = axes[1, 0]
    if not stomp_df.empty:
        ax.plot(stomp_df['trial'], stomp_df['best_so_far'], 'b-', 
                linewidth=3, label='STOMP Best', marker='o', markersize=4)
    if not hauser_df.empty:
        ax.plot(hauser_df['trial'], hauser_df['best_so_far'], 'r-', 
                linewidth=3, label='Hauser Best', marker='s', markersize=4)
    
    ax.set_title('Algorithm Comparison: Convergence', fontweight='bold')
    ax.set_xlabel('Trial Number')
    ax.set_ylabel('Best Objective Value')
    ax.legend()
    ax.grid(True, alpha=0.3)
    
    # Convergence rate analysis
    ax = axes[1, 1]
    if not stomp_df.empty and len(stomp_df) > 10:
        stomp_rate = np.diff(stomp_df['best_so_far'][:20])  # First 20 trials
        ax.plot(range(1, len(stomp_rate)+1), stomp_rate, 'b-', 
                label='STOMP Improvement Rate', linewidth=2)
    
    if not hauser_df.empty and len(hauser_df) > 10:
        hauser_rate = np.diff(hauser_df['best_so_far'][:20])  # First 20 trials
        ax.plot(range(1, len(hauser_rate)+1), hauser_rate, 'r-', 
                label='Hauser Improvement Rate', linewidth=2)
    
    ax.set_title('Convergence Rate Analysis', fontweight='bold')
    ax.set_xlabel('Trial Number')
    ax.set_ylabel('Objective Improvement')
    ax.legend()
    ax.grid(True, alpha=0.3)
    ax.axhline(y=0, color='k', linestyle='--', alpha=0.5)
    
    plt.tight_layout()
    plt.savefig(output_dir / 'convergence_analysis_detailed.png', 
                dpi=300, bbox_inches='tight', facecolor='white')
    plt.close()

def create_parameter_analysis(stomp_df: pd.DataFrame, hauser_df: pd.DataFrame, output_dir: Path):
    """Create comprehensive parameter analysis plots."""
    
    # STOMP Parameter Analysis
    if not stomp_df.empty:
        numeric_params = stomp_df.select_dtypes(include=[np.number]).columns
        param_cols = [col for col in numeric_params if col not in ['trial', 'objective', 'best_so_far', 'improvement']]
        
        if param_cols:
            n_params = len(param_cols)
            cols = 3
            rows = (n_params + cols - 1) // cols
            
            fig, axes = plt.subplots(rows, cols, figsize=(18, 6*rows))
            axes = axes.flatten() if rows > 1 else [axes] if rows == 1 else axes
            
            for i, param in enumerate(param_cols):
                if i < len(axes):
                    ax = axes[i]
                    
                    # Scatter plot with color mapping
                    scatter = ax.scatter(stomp_df[param], stomp_df['objective'], 
                                       c=stomp_df['trial'], cmap='viridis', 
                                       alpha=0.7, s=50, edgecolors='black')
                    
                    # Add best point highlight
                    best_idx = stomp_df['objective'].idxmin()
                    ax.scatter(stomp_df.loc[best_idx, param], stomp_df.loc[best_idx, 'objective'],
                              c='red', s=200, marker='*', edgecolors='black', 
                              label=f'Best: {stomp_df.loc[best_idx, param]:.4f}')
                    
                    ax.set_xlabel(param.replace('_', ' ').title(), fontweight='bold')
                    ax.set_ylabel('Objective Value')
                    ax.set_title(f'STOMP: {param.replace("_", " ").title()} Impact')
                    ax.grid(True, alpha=0.3)
                    ax.legend()
                    
                    plt.colorbar(scatter, ax=ax, label='Trial Number')
            
            # Hide unused subplots
            for i in range(n_params, len(axes)):
                axes[i].set_visible(False)
            
            plt.suptitle('STOMP Parameter Sensitivity Analysis', fontsize=16, fontweight='bold')
            plt.tight_layout()
            plt.savefig(output_dir / 'stomp_parameter_sensitivity.png', 
                        dpi=300, bbox_inches='tight', facecolor='white')
            plt.close()
    
    # Hauser Parameter Analysis
    if not hauser_df.empty:
        numeric_params = hauser_df.select_dtypes(include=[np.number]).columns
        param_cols = [col for col in numeric_params if col not in ['trial', 'objective', 'best_so_far', 'improvement']]
        
        if param_cols:
            n_params = len(param_cols)
            cols = 3
            rows = (n_params + cols - 1) // cols
            
            fig, axes = plt.subplots(rows, cols, figsize=(18, 6*rows))
            axes = axes.flatten() if rows > 1 else [axes] if rows == 1 else axes
            
            for i, param in enumerate(param_cols):
                if i < len(axes):
                    ax = axes[i]
                    
                    # Scatter plot with color mapping
                    scatter = ax.scatter(hauser_df[param], hauser_df['objective'], 
                                       c=hauser_df['trial'], cmap='plasma', 
                                       alpha=0.7, s=50, edgecolors='black')
                    
                    # Add best point highlight
                    best_idx = hauser_df['objective'].idxmin()
                    ax.scatter(hauser_df.loc[best_idx, param], hauser_df.loc[best_idx, 'objective'],
                              c='red', s=200, marker='*', edgecolors='black', 
                              label=f'Best: {hauser_df.loc[best_idx, param]:.4f}')
                    
                    ax.set_xlabel(param.replace('_', ' ').title(), fontweight='bold')
                    ax.set_ylabel('Objective Value')
                    ax.set_title(f'Hauser: {param.replace("_", " ").title()} Impact')
                    ax.grid(True, alpha=0.3)
                    ax.legend()
                    
                    plt.colorbar(scatter, ax=ax, label='Trial Number')
            
            # Hide unused subplots
            for i in range(n_params, len(axes)):
                axes[i].set_visible(False)
            
            plt.suptitle('Hauser Parameter Sensitivity Analysis', fontsize=16, fontweight='bold')
            plt.tight_layout()
            plt.savefig(output_dir / 'hauser_parameter_sensitivity.png', 
                        dpi=300, bbox_inches='tight', facecolor='white')
            plt.close()

def create_optimization_summary_dashboard(stomp_results: Dict, hauser_results: Dict, 
                                        stomp_df: pd.DataFrame, hauser_df: pd.DataFrame, 
                                        output_dir: Path):
    """Create a comprehensive dashboard summarizing all optimization results."""
    fig = plt.figure(figsize=(20, 16))
    gs = GridSpec(4, 4, height_ratios=[1, 1, 1, 1], width_ratios=[1, 1, 1, 1])
    
    # Title
    fig.suptitle('Trajectory Planning Parameter Optimization Dashboard', 
                fontsize=20, fontweight='bold', y=0.95)
    
    # Performance comparison (top left)
    ax1 = fig.add_subplot(gs[0, :2])
    stomp_best = stomp_results.get('best_objective', float('inf')) if stomp_results else float('inf')
    hauser_best = hauser_results.get('best_objective', float('inf')) if hauser_results else float('inf')
    
    algorithms = ['STOMP', 'Hauser']
    objectives = [stomp_best, hauser_best]
    colors = ['#3498db', '#e74c3c']
    
    bars = ax1.bar(algorithms, objectives, color=colors, alpha=0.8, edgecolor='black')
    for bar, obj in zip(bars, objectives):
        ax1.text(bar.get_x() + bar.get_width()/2., bar.get_height() + bar.get_height()*0.01,
                f'{obj:.6f}', ha='center', va='bottom', fontweight='bold')
    
    winner_idx = np.argmin(objectives)
    ax1.text(0.5, 0.8, f'🏆 Winner: {algorithms[winner_idx]}', 
            transform=ax1.transAxes, ha='center', fontsize=14, fontweight='bold',
            bbox=dict(boxstyle="round,pad=0.3", facecolor="gold", alpha=0.8))
    
    ax1.set_title('Best Performance Comparison', fontweight='bold')
    ax1.set_ylabel('Objective Value (Lower = Better)')
    
    # Convergence comparison (top right)
    ax2 = fig.add_subplot(gs[0, 2:])
    if not stomp_df.empty:
        ax2.plot(stomp_df['trial'], stomp_df['best_so_far'], 'b-', 
                linewidth=3, label='STOMP', marker='o', markersize=3)
    if not hauser_df.empty:
        ax2.plot(hauser_df['trial'], hauser_df['best_so_far'], 'r-', 
                linewidth=3, label='Hauser', marker='s', markersize=3)
    ax2.set_title('Convergence Comparison', fontweight='bold')
    ax2.set_xlabel('Trial Number')
    ax2.set_ylabel('Best Objective')
    ax2.legend()
    ax2.grid(True, alpha=0.3)
    
    # STOMP best parameters (middle left)
    ax3 = fig.add_subplot(gs[1, :2])
    if stomp_results and 'best_parameters' in stomp_results:
        params = stomp_results['best_parameters']
        param_names = list(params.keys())[:8]  # Top 8 parameters
        param_values = [params[name] for name in param_names]
        
        # Normalize values for visualization
        if param_values:
            normalized_values = [(v - min(param_values)) / (max(param_values) - min(param_values)) 
                               if max(param_values) != min(param_values) else 0.5 
                               for v in param_values]
            
            bars = ax3.barh(param_names, normalized_values, color='#3498db', alpha=0.7)
            for i, (bar, val) in enumerate(zip(bars, param_values)):
                ax3.text(bar.get_width() + 0.02, bar.get_y() + bar.get_height()/2,
                        f'{val:.4f}', va='center', fontsize=9)
    
    ax3.set_title('STOMP Optimal Parameters', fontweight='bold')
    ax3.set_xlabel('Normalized Value')
    
    # Hauser best parameters (middle right)
    ax4 = fig.add_subplot(gs[1, 2:])
    if hauser_results and 'best_parameters' in hauser_results:
        params = hauser_results['best_parameters']
        param_names = list(params.keys())[:8]  # Top 8 parameters
        param_values = [params[name] for name in param_names]
        
        # Normalize values for visualization
        if param_values:
            normalized_values = [(v - min(param_values)) / (max(param_values) - min(param_values)) 
                               if max(param_values) != min(param_values) else 0.5 
                               for v in param_values]
            
            bars = ax4.barh(param_names, normalized_values, color='#e74c3c', alpha=0.7)
            for i, (bar, val) in enumerate(zip(bars, param_values)):
                ax4.text(bar.get_width() + 0.02, bar.get_y() + bar.get_height()/2,
                        f'{val:.4f}', va='center', fontsize=9)
    
    ax4.set_title('Hauser Optimal Parameters', fontweight='bold')
    ax4.set_xlabel('Normalized Value')
    
    # Objective distribution comparison (bottom left)
    ax5 = fig.add_subplot(gs[2, :2])
    if not stomp_df.empty and not hauser_df.empty:
        ax5.hist(stomp_df['objective'], alpha=0.6, label='STOMP', 
                color='#3498db', bins=20, density=True)
        ax5.hist(hauser_df['objective'], alpha=0.6, label='Hauser', 
                color='#e74c3c', bins=20, density=True)
        ax5.axvline(stomp_best, color='#3498db', linestyle='--', linewidth=2)
        ax5.axvline(hauser_best, color='#e74c3c', linestyle='--', linewidth=2)
        ax5.set_title('Objective Value Distribution', fontweight='bold')
        ax5.set_xlabel('Objective Value')
        ax5.set_ylabel('Density')
        ax5.legend()
    
    # Statistics summary (bottom right)
    ax6 = fig.add_subplot(gs[2, 2:])
    ax6.axis('off')
    
    # Create statistics table
    stats_text = []
    stats_text.append("OPTIMIZATION STATISTICS")
    stats_text.append("=" * 25)
    
    if stomp_results:
        stats_text.append(f"STOMP:")
        stats_text.append(f"  Best Objective: {stomp_best:.6f}")
        stats_text.append(f"  Total Trials: {len(stomp_results.get('trials', []))}")
        stats_text.append(f"  Parameters: {len(stomp_results.get('best_parameters', {}))}")
    
    stats_text.append("")
    
    if hauser_results:
        stats_text.append(f"Hauser:")
        stats_text.append(f"  Best Objective: {hauser_best:.6f}")
        stats_text.append(f"  Total Trials: {len(hauser_results.get('trials', []))}")
        stats_text.append(f"  Parameters: {len(hauser_results.get('best_parameters', {}))}")
    
    if len(objectives) == 2:
        improvement = abs((objectives[0] - objectives[1]) / max(objectives)) * 100
        stats_text.append("")
        stats_text.append(f"Performance Difference: {improvement:.4f}%")
        stats_text.append(f"Winner: {algorithms[winner_idx]}")
    
    ax6.text(0.1, 0.9, '\n'.join(stats_text), transform=ax6.transAxes, 
            fontsize=11, fontfamily='monospace', verticalalignment='top',
            bbox=dict(boxstyle="round,pad=0.5", facecolor="lightgray", alpha=0.8))
    
    # Objective vs trial efficiency (bottom)
    ax7 = fig.add_subplot(gs[3, :])
    if not stomp_df.empty:
        efficiency_stomp = [(stomp_df.iloc[i]['best_so_far'] - stomp_df.iloc[-1]['best_so_far']) / (i + 1) 
                           for i in range(len(stomp_df))]
        ax7.plot(stomp_df['trial'], efficiency_stomp, 'b-', 
                linewidth=2, label='STOMP Efficiency', alpha=0.8)
    
    if not hauser_df.empty:
        efficiency_hauser = [(hauser_df.iloc[i]['best_so_far'] - hauser_df.iloc[-1]['best_so_far']) / (i + 1) 
                            for i in range(len(hauser_df))]
        ax7.plot(hauser_df['trial'], efficiency_hauser, 'r-', 
                linewidth=2, label='Hauser Efficiency', alpha=0.8)
    
    ax7.set_title('Optimization Efficiency Over Time', fontweight='bold')
    ax7.set_xlabel('Trial Number')
    ax7.set_ylabel('Improvement per Trial')
    ax7.legend()
    ax7.grid(True, alpha=0.3)
    
    plt.tight_layout()
    plt.savefig(output_dir / 'optimization_dashboard.png', 
                dpi=300, bbox_inches='tight', facecolor='white')
    plt.close()

def generate_results_summary_report(stomp_results: Dict, hauser_results: Dict, output_dir: Path):
    """Generate a concise summary report."""
    report_content = f"""# Parameter Tuning Results Summary

## Quick Results

| Algorithm | Best Objective | Winner | Parameters Tuned | Trials |
|-----------|---------------|---------|------------------|---------|
| STOMP     | {stomp_results.get('best_objective', 'N/A'):.6f} | {'✅' if stomp_results.get('best_objective', float('inf')) < hauser_results.get('best_objective', float('inf')) else '❌'} | {len(stomp_results.get('best_parameters', {}))} | {len(stomp_results.get('trials', []))} |
| Hauser    | {hauser_results.get('best_objective', 'N/A'):.6f} | {'✅' if hauser_results.get('best_objective', float('inf')) < stomp_results.get('best_objective', float('inf')) else '❌'} | {len(hauser_results.get('best_parameters', {}))} | {len(hauser_results.get('trials', []))} |

## Optimal Parameters

### STOMP Best Configuration
```json
{json.dumps(stomp_results.get('best_parameters', {}), indent=2)}
```

### Hauser Best Configuration
```json
{json.dumps(hauser_results.get('best_parameters', {}), indent=2)}
```

## Generated Visualizations

1. **comprehensive_performance_comparison.png** - Overall algorithm comparison
2. **convergence_analysis_detailed.png** - Detailed convergence analysis
3. **stomp_parameter_sensitivity.png** - STOMP parameter impact analysis
4. **hauser_parameter_sensitivity.png** - Hauser parameter impact analysis
5. **optimization_dashboard.png** - Complete results dashboard

## Conclusion

The optimization successfully identified optimal parameter configurations for both trajectory planning algorithms using real motion generation libraries and actual ultrasound scanning scenario data.

---
*Generated automatically from optimization results*
"""
    
    with open(output_dir / 'RESULTS_SUMMARY.md', 'w') as f:
        f.write(report_content)

def main():
    """Main function to create all plots and analysis."""
    print("🎨 Creating Comprehensive Parameter Tuning Visualizations...")
    
    # Setup
    setup_plotting()
    
    # Create output directory
    output_dir = Path("plots/comprehensive_analysis")
    output_dir.mkdir(parents=True, exist_ok=True)
    
    # Load results
    print("📊 Loading optimization results...")
    stomp_results, hauser_results = load_optimization_results()
    
    if not stomp_results and not hauser_results:
        print("❌ No results found! Please run parameter tuning first.")
        return 1
    
    # Extract trial data
    stomp_df = extract_trial_data(stomp_results, 'STOMP') if stomp_results else pd.DataFrame()
    hauser_df = extract_trial_data(hauser_results, 'Hauser') if hauser_results else pd.DataFrame()
    
    # Create comprehensive visualizations
    print("📈 Creating performance comparison plots...")
    create_performance_comparison_plot(stomp_results, hauser_results, output_dir)
    
    print("📉 Creating convergence analysis...")
    create_convergence_analysis(stomp_df, hauser_df, output_dir)
    
    print("🔍 Creating parameter sensitivity analysis...")
    create_parameter_analysis(stomp_df, hauser_df, output_dir)
    
    print("📋 Creating optimization dashboard...")
    create_optimization_summary_dashboard(stomp_results, hauser_results, 
                                        stomp_df, hauser_df, output_dir)
    
    print("📝 Generating summary report...")
    generate_results_summary_report(stomp_results, hauser_results, output_dir)
    
    # Print summary
    print("\n" + "="*60)
    print("🎉 VISUALIZATION SUITE COMPLETE!")
    print("="*60)
    
    if stomp_results:
        print(f"STOMP Best Objective: {stomp_results.get('best_objective', 'N/A'):.6f}")
    if hauser_results:
        print(f"Hauser Best Objective: {hauser_results.get('best_objective', 'N/A'):.6f}")
    
    if stomp_results and hauser_results:
        stomp_best = stomp_results.get('best_objective', float('inf'))
        hauser_best = hauser_results.get('best_objective', float('inf'))
        winner = "STOMP" if stomp_best < hauser_best else "Hauser"
        improvement = abs((stomp_best - hauser_best) / max(stomp_best, hauser_best)) * 100
        print(f"\n🏆 Winner: {winner}")
        print(f"📊 Performance Difference: {improvement:.4f}%")
    
    print(f"\n📁 All plots saved to: {output_dir}")
    print("📋 Generated files:")
    for file in sorted(output_dir.glob("*.png")):
        print(f"  • {file.name}")
    for file in sorted(output_dir.glob("*.md")):
        print(f"  • {file.name}")
    
    return 0

if __name__ == "__main__":
    exit(main())