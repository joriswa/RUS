#!/usr/bin/env python3
"""
STOMP Optimization Results Dashboard
Quick interactive analysis of the optimization results
"""

import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import json
from pathlib import Path

def load_and_analyze_results():
    """Load and provide quick analysis of STOMP results."""
    
    # Load data
    data_dir = Path("trajectory_optimization/data/results/stomp")
    trials_df = pd.read_csv(data_dir / "trials.csv")
    
    with open(data_dir / "best_config.json", 'r') as f:
        best_config = json.load(f)
    
    print("=" * 60)
    print("🚀 STOMP OPTIMIZATION RESULTS DASHBOARD")
    print("=" * 60)
    
    # Basic statistics
    print(f"\n📊 OPTIMIZATION SUMMARY")
    print(f"   Total Trials: {len(trials_df)}")
    print(f"   Best Value: {best_config['best_value']:.4f}")
    print(f"   Mean Value: {trials_df['value'].mean():.4f} ± {trials_df['value'].std():.4f}")
    print(f"   Median Value: {trials_df['value'].median():.4f}")
    print(f"   Improvement: {((trials_df['value'].iloc[0] - best_config['best_value']) / trials_df['value'].iloc[0] * 100):.1f}%")
    
    # Best trial info
    best_trial_idx = trials_df['value'].idxmin()
    best_trial = trials_df.loc[best_trial_idx]
    print(f"\n🎯 BEST TRIAL")
    print(f"   Trial Number: {best_trial['number']}")
    print(f"   Objective Value: {best_trial['value']:.4f}")
    print(f"   Planning Time: {best_trial['user_attrs_planning_time']:.2f}s")
    print(f"   Success Rate: {best_trial['user_attrs_success_rate']:.2f}")
    print(f"   Safety Clearance: {best_trial['user_attrs_safety_clearance']:.4f}")
    
    # Parameter analysis
    param_columns = [col for col in trials_df.columns if col.startswith('params_stomp_')]
    print(f"\n🔧 PARAMETER IMPORTANCE (Top 5)")
    
    param_importance = []
    for param in param_columns:
        correlation = abs(np.corrcoef(trials_df[param], trials_df['value'])[0, 1])
        if not np.isnan(correlation):
            param_name = param.replace('params_stomp_', '').replace('_', ' ')
            param_importance.append((param_name, correlation))
    
    param_importance.sort(key=lambda x: x[1], reverse=True)
    for i, (name, importance) in enumerate(param_importance[:5], 1):
        print(f"   {i}. {name.title()}: {importance:.4f}")
    
    # Best configuration
    print(f"\n⚙️  OPTIMAL CONFIGURATION")
    for param, value in best_config['best_params'].items():
        param_name = param.replace('stomp_', '').replace('_', ' ').title()
        if isinstance(value, float):
            print(f"   {param_name}: {value:.4f}")
        else:
            print(f"   {param_name}: {value}")
    
    # Convergence analysis
    print(f"\n📈 CONVERGENCE ANALYSIS")
    running_min = trials_df['value'].cummin()
    
    # Find when we achieved 90% of final improvement
    total_improvement = trials_df['value'].iloc[0] - best_config['best_value']
    target_value = trials_df['value'].iloc[0] - 0.9 * total_improvement
    convergence_trial = trials_df[trials_df['value'] <= target_value]['number'].iloc[0] if len(trials_df[trials_df['value'] <= target_value]) > 0 else len(trials_df)
    
    print(f"   90% Convergence at Trial: {convergence_trial}")
    print(f"   Final 10% of Trials Improvement: {(running_min.iloc[-40:].iloc[0] - running_min.iloc[-1]):.4f}")
    print(f"   Last Improvement at Trial: {trials_df[running_min.diff() < 0]['number'].iloc[-1] if len(trials_df[running_min.diff() < 0]) > 0 else 'No improvement'}")
    
    # Performance distribution
    print(f"\n🎯 PERFORMANCE DISTRIBUTION")
    percentiles = [10, 25, 50, 75, 90]
    values = [np.percentile(trials_df['value'], p) for p in percentiles]
    for p, v in zip(percentiles, values):
        print(f"   {p}th percentile: {v:.4f}")
    
    # Top 10 trials
    print(f"\n🏆 TOP 10 TRIALS")
    top_trials = trials_df.nsmallest(10, 'value')
    for _, trial in top_trials.iterrows():
        print(f"   Trial {int(trial['number']):3d}: {trial['value']:.4f} "
              f"(Planning: {trial['user_attrs_planning_time']:.1f}s, "
              f"Success: {trial['user_attrs_success_rate']:.2f})")
    
    print(f"\n📁 FILES GENERATED")
    viz_dir = Path("trajectory_optimization/data/visualizations")
    if viz_dir.exists():
        for file in viz_dir.glob("*.png"):
            print(f"   📊 {file.name}")
        for file in viz_dir.glob("*.md"):
            print(f"   📝 {file.name}")
    
    print(f"\n✅ Analysis complete! Check the visualizations folder for detailed plots.")
    print("=" * 60)
    
    return trials_df, best_config

def quick_plot():
    """Generate a quick summary plot."""
    data_dir = Path("trajectory_optimization/data/results/stomp")
    trials_df = pd.read_csv(data_dir / "trials.csv")
    
    with open(data_dir / "best_config.json", 'r') as f:
        best_config = json.load(f)
    
    fig, axes = plt.subplots(1, 2, figsize=(12, 5))
    
    # Convergence plot
    ax1 = axes[0]
    ax1.plot(trials_df['number'], trials_df['value'], alpha=0.5, linewidth=0.8, color='lightblue')
    running_min = trials_df['value'].cummin()
    ax1.plot(trials_df['number'], running_min, color='red', linewidth=2, label='Running Best')
    
    best_idx = trials_df['value'].idxmin()
    best_trial = trials_df.loc[best_idx]
    ax1.scatter(best_trial['number'], best_trial['value'], color='red', s=100, 
               marker='*', zorder=5, label=f'Best: {best_trial["value"]:.4f}')
    
    ax1.set_xlabel('Trial Number')
    ax1.set_ylabel('Objective Value')
    ax1.set_title('STOMP Optimization Convergence')
    ax1.legend()
    ax1.grid(True, alpha=0.3)
    
    # Distribution plot
    ax2 = axes[1]
    ax2.hist(trials_df['value'], bins=30, alpha=0.7, color='skyblue', edgecolor='black')
    ax2.axvline(trials_df['value'].mean(), color='orange', linestyle='--', 
               label=f'Mean: {trials_df["value"].mean():.4f}')
    ax2.axvline(best_config['best_value'], color='red', linestyle='-', linewidth=2,
               label=f'Best: {best_config["best_value"]:.4f}')
    ax2.set_xlabel('Objective Value')
    ax2.set_ylabel('Frequency')
    ax2.set_title('Objective Value Distribution')
    ax2.legend()
    ax2.grid(True, alpha=0.3)
    
    plt.tight_layout()
    
    # Save quick plot
    output_path = Path("trajectory_optimization/data/quick_summary.png")
    plt.savefig(output_path, dpi=150, bbox_inches='tight')
    print(f"\n📊 Quick summary plot saved to: {output_path}")
    
    plt.show()

if __name__ == "__main__":
    # Run analysis
    trials_df, best_config = load_and_analyze_results()
    
    # Generate quick plot
    quick_plot()
