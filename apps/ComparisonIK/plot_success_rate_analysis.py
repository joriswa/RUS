#!/usr/bin/env python3
"""
Success Rate Analysis: Bar charts for SA-Optimized vs Newton-Raphson
Generates separate plots for:
1. Overall success rate (all attempts)
2. Collision-free success rate
"""

import pandas as pd
import matplotlib.pyplot as plt
import seaborn as sns
import numpy as np
import os
from datetime import datetime

def plot_success_rate_analysis():
    """Generate success rate bar charts for different categories."""
    
    print("📊 SUCCESS RATE ANALYSIS")
    print("="*50)
    
    # Load data
    df = pd.read_csv('two_method_comparison_results.csv')
    print(f"📊 Loaded {len(df)} data points")
    
    # Set consistent styling
    plt.style.use('default')
    
    # Create two separate plots
    fig, axes = plt.subplots(1, 2, figsize=(12, 6))
    fig.suptitle('IK Method Success Rate Comparison', fontsize=16, fontweight='bold')
    
    # Define consistent colors (default matplotlib colors)
    colors = ['#1f77b4', '#ff7f0e']  # Blue, Orange (default matplotlib)
    method_colors = {'Newton-Raphson': colors[0], 'SA-Optimized': colors[1]}
    
    # 1. Overall Success Rate (All Attempts)
    ax1 = axes[0]
    
    # Calculate overall success rates
    overall_stats = df.groupby('method').agg({
        'success': ['count', 'sum', 'mean']
    }).round(4)
    overall_stats.columns = ['total_attempts', 'successful_attempts', 'success_rate']
    overall_stats['success_percentage'] = overall_stats['success_rate'] * 100
    overall_stats = overall_stats.reset_index()
    
    bars1 = ax1.bar(overall_stats['method'], overall_stats['success_percentage'], 
                    color=[method_colors[method] for method in overall_stats['method']],
                    alpha=0.8, edgecolor='black')
    
    ax1.set_title('Overall Success Rate\n(All Attempts)', fontsize=14, fontweight='bold')
    ax1.set_ylabel('Success Rate (%)')
    ax1.set_xlabel('Method')
    ax1.set_ylim(0, 100)
    ax1.grid(axis='y', alpha=0.3)
    
    # Add percentage labels on bars
    for i, (_, row) in enumerate(overall_stats.iterrows()):
        height = row['success_percentage']
        ax1.text(i, height + 2, 
                f"{height:.1f}%\n({row['successful_attempts']:.0f}/{row['total_attempts']:.0f})", 
                ha='center', va='bottom', fontweight='bold', fontsize=11)
    
    # 2. Overall Collision-Free Rate (All Attempts)
    ax2 = axes[1]
    
    # Calculate overall collision-free rate (all attempts, not just successful)
    overall_cf_stats = df.groupby('method').agg({
        'collision_free': ['count', 'sum', 'mean']
    }).round(4)
    overall_cf_stats.columns = ['total_attempts', 'collision_free_attempts', 'cf_rate']
    overall_cf_stats['cf_percentage'] = overall_cf_stats['cf_rate'] * 100
    overall_cf_stats = overall_cf_stats.reset_index()
    
    bars2 = ax2.bar(overall_cf_stats['method'], overall_cf_stats['cf_percentage'], 
                    color=[method_colors[method] for method in overall_cf_stats['method']],
                    alpha=0.8, edgecolor='black')
    
    ax2.set_title('Overall Collision-Free Rate\n(All Attempts)', fontsize=14, fontweight='bold')
    ax2.set_ylabel('Collision-Free Rate (%)')
    ax2.set_xlabel('Method')
    ax2.set_ylim(0, 100)
    ax2.grid(axis='y', alpha=0.3)
    
    # Add percentage labels on bars
    for i, (_, row) in enumerate(overall_cf_stats.iterrows()):
        height = row['cf_percentage']
        ax2.text(i, height + 2, 
                f"{height:.1f}%\n({row['collision_free_attempts']:.0f}/{row['total_attempts']:.0f})", 
                ha='center', va='bottom', fontweight='bold', fontsize=11)
    
    plt.tight_layout()
    
    # Save plots
    os.makedirs('plots', exist_ok=True)
    plt.savefig('plots/success_rate_analysis.png', dpi=300, bbox_inches='tight')
    plt.savefig('plots/success_rate_analysis.pdf', bbox_inches='tight')
    plt.close()
    
    # Print summary statistics
    print(f"\n📈 SUCCESS RATE SUMMARY")
    print("-" * 40)
    
    print(f"\nOverall Success Rates:")
    for _, row in overall_stats.iterrows():
        print(f"  {row['method']}: {row['success_percentage']:.1f}% "
              f"({row['successful_attempts']:.0f}/{row['total_attempts']:.0f})")
    
    print(f"\nOverall Collision-Free Rates:")
    for _, row in overall_cf_stats.iterrows():
        print(f"  {row['method']}: {row['cf_percentage']:.1f}% "
              f"({row['collision_free_attempts']:.0f}/{row['total_attempts']:.0f})")
    
    # Additional breakdown
    print(f"\nDetailed Breakdown:")
    for method in df['method'].unique():
        method_data = df[df['method'] == method]
        total = len(method_data)
        successful = method_data['success'].sum()
        collision_free = method_data['collision_free'].sum()
        both_success_cf = method_data[(method_data['success'] == 1) & 
                                     (method_data['collision_free'] == 1)].shape[0]
        
        print(f"  {method}:")
        print(f"    Total attempts: {total}")
        print(f"    Successful: {successful} ({successful/total*100:.1f}%)")
        print(f"    Collision-free: {collision_free} ({collision_free/total*100:.1f}%)")
        print(f"    Both successful & collision-free: {both_success_cf} ({both_success_cf/total*100:.1f}%)")
    
    print(f"\n✅ Generated: plots/success_rate_analysis.png")
    print(f"✅ Generated: plots/success_rate_analysis.pdf")

if __name__ == "__main__":
    plot_success_rate_analysis()
