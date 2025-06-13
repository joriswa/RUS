#!/usr/bin/env python3
"""
Two Execution Time Boxplots

Creates two separate boxplots:
1. Successful solutions only
2. All solutions (including failures)
"""

import pandas as pd
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import seaborn as sns

def create_two_boxplots():
    """Create two separate boxplots for execution times."""
    
    print("Loading data...")
    # Load data
    df = pd.read_csv('two_method_comparison_results.csv')
    
    # Filter successful solutions
    df_success = df[df['success'] == True]
    
    print(f"Loaded data:")
    print(f"  Total solutions: {len(df)}")
    print(f"  Successful solutions: {len(df_success)}")
    print(f"  Failed solutions: {len(df) - len(df_success)}")
    
    print("Creating figure...")
    # Create figure with two subplots
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(14, 6))
    
    print("Creating first boxplot...")
    # Plot 1: Successful solutions only
    sns.boxplot(data=df_success, x='method', y='time_ms', ax=ax1)
    ax1.set_title('Execution Time - Successful Solutions Only')
    ax1.set_xlabel('IK Method')
    ax1.set_ylabel('Execution Time (ms)')
    ax1.grid(True, alpha=0.3)
    
    print("Adding statistics to first plot...")
    # Add statistics for successful solutions
    for i, method in enumerate(df_success['method'].unique()):
        method_data = df_success[df_success['method'] == method]
        n_success = len(method_data)
        mean_time = method_data['time_ms'].mean()
        ax1.text(i, ax1.get_ylim()[1] * 0.9, 
                f'n={n_success}\nmean={mean_time:.1f}ms', 
                ha='center', va='top', 
                bbox=dict(boxstyle='round', facecolor='C0', alpha=0.3))
    
    print("Creating second boxplot...")
    # Plot 2: All solutions (including failures)
    sns.boxplot(data=df, x='method', y='time_ms', ax=ax2)
    ax2.set_title('Execution Time - All Solutions')
    ax2.set_xlabel('IK Method')
    ax2.set_ylabel('Execution Time (ms)')
    ax2.grid(True, alpha=0.3)
    
    print("Adding statistics to second plot...")
    # Add statistics for all solutions
    for i, method in enumerate(df['method'].unique()):
        method_data = df[df['method'] == method]
        n_total = len(method_data)
        n_success = len(method_data[method_data['success'] == True])
        mean_time = method_data['time_ms'].mean()
        success_rate = n_success / n_total * 100
        ax2.text(i, ax2.get_ylim()[1] * 0.9, 
                f'n={n_total}\nsuccess={success_rate:.1f}%\nmean={mean_time:.1f}ms', 
                ha='center', va='top', 
                bbox=dict(boxstyle='round', facecolor='C1', alpha=0.3))
    
    print("Finalizing plot...")
    # Overall title
    fig.suptitle('Execution Time Comparison: Successful vs All Solutions', fontsize=14, fontweight='bold')
    
    # Adjust layout
    plt.tight_layout()
    
    print("Saving plots...")
    # Save plots
    plt.savefig('plots/two_execution_time_boxplots.png', dpi=150, bbox_inches='tight')
    plt.savefig('plots/two_execution_time_boxplots.pdf', bbox_inches='tight')
    
    print("\nSaved plots:")
    print("  - plots/two_execution_time_boxplots.png")
    print("  - plots/two_execution_time_boxplots.pdf")
    
    plt.close()
    
    # Print detailed statistics
    print("\n=== DETAILED STATISTICS ===")
    
    print("\nSuccessful Solutions Only:")
    for method in df_success['method'].unique():
        method_data = df_success[df_success['method'] == method]['time_ms']
        print(f"  {method}:")
        print(f"    Count: {len(method_data)}")
        print(f"    Mean: {method_data.mean():.1f}ms")
        print(f"    Median: {method_data.median():.1f}ms")
        print(f"    Range: {method_data.min():.1f} - {method_data.max():.1f}ms")
    
    print("\nAll Solutions (including failures):")
    for method in df['method'].unique():
        method_data = df[df['method'] == method]
        time_data = method_data['time_ms']
        success_count = method_data['success'].sum()
        total_count = len(method_data)
        
        print(f"  {method}:")
        print(f"    Total attempts: {total_count}")
        print(f"    Successful: {success_count} ({success_count/total_count*100:.1f}%)")
        print(f"    Mean time (all): {time_data.mean():.1f}ms")
        print(f"    Median time (all): {time_data.median():.1f}ms")
        print(f"    Range: {time_data.min():.1f} - {time_data.max():.1f}ms")

if __name__ == "__main__":
    print("🤖 CREATING TWO EXECUTION TIME BOXPLOTS")
    print("="*50)
    create_two_boxplots()
    print("="*50)
