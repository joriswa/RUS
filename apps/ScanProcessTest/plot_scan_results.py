#!/usr/bin/env python3
"""
Comprehensive plotting script for Scan Process Test results.
Generates publication-quality plots optimized for printing from CSV test data.

This script creates multiple visualization types:
- Performance analysis plots
- Noise impact analysis
- Statistical summaries
- Success rate analysis
- Trajectory analysis

Usage:
    python plot_scan_results.py [csv_file]
    
If no CSV file is specified, searches for the most recent scan_test_results_*.csv file.
"""

import pandas as pd
import numpy as np
import matplotlib
matplotlib.use('Agg')  # Use non-interactive backend for headless operation
import matplotlib.pyplot as plt
import seaborn as sns
from pathlib import Path
import argparse
import glob
import sys
from datetime import datetime
import warnings
warnings.filterwarnings('ignore')

# Configure matplotlib for publication-quality plots
plt.rcParams.update({
    'font.size': 10,
    'font.family': 'serif',
    'font.serif': ['Times New Roman', 'DejaVu Serif'],
    'axes.titlesize': 12,
    'axes.labelsize': 10,
    'xtick.labelsize': 9,
    'ytick.labelsize': 9,
    'legend.fontsize': 9,
    'figure.titlesize': 14,
    'figure.dpi': 300,
    'savefig.dpi': 300,
    'savefig.bbox': 'tight',
    'savefig.pad_inches': 0.1,
    'lines.linewidth': 1.5,
    'lines.markersize': 4,
    'axes.grid': True,
    'grid.alpha': 0.3,
    'axes.axisbelow': True
})

# Color palette for consistent visualization
COLORS = {
    'primary': '#2E86AB',
    'secondary': '#A23B72', 
    'accent': '#F18F01',
    'success': '#C73E1D',
    'neutral': '#6C757D',
    'light': '#E9ECEF'
}

class ScanResultsPlotter:
    def __init__(self, csv_file):
        """Initialize the plotter with CSV data."""
        self.csv_file = Path(csv_file)
        self.data = self.load_and_preprocess_data()
        self.output_dir = self.csv_file.parent / 'plots'
        self.output_dir.mkdir(exist_ok=True)
        
    def load_and_preprocess_data(self):
        """Load CSV data and add derived columns."""
        try:
            df = pd.read_csv(self.csv_file)
            
            # Convert noise_level to degrees for better readability
            df['noise_degrees'] = df['noise_level'] * 180 / np.pi
            
            # Parse trajectory information
            df['avg_trajectory_size'] = df['trajectory_sizes'].apply(self.parse_trajectory_sizes)
            df['num_contact_trajectories'] = df['trajectory_types'].apply(self.count_contact_trajectories)
            df['num_free_trajectories'] = df['num_trajectories'] - df['num_contact_trajectories']
            
            # Calculate efficiency metrics
            df['planning_efficiency'] = df['num_trajectories'] / df['trajectory_planning_time_ms'] * 1000  # trajectories per second
            df['total_efficiency'] = df['num_trajectories'] / df['total_time_ms'] * 1000
            
            print(f"Loaded {len(df)} test results from {self.csv_file}")
            print(f"Noise levels tested: {sorted(df['noise_level'].unique())}")
            print(f"Success rate: {df['success'].mean():.1%}")
            
            return df
            
        except Exception as e:
            print(f"Error loading CSV file: {e}")
            sys.exit(1)
    
    def parse_trajectory_sizes(self, sizes_str):
        """Parse semicolon-separated trajectory sizes and return average."""
        try:
            if pd.isna(sizes_str) or sizes_str == '""' or sizes_str == '':
                return 0
            # Remove quotes and split
            clean_str = str(sizes_str).strip('"')
            if not clean_str:
                return 0
            sizes = [int(x) for x in clean_str.split(';') if x.strip()]
            return np.mean(sizes) if sizes else 0
        except:
            return 0
    
    def count_contact_trajectories(self, types_str):
        """Count number of contact trajectories."""
        try:
            if pd.isna(types_str) or types_str == '""' or types_str == '':
                return 0
            # Remove quotes and split
            clean_str = str(types_str).strip('"')
            if not clean_str:
                return 0
            types = [x.strip() for x in clean_str.split(';') if x.strip()]
            return sum(1 for t in types if t == 'contact')
        except:
            return 0
    
    def create_performance_overview(self):
        """Create comprehensive performance overview plot."""
        fig, axes = plt.subplots(2, 3, figsize=(15, 10))
        fig.suptitle('Scan Process Test - Performance Overview', fontweight='bold')
        
        # 1. Planning time vs noise level
        ax = axes[0, 0]
        successful_data = self.data[self.data['success']]
        sns.boxplot(data=successful_data, x='noise_degrees', y='trajectory_planning_time_ms', 
                   color=COLORS['primary'], ax=ax)
        ax.set_title('Planning Time vs Noise Level')
        ax.set_xlabel('Noise Level (degrees)')
        ax.set_ylabel('Planning Time (ms)')
        ax.tick_params(axis='x', rotation=45)
        
        # 2. Success rate vs noise level
        ax = axes[0, 1]
        success_by_noise = self.data.groupby('noise_degrees')['success'].agg(['mean', 'count']).reset_index()
        bars = ax.bar(success_by_noise['noise_degrees'], success_by_noise['mean'], 
                     color=COLORS['success'], alpha=0.7)
        ax.set_title('Success Rate vs Noise Level')
        ax.set_xlabel('Noise Level (degrees)')
        ax.set_ylabel('Success Rate')
        ax.set_ylim(0, 1.1)
        # Add count labels on bars
        for i, (bar, count) in enumerate(zip(bars, success_by_noise['count'])):
            height = bar.get_height()
            ax.text(bar.get_x() + bar.get_width()/2., height + 0.02,
                   f'n={count}', ha='center', va='bottom', fontsize=8)
        
        # 3. Total time breakdown
        ax = axes[0, 2]
        time_columns = ['csv_load_time_ms', 'planner_init_time_ms', 'trajectory_planning_time_ms']
        time_means = successful_data[time_columns].mean()
        colors = [COLORS['neutral'], COLORS['accent'], COLORS['primary']]
        wedges, texts, autotexts = ax.pie(time_means.values, labels=['CSV Load', 'Init', 'Planning'], 
                                         colors=colors, autopct='%1.1f%%', startangle=90)
        ax.set_title('Average Time Breakdown')
        
        # 4. Trajectory count distribution
        ax = axes[1, 0]
        ax.hist(successful_data['num_trajectories'], bins=range(int(successful_data['num_trajectories'].min()), 
                int(successful_data['num_trajectories'].max())+2), 
                color=COLORS['secondary'], alpha=0.7, edgecolor='black')
        ax.set_title('Trajectory Count Distribution')
        ax.set_xlabel('Number of Trajectories')
        ax.set_ylabel('Frequency')
        
        # 5. Planning efficiency vs noise
        ax = axes[1, 1]
        sns.scatterplot(data=successful_data, x='noise_degrees', y='planning_efficiency', 
                       color=COLORS['primary'], s=60, alpha=0.7, ax=ax)
        # Add trend line
        z = np.polyfit(successful_data['noise_degrees'], successful_data['planning_efficiency'], 1)
        p = np.poly1d(z)
        ax.plot(successful_data['noise_degrees'].unique(), 
               p(successful_data['noise_degrees'].unique()), 
               color=COLORS['accent'], linestyle='--', alpha=0.8)
        ax.set_title('Planning Efficiency vs Noise')
        ax.set_xlabel('Noise Level (degrees)')
        ax.set_ylabel('Trajectories per Second')
        
        # 6. Trajectory size analysis
        ax = axes[1, 2]
        sns.boxplot(data=successful_data, x='noise_degrees', y='avg_trajectory_size', 
                   color=COLORS['accent'], ax=ax)
        ax.set_title('Average Trajectory Size vs Noise')
        ax.set_xlabel('Noise Level (degrees)')
        ax.set_ylabel('Average Trajectory Size')
        ax.tick_params(axis='x', rotation=45)
        
        plt.tight_layout()
        output_file = self.output_dir / 'performance_overview.png'
        plt.savefig(output_file, dpi=300, bbox_inches='tight')
        print(f"Saved: {output_file}")
        plt.close()
    
    def create_noise_impact_analysis(self):
        """Create detailed noise impact analysis."""
        fig, axes = plt.subplots(2, 2, figsize=(12, 10))
        fig.suptitle('Noise Impact Analysis on Planning Performance', fontweight='bold')
        
        successful_data = self.data[self.data['success']]
        
        # 1. Planning time distribution by noise level
        ax = axes[0, 0]
        noise_levels = sorted(successful_data['noise_degrees'].unique())
        planning_times_by_noise = [successful_data[successful_data['noise_degrees'] == nl]['trajectory_planning_time_ms'].values 
                                  for nl in noise_levels]
        
        box_plot = ax.boxplot(planning_times_by_noise, labels=[f'{nl:.3f}°' for nl in noise_levels], 
                             patch_artist=True)
        for patch, color in zip(box_plot['boxes'], plt.cm.viridis(np.linspace(0, 1, len(noise_levels)))):
            patch.set_facecolor(color)
            patch.set_alpha(0.7)
        
        ax.set_title('Planning Time Distribution by Noise Level')
        ax.set_xlabel('Noise Level')
        ax.set_ylabel('Planning Time (ms)')
        ax.tick_params(axis='x', rotation=45)
        
        # 2. Trajectory type distribution
        ax = axes[0, 1]
        contact_ratio = successful_data['num_contact_trajectories'] / successful_data['num_trajectories']
        free_ratio = successful_data['num_free_trajectories'] / successful_data['num_trajectories']
        
        width = 0.35
        noise_pos = np.arange(len(noise_levels))
        
        contact_means = [contact_ratio[successful_data['noise_degrees'] == nl].mean() for nl in noise_levels]
        free_means = [free_ratio[successful_data['noise_degrees'] == nl].mean() for nl in noise_levels]
        
        bars1 = ax.bar(noise_pos - width/2, contact_means, width, label='Contact', color=COLORS['primary'])
        bars2 = ax.bar(noise_pos + width/2, free_means, width, label='Free', color=COLORS['accent'])
        
        ax.set_title('Trajectory Type Distribution by Noise')
        ax.set_xlabel('Noise Level')
        ax.set_ylabel('Proportion of Trajectories')
        ax.set_xticks(noise_pos)
        ax.set_xticklabels([f'{nl:.3f}°' for nl in noise_levels])
        ax.legend()
        ax.tick_params(axis='x', rotation=45)
        
        # 3. Correlation matrix
        ax = axes[1, 0]
        corr_columns = ['noise_degrees', 'trajectory_planning_time_ms', 'num_trajectories', 
                       'avg_trajectory_size', 'planning_efficiency']
        corr_matrix = successful_data[corr_columns].corr()
        
        im = ax.imshow(corr_matrix, cmap='RdBu_r', aspect='auto', vmin=-1, vmax=1)
        ax.set_xticks(range(len(corr_columns)))
        ax.set_yticks(range(len(corr_columns)))
        ax.set_xticklabels(['Noise', 'Plan Time', 'Num Traj', 'Avg Size', 'Efficiency'], rotation=45)
        ax.set_yticklabels(['Noise', 'Plan Time', 'Num Traj', 'Avg Size', 'Efficiency'])
        
        # Add correlation values
        for i in range(len(corr_columns)):
            for j in range(len(corr_columns)):
                text = ax.text(j, i, f'{corr_matrix.iloc[i, j]:.2f}',
                             ha="center", va="center", color="black", fontweight='bold')
        
        ax.set_title('Performance Metrics Correlation')
        plt.colorbar(im, ax=ax, shrink=0.8)
        
        # 4. Statistical summary table
        ax = axes[1, 1]
        ax.axis('tight')
        ax.axis('off')
        
        # Create summary statistics
        summary_stats = []
        for nl in noise_levels:
            subset = successful_data[successful_data['noise_degrees'] == nl]
            stats = {
                'Noise (°)': f'{nl:.3f}',
                'Trials': len(subset),
                'Avg Time (ms)': f'{subset["trajectory_planning_time_ms"].mean():.1f}',
                'Std Time (ms)': f'{subset["trajectory_planning_time_ms"].std():.1f}',
                'Avg Trajectories': f'{subset["num_trajectories"].mean():.1f}',
                'Success Rate': f'{len(subset)/len(self.data[self.data["noise_degrees"]==nl]):.1%}'
            }
            summary_stats.append(stats)
        
        df_summary = pd.DataFrame(summary_stats)
        table = ax.table(cellText=df_summary.values, colLabels=df_summary.columns,
                        cellLoc='center', loc='center', bbox=[0, 0, 1, 1])
        table.auto_set_font_size(False)
        table.set_fontsize(8)
        table.scale(1, 1.5)
        ax.set_title('Statistical Summary by Noise Level', pad=20)
        
        plt.tight_layout()
        output_file = self.output_dir / 'noise_impact_analysis.png'
        plt.savefig(output_file, dpi=300, bbox_inches='tight')
        print(f"Saved: {output_file}")
        plt.close()
    
    def create_time_analysis_plots(self):
        """Create detailed timing analysis plots."""
        fig, axes = plt.subplots(2, 2, figsize=(12, 10))
        fig.suptitle('Detailed Timing Analysis', fontweight='bold')
        
        successful_data = self.data[self.data['success']]
        
        # 1. Time component breakdown by noise level
        ax = axes[0, 0]
        noise_levels = sorted(successful_data['noise_degrees'].unique())
        
        # Calculate means for each noise level
        csv_means = [successful_data[successful_data['noise_degrees'] == nl]['csv_load_time_ms'].mean() 
                    for nl in noise_levels]
        init_means = [successful_data[successful_data['noise_degrees'] == nl]['planner_init_time_ms'].mean() 
                     for nl in noise_levels]
        planning_means = [successful_data[successful_data['noise_degrees'] == nl]['trajectory_planning_time_ms'].mean() 
                         for nl in noise_levels]
        
        width = 0.6
        x_pos = np.arange(len(noise_levels))
        
        ax.bar(x_pos, csv_means, width, label='CSV Load', color=COLORS['neutral'])
        ax.bar(x_pos, init_means, width, bottom=csv_means, label='Planner Init', color=COLORS['accent'])
        ax.bar(x_pos, planning_means, width, 
               bottom=np.array(csv_means) + np.array(init_means), 
               label='Trajectory Planning', color=COLORS['primary'])
        
        ax.set_title('Time Component Breakdown by Noise Level')
        ax.set_xlabel('Noise Level (degrees)')
        ax.set_ylabel('Time (ms)')
        ax.set_xticks(x_pos)
        ax.set_xticklabels([f'{nl:.3f}' for nl in noise_levels])
        ax.legend()
        
        # 2. Planning time vs total time correlation
        ax = axes[0, 1]
        scatter = ax.scatter(successful_data['total_time_ms'], successful_data['trajectory_planning_time_ms'], 
                           c=successful_data['noise_degrees'], cmap='viridis', s=60, alpha=0.7)
        
        # Add diagonal reference line
        min_time = min(successful_data['total_time_ms'].min(), successful_data['trajectory_planning_time_ms'].min())
        max_time = max(successful_data['total_time_ms'].max(), successful_data['trajectory_planning_time_ms'].max())
        ax.plot([min_time, max_time], [min_time, max_time], 'r--', alpha=0.5, label='Perfect correlation')
        
        ax.set_title('Planning vs Total Time Correlation')
        ax.set_xlabel('Total Time (ms)')
        ax.set_ylabel('Planning Time (ms)')
        plt.colorbar(scatter, ax=ax, label='Noise Level (°)')
        ax.legend()
        
        # 3. Planning efficiency trends
        ax = axes[1, 0]
        for nl in noise_levels:
            subset = successful_data[successful_data['noise_degrees'] == nl]
            ax.plot(subset['trial_number'], subset['planning_efficiency'], 'o-', 
                   label=f'{nl:.3f}°', alpha=0.7, markersize=4)
        
        ax.set_title('Planning Efficiency Trends by Trial')
        ax.set_xlabel('Trial Number')
        ax.set_ylabel('Planning Efficiency (traj/sec)')
        ax.legend(title='Noise Level', bbox_to_anchor=(1.05, 1), loc='upper left')
        
        # 4. Time variability analysis
        ax = axes[1, 1]
        
        # Calculate coefficient of variation for each noise level
        cv_data = []
        for nl in noise_levels:
            subset = successful_data[successful_data['noise_degrees'] == nl]
            if len(subset) > 1:  # Need at least 2 samples for std
                cv = subset['trajectory_planning_time_ms'].std() / subset['trajectory_planning_time_ms'].mean()
                cv_data.append(cv)
            else:
                cv_data.append(0)
        
        bars = ax.bar(range(len(noise_levels)), cv_data, color=COLORS['secondary'], alpha=0.7)
        ax.set_title('Planning Time Variability (Coefficient of Variation)')
        ax.set_xlabel('Noise Level (degrees)')
        ax.set_ylabel('Coefficient of Variation')
        ax.set_xticks(range(len(noise_levels)))
        ax.set_xticklabels([f'{nl:.3f}' for nl in noise_levels])
        
        # Add value labels on bars
        for bar, cv in zip(bars, cv_data):
            height = bar.get_height()
            ax.text(bar.get_x() + bar.get_width()/2., height + 0.001,
                   f'{cv:.3f}', ha='center', va='bottom', fontsize=8)
        
        plt.tight_layout()
        output_file = self.output_dir / 'timing_analysis.png'
        plt.savefig(output_file, dpi=300, bbox_inches='tight')
        print(f"Saved: {output_file}")
        plt.close()
    
    def create_trajectory_analysis_plots(self):
        """Create detailed trajectory analysis plots."""
        fig, axes = plt.subplots(2, 2, figsize=(12, 10))
        fig.suptitle('Trajectory Analysis', fontweight='bold')
        
        successful_data = self.data[self.data['success']]
        
        # 1. Trajectory count vs planning time
        ax = axes[0, 0]
        scatter = ax.scatter(successful_data['num_trajectories'], successful_data['trajectory_planning_time_ms'],
                           c=successful_data['noise_degrees'], cmap='viridis', s=60, alpha=0.7)
        
        # Add trend line
        z = np.polyfit(successful_data['num_trajectories'], successful_data['trajectory_planning_time_ms'], 1)
        p = np.poly1d(z)
        ax.plot(successful_data['num_trajectories'], p(successful_data['num_trajectories']), 
               'r--', alpha=0.8, label=f'Trend: y = {z[0]:.1f}x + {z[1]:.1f}')
        
        ax.set_title('Trajectory Count vs Planning Time')
        ax.set_xlabel('Number of Trajectories')
        ax.set_ylabel('Planning Time (ms)')
        plt.colorbar(scatter, ax=ax, label='Noise Level (°)')
        ax.legend()
        
        # 2. Average trajectory size distribution
        ax = axes[0, 1]
        noise_levels = sorted(successful_data['noise_degrees'].unique())
        colors = plt.cm.viridis(np.linspace(0, 1, len(noise_levels)))
        
        for i, nl in enumerate(noise_levels):
            subset = successful_data[successful_data['noise_degrees'] == nl]
            ax.hist(subset['avg_trajectory_size'], bins=15, alpha=0.6, 
                   label=f'{nl:.3f}°', color=colors[i], density=True)
        
        ax.set_title('Average Trajectory Size Distribution')
        ax.set_xlabel('Average Trajectory Size')
        ax.set_ylabel('Density')
        ax.legend(title='Noise Level')
        
        # 3. Contact vs Free trajectory analysis
        ax = axes[1, 0]
        contact_data = []
        free_data = []
        labels = []
        
        for nl in noise_levels:
            subset = successful_data[successful_data['noise_degrees'] == nl]
            contact_data.append(subset['num_contact_trajectories'].mean())
            free_data.append(subset['num_free_trajectories'].mean())
            labels.append(f'{nl:.3f}°')
        
        x_pos = np.arange(len(labels))
        width = 0.35
        
        bars1 = ax.bar(x_pos - width/2, contact_data, width, label='Contact', color=COLORS['primary'])
        bars2 = ax.bar(x_pos + width/2, free_data, width, label='Free', color=COLORS['accent'])
        
        ax.set_title('Average Trajectory Counts by Type')
        ax.set_xlabel('Noise Level')
        ax.set_ylabel('Average Number of Trajectories')
        ax.set_xticks(x_pos)
        ax.set_xticklabels(labels)
        ax.legend()
        
        # Add value labels on bars
        for bars in [bars1, bars2]:
            for bar in bars:
                height = bar.get_height()
                ax.text(bar.get_x() + bar.get_width()/2., height + 0.02,
                       f'{height:.1f}', ha='center', va='bottom', fontsize=8)
        
        # 4. Trajectory efficiency (size per planning time)
        ax = axes[1, 1]
        successful_data['trajectory_size_efficiency'] = (
            successful_data['avg_trajectory_size'] * successful_data['num_trajectories'] / 
            successful_data['trajectory_planning_time_ms'] * 1000
        )
        
        sns.boxplot(data=successful_data, x='noise_degrees', y='trajectory_size_efficiency', 
                   color=COLORS['secondary'], ax=ax)
        ax.set_title('Trajectory Size Efficiency vs Noise')
        ax.set_xlabel('Noise Level (degrees)')
        ax.set_ylabel('Total Trajectory Points per Second')
        ax.tick_params(axis='x', rotation=45)
        
        plt.tight_layout()
        output_file = self.output_dir / 'trajectory_analysis.png'
        plt.savefig(output_file, dpi=300, bbox_inches='tight')
        print(f"Saved: {output_file}")
        plt.close()
    
    def create_comprehensive_summary(self):
        """Create a comprehensive summary report."""
        fig = plt.figure(figsize=(16, 12))
        gs = fig.add_gridspec(4, 4, hspace=0.3, wspace=0.3)
        
        successful_data = self.data[self.data['success']]
        noise_levels = sorted(successful_data['noise_degrees'].unique())
        
        # Title
        fig.suptitle('Scan Process Test - Comprehensive Performance Report', fontsize=16, fontweight='bold')
        
        # 1. Key Performance Indicators (top row)
        ax1 = fig.add_subplot(gs[0, :2])
        ax1.axis('off')
        
        # Calculate key metrics
        total_trials = len(self.data)
        successful_trials = len(successful_data)
        success_rate = successful_trials / total_trials
        avg_planning_time = successful_data['trajectory_planning_time_ms'].mean()
        std_planning_time = successful_data['trajectory_planning_time_ms'].std()
        avg_trajectories = successful_data['num_trajectories'].mean()
        
        kpi_text = f"""
        KEY PERFORMANCE INDICATORS
        ═══════════════════════════════════
        Total Test Trials: {total_trials}
        Successful Trials: {successful_trials} ({success_rate:.1%})
        
        Average Planning Time: {avg_planning_time:.1f} ± {std_planning_time:.1f} ms
        Average Trajectories: {avg_trajectories:.1f}
        
        Noise Levels Tested: {len(noise_levels)} ({min(noise_levels):.3f}° to {max(noise_levels):.3f}°)
        Test Duration Range: {successful_data['total_time_ms'].min():.0f} - {successful_data['total_time_ms'].max():.0f} ms
        """
        
        ax1.text(0, 0.5, kpi_text, fontsize=10, fontfamily='monospace', 
                verticalalignment='center', bbox=dict(boxstyle="round,pad=0.5", facecolor=COLORS['light']))
        
        # 2. Performance trend summary (top right)
        ax2 = fig.add_subplot(gs[0, 2:])
        
        # Calculate performance trends
        trend_data = []
        for nl in noise_levels:
            subset = successful_data[successful_data['noise_degrees'] == nl]
            trend_data.append({
                'noise': nl,
                'avg_time': subset['trajectory_planning_time_ms'].mean(),
                'success_rate': len(subset) / len(self.data[self.data['noise_degrees'] == nl])
            })
        
        trend_df = pd.DataFrame(trend_data)
        
        ax2_twin = ax2.twinx()
        
        line1 = ax2.plot(trend_df['noise'], trend_df['avg_time'], 'o-', color=COLORS['primary'], 
                        linewidth=2, markersize=6, label='Planning Time')
        line2 = ax2_twin.plot(trend_df['noise'], trend_df['success_rate'], 's-', color=COLORS['success'], 
                             linewidth=2, markersize=6, label='Success Rate')
        
        ax2.set_xlabel('Noise Level (degrees)')
        ax2.set_ylabel('Average Planning Time (ms)', color=COLORS['primary'])
        ax2_twin.set_ylabel('Success Rate', color=COLORS['success'])
        ax2.set_title('Performance Trends vs Noise Level')
        
        # Combine legends
        lines = line1 + line2
        labels = [l.get_label() for l in lines]
        ax2.legend(lines, labels, loc='upper right')
        
        # 3. Detailed statistics table (middle left)
        ax3 = fig.add_subplot(gs[1:3, :2])
        ax3.axis('off')
        
        # Create detailed statistics table
        stats_data = []
        for nl in noise_levels:
            subset = successful_data[successful_data['noise_degrees'] == nl]
            all_subset = self.data[self.data['noise_degrees'] == nl]
            
            stats = {
                'Noise (°)': f'{nl:.3f}',
                'Trials': f'{len(subset)}/{len(all_subset)}',
                'Success': f'{len(subset)/len(all_subset):.1%}',
                'Avg Time': f'{subset["trajectory_planning_time_ms"].mean():.0f}',
                'Std Time': f'{subset["trajectory_planning_time_ms"].std():.0f}',
                'Min Time': f'{subset["trajectory_planning_time_ms"].min():.0f}',
                'Max Time': f'{subset["trajectory_planning_time_ms"].max():.0f}',
                'Avg Traj': f'{subset["num_trajectories"].mean():.1f}',
                'Efficiency': f'{subset["planning_efficiency"].mean():.2f}'
            }
            stats_data.append(stats)
        
        stats_df = pd.DataFrame(stats_data)
        
        # Create table
        table = ax3.table(cellText=stats_df.values, colLabels=stats_df.columns,
                         cellLoc='center', loc='center', bbox=[0, 0, 1, 1])
        table.auto_set_font_size(False)
        table.set_fontsize(8)
        table.scale(1, 2)
        
        # Style the table
        for i in range(len(stats_df.columns)):
            table[(0, i)].set_facecolor(COLORS['neutral'])
            table[(0, i)].set_text_props(weight='bold', color='white')
        
        ax3.set_title('Detailed Performance Statistics by Noise Level', pad=20, fontweight='bold')
        
        # 4. Distribution plots (middle right and bottom)
        ax4 = fig.add_subplot(gs[1, 2:])
        
        # Planning time distribution
        planning_times_all = [successful_data[successful_data['noise_degrees'] == nl]['trajectory_planning_time_ms'].values 
                             for nl in noise_levels]
        colors = plt.cm.viridis(np.linspace(0, 1, len(noise_levels)))
        
        violin_parts = ax4.violinplot(planning_times_all, positions=range(len(noise_levels)), 
                                     showmeans=True, showmedians=True)
        
        for pc, color in zip(violin_parts['bodies'], colors):
            pc.set_facecolor(color)
            pc.set_alpha(0.7)
        
        ax4.set_title('Planning Time Distribution by Noise Level')
        ax4.set_xlabel('Noise Level Index')
        ax4.set_ylabel('Planning Time (ms)')
        ax4.set_xticks(range(len(noise_levels)))
        ax4.set_xticklabels([f'{nl:.3f}°' for nl in noise_levels])
        
        # 5. Correlation heatmap (bottom left)
        ax5 = fig.add_subplot(gs[2, 2:])
        
        corr_columns = ['noise_degrees', 'trajectory_planning_time_ms', 'num_trajectories', 
                       'avg_trajectory_size', 'planning_efficiency']
        corr_matrix = successful_data[corr_columns].corr()
        
        im = ax5.imshow(corr_matrix, cmap='RdBu_r', aspect='auto', vmin=-1, vmax=1)
        ax5.set_xticks(range(len(corr_columns)))
        ax5.set_yticks(range(len(corr_columns)))
        ax5.set_xticklabels(['Noise', 'Plan Time', 'Trajectories', 'Avg Size', 'Efficiency'], rotation=45)
        ax5.set_yticklabels(['Noise', 'Plan Time', 'Trajectories', 'Avg Size', 'Efficiency'])
        
        # Add correlation values
        for i in range(len(corr_columns)):
            for j in range(len(corr_columns)):
                text = ax5.text(j, i, f'{corr_matrix.iloc[i, j]:.2f}',
                               ha="center", va="center", color="black", fontweight='bold', fontsize=8)
        
        ax5.set_title('Performance Metrics Correlation Matrix')
        
        # 6. Test metadata (bottom)
        ax6 = fig.add_subplot(gs[3, :])
        ax6.axis('off')
        
        # Generate metadata text
        metadata_text = f"""
        TEST METADATA AND CONFIGURATION
        ═══════════════════════════════════════════════════════════════════════════════════════════════════════════════════════
        
        Data File: {self.csv_file.name}                    Generated: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}
        Total Test Duration: {(successful_data['total_time_ms'].sum()/1000/60):.1f} minutes     Plots Output: {self.output_dir.name}/
        
        Analysis Summary:
        • Performance decreases moderately with increased noise (correlation: {corr_matrix.loc['noise_degrees', 'trajectory_planning_time_ms']:.3f})
        • Success rate remains high across all noise levels: {success_rate:.1%} overall
        • Planning efficiency shows {'positive' if corr_matrix.loc['noise_degrees', 'planning_efficiency'] > 0 else 'negative'} correlation with noise
        • Most stable performance at {noise_levels[np.argmin([successful_data[successful_data['noise_degrees']==nl]['trajectory_planning_time_ms'].std() for nl in noise_levels])]:.3f}° noise level
        """
        
        ax6.text(0, 0.5, metadata_text, fontsize=9, fontfamily='monospace', 
                verticalalignment='center', bbox=dict(boxstyle="round,pad=0.5", facecolor=COLORS['light'], alpha=0.8))
        
        output_file = self.output_dir / 'comprehensive_summary.png'
        plt.savefig(output_file, dpi=300, bbox_inches='tight')
        print(f"Saved: {output_file}")
        plt.close()
    
    def generate_all_plots(self):
        """Generate all analysis plots."""
        print(f"\nGenerating comprehensive plots for {len(self.data)} test results...")
        print(f"Output directory: {self.output_dir}")
        
        self.create_performance_overview()
        self.create_noise_impact_analysis()
        self.create_time_analysis_plots()
        self.create_trajectory_analysis_plots()
        self.create_comprehensive_summary()
        
        print(f"\n✓ All plots generated successfully!")
        print(f"  Output directory: {self.output_dir}")
        print(f"  Generated files:")
        for plot_file in sorted(self.output_dir.glob('*.png')):
            print(f"    - {plot_file.name}")

def find_latest_csv():
    """Find the most recent scan_test_results CSV file."""
    csv_files = glob.glob('**/scan_test_results_*.csv', recursive=True)
    if not csv_files:
        return None
    
    # Sort by modification time and return the most recent
    csv_files.sort(key=lambda x: Path(x).stat().st_mtime, reverse=True)
    return csv_files[0]

def main():
    parser = argparse.ArgumentParser(description='Generate comprehensive plots from scan process test results')
    parser.add_argument('csv_file', nargs='?', help='Path to CSV results file (optional)')
    parser.add_argument('--output-dir', help='Output directory for plots (default: plots/ next to CSV)')
    
    args = parser.parse_args()
    
    # Find CSV file
    csv_file = args.csv_file
    if not csv_file:
        csv_file = find_latest_csv()
        if not csv_file:
            print("No CSV file specified and no scan_test_results_*.csv files found.")
            print("Please specify a CSV file or run the scan process test first.")
            sys.exit(1)
        print(f"Using most recent CSV file: {csv_file}")
    
    if not Path(csv_file).exists():
        print(f"Error: CSV file not found: {csv_file}")
        sys.exit(1)
    
    # Generate plots
    plotter = ScanResultsPlotter(csv_file)
    
    if args.output_dir:
        plotter.output_dir = Path(args.output_dir)
        plotter.output_dir.mkdir(parents=True, exist_ok=True)
    
    plotter.generate_all_plots()

if __name__ == '__main__':
    main()
