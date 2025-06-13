#!/usr/bin/env python3
"""
Individual Parameter Configuration Analysis - Generate one PDF per parameter config
"""

import pandas as pd
import matplotlib.pyplot as plt
import seaborn as sns
from matplotlib.backends.backend_pdf import PdfPages
import numpy as np
import argparse
from pathlib import Path
import sys

class IndividualConfigAnalyzer:
    def __init__(self, data_file: str, output_dir: str):
        self.data_file = Path(data_file)
        self.output_dir = Path(output_dir)
        self.output_dir.mkdir(exist_ok=True)
        
        # Create subdirectories for organization
        self.stomp_dir = self.output_dir / "stomp_configs"
        self.hauser_dir = self.output_dir / "hauser_configs"
        self.stomp_dir.mkdir(exist_ok=True)
        self.hauser_dir.mkdir(exist_ok=True)
        
        # Algorithm family mapping
        self.algorithm_names = {
            0: 'STOMP',
            1: 'Hauser RRT',
            2: 'Hauser RRT*', 
            3: 'Hauser Informed RRT*',
            4: 'Hauser Bi-RRT'
        }
        
        # Load and preprocess data
        self.data = self.load_data()
        
        # Define metrics to analyze
        self.metrics = {
            'planning_time_ms': {'title': 'Planning Time', 'ylabel': 'Time (ms)', 'unit': 'ms'},
            'execution_time_ms': {'title': 'Execution Time', 'ylabel': 'Time (ms)', 'unit': 'ms'},
            'iterations_used': {'title': 'Iterations Used', 'ylabel': 'Iterations', 'unit': 'iterations'},
            'trajectory_length': {'title': 'Trajectory Length', 'ylabel': 'Length', 'unit': 'length units'},
            'path_quality': {'title': 'Path Quality', 'ylabel': 'Quality Score', 'unit': 'score'},
            'joint_smoothness': {'title': 'Joint Smoothness', 'ylabel': 'Smoothness Score', 'unit': 'score'}
        }
        
        print(f"📊 Loaded {len(self.data)} evaluation results")
        print(f"📋 Algorithms: {list(self.data['algorithm_name'].unique())}")
        
        # Get all unique configurations
        self.all_configs = self.get_all_configurations()
        print(f"🔧 Found {len(self.all_configs)} unique parameter configurations")
        
        # Print each configuration
        for config in self.all_configs:
            print(f"  - {config['config_id']}: {config['sample_count']} samples")
    
    def load_data(self) -> pd.DataFrame:
        """Load CSV data and add algorithm names."""
        try:
            df = pd.read_csv(self.data_file)
            df['algorithm_name'] = df['algorithm'].map(self.algorithm_names)
            
            # Add algorithm family grouping
            df['algorithm_family'] = df['algorithm_name'].apply(
                lambda x: 'STOMP' if x == 'STOMP' else 'Hauser'
            )
            
            # Create unique identifier for each config
            df['config_id'] = df['algorithm_name'] + ' - Config ' + df['config'].astype(str)
            
            return df
            
        except Exception as e:
            print(f"❌ Error loading data: {e}")
            sys.exit(1)
    
    def get_all_configurations(self):
        """Get all unique algorithm + configuration combinations."""
        configs = []
        
        # Group by algorithm and config
        for (algorithm, config), group in self.data.groupby(['algorithm_name', 'config']):
            if pd.notna(algorithm) and len(group) > 0:
                configs.append({
                    'algorithm': algorithm,
                    'config': config,
                    'config_id': f"{algorithm} - Config {config}",
                    'family': 'STOMP' if algorithm == 'STOMP' else 'Hauser',
                    'sample_count': len(group)
                })
        
        return configs
    
    def create_config_pdf(self, config_info: dict):
        """Create a comprehensive PDF for one specific parameter configuration."""
        
        algorithm = config_info['algorithm']
        config = config_info['config']
        config_id = config_info['config_id']
        family = config_info['family']
        
        print(f"📈 Generating analysis for {config_id}...")
        
        # Determine output directory and filename
        output_dir = self.stomp_dir if family == 'STOMP' else self.hauser_dir
        safe_filename = config_id.replace(' ', '_').replace('*', 'star').lower()
        filename = output_dir / f"{safe_filename}_analysis.pdf"
        
        # Filter data for this specific configuration
        config_data = self.data[
            (self.data['algorithm_name'] == algorithm) & 
            (self.data['config'] == config)
        ].copy()
        
        if len(config_data) == 0:
            print(f"⚠️ No data for {config_id}")
            return
        
        with PdfPages(filename) as pdf:
            # Page 1: Overview with all metrics
            fig = plt.figure(figsize=(16, 12))
            fig.suptitle(f'{config_id} - Complete Performance Analysis', fontsize=16, fontweight='bold')
            
            # Create a 3x2 grid for all 6 metrics
            metrics_list = list(self.metrics.keys())
            for i, (metric, info) in enumerate(self.metrics.items()):
                ax = plt.subplot(3, 2, i+1)
                
                # Filter data for this metric
                metric_data = config_data.dropna(subset=[metric])
                if metric in ['execution_time_ms', 'trajectory_length', 'path_quality', 'joint_smoothness']:
                    metric_data = metric_data[metric_data['success'] == 1]
                if metric in ['trajectory_length', 'path_quality', 'joint_smoothness']:
                    metric_data = metric_data[metric_data[metric] > 0]
                
                if len(metric_data) > 0:
                    # Create boxplot
                    values = metric_data[metric].values
                    color = '#1f77b4' if family == 'STOMP' else '#ff7f0e'
                    
                    bp = ax.boxplot([values], patch_artist=True)
                    bp['boxes'][0].set_facecolor(color)
                    bp['boxes'][0].set_alpha(0.7)
                    
                    ax.set_title(f'{info["title"]}')
                    ax.set_ylabel(info['ylabel'])
                    ax.set_xticklabels([config_id])
                    ax.grid(True, alpha=0.3)
                    
                    # Add statistics text
                    mean_val = values.mean()
                    std_val = values.std()
                    median_val = np.median(values)
                    ax.text(0.02, 0.98, f'μ={mean_val:.2f}\nσ={std_val:.2f}\nMed={median_val:.2f}', 
                           transform=ax.transAxes, verticalalignment='top',
                           bbox=dict(boxstyle='round', facecolor='white', alpha=0.8))
                else:
                    ax.text(0.5, 0.5, 'No valid data', ha='center', va='center', transform=ax.transAxes)
                    ax.set_title(f'{info["title"]} - No Data')
            
            plt.tight_layout()
            pdf.savefig(fig, bbox_inches='tight')
            plt.close()
            
            # Page 2: Detailed analysis for each metric
            for metric, info in self.metrics.items():
                fig, ((ax1, ax2), (ax3, ax4)) = plt.subplots(2, 2, figsize=(16, 12))
                fig.suptitle(f'{config_id} - {info["title"]} Detailed Analysis', fontsize=16, fontweight='bold')
                
                # Filter data for this metric
                metric_data = config_data.dropna(subset=[metric])
                if metric in ['execution_time_ms', 'trajectory_length', 'path_quality', 'joint_smoothness']:
                    metric_data = metric_data[metric_data['success'] == 1]
                if metric in ['trajectory_length', 'path_quality', 'joint_smoothness']:
                    metric_data = metric_data[metric_data[metric] > 0]
                
                if len(metric_data) == 0:
                    # No data page
                    ax1.text(0.5, 0.5, f'No valid data for {info["title"]}', 
                            ha='center', va='center', transform=ax1.transAxes, fontsize=14)
                    ax2.axis('off')
                    ax3.axis('off') 
                    ax4.axis('off')
                else:
                    values = metric_data[metric].values
                    color = '#1f77b4' if family == 'STOMP' else '#ff7f0e'
                    
                    # 1. Distribution histogram
                    ax1.hist(values, bins=min(15, len(values)//2 + 1), 
                            color=color, alpha=0.7, edgecolor='black')
                    ax1.set_title(f'{info["title"]} Distribution')
                    ax1.set_xlabel(info['ylabel'])
                    ax1.set_ylabel('Frequency')
                    ax1.grid(True, alpha=0.3)
                    
                    # Add statistics lines
                    mean_val = values.mean()
                    median_val = np.median(values)
                    ax1.axvline(mean_val, color='red', linestyle='--', label=f'Mean: {mean_val:.2f}')
                    ax1.axvline(median_val, color='green', linestyle='--', label=f'Median: {median_val:.2f}')
                    ax1.legend()
                    
                    # 2. Values by trial/run
                    trial_data = metric_data.groupby('trial_id')[metric].mean().reset_index()
                    ax2.plot(trial_data['trial_id'], trial_data[metric], 'o-', color=color, alpha=0.7)
                    ax2.set_title(f'{info["title"]} by Trial')
                    ax2.set_xlabel('Trial ID')
                    ax2.set_ylabel(info['ylabel'])
                    ax2.grid(True, alpha=0.3)
                    
                    # 3. Values by pose
                    pose_data = metric_data.groupby('pose_id')[metric].mean().reset_index()
                    if len(pose_data) > 1:
                        ax3.scatter(pose_data['pose_id'], pose_data[metric], 
                                  color=color, alpha=0.7, s=50)
                        ax3.set_title(f'{info["title"]} by Pose')
                        ax3.set_xlabel('Pose ID')
                        ax3.set_ylabel(info['ylabel'])
                        ax3.grid(True, alpha=0.3)
                    else:
                        ax3.text(0.5, 0.5, 'Single pose data', ha='center', va='center', transform=ax3.transAxes)
                    
                    # 4. Statistics table
                    ax4.axis('off')
                    stats = {
                        'Count': len(values),
                        'Mean': f"{values.mean():.3f}",
                        'Std Dev': f"{values.std():.3f}",
                        'Min': f"{values.min():.3f}",
                        'Max': f"{values.max():.3f}",
                        'Median': f"{np.median(values):.3f}",
                        'Q1': f"{np.percentile(values, 25):.3f}",
                        'Q3': f"{np.percentile(values, 75):.3f}"
                    }
                    
                    # Create table
                    table_data = [[k, v] for k, v in stats.items()]
                    table = ax4.table(cellText=table_data,
                                    colLabels=['Statistic', 'Value'],
                                    cellLoc='center',
                                    loc='center')
                    table.auto_set_font_size(False)
                    table.set_fontsize(12)
                    table.scale(1.2, 2)
                    ax4.set_title(f'{info["title"]} Statistics', pad=20)
                
                plt.tight_layout()
                pdf.savefig(fig, bbox_inches='tight')
                plt.close()
        
        print(f"✅ Saved {config_id} analysis to: {filename}")
    
    def generate_all_config_analyses(self):
        """Generate a PDF for each parameter configuration."""
        
        print(f"\n🚀 Generating individual parameter configuration analyses...")
        print("=" * 70)
        
        print(f"\n📁 STOMP configurations will be saved to: {self.stomp_dir}")
        print(f"📁 Hauser configurations will be saved to: {self.hauser_dir}")
        
        # Generate analysis for each configuration
        for config_info in self.all_configs:
            self.create_config_pdf(config_info)
        
        print("\n" + "=" * 70)
        print("✅ All parameter configuration analyses generated!")
        print(f"📁 STOMP configs: {self.stomp_dir}")
        print(f"📁 Hauser configs: {self.hauser_dir}")
        print(f"📊 Total configurations analyzed: {len(self.all_configs)}")

def main():
    parser = argparse.ArgumentParser(description='Individual Parameter Configuration Analyzer')
    parser.add_argument('data_file', help='CSV file with evaluation results')
    parser.add_argument('--output-dir', '-o', default='results/individual_configs', help='Output directory for PDFs')
    
    args = parser.parse_args()
    
    if not Path(args.data_file).exists():
        print(f"❌ Data file not found: {args.data_file}")
        sys.exit(1)
    
    analyzer = IndividualConfigAnalyzer(args.data_file, args.output_dir)
    analyzer.generate_all_config_analyses()

if __name__ == "__main__":
    main()
