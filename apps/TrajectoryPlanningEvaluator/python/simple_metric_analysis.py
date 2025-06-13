#!/usr/bin/env python3
"""
Simple Metric Analysis - Generate one PDF per metric showing scores grouped by algorithm family
"""

import pandas as pd
import matplotlib.pyplot as plt
import seaborn as sns
from matplotlib.backends.backend_pdf import PdfPages
import numpy as np
import argparse
from pathlib import Path
import sys

class SimpleMetricAnalyzer:
    def __init__(self, data_file: str, output_dir: str):
        self.data_file = Path(data_file)
        self.output_dir = Path(output_dir)
        self.output_dir.mkdir(exist_ok=True)
        
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
            'planning_time_ms': {'title': 'Planning Time', 'ylabel': 'Time (ms)'},
            'execution_time_ms': {'title': 'Execution Time', 'ylabel': 'Time (ms)'},
            'iterations_used': {'title': 'Iterations Used', 'ylabel': 'Iterations'},
            'trajectory_length': {'title': 'Trajectory Length', 'ylabel': 'Length'},
            'path_quality': {'title': 'Path Quality', 'ylabel': 'Quality Score'},
            'joint_smoothness': {'title': 'Joint Smoothness', 'ylabel': 'Smoothness Score'}
        }
        
        print(f"📊 Loaded {len(self.data)} evaluation results")
        print(f"📋 Algorithms: {list(self.data['algorithm_name'].unique())}")
    
    def load_data(self) -> pd.DataFrame:
        """Load CSV data and add algorithm names."""
        try:
            df = pd.read_csv(self.data_file)
            df['algorithm_name'] = df['algorithm'].map(self.algorithm_names)
            
            # Add algorithm family grouping
            df['algorithm_family'] = df['algorithm_name'].apply(
                lambda x: 'STOMP' if x == 'STOMP' else 'Hauser'
            )
            
            # Create combined identifier for grouping
            df['algorithm_config'] = df['algorithm_name'] + ' (Config ' + df['config'].astype(str) + ')'
            
            return df
            
        except Exception as e:
            print(f"❌ Error loading data: {e}")
            sys.exit(1)
    
    def create_metric_pdf(self, metric: str, info: dict):
        """Create a single PDF for one metric showing scores grouped by algorithm family."""
        
        print(f"📈 Generating {info['title']} analysis...")
        
        filename = self.output_dir / f"{metric}_algorithm_comparison.pdf"
        
        with PdfPages(filename) as pdf:
            # Filter data - only successful runs for some metrics
            plot_data = self.data.copy()
            if metric in ['execution_time_ms', 'trajectory_length', 'path_quality', 'joint_smoothness']:
                plot_data = plot_data[plot_data['success'] == 1]
            
            plot_data = plot_data.dropna(subset=[metric])
            if metric in ['trajectory_length', 'path_quality', 'joint_smoothness']:
                plot_data = plot_data[plot_data[metric] > 0]
            
            if len(plot_data) == 0:
                print(f"⚠️ No data available for {metric}")
                return
            
            # Create figure with 2x2 layout
            fig, ((ax1, ax2), (ax3, ax4)) = plt.subplots(2, 2, figsize=(16, 12))
            fig.suptitle(f'{info["title"]} - Algorithm Family Comparison', fontsize=16, fontweight='bold')
            
            # 1. Main boxplot - STOMP vs Hauser families
            stomp_data = plot_data[plot_data['algorithm_family'] == 'STOMP'][metric]
            hauser_data = plot_data[plot_data['algorithm_family'] == 'Hauser'][metric]
            
            box_data = [stomp_data, hauser_data]
            box_labels = ['STOMP Family', 'Hauser Family']
            
            bp1 = ax1.boxplot(box_data, patch_artist=True, tick_labels=box_labels)
            bp1['boxes'][0].set_facecolor('#1f77b4')  # Blue for STOMP
            bp1['boxes'][1].set_facecolor('#ff7f0e')  # Orange for Hauser
            
            ax1.set_title(f'{info["title"]} by Algorithm Family')
            ax1.set_ylabel(info['ylabel'])
            ax1.grid(True, alpha=0.3)
            
            # 2. Individual algorithm boxplots
            algorithms = sorted(plot_data['algorithm_name'].unique())
            alg_data = []
            alg_labels = []
            alg_colors = []
            
            for alg in algorithms:
                alg_subset = plot_data[plot_data['algorithm_name'] == alg][metric]
                if len(alg_subset) > 0:
                    alg_data.append(alg_subset)
                    alg_labels.append(alg)
                    alg_colors.append('#1f77b4' if alg == 'STOMP' else '#ff7f0e')
            
            if alg_data:
                bp2 = ax2.boxplot(alg_data, patch_artist=True)
                for i, patch in enumerate(bp2['boxes']):
                    patch.set_facecolor(alg_colors[i])
                    patch.set_alpha(0.7)
                
                ax2.set_title(f'{info["title"]} by Individual Algorithm')
                ax2.set_ylabel(info['ylabel'])
                ax2.set_xticklabels(alg_labels, rotation=45)
                ax2.grid(True, alpha=0.3)
            
            # 3. Configuration-level detail for STOMP
            stomp_configs = plot_data[plot_data['algorithm_family'] == 'STOMP']
            if len(stomp_configs) > 0:
                stomp_config_data = []
                stomp_config_labels = []
                
                for config in sorted(stomp_configs['config'].unique()):
                    config_subset = stomp_configs[stomp_configs['config'] == config][metric]
                    if len(config_subset) > 0:
                        stomp_config_data.append(config_subset)
                        stomp_config_labels.append(f'Config {config}')
                
                if stomp_config_data:
                    bp3 = ax3.boxplot(stomp_config_data, patch_artist=True)
                    for patch in bp3['boxes']:
                        patch.set_facecolor('#1f77b4')
                        patch.set_alpha(0.7)
                    
                    ax3.set_title(f'STOMP {info["title"]} by Configuration')
                    ax3.set_ylabel(info['ylabel'])
                    ax3.set_xticklabels(stomp_config_labels, rotation=45)
                    ax3.grid(True, alpha=0.3)
            
            # 4. Configuration-level detail for Hauser family
            hauser_configs = plot_data[plot_data['algorithm_family'] == 'Hauser']
            if len(hauser_configs) > 0:
                # Group by algorithm and config
                hauser_groups = hauser_configs.groupby(['algorithm_name', 'config'])
                hauser_config_data = []
                hauser_config_labels = []
                
                for (alg, config), group in hauser_groups:
                    if len(group) > 0:
                        hauser_config_data.append(group[metric])
                        hauser_config_labels.append(f'{alg}\nConfig {config}')
                
                if hauser_config_data:
                    bp4 = ax4.boxplot(hauser_config_data, patch_artist=True)
                    for patch in bp4['boxes']:
                        patch.set_facecolor('#ff7f0e')
                        patch.set_alpha(0.7)
                    
                    ax4.set_title(f'Hauser Family {info["title"]} by Algorithm & Configuration')
                    ax4.set_ylabel(info['ylabel'])
                    ax4.set_xticklabels(hauser_config_labels, rotation=45, fontsize=8)
                    ax4.grid(True, alpha=0.3)
            
            plt.tight_layout()
            pdf.savefig(fig, bbox_inches='tight')
            plt.close()
        
        print(f"✅ Saved {info['title']} analysis to: {filename}")
    
    def generate_all_metric_pdfs(self):
        """Generate a PDF for each metric."""
        
        print("\n🚀 Generating simple metric analyses...")
        print("=" * 50)
        
        for metric, info in self.metrics.items():
            self.create_metric_pdf(metric, info)
        
        print("\n" + "=" * 50)
        print("✅ All metric analyses generated!")
        print(f"📁 PDFs saved to: {self.output_dir}")

def main():
    parser = argparse.ArgumentParser(description='Simple Trajectory Planning Metric Visualizer')
    parser.add_argument('data_file', help='CSV file with evaluation results')
    parser.add_argument('--output-dir', '-o', default='results', help='Output directory for PDFs')
    
    args = parser.parse_args()
    
    if not Path(args.data_file).exists():
        print(f"❌ Data file not found: {args.data_file}")
        sys.exit(1)
    
    analyzer = SimpleMetricAnalyzer(args.data_file, args.output_dir)
    analyzer.generate_all_metric_pdfs()

if __name__ == "__main__":
    main()
