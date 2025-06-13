#!/usr/bin/env python3
"""
Metric-focused Hauser Configuration Analysis - Data analysis only (no plotting)
"""

import pandas as pd
import numpy as np
import argparse
from pathlib import Path
import sys

class HauserMetricAnalyzer:
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
        print(f"📋 Total algorithms: {list(self.data['algorithm_name'].unique())}")
        
        # Filter for Hauser only
        self.hauser_data = self.data[self.data['algorithm_name'] != 'STOMP'].copy()
        print(f"🎯 Hauser algorithms: {list(self.hauser_data['algorithm_name'].unique())}")
        print(f"📈 Hauser configurations: {len(self.hauser_data.groupby(['algorithm_name', 'config']))}")
    
    def load_data(self) -> pd.DataFrame:
        """Load CSV data and add algorithm names."""
        try:
            df = pd.read_csv(self.data_file)
            df['algorithm_name'] = df['algorithm'].map(self.algorithm_names)
            return df
        except Exception as e:
            print(f"❌ Error loading data: {e}")
            sys.exit(1)
    
    def analyze_metric_hauser_data(self, metric: str, info: dict):
        """Analyze one metric for Hauser algorithm configurations (data only, no plotting)."""
        
        print(f"📈 Analyzing {info['title']} - Hauser configurations...")
        
        # Filter data for this metric (Hauser only)
        plot_data = self.hauser_data.copy()
        
        # Apply metric-specific filtering
        if metric in ['execution_time_ms', 'trajectory_length', 'path_quality', 'joint_smoothness']:
            plot_data = plot_data[plot_data['success'] == 1]
        
        plot_data = plot_data.dropna(subset=[metric])
        if metric in ['trajectory_length', 'path_quality', 'joint_smoothness']:
            plot_data = plot_data[plot_data[metric] > 0]
        
        if len(plot_data) == 0:
            print(f"⚠️ No Hauser data available for {metric}")
            return
        
        # Analyze each Hauser algorithm+config combination
        print(f"\n{info['title']} Analysis:")
        print("=" * 50)
        
        for (algorithm, config), group in plot_data.groupby(['algorithm_name', 'config']):
            if len(group) > 0:
                values = group[metric].values
                
                # Calculate statistics
                mean_val = np.mean(values)
                median_val = np.median(values)
                std_val = np.std(values)
                min_val = np.min(values)
                max_val = np.max(values)
                count = len(values)
                
                print(f"{algorithm} - {config}:")
                print(f"  Count: {count}")
                print(f"  Mean: {mean_val:.3f} {info['ylabel'].lower()}")
                print(f"  Median: {median_val:.3f} {info['ylabel'].lower()}")
                print(f"  Std Dev: {std_val:.3f}")
                print(f"  Min: {min_val:.3f}")
                print(f"  Max: {max_val:.3f}")
                print()
        
        print(f"✅ Completed {info['title']} Hauser analysis")
    
    def generate_all_hauser_metrics(self):
        """Analyze all metrics for Hauser configurations (data only, no plotting)."""
        
        print(f"\n🚀 Analyzing Hauser-only metric data...")
        print("=" * 60)
        
        for metric, info in self.metrics.items():
            self.analyze_metric_hauser_data(metric, info)
        
        print("\n" + "=" * 60)
        print("✅ All Hauser metric analyses completed!")
        print(f"🎯 Focus: Hauser algorithm configurations only")

def main():
    print("🚀 Starting Hauser-only metric analysis...")
    
    parser = argparse.ArgumentParser(description='Hauser-only Metric Data Analyzer')
    parser.add_argument('data_file', help='CSV file with evaluation results')
    parser.add_argument('--output-dir', '-o', default='results/hauser_metrics', help='Output directory (not used for data analysis)')
    
    args = parser.parse_args()
    print(f"📁 Input file: {args.data_file}")
    
    if not Path(args.data_file).exists():
        print(f"❌ Data file not found: {args.data_file}")
        sys.exit(1)
    
    analyzer = HauserMetricAnalyzer(args.data_file, args.output_dir)
    analyzer.generate_all_hauser_metrics()

if __name__ == "__main__":
    main()
