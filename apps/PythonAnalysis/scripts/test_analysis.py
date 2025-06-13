#!/usr/bin/env python3
"""
Test script to verify the analysis works
"""

import pandas as pd
import numpy as np
import matplotlib
matplotlib.use('Agg')  # Use non-interactive backend
import matplotlib.pyplot as plt
from pathlib import Path

def main():
    print("Starting test analysis...")
    
    # Create output directory
    output_dir = Path("results/test_analysis")
    output_dir.mkdir(parents=True, exist_ok=True)
    print(f"Created output directory: {output_dir}")
    
    # Generate simple test data
    np.random.seed(42)
    algorithms = ['STOMP', 'HAUSER_RRT', 'HAUSER_RRT_STAR', 'HAUSER_INFORMED_RRT']
    
    data = []
    for i, alg in enumerate(algorithms):
        for trial in range(20):
            data.append({
                'algorithm': alg,
                'planning_time_ms': np.random.normal(500 + i*100, 50),
                'success': np.random.choice([0, 1], p=[0.2, 0.8])
            })
    
    df = pd.DataFrame(data)
    print(f"Generated {len(df)} data points")
    
    # Save data
    data_file = output_dir / "test_data.csv"
    df.to_csv(data_file, index=False)
    print(f"Saved data to: {data_file}")
    
    # Create simple plot
    plt.figure(figsize=(10, 6))
    
    planning_data = [df[df['algorithm'] == alg]['planning_time_ms'].values for alg in algorithms]
    plt.boxplot(planning_data, labels=algorithms)
    plt.title('Test Planning Time Analysis')
    plt.ylabel('Planning Time (ms)')
    plt.xticks(rotation=45)
    plt.grid(True, alpha=0.3)
    
    plot_file = output_dir / "test_boxplot.png"
    plt.savefig(plot_file, dpi=300, bbox_inches='tight')
    plt.close()
    print(f"Saved plot to: {plot_file}")
    
    print("Test analysis completed successfully!")

if __name__ == "__main__":
    main()
