#!/usr/bin/env python3
import pandas as pd
import matplotlib.pyplot as plt
from matplotlib.backends.backend_pdf import PdfPages
import sys
from pathlib import Path

# Simple test
print("Starting simple analysis...")

data_file = sys.argv[1]
output_dir = sys.argv[2]

print(f"Data file: {data_file}")
print(f"Output dir: {output_dir}")

# Create output directory
Path(output_dir).mkdir(exist_ok=True)

# Load data
print("Loading data...")
df = pd.read_csv(data_file)
print(f"Loaded {len(df)} rows")

# Algorithm mapping
algorithm_names = {
    0: 'STOMP',
    1: 'Hauser RRT',
    2: 'Hauser RRT*', 
    3: 'Hauser Informed RRT*',
    4: 'Hauser Bi-RRT'
}

df['algorithm_name'] = df['algorithm'].map(algorithm_names)
df['algorithm_family'] = df['algorithm_name'].apply(
    lambda x: 'STOMP' if x == 'STOMP' else 'Hauser'
)

print("Unique algorithms:", df['algorithm_name'].unique())

# Create one simple plot for planning time
metric = 'planning_time_ms'
plot_data = df.dropna(subset=[metric])

print(f"Creating plot for {metric}...")

filename = Path(output_dir) / f"{metric}_simple.pdf"

with PdfPages(filename) as pdf:
    fig, ax = plt.subplots(1, 1, figsize=(10, 6))
    
    # Simple boxplot by algorithm family
    stomp_data = plot_data[plot_data['algorithm_family'] == 'STOMP'][metric]
    hauser_data = plot_data[plot_data['algorithm_family'] == 'Hauser'][metric]
    
    box_data = [stomp_data, hauser_data]
    box_labels = ['STOMP', 'Hauser']
    
    bp = ax.boxplot(box_data, patch_artist=True, labels=box_labels)
    bp['boxes'][0].set_facecolor('#1f77b4')  # Blue for STOMP
    bp['boxes'][1].set_facecolor('#ff7f0e')  # Orange for Hauser
    
    ax.set_title('Planning Time by Algorithm Family')
    ax.set_ylabel('Time (ms)')
    ax.grid(True, alpha=0.3)
    
    plt.tight_layout()
    pdf.savefig(fig, bbox_inches='tight')
    plt.close()

print(f"Saved plot to: {filename}")
print("Done!")
