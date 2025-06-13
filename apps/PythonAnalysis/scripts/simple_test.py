#!/usr/bin/env python3
import pandas as pd
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from pathlib import Path

# Simple test
print("Starting SinglePose analysis...")

# Create output directory
output_dir = Path("results/single_pose_final_analysis")
output_dir.mkdir(parents=True, exist_ok=True)
print(f"Created directory: {output_dir}")

# Generate simple data
np.random.seed(42)
algorithms = ['STOMP', 'HAUSER_RRT', 'HAUSER_RRT_STAR', 'HAUSER_INFORMED_RRT']

data = []
for alg in algorithms:
    for i in range(20):
        data.append({
            'algorithm': alg,
            'planning_time_ms': np.random.normal(500, 100),
            'success': 1
        })

df = pd.DataFrame(data)
print(f"Generated {len(df)} data points")

# Save data
data_file = output_dir / "test_data.csv"
df.to_csv(data_file, index=False)
print(f"Saved to: {data_file}")

# Create simple plot
fig, ax = plt.subplots(figsize=(10, 6))
planning_data = [df[df['algorithm'] == alg]['planning_time_ms'].values for alg in algorithms]
ax.boxplot(planning_data, labels=algorithms)
ax.set_title('SinglePose Planning Time Analysis')
ax.set_ylabel('Planning Time (ms)')
plt.xticks(rotation=45)
plt.grid(True, alpha=0.3)

plot_file = output_dir / "simple_boxplot.png"
plt.savefig(plot_file, dpi=300, bbox_inches='tight')
plt.close()
print(f"Saved plot: {plot_file}")

print("Analysis complete!")
