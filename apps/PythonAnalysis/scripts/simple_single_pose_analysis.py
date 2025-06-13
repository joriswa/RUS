#!/usr/bin/env python3
import pandas as pd
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from pathlib import Path

# Test simple execution
print("🚀 SinglePose Evaluator Execution Time Analysis")
print("=" * 50)

# Generate simple test data
np.random.seed(42)
algorithms = ['STOMP', 'HAUSER_RRT', 'HAUSER_RRT_STAR', 'HAUSER_INFORMED_RRT']
data = []

for i, algorithm in enumerate(algorithms):
    for trial in range(20):
        base_time = [850, 450, 720, 620][i]
        planning_time = max(50, np.random.normal(base_time, 150))
        success = np.random.random() < [0.85, 0.75, 0.82, 0.78][i]
        
        data.append({
            'algorithm': algorithm,
            'planning_time_ms': planning_time,
            'success': 1 if success else 0,
            'iterations': np.random.randint(100, 2000),
            'execution_time': max(100, np.random.normal(800, 150)) if success else 0
        })

df = pd.DataFrame(data)
print(f"✅ Generated {len(df)} evaluation records")

# Create comprehensive 6-panel analysis
fig, axes = plt.subplots(2, 3, figsize=(20, 14))
fig.suptitle('SinglePose Evaluator: Comprehensive Performance Analysis\n' +
             'Execution Time & Computation Metrics for Trajectory Planning Algorithms', 
             fontsize=18, fontweight='bold', y=0.95)

colors = ['#3498db', '#e74c3c', '#2ecc71', '#f39c12']

# 1. Planning Time Distribution (Top Left)
ax1 = axes[0, 0]
planning_data = [df[df['algorithm'] == alg]['planning_time_ms'].values for alg in algorithms]
bp1 = ax1.boxplot(planning_data, tick_labels=algorithms, patch_artist=True, showmeans=True)

for patch, color in zip(bp1['boxes'], colors):
    patch.set_facecolor(color)
    patch.set_alpha(0.7)

ax1.set_title('Planning Time Distribution', fontweight='bold', fontsize=14)
ax1.set_ylabel('Planning Time (ms)', fontsize=12)
ax1.tick_params(axis='x', rotation=45)
ax1.grid(True, alpha=0.3)

# 2. Execution Time (Successful Trials) (Top Center)
ax2 = axes[0, 1]
successful_df = df[(df['success'] == 1) & (df['execution_time'] > 0)]
exec_data = [successful_df[successful_df['algorithm'] == alg]['execution_time'].values for alg in algorithms]
bp2 = ax2.boxplot(exec_data, tick_labels=algorithms, patch_artist=True, showmeans=True)

for patch, color in zip(bp2['boxes'], colors):
    patch.set_facecolor(color)
    patch.set_alpha(0.7)

ax2.set_title('Execution Time (Successful Trials)', fontweight='bold', fontsize=14)
ax2.set_ylabel('Execution Time (ms)', fontsize=12)
ax2.tick_params(axis='x', rotation=45)
ax2.grid(True, alpha=0.3)

# 3. Computation Efficiency (Top Right)
ax3 = axes[0, 2]
df['time_per_iteration'] = df['planning_time_ms'] / df['iterations']
efficiency_data = [df[df['algorithm'] == alg]['time_per_iteration'].values for alg in algorithms]
bp3 = ax3.boxplot(efficiency_data, tick_labels=algorithms, patch_artist=True, showmeans=True)

for patch, color in zip(bp3['boxes'], colors):
    patch.set_facecolor(color)
    patch.set_alpha(0.7)

ax3.set_title('Computation Efficiency', fontweight='bold', fontsize=14)
ax3.set_ylabel('Time per Iteration (ms)', fontsize=12)
ax3.tick_params(axis='x', rotation=45)
ax3.grid(True, alpha=0.3)

# 4. Success Rate Comparison (Bottom Left)
ax4 = axes[1, 0]
success_rates = [df[df['algorithm'] == alg]['success'].mean() * 100 for alg in algorithms]
bars = ax4.bar(algorithms, success_rates, color=colors, alpha=0.7)

ax4.set_title('Success Rate by Algorithm', fontweight='bold', fontsize=14)
ax4.set_ylabel('Success Rate (%)', fontsize=12)
ax4.tick_params(axis='x', rotation=45)
ax4.grid(True, alpha=0.3, axis='y')
ax4.set_ylim(0, 100)

# Add percentage labels
for bar, rate in zip(bars, success_rates):
    height = bar.get_height()
    ax4.text(bar.get_x() + bar.get_width()/2., height + 1,
            f'{rate:.1f}%', ha='center', va='bottom', fontweight='bold')

# 5. Iterations Distribution (Bottom Center)
ax5 = axes[1, 1]
iter_data = [df[df['algorithm'] == alg]['iterations'].values for alg in algorithms]
bp5 = ax5.boxplot(iter_data, tick_labels=algorithms, patch_artist=True, showmeans=True)

for patch, color in zip(bp5['boxes'], colors):
    patch.set_facecolor(color)
    patch.set_alpha(0.7)

ax5.set_title('Iterations Required', fontweight='bold', fontsize=14)
ax5.set_ylabel('Number of Iterations', fontsize=12)
ax5.tick_params(axis='x', rotation=45)
ax5.grid(True, alpha=0.3)

# 6. Success vs Failed Planning Times (Bottom Right)
ax6 = axes[1, 2]
successful = df[df['success'] == 1]
failed = df[df['success'] == 0]

success_times = [successful[successful['algorithm'] == alg]['planning_time_ms'].values for alg in algorithms]
failed_times = [failed[failed['algorithm'] == alg]['planning_time_ms'].values for alg in algorithms]

x_pos = np.arange(len(algorithms))
width = 0.35

bp6_success = ax6.boxplot(success_times, positions=x_pos - width/2, widths=width*0.8, 
                         patch_artist=True, showmeans=True)
bp6_failed = ax6.boxplot(failed_times, positions=x_pos + width/2, widths=width*0.8, 
                        patch_artist=True, showmeans=True)

for patch in bp6_success['boxes']:
    patch.set_facecolor('#2ecc71')
    patch.set_alpha(0.7)

for patch in bp6_failed['boxes']:
    patch.set_facecolor('#e74c3c')
    patch.set_alpha(0.7)

ax6.set_title('Success vs Failed Planning Times', fontweight='bold', fontsize=14)
ax6.set_ylabel('Planning Time (ms)', fontsize=12)
ax6.set_xticks(x_pos)
ax6.set_xticklabels(algorithms, rotation=45)
ax6.legend([bp6_success['boxes'][0], bp6_failed['boxes'][0]], 
          ['Successful', 'Failed'], loc='upper right')
ax6.grid(True, alpha=0.3)

plt.tight_layout()
output_dir = Path("./comprehensive_single_pose_results")
output_dir.mkdir(exist_ok=True)
output_dir.mkdir(exist_ok=True)
plt.savefig(output_dir / "single_pose_execution_time_boxplots.png", dpi=300, bbox_inches='tight')
plt.close()

print(f"✅ Saved boxplot to: {output_dir / 'single_pose_execution_time_boxplots.png'}")

# Create performance summary
summary = df.groupby('algorithm').agg({
    'success': 'mean',
    'planning_time_ms': ['mean', 'std', 'min', 'max'],
    'iterations': 'mean'
}).round(2)

print("\n🎯 PERFORMANCE SUMMARY")
print("=" * 60)
for algorithm in algorithms:
    alg_data = df[df['algorithm'] == algorithm]
    success_rate = alg_data['success'].mean() * 100
    avg_time = alg_data['planning_time_ms'].mean()
    std_time = alg_data['planning_time_ms'].std()
    
    print(f"{algorithm}:")
    print(f"  Success Rate: {success_rate:.1f}%")
    print(f"  Avg Planning Time: {avg_time:.1f} ± {std_time:.1f} ms")
    print()

print("✨ Analysis complete!")
