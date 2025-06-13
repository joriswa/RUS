#!/usr/bin/env python3
"""
Underlying Cost Function Visualization

Plot the cost function landscape for selectGoalPose over joint 7 configuration.
Uses real PathPlanner cost evaluation to show actual optimization landscape.
Shows valid cost ranges and infeasible regions (marked in red).
"""

import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
import sys

def plot_cost_function(pose_id=None):
    """Plot cost function vs q7 for a specific pose or all poses."""
    
    # Load data
    df = pd.read_csv('underlying_cost_function_data.csv')
    
    if pose_id is not None:
        # Plot single pose
        pose_data = df[df['pose_id'] == pose_id]
        if pose_data.empty:
            print(f"No data found for pose {pose_id}")
            return
        
        plt.figure(figsize=(14, 10))
        
        # Separate successful and failed evaluations
        successful_data = pose_data[pose_data['success'] == 1]
        failed_data = pose_data[pose_data['success'] == 0]
        
        # Plot successful evaluations with finite costs
        if not successful_data.empty:
            finite_mask = np.isfinite(successful_data['cost'])
            valid_data = successful_data[finite_mask]
            
            if not valid_data.empty:
                # Create a smooth cost curve
                q7_vals = valid_data['q7'].values
                cost_vals = valid_data['cost'].values
                
                # Sort by q7 for proper line plotting
                sort_idx = np.argsort(q7_vals)
                q7_sorted = q7_vals[sort_idx]
                cost_sorted = cost_vals[sort_idx]
                
                # Plot as scatter points and line
                plt.scatter(q7_sorted, cost_sorted, c='blue', alpha=0.7, s=30, 
                           label=f'Valid solutions (n={len(valid_data)})', zorder=3)
                plt.plot(q7_sorted, cost_sorted, 'b-', alpha=0.5, linewidth=1, zorder=2)
                
                # Find optimal q7 (minimum cost)
                min_idx = np.argmin(cost_sorted)
                opt_q7 = q7_sorted[min_idx]
                opt_cost = cost_sorted[min_idx]
                plt.scatter(opt_q7, opt_cost, c='green', s=100, marker='*', 
                           label=f'Optimal: q7={opt_q7:.3f}, cost={opt_cost:.2f}', zorder=4)
                
                # Set plot limits based on valid data
                q7_range = q7_sorted.max() - q7_sorted.min()
                cost_range = cost_sorted.max() - cost_sorted.min()
                
                plt.xlim(q7_sorted.min() - 0.1 * q7_range, q7_sorted.max() + 0.1 * q7_range)
                plt.ylim(cost_sorted.min() - 0.1 * cost_range, cost_sorted.max() + 0.1 * cost_range)
        
        # Mark infeasible regions
        if not failed_data.empty:
            # Get full q7 range from data
            all_q7 = np.sort(pose_data['q7'].values)
            q7_step = np.mean(np.diff(all_q7)) if len(all_q7) > 1 else 0.1
            
            # Group consecutive failed q7 values
            failed_q7_sorted = np.sort(failed_data['q7'].values)
            
            if len(failed_q7_sorted) > 0:
                # Find continuous regions of infeasibility
                regions = []
                start = failed_q7_sorted[0]
                end = failed_q7_sorted[0]
                
                for i in range(1, len(failed_q7_sorted)):
                    if failed_q7_sorted[i] - failed_q7_sorted[i-1] <= q7_step * 1.5:
                        end = failed_q7_sorted[i]
                    else:
                        regions.append((start, end))
                        start = failed_q7_sorted[i]
                        end = failed_q7_sorted[i]
                regions.append((start, end))
                
                # Plot infeasible regions
                for i, (region_start, region_end) in enumerate(regions):
                    plt.axvspan(region_start - q7_step/2, region_end + q7_step/2, 
                              alpha=0.3, color='red', 
                              label='Infeasible regions' if i == 0 else "", zorder=1)
        
        pose_name = pose_data['pose_name'].iloc[0]
        plt.title(f'PathPlanner Cost Function Landscape - {pose_name}\n'
                 f'Joint 7 Configuration vs Optimization Cost', fontsize=14, fontweight='bold')
        plt.xlabel('Joint 7 (q7) [radians]', fontsize=12)
        plt.ylabel('PathPlanner Cost Function Value', fontsize=12)
        plt.legend(fontsize=10)
        plt.grid(True, alpha=0.3)
        
        # Add statistics text box
        if not successful_data.empty:
            stats_text = f"Statistics:\n"
            stats_text += f"• Feasible q7 range: {len(valid_data)} points\n"
            stats_text += f"• Success rate: {len(successful_data)/len(pose_data)*100:.1f}%\n"
            stats_text += f"• Cost range: {cost_sorted.min():.2f} - {cost_sorted.max():.2f}\n"
            stats_text += f"• Optimal cost: {opt_cost:.2f}"
            
            plt.text(0.02, 0.98, stats_text, transform=plt.gca().transAxes, 
                    verticalalignment='top', bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.8),
                    fontsize=9)
        
        # Save plot
        plt.tight_layout()
        plt.savefig(f'plots/cost_function_pose_{pose_id}.png', dpi=150, bbox_inches='tight')
        print(f"Saved: plots/cost_function_pose_{pose_id}.png")
        plt.show()
        
    else:
        # Plot all poses in a grid
        poses = sorted(df['pose_id'].unique())
        n_poses = len(poses)
        
        # Create subplot grid - larger for better visibility
        cols = min(3, n_poses)  # Reduced columns for larger subplots
        rows = (n_poses + cols - 1) // cols
        
        fig, axes = plt.subplots(rows, cols, figsize=(6*cols, 4*rows))
        if n_poses == 1:
            axes = [axes]
        elif rows == 1:
            axes = axes.reshape(1, -1)
        
        for i, pose_id in enumerate(poses):
            row = i // cols
            col = i % cols
            ax = axes[row, col] if rows > 1 else axes[col]
            
            pose_data = df[df['pose_id'] == pose_id]
            
            # Separate successful and failed evaluations
            successful_data = pose_data[pose_data['success'] == 1]
            failed_data = pose_data[pose_data['success'] == 0]
            
            if not successful_data.empty:
                finite_mask = np.isfinite(successful_data['cost'])
                valid_data = successful_data[finite_mask]
                
                if not valid_data.empty:
                    # Plot valid costs
                    q7_vals = valid_data['q7'].values
                    cost_vals = valid_data['cost'].values
                    
                    # Sort for line plotting
                    sort_idx = np.argsort(q7_vals)
                    q7_sorted = q7_vals[sort_idx]
                    cost_sorted = cost_vals[sort_idx]
                    
                    ax.scatter(q7_sorted, cost_sorted, c='blue', alpha=0.7, s=15)
                    ax.plot(q7_sorted, cost_sorted, 'b-', alpha=0.4, linewidth=0.8)
                    
                    # Mark optimal point
                    min_idx = np.argmin(cost_sorted)
                    opt_q7 = q7_sorted[min_idx]
                    opt_cost = cost_sorted[min_idx]
                    ax.scatter(opt_q7, opt_cost, c='green', s=50, marker='*', zorder=3)
                    
                    # Set limits
                    q7_range = q7_sorted.max() - q7_sorted.min()
                    cost_range = cost_sorted.max() - cost_sorted.min()
                    ax.set_xlim(q7_sorted.min() - 0.1 * q7_range, q7_sorted.max() + 0.1 * q7_range)
                    ax.set_ylim(cost_sorted.min() - 0.1 * cost_range, cost_sorted.max() + 0.1 * cost_range)
                    
                    # Mark infeasible regions
                    if not failed_data.empty:
                        all_q7 = np.sort(pose_data['q7'].values)
                        q7_step = np.mean(np.diff(all_q7)) if len(all_q7) > 1 else 0.1
                        
                        failed_q7_sorted = np.sort(failed_data['q7'].values)
                        
                        # Group consecutive failed regions
                        if len(failed_q7_sorted) > 0:
                            regions = []
                            start = failed_q7_sorted[0]
                            end = failed_q7_sorted[0]
                            
                            for j in range(1, len(failed_q7_sorted)):
                                if failed_q7_sorted[j] - failed_q7_sorted[j-1] <= q7_step * 1.5:
                                    end = failed_q7_sorted[j]
                                else:
                                    regions.append((start, end))
                                    start = failed_q7_sorted[j]
                                    end = failed_q7_sorted[j]
                            regions.append((start, end))
                            
                            # Plot regions within current view
                            for region_start, region_end in regions:
                                if region_start <= ax.get_xlim()[1] and region_end >= ax.get_xlim()[0]:
                                    ax.axvspan(max(region_start - q7_step/2, ax.get_xlim()[0]), 
                                             min(region_end + q7_step/2, ax.get_xlim()[1]), 
                                             alpha=0.2, color='red')
                else:
                    ax.text(0.5, 0.5, 'No valid\nsolutions', transform=ax.transAxes, 
                           ha='center', va='center', fontsize=10, 
                           bbox=dict(boxstyle='round', facecolor='lightcoral', alpha=0.7))
            else:
                ax.text(0.5, 0.5, 'No valid\nsolutions', transform=ax.transAxes, 
                       ha='center', va='center', fontsize=10,
                       bbox=dict(boxstyle='round', facecolor='lightcoral', alpha=0.7))
            
            # Calculate success rate for this pose
            success_rate = len(successful_data) / len(pose_data) * 100 if len(pose_data) > 0 else 0
            
            ax.set_title(f'Pose {pose_id} ({success_rate:.0f}% feasible)', fontsize=11, fontweight='bold')
            ax.set_xlabel('q7 [rad]', fontsize=9)
            ax.set_ylabel('Cost', fontsize=9)
            ax.grid(True, alpha=0.2)
            ax.tick_params(labelsize=8)
        
        # Hide empty subplots
        for i in range(n_poses, rows * cols):
            row = i // cols
            col = i % cols
            axes[row, col].set_visible(False)
        
        plt.suptitle('PathPlanner Cost Function Landscapes - All Poses\n'
                    'Blue: Feasible solutions, Green stars: Optimal q7, Red: Infeasible regions', 
                    fontsize=14, fontweight='bold')
        plt.tight_layout()
        plt.savefig('plots/cost_function_all_poses.png', dpi=150, bbox_inches='tight')
        print("Saved: plots/cost_function_all_poses.png")
        plt.show()

if __name__ == "__main__":
    # Parse command line argument for pose ID
    if len(sys.argv) > 1:
        try:
            pose_id = int(sys.argv[1])
            plot_cost_function(pose_id)
        except ValueError:
            print("Usage: python plot_underlying_cost_function.py [pose_id]")
            print("       pose_id should be an integer")
    else:
        # Show available poses
        df = pd.read_csv('underlying_cost_function_data.csv')
        poses = sorted(df['pose_id'].unique())
        print(f"Available poses: {poses}")
        print("Usage: python plot_underlying_cost_function.py [pose_id]")
        print("       or run without arguments to plot all poses")
        
        # Plot all poses by default
        plot_cost_function()



