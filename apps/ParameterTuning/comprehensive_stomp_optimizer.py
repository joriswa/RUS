#!/usr/bin/env python3
"""
Comprehensive Multi-Objective STOMP Parameter Optimization
Methodologically sound approach using NSGAII with proper objective separation
"""

import os
import sys
import time
import json
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
from pathlib import Path
from typing import Tuple, Dict, Any, List, Optional
import optuna
from optuna.samplers import NSGAIISampler
import logging

# Import our parameter evaluation functions
from test_evaluator_simple import create_config_file, run_evaluator, parse_results

# Configure logging
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')
logger = logging.getLogger(__name__)

class ComprehensiveSTOMPOptimizer:
    """
    Comprehensive multi-objective STOMP optimizer using methodologically sound NSGAII approach.
    
    Key principles:
    - NSGAII for true multi-objective optimization without problematic weighted scalarization
    - Optimizes all STOMP parameters including cost weights
    - Fixed learning_rate = 1.0 as requested
    - Comprehensive visualization for decision support
    """
    
    def __init__(self, trajectory_pairs: int = 20, timeout_per_trial: int = 360):
        """Initialize the comprehensive optimizer."""
        self.trajectory_pairs = trajectory_pairs
        self.timeout_per_trial = timeout_per_trial
        
        # Ensure we're in the correct directory
        script_dir = Path(__file__).parent
        os.chdir(script_dir)
        
        # Create results directory
        self.results_dir = Path("comprehensive_optimization_results")
        self.results_dir.mkdir(exist_ok=True)
        
        # Create plots directory
        self.plots_dir = self.results_dir / "plots"
        self.plots_dir.mkdir(exist_ok=True)
    
    def suggest_parameters(self, trial: optuna.Trial) -> Dict[str, Any]:
        """
        Suggest STOMP parameters for optimization.
        
        Design decisions:
        - learning_rate = 1.0 (fixed as requested)
        - All other parameters including cost weights optimized
        - Cost weights are crucial for STOMP behavior and should be optimized
        """
        # Individual joint standard deviations for all 7 joints
        joint_std_devs = [
            trial.suggest_float(f'joint_std_dev_{i}', 0.01, 0.2) for i in range(7)
        ]
        
        return {
            'temperature': trial.suggest_float('temperature', 0.1, 4.0),
            'learning_rate': 1.0,  # Fixed as requested
            'max_iterations': trial.suggest_int('max_iterations', 50, 500),
            'N': trial.suggest_int('N', 30, 120),
            'num_noisy_trajectories': trial.suggest_int('num_noisy_trajectories', 3, 15),
            'num_best_samples': trial.suggest_int('num_best_samples', 2, 10),
            # Optimize cost weights - these directly influence STOMP behavior
            'obstacle_cost_weight': trial.suggest_float('obstacle_cost_weight', 1.0, 100.0),
            'constraint_cost_weight': trial.suggest_float('constraint_cost_weight', 0.1, 20.0),
            'control_cost_weight': trial.suggest_float('control_cost_weight', 1e-10, 1e-5, log=True),
            'joint_std_devs': joint_std_devs
        }
    
    def evaluate_parameters(self, stomp_params: Dict[str, Any]) -> Dict[str, float]:
        """Evaluate STOMP parameters and return performance metrics."""
        config_file, output_file = create_config_file(
            stomp_params, 
            f"comprehensive_trial_{int(time.time())}.json", 
            self.trajectory_pairs
        )
        
        try:
            success, stdout, stderr, returncode = run_evaluator(config_file, self.timeout_per_trial)
            
            if not success or returncode != 0:
                logger.warning(f"Evaluator failed: {stderr}")
                return {
                    'success_rate': 0.0,
                    'avg_planning_time_ms': 30000.0,  # Timeout value
                    'avg_path_length': 100.0,         # Large penalty value
                    'avg_clearance': 0.001            # Minimal clearance
                }
            
            results = parse_results(stdout, output_file)
            if results is None:
                return {
                    'success_rate': 0.0,
                    'avg_planning_time_ms': 30000.0,
                    'avg_path_length': 100.0,
                    'avg_clearance': 0.001
                }
            
            return {
                'success_rate': results.get('success_rate', 0.0),
                'avg_planning_time_ms': results.get('avg_planning_time_ms', 30000.0),
                'avg_path_length': results.get('avg_path_length', 100.0),
                'avg_clearance': results.get('avg_clearance', 0.001)
            }
            
        finally:
            # Clean up temporary files
            for file_path in [config_file, output_file]:
                if os.path.exists(file_path):
                    os.remove(file_path)
    
    def multi_objective_function(self, trial: optuna.Trial) -> Tuple[float, float, float, float]:
        """
        Multi-objective function for NSGAII optimization.
        
        Returns four objectives:
        1. success_rate (maximize)
        2. avg_planning_time_ms (minimize) 
        3. avg_path_length (minimize)
        4. avg_clearance (maximize)
        
        Note: NSGAII handles different scales naturally, no manual normalization needed
        """
        # Suggest parameters
        stomp_params = self.suggest_parameters(trial)
        
        # Store parameters in trial for analysis
        for key, value in stomp_params.items():
            if key != 'joint_std_devs':
                trial.set_user_attr(key, value)
            else:
                for i, std_dev in enumerate(value):
                    trial.set_user_attr(f'joint_std_dev_{i}', std_dev)
        
        # Evaluate parameters
        metrics = self.evaluate_parameters(stomp_params)
        
        # Store metrics in trial
        for key, value in metrics.items():
            trial.set_user_attr(key, value)
        
        # Return objectives for NSGAII (directions: maximize, minimize, minimize, maximize)
        return (
            metrics['success_rate'],           # Maximize
            metrics['avg_planning_time_ms'],   # Minimize
            metrics['avg_path_length'],        # Minimize
            metrics['avg_clearance']           # Maximize
        )
    
    def run_optimization(self, n_trials: int = 50) -> optuna.Study:
        """Run comprehensive multi-objective optimization."""
        logger.info(f"Starting comprehensive NSGAII optimization with {n_trials} trials")
        
        study = optuna.create_study(
            directions=["maximize", "minimize", "minimize", "maximize"],
            sampler=NSGAIISampler(seed=42, population_size=20),
            study_name="comprehensive_stomp_optimization",
            storage=f"sqlite:///{self.results_dir}/comprehensive_study.db",
            load_if_exists=True
        )
        
        study.optimize(self.multi_objective_function, n_trials=n_trials)
        
        logger.info(f"Optimization completed with {len(study.trials)} trials")
        logger.info(f"Pareto optimal solutions: {len(study.best_trials)}")
        
        return study
    
    def create_comprehensive_plots(self, study: optuna.Study) -> None:
        """Create comprehensive visualization suite for decision support."""
        logger.info("Creating comprehensive visualization suite...")
        
        # Extract data for plotting
        df = study.trials_dataframe()
        pareto_trials = study.best_trials
        
        # Create figure with subplots
        fig = plt.figure(figsize=(20, 16))
        
        # 1. Pareto front visualization (2x2 objective pairs)
        objective_names = ['Success Rate', 'Planning Time (ms)', 'Path Length', 'Clearance (m)']
        objective_directions = ['maximize', 'minimize', 'minimize', 'maximize']
        
        for i in range(4):
            for j in range(i+1, 4):
                plt.subplot(4, 4, i*4 + j + 1)
                
                # Plot all trials
                x_vals = [trial.values[j] for trial in study.trials if trial.values is not None]
                y_vals = [trial.values[i] for trial in study.trials if trial.values is not None]
                plt.scatter(x_vals, y_vals, alpha=0.6, s=20, c='lightblue', label='All trials')
                
                # Plot Pareto front
                pareto_x = [trial.values[j] for trial in pareto_trials]
                pareto_y = [trial.values[i] for trial in pareto_trials]
                plt.scatter(pareto_x, pareto_y, alpha=0.9, s=40, c='red', label='Pareto optimal')
                
                plt.xlabel(f'{objective_names[j]} ({objective_directions[j]})')
                plt.ylabel(f'{objective_names[i]} ({objective_directions[i]})')
                plt.title(f'{objective_names[i]} vs {objective_names[j]}')
                plt.legend()
                plt.grid(True, alpha=0.3)
        
        plt.tight_layout()
        plt.savefig(self.plots_dir / 'pareto_front_analysis.png', dpi=300, bbox_inches='tight')
        plt.close()
        
        # 2. Parallel coordinates plot
        self._create_parallel_coordinates_plot(pareto_trials)
        
        # 4. Parameter sensitivity analysis
        self._create_parameter_sensitivity_plot(study)
        
        # 5. Cost weight analysis  
        self._create_cost_weight_analysis(study)
        
        # 6. Convergence analysis
        self._create_convergence_plot(study)
        
        # 7. Decision support summary
        self._create_decision_support_summary(study)
        
        logger.info(f"Plots saved to {self.plots_dir}")
    
    def _create_parallel_coordinates_plot(self, pareto_trials: List) -> None:
        """Create parallel coordinates plot for Pareto solutions."""
        fig, ax = plt.subplots(figsize=(14, 8))
        
        # Normalize objectives to [0, 1] for visualization
        objectives = np.array([[trial.values[0], trial.values[1], trial.values[2], trial.values[3]] 
                              for trial in pareto_trials])
        
        if len(objectives) > 0:
            # Normalize each objective
            normalized_objectives = np.zeros_like(objectives)
            for i in range(4):
                obj_min, obj_max = objectives[:, i].min(), objectives[:, i].max()
                if obj_max > obj_min:
                    if i in [0, 3]:  # Success rate and clearance (maximize)
                        normalized_objectives[:, i] = (objectives[:, i] - obj_min) / (obj_max - obj_min)
                    else:  # Planning time and path length (minimize) 
                        normalized_objectives[:, i] = 1 - (objectives[:, i] - obj_min) / (obj_max - obj_min)
                else:
                    normalized_objectives[:, i] = 0.5
            
            # Plot parallel coordinates
            x_positions = [0, 1, 2, 3]
            labels = ['Success Rate\n(higher=better)', 'Planning Time\n(lower=better)', 
                     'Path Length\n(lower=better)', 'Clearance\n(higher=better)']
            
            for i, solution in enumerate(normalized_objectives):
                ax.plot(x_positions, solution, 'o-', alpha=0.7, linewidth=2, 
                       label=f'Solution {i+1}' if i < 10 else '')
            
            ax.set_xticks(x_positions)
            ax.set_xticklabels(labels)
            ax.set_ylabel('Normalized Performance (0=worst, 1=best)')
            ax.set_title('Parallel Coordinates Plot: Pareto Optimal Solutions')
            ax.grid(True, alpha=0.3)
            ax.set_ylim(-0.05, 1.05)
            
            if len(normalized_objectives) <= 10:
                ax.legend(bbox_to_anchor=(1.05, 1), loc='upper left')
        
        plt.tight_layout()
        plt.savefig(self.plots_dir / 'parallel_coordinates.png', dpi=300, bbox_inches='tight')
        plt.close()
    
    def _create_parameter_sensitivity_plot(self, study: optuna.Study) -> None:
        """Create parameter sensitivity analysis."""
        fig, axes = plt.subplots(2, 2, figsize=(16, 12))
        axes = axes.flatten()
        
        # Get data for successful trials only
        successful_trials = [trial for trial in study.trials 
                           if trial.values is not None and trial.user_attrs.get('success_rate', 0) > 0]
        
        if len(successful_trials) < 2:
            return
        
        parameters = ['temperature', 'max_iterations', 'N', 'num_noisy_trajectories']
        objective_names = ['Success Rate', 'Planning Time (ms)', 'Path Length', 'Clearance (m)']
        
        for i, obj_name in enumerate(objective_names):
            ax = axes[i]
            
            for param in parameters:
                param_values = []
                obj_values = []
                
                for trial in successful_trials:
                    if param in trial.user_attrs and trial.values[i] is not None:
                        param_values.append(trial.user_attrs[param])
                        obj_values.append(trial.values[i])
                
                if len(param_values) > 1:
                    # Calculate correlation
                    correlation = np.corrcoef(param_values, obj_values)[0, 1]
                    ax.scatter(param_values, obj_values, alpha=0.6, s=30, 
                             label=f'{param} (r={correlation:.2f})')
            
            ax.set_ylabel(obj_name)
            ax.set_title(f'Parameter Sensitivity: {obj_name}')
            ax.legend()
            ax.grid(True, alpha=0.3)
        
        plt.tight_layout()
        plt.savefig(self.plots_dir / 'parameter_sensitivity.png', dpi=300, bbox_inches='tight')
        plt.close()
    
    def _create_cost_weight_analysis(self, study: optuna.Study) -> None:
        """Create cost weight analysis plot."""
        fig, axes = plt.subplots(2, 2, figsize=(16, 12))
        axes = axes.flatten()
        
        # Get successful trials
        successful_trials = [trial for trial in study.trials 
                           if trial.values is not None and trial.user_attrs.get('success_rate', 0) > 0]
        
        if len(successful_trials) < 2:
            return
        
        cost_weights = ['obstacle_cost_weight', 'constraint_cost_weight', 'control_cost_weight']
        objective_names = ['Success Rate', 'Planning Time (ms)', 'Path Length', 'Clearance (m)']
        
        # Plot 1: Cost weight distributions for successful trials
        ax = axes[0]
        weight_data = []
        labels = []
        
        for weight in cost_weights:
            values = [trial.user_attrs.get(weight, 0) for trial in successful_trials]
            if weight == 'control_cost_weight':
                values = [np.log10(v) if v > 0 else -10 for v in values]  # Log scale
                labels.append(f'{weight}\n(log10)')
            else:
                labels.append(weight.replace('_', '\n'))
            weight_data.append(values)
        
        ax.boxplot(weight_data, labels=labels)
        ax.set_title('Cost Weight Distributions (Successful Trials)')
        ax.set_ylabel('Weight Value')
        ax.grid(True, alpha=0.3)
        
        # Plot 2-4: Cost weights vs objectives
        for i, obj_name in enumerate(objective_names[1:], 1):  # Skip success rate
            ax = axes[i]
            
            # Show obstacle_cost_weight vs objective
            obstacle_weights = [trial.user_attrs.get('obstacle_cost_weight', 0) for trial in successful_trials]
            obj_values = [trial.values[i] for trial in successful_trials]
            
            if len(obstacle_weights) > 1:
                correlation = np.corrcoef(obstacle_weights, obj_values)[0, 1]
                ax.scatter(obstacle_weights, obj_values, alpha=0.6, s=40)
                ax.set_xlabel('Obstacle Cost Weight')
                ax.set_ylabel(obj_name)
                ax.set_title(f'{obj_name} vs Obstacle Weight (r={correlation:.2f})')
                ax.grid(True, alpha=0.3)
        
        plt.tight_layout()
        plt.savefig(self.plots_dir / 'cost_weight_analysis.png', dpi=300, bbox_inches='tight')
        plt.close()
    
    def _create_convergence_plot(self, study: optuna.Study) -> None:
        """Create convergence analysis plot."""
        fig, axes = plt.subplots(2, 2, figsize=(16, 10))
        axes = axes.flatten()
        
        # Track how objectives evolve over trials
        objective_names = ['Success Rate', 'Planning Time (ms)', 'Path Length', 'Clearance (m)']
        directions = [1, -1, -1, 1]  # 1 for maximize, -1 for minimize
        
        for i, (obj_name, direction) in enumerate(zip(objective_names, directions)):
            ax = axes[i]
            
            trial_numbers = []
            best_values = []
            current_best = float('-inf') if direction == 1 else float('inf')
            
            for trial in study.trials:
                if trial.values is not None:
                    current_value = trial.values[i]
                    if direction == 1:  # Maximize
                        if current_value > current_best:
                            current_best = current_value
                    else:  # Minimize
                        if current_value < current_best:
                            current_best = current_value
                    
                    trial_numbers.append(trial.number)
                    best_values.append(current_best)
            
            if trial_numbers:
                ax.plot(trial_numbers, best_values, 'b-', linewidth=2)
                ax.set_xlabel('Trial Number')
                ax.set_ylabel(f'Best {obj_name}')
                ax.set_title(f'Convergence: {obj_name}')
                ax.grid(True, alpha=0.3)
        
        plt.tight_layout()
        plt.savefig(self.plots_dir / 'convergence_analysis.png', dpi=300, bbox_inches='tight')
        plt.close()
    
    def _create_decision_support_summary(self, study: optuna.Study) -> None:
        """Create decision support summary."""
        pareto_trials = study.best_trials
        
        if not pareto_trials:
            return
        
        # Create summary figure
        fig, ((ax1, ax2), (ax3, ax4)) = plt.subplots(2, 2, figsize=(16, 12))
        
        # 1. Success rate vs planning time (most critical trade-off)
        success_rates = [trial.values[0] for trial in pareto_trials]
        planning_times = [trial.values[1] for trial in pareto_trials]
        
        ax1.scatter(planning_times, success_rates, s=80, c='red', alpha=0.7)
        for i, (x, y) in enumerate(zip(planning_times, success_rates)):
            ax1.annotate(f'S{i+1}', (x, y), xytext=(5, 5), textcoords='offset points')
        ax1.set_xlabel('Planning Time (ms)')
        ax1.set_ylabel('Success Rate')
        ax1.set_title('Critical Trade-off: Success Rate vs Planning Time')
        ax1.grid(True, alpha=0.3)
        
        # 2. Path length vs clearance (quality trade-off)
        path_lengths = [trial.values[2] for trial in pareto_trials]
        clearances = [trial.values[3] for trial in pareto_trials]
        
        ax2.scatter(path_lengths, clearances, s=80, c='blue', alpha=0.7)
        for i, (x, y) in enumerate(zip(path_lengths, clearances)):
            ax2.annotate(f'S{i+1}', (x, y), xytext=(5, 5), textcoords='offset points')
        ax2.set_xlabel('Path Length')
        ax2.set_ylabel('Clearance (m)')
        ax2.set_title('Quality Trade-off: Path Length vs Clearance')
        ax2.grid(True, alpha=0.3)
        
        # 3. Parameter distribution for Pareto solutions
        temperatures = [trial.user_attrs.get('temperature', 0) for trial in pareto_trials]
        obstacle_weights = [trial.user_attrs.get('obstacle_cost_weight', 0) for trial in pareto_trials]
        
        ax3.scatter(temperatures, obstacle_weights, s=80, c='green', alpha=0.7)
        for i, (x, y) in enumerate(zip(temperatures, obstacle_weights)):
            ax3.annotate(f'S{i+1}', (x, y), xytext=(5, 5), textcoords='offset points')
        ax3.set_xlabel('Temperature')
        ax3.set_ylabel('Obstacle Cost Weight')
        ax3.set_title('Pareto Solutions: Key Parameters')
        ax3.grid(True, alpha=0.3)
        
        # 4. Solution ranking table
        ax4.axis('off')
        
        # Create ranking based on balanced performance
        rankings = []
        for i, trial in enumerate(pareto_trials):
            # Normalize and combine objectives (higher is better for all)
            norm_success = trial.values[0]  # Already 0-1
            norm_time = 1 - min(1, trial.values[1] / 30000)  # Invert planning time
            norm_length = 1 - min(1, trial.values[2] / 100)  # Invert path length  
            norm_clearance = min(1, trial.values[3] / 0.2)  # Normalize clearance
            
            score = (norm_success + norm_time + norm_length + norm_clearance) / 4
            rankings.append((i+1, score, trial.values))
        
        rankings.sort(key=lambda x: x[1], reverse=True)
        
        # Create table
        table_data = [['Rank', 'Solution', 'Score', 'Success Rate', 'Time (ms)', 'Path Length', 'Clearance']]
        for rank, (sol_id, score, values) in enumerate(rankings[:5], 1):
            table_data.append([
                f'{rank}', f'S{sol_id}', f'{score:.6f}', 
                f'{values[0]:.6f}', f'{values[1]:.3f}', 
                f'{values[2]:.6f}', f'{values[3]:.6f}'
            ])
        
        table = ax4.table(cellText=table_data[1:], colLabels=table_data[0],
                         cellLoc='center', loc='center')
        table.auto_set_font_size(False)
        table.set_fontsize(10)
        table.scale(1, 2)
        ax4.set_title('Top 5 Balanced Solutions', pad=20)
        
        plt.tight_layout()
        plt.savefig(self.plots_dir / 'decision_support_summary.png', dpi=300, bbox_inches='tight')
        plt.close()
    
    def save_results(self, study: optuna.Study) -> None:
        """Save comprehensive analysis results."""
        results_file = self.results_dir / 'comprehensive_analysis.json'
        
        # Prepare comprehensive analysis
        analysis = {
            'study_info': {
                'study_name': study.study_name,
                'total_trials': len(study.trials),
                'pareto_solutions': len(study.best_trials),
                'timestamp': time.time(),
                'methodology': 'NSGAII multi-objective optimization'
            },
            'design_decisions': {
                'fixed_parameters': {
                    'learning_rate': 1.0
                },
                'optimized_parameters': [
                    'temperature', 'max_iterations', 'N', 
                    'num_noisy_trajectories', 'num_best_samples', 'joint_std_devs',
                    'obstacle_cost_weight', 'constraint_cost_weight', 'control_cost_weight'
                ],
                'objectives': [
                    'success_rate (maximize)',
                    'avg_planning_time_ms (minimize)', 
                    'avg_path_length (minimize)',
                    'avg_clearance (maximize)'
                ]
            },
            'methodology_notes': {
                'approach': 'NSGAII for methodologically sound multi-objective optimization',
                'no_weighted_scalarization': 'Avoided problematic weighted sum of different scales',
                'natural_scale_handling': 'NSGAII handles different objective scales naturally',
                'comprehensive_optimization': 'All STOMP parameters including cost weights optimized'
            }
        }
        
        # Add Pareto solutions
        pareto_solutions = []
        for i, trial in enumerate(study.best_trials):
            solution = {
                'solution_id': i + 1,
                'objectives': {
                    'success_rate': trial.values[0],
                    'planning_time_ms': trial.values[1],
                    'path_length': trial.values[2],
                    'clearance_m': trial.values[3]
                },
                'parameters': {k: v for k, v in trial.user_attrs.items() 
                             if not k.startswith('success_rate') and not k.startswith('avg_')}
            }
            pareto_solutions.append(solution)
        
        analysis['pareto_solutions'] = pareto_solutions
        
        # Save analysis
        with open(results_file, 'w') as f:
            json.dump(analysis, f, indent=2, default=str)
        
        # Save trial data
        df = study.trials_dataframe()
        df.to_csv(self.results_dir / 'all_trials.csv', index=False)
        
        logger.info(f"Results saved to {results_file}")
    
    def print_summary(self, study: optuna.Study) -> None:
        """Print comprehensive optimization summary."""
        print(f"\n{'='*80}")
        print("COMPREHENSIVE STOMP MULTI-OBJECTIVE OPTIMIZATION SUMMARY")
        print(f"{'='*80}")
        print(f"Methodology: NSGAII (methodologically sound for unknown preferences)")
        print(f"Total trials: {len(study.trials)}")
        print(f"Pareto optimal solutions: {len(study.best_trials)}")
        print(f"Fixed learning_rate: 1.0")
        
        print(f"\nObjectives (no problematic weighted scalarization):")
        print(f"  1. Success Rate (maximize)")
        print(f"  2. Planning Time (minimize)")  
        print(f"  3. Path Length (minimize)")
        print(f"  4. Clearance (maximize)")
        
        if study.best_trials:
            print(f"\nTop 3 Pareto Solutions:")
            for i, trial in enumerate(study.best_trials[:3], 1):
                print(f"  Solution {i}:")
                print(f"    Success Rate: {trial.values[0]:.6f}")
                print(f"    Planning Time: {trial.values[1]:.3f}ms")
                print(f"    Path Length: {trial.values[2]:.6f}")
                print(f"    Clearance: {trial.values[3]:.6f}m")
                print(f"    Temperature: {trial.user_attrs.get('temperature', 'N/A'):.4f}")
                print(f"    N: {trial.user_attrs.get('N', 'N/A')}")
                print(f"    Obstacle Weight: {trial.user_attrs.get('obstacle_cost_weight', 'N/A'):.3f}")
                print(f"    Control Weight: {trial.user_attrs.get('control_cost_weight', 'N/A'):.2e}")
        
        print(f"\nVisualization suite created in: {self.plots_dir}")
        print(f"Results saved in: {self.results_dir}")
        print(f"{'='*80}")


def main():
    """Main function for comprehensive optimization."""
    import argparse
    
    parser = argparse.ArgumentParser(description="Comprehensive STOMP Multi-Objective Optimization")
    parser.add_argument("--trials", type=int, default=50,
                       help="Number of optimization trials")
    parser.add_argument("--trajectory-pairs", type=int, default=20,
                       help="Trajectory pairs per evaluation")
    parser.add_argument("--timeout", type=int, default=360,
                       help="Timeout per trial in seconds")
    
    args = parser.parse_args()
    
    # Create and run comprehensive optimizer
    optimizer = ComprehensiveSTOMPOptimizer(
        trajectory_pairs=args.trajectory_pairs,
        timeout_per_trial=args.timeout
    )
    
    print("🎯 Starting Comprehensive Multi-Objective STOMP Optimization...")
    print("📋 Methodology: NSGAII (methodologically sound, no problematic weighted sums)")
    print("🔧 Fixed learning_rate = 1.0, optimizing all other parameters including cost weights")
    
    # Run optimization
    study = optimizer.run_optimization(n_trials=args.trials)
    
    # Create comprehensive visualizations
    optimizer.create_comprehensive_plots(study)
    
    # Save results
    optimizer.save_results(study)
    
    # Print summary
    optimizer.print_summary(study)
    
    print(f"\n✅ Comprehensive optimization completed!")
    print(f"📊 Decision support plots available in: {optimizer.plots_dir}")
    print(f"📈 Use plots to justify parameter selection based on your priorities")


if __name__ == "__main__":
    main()
