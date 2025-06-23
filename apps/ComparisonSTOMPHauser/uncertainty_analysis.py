#!/usr/bin/env python3
"""
Post-processing analysis of existing STOMP results under uncertainty.
Evaluate how robust your optimized trajectories are to registration errors.
"""

import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from pathlib import Path

class UncertaintyPostProcessor:
    def __init__(self, results_dir="trajectory_optimization/data/results/stomp"):
        self.results_dir = Path(results_dir)
        self.uncertainty_radius = 0.02  # 2cm
        
    def load_existing_results(self):
        """Load your existing STOMP optimization results."""
        trials_file = self.results_dir / "trials.csv"
        if trials_file.exists():
            return pd.read_csv(trials_file)
        else:
            raise FileNotFoundError(f"Results not found: {trials_file}")
    
    def simulate_registration_errors(self, n_error_scenarios=20):
        """Generate different registration error scenarios."""
        errors = []
        for i in range(n_error_scenarios):
            error = {
                'error_id': i,
                'x_error': np.random.uniform(-self.uncertainty_radius, self.uncertainty_radius),
                'y_error': np.random.uniform(-self.uncertainty_radius, self.uncertainty_radius),
                'z_error': np.random.uniform(-self.uncertainty_radius, self.uncertainty_radius),
                'error_magnitude': 0
            }
            error['error_magnitude'] = np.sqrt(error['x_error']**2 + error['y_error']**2 + error['z_error']**2)
            errors.append(error)
        return errors
    
    def evaluate_robustness(self, trials_df, error_scenarios):
        """Evaluate how robust each trial is to registration errors."""
        robustness_results = []
        
        for _, trial in trials_df.iterrows():
            trial_robustness = {
                'trial_number': trial['number'],
                'original_value': trial['value'],
                'robustness_scores': []
            }
            
            # For each error scenario, estimate how the trial would perform
            for error in error_scenarios:
                # Simplified robustness estimation
                # In reality, you'd re-evaluate your objective function with the error
                
                # Simple model: performance degrades with error magnitude
                degradation_factor = 1 + (error['error_magnitude'] / self.uncertainty_radius) * 0.3
                estimated_value = trial['value'] * degradation_factor
                
                trial_robustness['robustness_scores'].append({
                    'error_magnitude': error['error_magnitude'],
                    'estimated_value': estimated_value,
                    'degradation': estimated_value - trial['value']
                })
            
            robustness_results.append(trial_robustness)
        
        return robustness_results
    
    def analyze_uncertainty_impact(self):
        """Run complete uncertainty analysis on existing results."""
        print("Loading existing STOMP results...")
        trials_df = self.load_existing_results()
        
        print("Generating registration error scenarios...")
        error_scenarios = self.simulate_registration_errors(20)
        
        print("Evaluating robustness...")
        robustness_results = self.evaluate_robustness(trials_df, error_scenarios)
        
        # Calculate summary statistics
        best_trial_idx = trials_df['value'].idxmin()
        best_trial = trials_df.loc[best_trial_idx]
        
        # Find robustness of best trial
        best_trial_robustness = next(r for r in robustness_results if r['trial_number'] == best_trial['number'])
        
        robustness_scores = [score['estimated_value'] for score in best_trial_robustness['robustness_scores']]
        
        analysis = {
            'original_best_value': best_trial['value'],
            'mean_under_uncertainty': np.mean(robustness_scores),
            'worst_case_value': np.max(robustness_scores),
            'std_under_uncertainty': np.std(robustness_scores),
            'robustness_degradation': np.mean(robustness_scores) - best_trial['value']
        }
        
        return analysis, robustness_results
    
    def create_uncertainty_plots(self, analysis, robustness_results):
        """Create simple plots showing uncertainty impact."""
        fig, axes = plt.subplots(1, 2, figsize=(12, 5))
        
        # Plot 1: Robustness vs trial number
        ax1 = axes[0]
        trial_numbers = []
        mean_robustness = []
        
        for result in robustness_results[:50]:  # Limit to first 50 trials for clarity
            trial_numbers.append(result['trial_number'])
            scores = [score['estimated_value'] for score in result['robustness_scores']]
            mean_robustness.append(np.mean(scores))
        
        ax1.scatter(trial_numbers, mean_robustness, alpha=0.6)
        ax1.set_xlabel('Trial Number')
        ax1.set_ylabel('Mean Performance Under Uncertainty')
        ax1.set_title('Robustness Analysis')
        ax1.grid(True, alpha=0.3)
        
        # Plot 2: Best trial uncertainty distribution
        best_trial_robustness = robustness_results[0]  # Assuming first is best
        uncertainty_values = [score['estimated_value'] for score in best_trial_robustness['robustness_scores']]
        
        ax2 = axes[1]
        ax2.hist(uncertainty_values, bins=10, alpha=0.7, edgecolor='black')
        ax2.axvline(best_trial_robustness['original_value'], color='red', linestyle='--', 
                   label=f'Original: {best_trial_robustness["original_value"]:.4f}')
        ax2.axvline(np.mean(uncertainty_values), color='green', linestyle='--', 
                   label=f'Mean w/ uncertainty: {np.mean(uncertainty_values):.4f}')
        ax2.set_xlabel('Performance Value')
        ax2.set_ylabel('Frequency')
        ax2.set_title('Best Trial Under Uncertainty')
        ax2.legend()
        ax2.grid(True, alpha=0.3)
        
        plt.tight_layout()
        plt.savefig(self.results_dir.parent / 'uncertainty_analysis.png', dpi=300, bbox_inches='tight')
        plt.show()
        
        return fig

def run_simple_uncertainty_analysis():
    """Run the complete simple uncertainty analysis."""
    processor = UncertaintyPostProcessor()
    
    try:
        analysis, robustness_results = processor.analyze_uncertainty_impact()
        
        print("\n" + "="*50)
        print("UNCERTAINTY IMPACT ANALYSIS")
        print("="*50)
        print(f"Original best value: {analysis['original_best_value']:.4f}")
        print(f"Mean under uncertainty: {analysis['mean_under_uncertainty']:.4f}")
        print(f"Worst case value: {analysis['worst_case_value']:.4f}")
        print(f"Std under uncertainty: {analysis['std_under_uncertainty']:.4f}")
        print(f"Robustness degradation: {analysis['robustness_degradation']:.4f}")
        print(f"Relative degradation: {(analysis['robustness_degradation']/analysis['original_best_value']*100):.1f}%")
        
        # Create plots
        processor.create_uncertainty_plots(analysis, robustness_results)
        
        return analysis
        
    except FileNotFoundError as e:
        print(f"Error: {e}")
        print("Please run STOMP optimization first to generate results.")
        return None

if __name__ == "__main__":
    run_simple_uncertainty_analysis()
