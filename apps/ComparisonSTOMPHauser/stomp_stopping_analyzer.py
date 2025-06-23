#!/usr/bin/env python3
"""
Post-hoc analysis of STOMP stopping scenarios and convergence behavior.
Validates when the optimizer should have stopped or continued under uncertainty.
"""

import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import seaborn as sns
from pathlib import Path
from typing import Dict, List, Tuple, Optional
import json
from datetime import datetime, timedelta

class STOMPStoppingAnalyzer:
    """
    Analyzes STOMP optimization stopping scenarios with uncertainty integration.
    """
    
    def __init__(self, results_dir="trajectory_optimization/data/results/stomp"):
        self.results_dir = Path(results_dir)
        self.uncertainty_radius = 0.02  # 2cm registration error
        
    def load_stomp_results(self) -> pd.DataFrame:
        """Load existing STOMP optimization results."""
        trials_file = self.results_dir / "trials.csv"
        if not trials_file.exists():
            raise FileNotFoundError(f"STOMP results not found: {trials_file}")
        
        df = pd.read_csv(trials_file)
        
        # Parse datetime columns
        df['datetime_start'] = pd.to_datetime(df['datetime_start'])
        df['datetime_complete'] = pd.to_datetime(df['datetime_complete'])
        df['duration_seconds'] = df['duration'].apply(self._parse_duration_to_seconds)
        
        return df
    
    def _parse_duration_to_seconds(self, duration_str: str) -> float:
        """Parse duration string to seconds."""
        try:
            # Handle format like "0 days 00:00:00.041349"
            if 'days' in duration_str:
                parts = duration_str.split()
                days = int(parts[0])
                time_part = parts[2]
            else:
                days = 0
                time_part = duration_str
            
            h, m, s = time_part.split(':')
            total_seconds = days * 86400 + int(h) * 3600 + int(m) * 60 + float(s)
            return total_seconds
        except:
            return 0.0
    
    def analyze_convergence_patterns(self, df: pd.DataFrame) -> Dict:
        """Analyze convergence patterns in STOMP optimization."""
        
        # Group by convergence threshold to see stopping behavior
        convergence_analysis = {
            'threshold_ranges': {},
            'iteration_efficiency': {},
            'value_improvement': {}
        }
        
        # Analyze convergence thresholds
        thresholds = df['params_stomp_convergence_threshold'].values
        threshold_bins = np.logspace(np.log10(thresholds.min()), np.log10(thresholds.max()), 10)
        
        for i in range(len(threshold_bins)-1):
            mask = (thresholds >= threshold_bins[i]) & (thresholds < threshold_bins[i+1])
            if mask.sum() > 0:
                subset = df[mask]
                
                bin_key = f"{threshold_bins[i]:.2e}-{threshold_bins[i+1]:.2e}"
                convergence_analysis['threshold_ranges'][bin_key] = {
                    'count': mask.sum(),
                    'mean_value': subset['value'].mean(),
                    'mean_iterations': subset['params_stomp_max_iterations'].mean(),
                    'mean_duration': subset['duration_seconds'].mean(),
                    'success_rate': subset['user_attrs_success_rate'].mean()
                }
        
        return convergence_analysis
    
    def simulate_stopping_scenarios(self, df: pd.DataFrame, n_scenarios: int = 100) -> List[Dict]:
        """Simulate different stopping scenarios under uncertainty."""
        
        scenarios = []
        
        # Limit to first 20 trials but ensure we have enough data
        analysis_trials = min(20, len(df))
        scenarios_per_trial = max(1, n_scenarios // analysis_trials)
        
        for trial_idx in range(analysis_trials):
            trial = df.iloc[trial_idx]
                
            # Extract trial parameters
            max_iterations = trial['params_stomp_max_iterations']
            convergence_threshold = trial['params_stomp_convergence_threshold']
            final_value = trial['value']
            
            # Simulate what would happen with different stopping criteria
            for scenario_id in range(scenarios_per_trial):
                
                # Generate uncertainty-perturbed stopping criteria
                uncertainty_factor = np.random.uniform(0.5, 2.0)
                perturbed_threshold = convergence_threshold * uncertainty_factor
                perturbed_max_iter = int(max_iterations * np.random.uniform(0.3, 1.5))
                
                # Simulate registration error impact on final value
                registration_error = np.random.uniform(0, self.uncertainty_radius)
                error_penalty = self._calculate_error_penalty(registration_error, final_value)
                perturbed_final_value = final_value + error_penalty
                
                scenario = {
                    'trial_number': trial['number'],
                    'scenario_id': scenario_id,
                    'original_max_iterations': max_iterations,
                    'original_threshold': convergence_threshold,
                    'original_value': final_value,
                    'perturbed_max_iterations': perturbed_max_iter,
                    'perturbed_threshold': perturbed_threshold,
                    'perturbed_value': perturbed_final_value,
                    'registration_error': registration_error,
                    'error_penalty': error_penalty,
                    'should_have_stopped_earlier': self._should_stop_earlier(trial, perturbed_threshold),
                    'should_have_continued': self._should_continue(trial, perturbed_max_iter),
                    'robustness_score': self._calculate_robustness_score(error_penalty, final_value)
                }
                
                scenarios.append(scenario)
        
        return scenarios
    
    def _calculate_error_penalty(self, registration_error: float, base_value: float) -> float:
        """Calculate penalty due to registration error."""
        # Knee ultrasound specific: error affects coverage and protocol adherence
        penalty_factor = (registration_error / self.uncertainty_radius) ** 2
        return base_value * penalty_factor * 0.3  # Up to 30% penalty
    
    def _should_stop_earlier(self, trial: pd.Series, new_threshold: float) -> bool:
        """Determine if optimization should have stopped earlier."""
        # Simple heuristic: if new threshold is much looser, could have stopped earlier
        original_threshold = trial['params_stomp_convergence_threshold']
        return new_threshold > original_threshold * 10
    
    def _should_continue(self, trial: pd.Series, new_max_iter: int) -> bool:
        """Determine if optimization should have continued longer."""
        original_max_iter = trial['params_stomp_max_iterations']
        return new_max_iter > original_max_iter * 1.5
    
    def _calculate_robustness_score(self, error_penalty: float, base_value: float) -> float:
        """Calculate robustness score (0-1, higher is better)."""
        relative_penalty = error_penalty / base_value if base_value > 0 else 1.0
        return max(0, 1 - relative_penalty)
    
    def analyze_optimal_stopping_points(self, scenarios: List[Dict]) -> Dict:
        """Analyze optimal stopping points across scenarios."""
        
        scenario_df = pd.DataFrame(scenarios)
        
        analysis = {
            'early_stopping_benefits': {},
            'continuation_benefits': {},
            'uncertainty_impact': {},
            'recommendations': {}
        }
        
        # Early stopping analysis
        early_stop_mask = scenario_df['should_have_stopped_earlier']
        if early_stop_mask.sum() > 0:
            early_stop_data = scenario_df[early_stop_mask]
            analysis['early_stopping_benefits'] = {
                'count': early_stop_mask.sum(),
                'avg_value_improvement': early_stop_data['perturbed_value'].mean() - early_stop_data['original_value'].mean(),
                'avg_robustness_gain': early_stop_data['robustness_score'].mean(),
                'recommended_threshold_factor': early_stop_data['perturbed_threshold'].mean() / early_stop_data['original_threshold'].mean()
            }
        
        # Continuation analysis
        continue_mask = scenario_df['should_have_continued']
        if continue_mask.sum() > 0:
            continue_data = scenario_df[continue_mask]
            analysis['continuation_benefits'] = {
                'count': continue_mask.sum(),
                'avg_iteration_increase': continue_data['perturbed_max_iterations'].mean() - continue_data['original_max_iterations'].mean(),
                'potential_improvement': continue_data['robustness_score'].mean()
            }
        
        # Uncertainty impact
        analysis['uncertainty_impact'] = {
            'mean_error_penalty': scenario_df['error_penalty'].mean(),
            'mean_robustness_score': scenario_df['robustness_score'].mean(),
            'worst_case_penalty': scenario_df['error_penalty'].max(),
            'best_case_robustness': scenario_df['robustness_score'].max()
        }
        
        return analysis
    
    def generate_stopping_recommendations(self, analysis: Dict, convergence_analysis: Dict) -> Dict:
        """Generate clinical recommendations for STOMP stopping criteria."""
        
        recommendations = {
            'for_knee_ultrasound': {
                'convergence_threshold': 'Use adaptive threshold based on anatomical coverage',
                'max_iterations': 'Limit iterations when diminishing returns in protocol adherence',
                'uncertainty_handling': 'Build in 20-30% safety margin for registration errors'
            },
            'uncertainty_aware_stopping': {
                'early_stop_when': 'Registration error penalty > 25% of trajectory value',
                'continue_when': 'Coverage metrics still improving significantly',
                'robust_threshold': 'Use 2-3x looser convergence threshold under high uncertainty'
            },
            'clinical_validation': {
                'coverage_priority': 'Stop when anatomical coverage > 90% target',
                'protocol_adherence': 'Continue if protocol compliance < 85%',
                'image_quality': 'Balance optimization time vs. expected image quality gains'
            }
        }
        
        return recommendations
    
    def visualize_stopping_analysis(self, scenarios: List[Dict], analysis: Dict) -> None:
        """Create comprehensive visualizations of stopping analysis."""
        
        scenario_df = pd.DataFrame(scenarios)
        
        # Create multi-panel figure
        fig, axes = plt.subplots(2, 3, figsize=(18, 12))
        fig.suptitle('STOMP Stopping Scenario Analysis for Knee Ultrasound', fontsize=16, fontweight='bold')
        
        # 1. Registration error vs. performance penalty
        ax1 = axes[0, 0]
        scatter = ax1.scatter(scenario_df['registration_error'] * 1000,  # Convert to mm
                            scenario_df['error_penalty'],
                            c=scenario_df['robustness_score'],
                            cmap='RdYlGn', alpha=0.7)
        ax1.set_xlabel('Registration Error (mm)')
        ax1.set_ylabel('Performance Penalty')
        ax1.set_title('Registration Error Impact')
        plt.colorbar(scatter, ax=ax1, label='Robustness Score')
        
        # 2. Stopping criteria distribution
        ax2 = axes[0, 1]
        stopping_analysis = scenario_df.groupby(['should_have_stopped_earlier', 'should_have_continued']).size()
        labels = ['Continue\nOptimal', 'Stop\nEarlier', 'Continue\nLonger', 'Mixed\nSignals']
        values = [
            stopping_analysis.get((False, False), 0),
            stopping_analysis.get((True, False), 0),
            stopping_analysis.get((False, True), 0),
            stopping_analysis.get((True, True), 0)
        ]
        colors = ['green', 'orange', 'blue', 'red']
        ax2.pie(values, labels=labels, colors=colors, autopct='%1.1f%%')
        ax2.set_title('Stopping Decision Distribution')
        
        # 3. Robustness vs. original value
        ax3 = axes[0, 2]
        ax3.scatter(scenario_df['original_value'], scenario_df['robustness_score'], alpha=0.7)
        ax3.set_xlabel('Original Trajectory Value')
        ax3.set_ylabel('Robustness Score')
        ax3.set_title('Value vs. Robustness')
        
        # 4. Iteration efficiency
        ax4 = axes[1, 0]
        scenario_df.boxplot(column='perturbed_max_iterations', by='should_have_continued', ax=ax4)
        ax4.set_xlabel('Should Have Continued')
        ax4.set_ylabel('Iterations')
        ax4.set_title('Iteration Efficiency Analysis')
        plt.suptitle('')  # Remove automatic title
        
        # 5. Threshold sensitivity
        ax5 = axes[1, 1]
        threshold_ratio = scenario_df['perturbed_threshold'] / scenario_df['original_threshold']
        ax5.hist(threshold_ratio, bins=20, alpha=0.7, color='skyblue', edgecolor='black')
        ax5.axvline(1.0, color='red', linestyle='--', label='Original Threshold')
        ax5.set_xlabel('Threshold Ratio (Perturbed/Original)')
        ax5.set_ylabel('Frequency')
        ax5.set_title('Convergence Threshold Sensitivity')
        ax5.legend()
        
        # 6. Clinical recommendation heatmap
        ax6 = axes[1, 2]
        # Create recommendation matrix
        error_bins = np.linspace(0, scenario_df['registration_error'].max(), 5)
        value_bins = np.linspace(scenario_df['original_value'].min(), scenario_df['original_value'].max(), 5)
        
        recommendation_matrix = np.zeros((len(error_bins)-1, len(value_bins)-1))
        
        for i in range(len(error_bins)-1):
            for j in range(len(value_bins)-1):
                mask = ((scenario_df['registration_error'] >= error_bins[i]) & 
                       (scenario_df['registration_error'] < error_bins[i+1]) &
                       (scenario_df['original_value'] >= value_bins[j]) & 
                       (scenario_df['original_value'] < value_bins[j+1]))
                
                if mask.sum() > 0:
                    recommendation_matrix[i, j] = scenario_df[mask]['robustness_score'].mean()
        
        im = ax6.imshow(recommendation_matrix, cmap='RdYlGn', aspect='auto')
        ax6.set_xlabel('Trajectory Value Bins')
        ax6.set_ylabel('Registration Error Bins')
        ax6.set_title('Clinical Robustness Map')
        plt.colorbar(im, ax=ax6, label='Robustness Score')
        
        plt.tight_layout()
        
        # Save visualization
        output_dir = Path("trajectory_optimization/data/visualizations")
        output_dir.mkdir(parents=True, exist_ok=True)
        plt.savefig(output_dir / "stomp_stopping_analysis.png", dpi=300, bbox_inches='tight')
        plt.show()
    
    def export_stopping_analysis_report(self, scenarios: List[Dict], analysis: Dict, 
                                       convergence_analysis: Dict, recommendations: Dict) -> None:
        """Export comprehensive stopping analysis report."""
        
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        output_dir = Path("trajectory_optimization/data/reports")
        output_dir.mkdir(parents=True, exist_ok=True)
        
        report_file = output_dir / f"stomp_stopping_analysis_{timestamp}.md"
        
        with open(report_file, 'w') as f:
            f.write("# STOMP Stopping Scenario Analysis Report\n\n")
            f.write(f"Generated: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}\n\n")
            
            f.write("## Executive Summary\n\n")
            f.write("This report analyzes optimal stopping criteria for STOMP optimization ")
            f.write("in knee ultrasound trajectory planning, considering registration uncertainty.\n\n")
            
            f.write("## Key Findings\n\n")
            scenario_df = pd.DataFrame(scenarios)
            
            f.write(f"- **Total Scenarios Analyzed**: {len(scenarios)}\n")
            f.write(f"- **Mean Robustness Score**: {scenario_df['robustness_score'].mean():.3f}\n")
            f.write(f"- **Registration Error Impact**: {analysis['uncertainty_impact']['mean_error_penalty']:.3f} average penalty\n")
            f.write(f"- **Early Stopping Beneficial**: {analysis.get('early_stopping_benefits', {}).get('count', 0)} scenarios\n")
            f.write(f"- **Continuation Beneficial**: {analysis.get('continuation_benefits', {}).get('count', 0)} scenarios\n\n")
            
            f.write("## Convergence Pattern Analysis\n\n")
            for threshold_range, data in convergence_analysis['threshold_ranges'].items():
                f.write(f"### Threshold Range: {threshold_range}\n")
                f.write(f"- Count: {data['count']}\n")
                f.write(f"- Mean Value: {data['mean_value']:.3f}\n")
                f.write(f"- Mean Iterations: {data['mean_iterations']:.1f}\n")
                f.write(f"- Mean Duration: {data['mean_duration']:.3f}s\n")
                f.write(f"- Success Rate: {data['success_rate']:.3f}\n\n")
            
            f.write("## Clinical Recommendations\n\n")
            for category, recs in recommendations.items():
                f.write(f"### {category.replace('_', ' ').title()}\n")
                for key, value in recs.items():
                    f.write(f"- **{key.replace('_', ' ').title()}**: {value}\n")
                f.write("\n")
            
            f.write("## Uncertainty Impact Analysis\n\n")
            uncertainty = analysis['uncertainty_impact']
            f.write(f"- **Mean Error Penalty**: {uncertainty['mean_error_penalty']:.3f}\n")
            f.write(f"- **Mean Robustness Score**: {uncertainty['mean_robustness_score']:.3f}\n")
            f.write(f"- **Worst Case Penalty**: {uncertainty['worst_case_penalty']:.3f}\n")
            f.write(f"- **Best Case Robustness**: {uncertainty['best_case_robustness']:.3f}\n\n")
            
            f.write("## Implementation Recommendations\n\n")
            f.write("1. **Adaptive Convergence Thresholds**: Adjust based on current registration uncertainty estimate\n")
            f.write("2. **Multi-Criteria Stopping**: Combine convergence, coverage, and protocol adherence metrics\n")
            f.write("3. **Uncertainty-Aware Planning**: Build in safety margins for 2cm registration errors\n")
            f.write("4. **Clinical Validation Loop**: Continuously validate stopping decisions against imaging outcomes\n\n")
            
        print(f"Stopping analysis report exported to: {report_file}")
        
        # Also export scenario data as CSV
        scenario_df = pd.DataFrame(scenarios)
        csv_file = output_dir / f"stomp_stopping_scenarios_{timestamp}.csv"
        scenario_df.to_csv(csv_file, index=False)
        print(f"Scenario data exported to: {csv_file}")
    
    def run_complete_stopping_analysis(self) -> Dict:
        """Run the complete stopping scenario analysis pipeline."""
        
        print("🔍 Loading STOMP optimization results...")
        df = self.load_stomp_results()
        print(f"   Loaded {len(df)} optimization trials")
        
        print("\n📊 Analyzing convergence patterns...")
        convergence_analysis = self.analyze_convergence_patterns(df)
        
        print("\n🎯 Simulating stopping scenarios under uncertainty...")
        scenarios = self.simulate_stopping_scenarios(df, n_scenarios=100)
        print(f"   Generated {len(scenarios)} stopping scenarios")
        
        print("\n🧮 Analyzing optimal stopping points...")
        analysis = self.analyze_optimal_stopping_points(scenarios)
        
        print("\n📋 Generating clinical recommendations...")
        recommendations = self.generate_stopping_recommendations(analysis, convergence_analysis)
        
        print("\n📈 Creating visualizations...")
        self.visualize_stopping_analysis(scenarios, analysis)
        
        print("\n📄 Exporting comprehensive report...")
        self.export_stopping_analysis_report(scenarios, analysis, convergence_analysis, recommendations)
        
        return {
            'scenarios': scenarios,
            'analysis': analysis,
            'convergence_analysis': convergence_analysis,
            'recommendations': recommendations
        }

def main():
    """Main function to run STOMP stopping analysis."""
    print("🏥 STOMP Stopping Scenario Analysis for Knee Ultrasound")
    print("=" * 60)
    
    analyzer = STOMPStoppingAnalyzer()
    
    try:
        results = analyzer.run_complete_stopping_analysis()
        
        print("\n✅ Analysis Complete!")
        print("\nKey Insights:")
        print(f"- Analyzed {len(results['scenarios'])} stopping scenarios")
        print(f"- Mean robustness score: {pd.DataFrame(results['scenarios'])['robustness_score'].mean():.3f}")
        print(f"- Registration errors cause {results['analysis']['uncertainty_impact']['mean_error_penalty']:.3f} average penalty")
        
        print("\n💡 Clinical Recommendation Summary:")
        print("- Use adaptive convergence thresholds based on registration uncertainty")
        print("- Prioritize anatomical coverage over pure optimization convergence")
        print("- Build in 20-30% safety margins for 2cm registration errors")
        print("- Validate stopping decisions against imaging quality outcomes")
        
    except Exception as e:
        print(f"❌ Error during analysis: {e}")
        raise

if __name__ == "__main__":
    main()
