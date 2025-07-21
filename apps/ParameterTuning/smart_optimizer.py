#!/usr/bin/env python3
"""
Smart Multi-Objective STOMP Parameter Optimization System
=========================================================

This is the main orchestrator that integrates all components:
1. Scenario feature extraction
2. ML-based parameter prediction
3. Multi-objective optimization with NSGA-II
4. Adaptive parameter selection
5. Performance evaluation and feedback

The system learns from optimization history and provides intelligent
parameter selection for new scenarios.

Authors: Smart Parameter Optimization System  
Date: July 2025
"""

import argparse
import logging
import json
import yaml
import pandas as pd
import numpy as np
from pathlib import Path
from typing import Dict, List, Any
import matplotlib.pyplot as plt
import seaborn as sns
from datetime import datetime

# Import our components
from scenario_analyzer import ScenarioAnalyzer, ScenarioFeatures
from adaptive_parameter_selector import AdaptiveParameterSelector
from multi_objective_optimizer import MultiObjectiveStompOptimizer

class SmartParameterOptimizer:
    """Main orchestrator for smart multi-objective parameter optimization"""
    
    def __init__(self, config_file: str):
        """Initialize the smart optimizer with configuration"""
        with open(config_file, 'r') as f:
            self.config = yaml.safe_load(f)
        
        self.output_dir = Path(self.config['output_dir'])
        self.output_dir.mkdir(parents=True, exist_ok=True)
        
        # Initialize components
        self.scenario_analyzer = ScenarioAnalyzer()
        self.adaptive_selector = AdaptiveParameterSelector(
            model_dir=self.config.get('model_dir', 'models')
        )
        
        # Results storage
        self.optimization_history = []
        
        logging.info(f"Initialized SmartParameterOptimizer with config: {config_file}")
    
    def optimize_scenario(self, scenario_config: Dict[str, Any]) -> Dict[str, Any]:
        """
        Optimize parameters for a single scenario using the complete pipeline:
        1. Extract scenario features
        2. Predict initial parameters with ML
        3. Run multi-objective optimization
        4. Update ML models with results
        """
        
        logging.info(f"Starting optimization for scenario: {scenario_config.get('name', 'unnamed')}")
        
        # Step 1: Extract scenario features
        logging.info("Step 1: Extracting scenario features...")
        features = self.scenario_analyzer.analyze_scenario(
            scenario_config['urdf_file'],
            scenario_config['environment_file'],
            scenario_config['poses_file']
        )
        
        # Step 2: Get ML-based parameter predictions
        logging.info("Step 2: Predicting parameters with ML...")
        ml_predictions = self.adaptive_selector.generate_parameter_candidates(
            features, n_candidates=self.config.get('ml_candidates', 3)
        )
        
        # Step 3: Run multi-objective optimization
        logging.info("Step 3: Running multi-objective optimization...")
        
        # Create scenarios list for multi-objective optimizer
        scenarios = [scenario_config]
        
        # Initialize multi-objective optimizer
        cpp_executable = self.config['cpp_executable']
        mo_optimizer = MultiObjectiveStompOptimizer(cpp_executable, scenarios)
        
        # Warm-start with ML predictions
        initial_population = self._convert_predictions_to_population(ml_predictions)
        
        # Run optimization
        mo_results = mo_optimizer.optimize(
            n_generations=self.config.get('generations', 30),
            population_size=self.config.get('population_size', 20)
        )
        
        # Step 4: Analyze results and select best solutions
        logging.info("Step 4: Analyzing optimization results...")
        analysis_results = self._analyze_optimization_results(
            features, ml_predictions, mo_results
        )
        
        # Step 5: Update ML models with performance feedback
        logging.info("Step 5: Updating ML models...")
        self._update_models_with_feedback(features, mo_results)
        
        # Compile final results
        final_results = {
            'scenario': scenario_config,
            'scenario_features': features.__dict__,
            'ml_predictions': [pred.to_dict() for pred in ml_predictions],
            'multi_objective_results': mo_results,
            'analysis': analysis_results,
            'timestamp': datetime.now().isoformat()
        }
        
        # Save results
        self._save_scenario_results(final_results)
        
        return final_results
    
    def optimize_multiple_scenarios(self, scenarios_config: List[Dict[str, Any]]) -> Dict[str, Any]:
        """Optimize parameters across multiple scenarios"""
        
        logging.info(f"Starting optimization for {len(scenarios_config)} scenarios")
        
        all_results = []
        scenario_comparisons = []
        
        for i, scenario_config in enumerate(scenarios_config):
            logging.info(f"Processing scenario {i+1}/{len(scenarios_config)}: {scenario_config.get('name', f'scenario_{i}')}")
            
            try:
                scenario_results = self.optimize_scenario(scenario_config)
                all_results.append(scenario_results)
                
                # Extract comparison metrics
                comparison = self._extract_comparison_metrics(scenario_results)
                scenario_comparisons.append(comparison)
                
            except Exception as e:
                logging.error(f"Failed to optimize scenario {i}: {e}")
                continue
        
        # Cross-scenario analysis
        cross_analysis = self._perform_cross_scenario_analysis(all_results)
        
        # Generate comprehensive report
        final_report = {
            'summary': {
                'total_scenarios': len(scenarios_config),
                'successful_optimizations': len(all_results),
                'optimization_timestamp': datetime.now().isoformat()
            },
            'individual_results': all_results,
            'scenario_comparisons': scenario_comparisons,
            'cross_scenario_analysis': cross_analysis,
            'recommendations': self._generate_recommendations(cross_analysis)
        }
        
        # Save comprehensive report
        self._save_comprehensive_report(final_report)
        
        # Generate visualizations
        self._generate_visualizations(final_report)
        
        return final_report
    
    def _convert_predictions_to_population(self, predictions: List) -> List[np.ndarray]:
        """Convert ML predictions to initial population for NSGA-II"""
        population = []
        
        for pred in predictions:
            # Convert parameters to optimization vector
            param_vector = np.array([
                pred.num_noisy_trajectories,
                pred.num_best_samples,
                pred.max_iterations,
                pred.learning_rate,
                pred.temperature,
                pred.dt,
                pred.velocity_limit,
                pred.acceleration_limit
            ])
            population.append(param_vector)
        
        return population
    
    def _analyze_optimization_results(self, features: ScenarioFeatures, 
                                    ml_predictions: List, mo_results: Dict) -> Dict[str, Any]:
        """Analyze optimization results and extract insights"""
        
        analysis = {
            'scenario_complexity_score': self._compute_scenario_complexity(features),
            'ml_prediction_quality': self._evaluate_ml_predictions(ml_predictions, mo_results),
            'pareto_front_analysis': self._analyze_pareto_front(mo_results),
            'convergence_analysis': self._analyze_convergence(mo_results),
            'parameter_sensitivity': self._analyze_parameter_sensitivity(mo_results)
        }
        
        return analysis
    
    def _compute_scenario_complexity(self, features: ScenarioFeatures) -> float:
        """Compute overall scenario complexity score"""
        # Weighted combination of key complexity indicators
        complexity = (
            0.3 * features.obstacle_density +
            0.2 * (1.0 - features.min_corridor_width) +
            0.2 * features.orientation_complexity +
            0.15 * features.configuration_space_obstacles +
            0.15 * features.manipulability_variation
        )
        
        return min(complexity, 1.0)
    
    def _evaluate_ml_predictions(self, ml_predictions: List, mo_results: Dict) -> Dict[str, float]:
        """Evaluate quality of ML predictions compared to optimization results"""
        
        if not mo_results.get('pareto_front'):
            return {'accuracy': 0.0, 'coverage': 0.0}
        
        # Find best compromise from Pareto front
        best_compromise = mo_results.get('best_compromise', {})
        
        if not best_compromise:
            return {'accuracy': 0.0, 'coverage': 0.0}
        
        # Compare ML prediction to best optimization result
        best_ml = ml_predictions[0] if ml_predictions else None
        
        if not best_ml:
            return {'accuracy': 0.0, 'coverage': 0.0}
        
        # Compute parameter similarity
        ml_params = np.array([
            best_ml.num_noisy_trajectories,
            best_ml.num_best_samples,
            best_ml.max_iterations,
            best_ml.learning_rate,
            best_ml.temperature,
            best_ml.dt,
            best_ml.velocity_limit,
            best_ml.acceleration_limit
        ])
        
        opt_params = np.array([
            best_compromise['parameters']['num_noisy_trajectories'],
            best_compromise['parameters']['num_best_samples'],
            best_compromise['parameters']['max_iterations'],
            best_compromise['parameters']['learning_rate'],
            best_compromise['parameters']['temperature'],
            best_compromise['parameters']['dt'],
            best_compromise['parameters']['velocity_limit'],
            best_compromise['parameters']['acceleration_limit']
        ])
        
        # Normalize parameters and compute similarity
        param_ranges = np.array([90, 45, 450, 0.9, 49, 0.19, 2.5, 0.9])  # Approximate ranges
        normalized_ml = ml_params / param_ranges
        normalized_opt = opt_params / param_ranges
        
        similarity = 1.0 - np.mean(np.abs(normalized_ml - normalized_opt))
        
        return {
            'accuracy': max(0.0, similarity),
            'coverage': 0.8,  # Placeholder - would measure coverage of Pareto front
            'confidence': np.mean(list(best_ml.confidence_scores.values()))
        }
    
    def _analyze_pareto_front(self, mo_results: Dict) -> Dict[str, Any]:
        """Analyze the Pareto front characteristics"""
        
        pareto_front = mo_results.get('pareto_front', [])
        
        if not pareto_front:
            return {'size': 0, 'diversity': 0.0, 'quality': 0.0}
        
        # Extract objective values
        objectives = []
        for solution in pareto_front:
            obj = solution['objectives']
            objectives.append([
                obj['planning_time'],
                obj['trajectory_quality'],
                obj['energy_consumption'],
                obj['collision_margin'],
                obj['success_rate']
            ])
        
        objectives = np.array(objectives)
        
        # Compute diversity (spread of solutions)
        diversity = np.std(objectives, axis=0).mean()
        
        # Compute quality (average objective values)
        quality_score = (
            1.0 / (1.0 + objectives[:, 0].mean()) +  # Lower planning time is better
            objectives[:, 1].mean() +                # Higher trajectory quality is better
            1.0 / (1.0 + objectives[:, 2].mean()) +  # Lower energy is better
            objectives[:, 3].mean() +                # Higher collision margin is better
            objectives[:, 4].mean()                  # Higher success rate is better
        ) / 5.0
        
        return {
            'size': len(pareto_front),
            'diversity': float(diversity),
            'quality': float(quality_score),
            'hypervolume': self._compute_hypervolume(objectives)
        }
    
    def _compute_hypervolume(self, objectives: np.ndarray) -> float:
        """Compute hypervolume of Pareto front (simplified)"""
        if len(objectives) == 0:
            return 0.0
        
        # Simplified hypervolume calculation
        # In practice, would use proper hypervolume algorithm
        normalized_objectives = objectives / (np.max(objectives, axis=0) + 1e-6)
        volume = np.prod(np.max(normalized_objectives, axis=0))
        
        return float(volume)
    
    def _analyze_convergence(self, mo_results: Dict) -> Dict[str, Any]:
        """Analyze optimization convergence"""
        
        history = mo_results.get('optimization_history', {})
        
        return {
            'generations': history.get('generations', 0),
            'evaluations': history.get('evaluations', 0),
            'convergence_rate': 'fast',  # Placeholder analysis
            'final_hypervolume': history.get('final_hypervolume', 0.0)
        }
    
    def _analyze_parameter_sensitivity(self, mo_results: Dict) -> Dict[str, float]:
        """Analyze parameter sensitivity across Pareto front"""
        
        pareto_front = mo_results.get('pareto_front', [])
        
        if len(pareto_front) < 2:
            return {}
        
        # Extract parameter values
        param_names = ['num_noisy_trajectories', 'num_best_samples', 'max_iterations',
                      'learning_rate', 'temperature', 'dt', 'velocity_limit', 'acceleration_limit']
        
        param_values = {name: [] for name in param_names}
        
        for solution in pareto_front:
            params = solution['parameters']
            for name in param_names:
                param_values[name].append(params[name])
        
        # Compute coefficient of variation for each parameter
        sensitivity = {}
        for name, values in param_values.items():
            if len(values) > 1:
                std_dev = np.std(values)
                mean_val = np.mean(values)
                cv = std_dev / (mean_val + 1e-6)
                sensitivity[name] = float(cv)
        
        return sensitivity
    
    def _update_models_with_feedback(self, features: ScenarioFeatures, mo_results: Dict):
        """Update ML models with optimization results"""
        
        best_compromise = mo_results.get('best_compromise')
        if not best_compromise:
            return
        
        # Update adaptive selector with performance feedback
        parameters = best_compromise['parameters']
        performance = best_compromise['objectives']
        
        self.adaptive_selector.update_performance(features, parameters, performance)
    
    def _extract_comparison_metrics(self, scenario_results: Dict) -> Dict[str, Any]:
        """Extract metrics for cross-scenario comparison"""
        
        analysis = scenario_results.get('analysis', {})
        
        return {
            'scenario_name': scenario_results['scenario'].get('name', 'unnamed'),
            'complexity_score': analysis.get('scenario_complexity_score', 0.0),
            'pareto_front_size': analysis.get('pareto_front_analysis', {}).get('size', 0),
            'optimization_quality': analysis.get('pareto_front_analysis', {}).get('quality', 0.0),
            'ml_prediction_accuracy': analysis.get('ml_prediction_quality', {}).get('accuracy', 0.0),
            'convergence_generations': analysis.get('convergence_analysis', {}).get('generations', 0)
        }
    
    def _perform_cross_scenario_analysis(self, all_results: List[Dict]) -> Dict[str, Any]:
        """Perform analysis across all scenarios"""
        
        if not all_results:
            return {}
        
        comparisons = [self._extract_comparison_metrics(result) for result in all_results]
        df = pd.DataFrame(comparisons)
        
        analysis = {
            'scenario_complexity_distribution': {
                'mean': float(df['complexity_score'].mean()),
                'std': float(df['complexity_score'].std()),
                'min': float(df['complexity_score'].min()),
                'max': float(df['complexity_score'].max())
            },
            'optimization_performance': {
                'avg_pareto_size': float(df['pareto_front_size'].mean()),
                'avg_quality': float(df['optimization_quality'].mean()),
                'avg_ml_accuracy': float(df['ml_prediction_accuracy'].mean())
            },
            'parameter_trends': self._analyze_parameter_trends(all_results),
            'scenario_clustering': self._cluster_scenarios(all_results)
        }
        
        return analysis
    
    def _analyze_parameter_trends(self, all_results: List[Dict]) -> Dict[str, Any]:
        """Analyze parameter trends across scenarios"""
        
        # Extract best parameters from each scenario
        best_params = []
        complexities = []
        
        for result in all_results:
            mo_results = result.get('multi_objective_results', {})
            best_compromise = mo_results.get('best_compromise')
            
            if best_compromise:
                best_params.append(best_compromise['parameters'])
                
                analysis = result.get('analysis', {})
                complexities.append(analysis.get('scenario_complexity_score', 0.0))
        
        if not best_params:
            return {}
        
        # Analyze correlations between complexity and parameters
        df_params = pd.DataFrame(best_params)
        df_params['complexity'] = complexities
        
        correlations = {}
        for param in df_params.columns:
            if param != 'complexity':
                corr = df_params[param].corr(df_params['complexity'])
                correlations[param] = float(corr) if not np.isnan(corr) else 0.0
        
        return {
            'complexity_correlations': correlations,
            'parameter_ranges': {
                param: {
                    'min': float(df_params[param].min()),
                    'max': float(df_params[param].max()),
                    'mean': float(df_params[param].mean())
                }
                for param in df_params.columns if param != 'complexity'
            }
        }
    
    def _cluster_scenarios(self, all_results: List[Dict]) -> Dict[str, Any]:
        """Cluster scenarios by complexity and characteristics"""
        
        # Extract scenario features
        features_list = []
        scenario_names = []
        
        for result in all_results:
            features = result.get('scenario_features', {})
            if features:
                # Convert to feature vector
                feature_vector = [
                    features.get('obstacle_density', 0),
                    features.get('workspace_coverage', 0),
                    features.get('orientation_complexity', 0),
                    features.get('path_length', 0),
                    features.get('manipulability_variation', 0)
                ]
                features_list.append(feature_vector)
                scenario_names.append(result['scenario'].get('name', 'unnamed'))
        
        if len(features_list) < 2:
            return {'clusters': [], 'cluster_centers': []}
        
        # Simple clustering (in practice would use proper clustering)
        features_array = np.array(features_list)
        n_clusters = min(3, len(features_list))
        
        try:
            from sklearn.cluster import KMeans
            kmeans = KMeans(n_clusters=n_clusters, random_state=42)
            cluster_labels = kmeans.fit_predict(features_array)
            
            clusters = []
            for i in range(n_clusters):
                cluster_scenarios = [scenario_names[j] for j in range(len(scenario_names)) 
                                   if cluster_labels[j] == i]
                clusters.append({
                    'cluster_id': i,
                    'scenarios': cluster_scenarios,
                    'size': len(cluster_scenarios)
                })
            
            return {
                'clusters': clusters,
                'cluster_centers': kmeans.cluster_centers_.tolist()
            }
            
        except ImportError:
            return {'clusters': [], 'cluster_centers': []}
    
    def _generate_recommendations(self, cross_analysis: Dict) -> List[str]:
        """Generate optimization recommendations based on analysis"""
        
        recommendations = []
        
        # Analyze ML prediction accuracy
        ml_accuracy = cross_analysis.get('optimization_performance', {}).get('avg_ml_accuracy', 0.0)
        if ml_accuracy < 0.7:
            recommendations.append(
                "ML prediction accuracy is low. Consider collecting more training data "
                "or retraining models with different algorithms."
            )
        
        # Analyze parameter trends
        trends = cross_analysis.get('parameter_trends', {})
        correlations = trends.get('complexity_correlations', {})
        
        strong_correlations = {k: v for k, v in correlations.items() if abs(v) > 0.7}
        if strong_correlations:
            rec = "Strong correlations found between scenario complexity and parameters: "
            rec += ", ".join([f"{k} ({v:.2f})" for k, v in strong_correlations.items()])
            rec += ". Consider complexity-adaptive parameter selection."
            recommendations.append(rec)
        
        # Analyze optimization quality
        avg_quality = cross_analysis.get('optimization_performance', {}).get('avg_quality', 0.0)
        if avg_quality < 0.5:
            recommendations.append(
                "Optimization quality is suboptimal. Consider increasing population size "
                "or number of generations for better convergence."
            )
        
        # Default recommendation
        if not recommendations:
            recommendations.append(
                "Optimization performance is satisfactory. Continue with current configuration."
            )
        
        return recommendations
    
    def _save_scenario_results(self, results: Dict[str, Any]):
        """Save results for individual scenario"""
        scenario_name = results['scenario'].get('name', 'unnamed')
        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        
        filename = f"scenario_{scenario_name}_{timestamp}.json"
        filepath = self.output_dir / filename
        
        with open(filepath, 'w') as f:
            json.dump(results, f, indent=2, default=str)
        
        logging.info(f"Scenario results saved to {filepath}")
    
    def _save_comprehensive_report(self, report: Dict[str, Any]):
        """Save comprehensive optimization report"""
        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        filename = f"optimization_report_{timestamp}.json"
        filepath = self.output_dir / filename
        
        with open(filepath, 'w') as f:
            json.dump(report, f, indent=2, default=str)
        
        logging.info(f"Comprehensive report saved to {filepath}")
    
    def _generate_visualizations(self, report: Dict[str, Any]):
        """Generate visualization plots"""
        
        try:
            # Extract data for plotting
            comparisons = report.get('scenario_comparisons', [])
            if not comparisons:
                return
            
            df = pd.DataFrame(comparisons)
            
            # Create figure with subplots
            fig, axes = plt.subplots(2, 2, figsize=(15, 12))
            
            # Plot 1: Complexity vs Quality
            axes[0, 0].scatter(df['complexity_score'], df['optimization_quality'])
            axes[0, 0].set_xlabel('Scenario Complexity')
            axes[0, 0].set_ylabel('Optimization Quality')
            axes[0, 0].set_title('Complexity vs Optimization Quality')
            
            # Plot 2: ML Accuracy Distribution
            axes[0, 1].hist(df['ml_prediction_accuracy'], bins=10, alpha=0.7)
            axes[0, 1].set_xlabel('ML Prediction Accuracy')
            axes[0, 1].set_ylabel('Frequency')
            axes[0, 1].set_title('ML Prediction Accuracy Distribution')
            
            # Plot 3: Pareto Front Size vs Complexity
            axes[1, 0].scatter(df['complexity_score'], df['pareto_front_size'])
            axes[1, 0].set_xlabel('Scenario Complexity')
            axes[1, 0].set_ylabel('Pareto Front Size')
            axes[1, 0].set_title('Complexity vs Pareto Front Size')
            
            # Plot 4: Convergence Analysis
            axes[1, 1].bar(range(len(df)), df['convergence_generations'])
            axes[1, 1].set_xlabel('Scenario Index')
            axes[1, 1].set_ylabel('Generations to Converge')
            axes[1, 1].set_title('Convergence Speed by Scenario')
            
            plt.tight_layout()
            
            timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
            plot_filename = f"optimization_analysis_{timestamp}.png"
            plot_path = self.output_dir / plot_filename
            
            plt.savefig(plot_path, dpi=300, bbox_inches='tight')
            plt.close()
            
            logging.info(f"Visualizations saved to {plot_path}")
            
        except Exception as e:
            logging.warning(f"Failed to generate visualizations: {e}")

def main():
    parser = argparse.ArgumentParser(description='Smart Multi-Objective STOMP Parameter Optimization')
    parser.add_argument('--config', required=True, help='Configuration YAML file')
    parser.add_argument('--scenarios', required=True, help='Scenarios configuration YAML file')
    parser.add_argument('--verbose', action='store_true', help='Verbose logging')
    
    args = parser.parse_args()
    
    # Setup logging
    level = logging.INFO if args.verbose else logging.WARNING
    logging.basicConfig(level=level, format='%(asctime)s - %(levelname)s - %(message)s')
    
    try:
        # Load scenarios configuration
        with open(args.scenarios, 'r') as f:
            scenarios_config = yaml.safe_load(f)
        
        # Initialize smart optimizer
        optimizer = SmartParameterOptimizer(args.config)
        
        # Run optimization
        if isinstance(scenarios_config, list):
            # Multiple scenarios
            results = optimizer.optimize_multiple_scenarios(scenarios_config)
        else:
            # Single scenario
            results = optimizer.optimize_scenario(scenarios_config)
        
        print("Smart parameter optimization completed successfully!")
        print(f"Results saved to: {optimizer.output_dir}")
        
        # Print summary
        if 'summary' in results:
            summary = results['summary']
            print(f"\nSummary:")
            print(f"- Total scenarios: {summary['total_scenarios']}")
            print(f"- Successful optimizations: {summary['successful_optimizations']}")
            
            # Print recommendations
            recommendations = results.get('recommendations', [])
            if recommendations:
                print(f"\nRecommendations:")
                for i, rec in enumerate(recommendations, 1):
                    print(f"{i}. {rec}")
        
    except Exception as e:
        logging.error(f"Smart parameter optimization failed: {e}")
        return 1
    
    return 0

if __name__ == '__main__':
    exit(main())
