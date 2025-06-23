#!/usr/bin/env python3
"""
Demonstration of Academic Analysis for Trajectory Planning Parameter Optimization

This script creates a simulated optimization study and generates a comprehensive
academic-quality analysis report including parameter importance, statistical 
significance testing, and publication-ready visualizations.
"""

import optuna
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import seaborn as sns
from pathlib import Path
import json
from academic_analysis import AcademicAnalyzer

def create_simulated_study():
    """
    Create a simulated Optuna study with realistic trajectory planning results
    """
    print("Creating simulated optimization study for academic analysis...")
    
    # Create storage
    storage = optuna.storages.RDBStorage('sqlite:///demo_optuna_studies.db')
    
    # Delete existing study if it exists
    try:
        optuna.delete_study(study_name='demo_trajectory_optimization', storage=storage)
        print('Deleted existing demo study')
    except:
        pass
    
    # Create new study
    study = optuna.create_study(
        study_name='demo_trajectory_optimization',
        direction='minimize',
        storage=storage,
        load_if_exists=False
    )
    
    # Set random seed for reproducible results
    np.random.seed(42)
    
    # Define realistic parameter relationships and performance characteristics
    def simulate_trajectory_performance(trial):
        """
        Simulate realistic trajectory planning performance based on parameters
        """
        algorithm = trial.suggest_categorical('algorithm', ['STOMP', 'HAUSER', 'RRT', 'RRT_STAR'])
        
        # Algorithm-specific parameter suggestions
        if algorithm == 'STOMP':
            iterations = trial.suggest_int('stomp_iterations', 50, 500)
            noise_std = trial.suggest_float('stomp_noise_std', 0.01, 0.5)
            timesteps = trial.suggest_int('stomp_timesteps', 10, 100)
            learning_rate = trial.suggest_float('stomp_learning_rate', 0.001, 0.1)
            
            # STOMP performance model: balance iterations vs noise
            base_performance = 15.0
            iteration_factor = max(0.5, 1.0 - (iterations - 200) / 300 * 0.3)
            noise_factor = 1.0 + (noise_std - 0.25) ** 2 * 2.0
            timestep_factor = 1.0 + abs(timesteps - 50) / 50 * 0.2
            lr_factor = 1.0 + abs(learning_rate - 0.05) / 0.05 * 0.15
            
            performance = base_performance * iteration_factor * noise_factor * timestep_factor * lr_factor
            
        elif algorithm == 'HAUSER':
            samples = trial.suggest_int('hauser_samples', 100, 2000)
            neighbor_radius = trial.suggest_float('hauser_neighbor_radius', 0.1, 2.0)
            max_iterations = trial.suggest_int('hauser_max_iterations', 50, 1000)
            
            # Hauser performance model: more samples generally better
            base_performance = 12.0
            sample_factor = max(0.6, 1.0 - (samples - 500) / 1500 * 0.4)
            radius_factor = 1.0 + abs(neighbor_radius - 0.8) / 0.8 * 0.3
            iter_factor = max(0.7, 1.0 - (max_iterations - 300) / 700 * 0.25)
            
            performance = base_performance * sample_factor * radius_factor * iter_factor
            
        elif algorithm == 'RRT':
            max_iterations = trial.suggest_int('rrt_max_iterations', 100, 5000)
            step_size = trial.suggest_float('rrt_step_size', 0.01, 0.5)
            goal_bias = trial.suggest_float('rrt_goal_bias', 0.05, 0.3)
            
            # RRT performance model: step size critical
            base_performance = 18.0
            iter_factor = max(0.5, 1.0 - (max_iterations - 1000) / 4000 * 0.3)
            step_factor = 1.0 + abs(step_size - 0.15) / 0.15 * 0.5
            bias_factor = 1.0 + abs(goal_bias - 0.15) / 0.15 * 0.2
            
            performance = base_performance * iter_factor * step_factor * bias_factor
            
        else:  # RRT_STAR
            max_iterations = trial.suggest_int('rrt_max_iterations', 100, 5000)
            step_size = trial.suggest_float('rrt_step_size', 0.01, 0.5)
            goal_bias = trial.suggest_float('rrt_goal_bias', 0.05, 0.3)
            radius = trial.suggest_float('rrt_star_radius', 0.1, 1.0)
            
            # RRT* performance model: better than RRT but slower
            base_performance = 10.0
            iter_factor = max(0.6, 1.0 - (max_iterations - 1500) / 3500 * 0.3)
            step_factor = 1.0 + abs(step_size - 0.12) / 0.12 * 0.4
            bias_factor = 1.0 + abs(goal_bias - 0.12) / 0.12 * 0.15
            radius_factor = 1.0 + abs(radius - 0.4) / 0.4 * 0.25
            
            performance = base_performance * iter_factor * step_factor * bias_factor * radius_factor
        
        # Add realistic noise and some cross-parameter interactions
        noise = np.random.normal(0, performance * 0.1)
        performance += noise
        
        # Ensure minimum positive performance
        return max(1.0, performance)
    
    # Run simulated optimization
    print("Running 150 simulated trials...")
    for i in range(150):
        trial = study.ask()
        objective_value = simulate_trajectory_performance(trial)
        study.tell(trial, objective_value)
        
        if (i + 1) % 25 == 0:
            print(f"Completed {i + 1} trials...")
    
    print(f"✓ Simulated study completed!")
    print(f"Best trial: {study.best_trial.number}")
    print(f"Best objective value: {study.best_value:.4f}")
    print("Best parameters:")
    for key, value in study.best_params.items():
        print(f"  {key}: {value}")
    
    return study

def generate_comprehensive_analysis():
    """
    Generate comprehensive academic analysis of the optimization results
    """
    print("\n" + "="*60)
    print("GENERATING COMPREHENSIVE ACADEMIC ANALYSIS")
    print("="*60)
    
    # Initialize analyzer
    analyzer = AcademicAnalyzer(
        study_name='demo_trajectory_optimization',
        storage_url='sqlite:///demo_optuna_studies.db'
    )
    
    # Generate complete analysis
    print("\n1. Loading optimization results...")
    # analyzer.load_study()  # This is done in __init__
    
    print("2. Analyzing parameter importance...")
    importance_results = analyzer.analyze_parameter_importance()
    
    print("3. Performing statistical analysis...")
    statistical_results = analyzer.analyze_performance_distribution()
    
    print("4. Computing parameter correlations...")
    correlation_results = analyzer.analyze_parameter_correlations()
    
    print("5. Analyzing convergence behavior...")
    convergence_results = analyzer.analyze_convergence()
    
    print("6. Generating publication-ready visualizations...")
    # Create individual plots
    analyzer.create_parameter_importance_plot(importance_results)
    analyzer.create_performance_distribution_plot(statistical_results)
    analyzer.create_convergence_plot(convergence_results)
    analyzer.create_parameter_interaction_plots()
    analyzer.create_optimization_trajectory_plot()
    
    print("7. Creating comprehensive academic report...")
    # For now, use the generate_full_report method
    report_content = analyzer.generate_full_report()
    
    # Save the report to file
    output_dir = Path('academic_analysis_output')
    output_dir.mkdir(exist_ok=True)
    report_path = output_dir / 'trajectory_optimization_analysis.md'
    
    with open(report_path, 'w') as f:
        f.write(report_content)
    
    print(f"\n✓ Academic analysis completed!")
    print(f"📊 Report saved to: {report_path}")
    print(f"📈 Visualizations saved to: academic_analysis_output/")
    
    return analyzer

def print_summary_statistics(analyzer):
    """
    Print key findings and statistics
    """
    print("\n" + "="*60)
    print("KEY FINDINGS AND RECOMMENDATIONS")
    print("="*60)
    
    # Load study data
    df = analyzer.study.trials_dataframe()
    
    # Algorithm performance comparison
    print("\n📈 Algorithm Performance Summary:")
    print("-" * 40)
    algo_stats = df.groupby('params_algorithm')['value'].agg(['mean', 'std', 'count'])
    for algo, stats in algo_stats.iterrows():
        print(f"{algo:10s}: μ={stats['mean']:6.2f} ± {stats['std']:5.2f} (n={int(stats['count']):3d})")
    
    # Best performing configuration
    best_trial = analyzer.study.best_trial
    print(f"\n🏆 Best Configuration (Objective: {best_trial.value:.3f}):")
    print("-" * 40)
    for param, value in best_trial.params.items():
        if isinstance(value, float):
            print(f"  {param:20s}: {value:.4f}")
        else:
            print(f"  {param:20s}: {value}")
    
    # Parameter importance
    importance = optuna.importance.get_param_importances(analyzer.study)
    print(f"\n🔍 Most Important Parameters:")
    print("-" * 40)
    for i, (param, imp) in enumerate(sorted(importance.items(), key=lambda x: x[1], reverse=True)[:5]):
        print(f"  {i+1}. {param:20s}: {imp:.3f}")
    
    print(f"\n📋 Analysis Summary:")
    print("-" * 40)
    print(f"  Total trials evaluated: {len(df)}")
    print(f"  Algorithms compared: {df['params_algorithm'].nunique()}")
    print(f"  Parameter space dimensions: {len([col for col in df.columns if col.startswith('params_')])}")
    print(f"  Performance improvement: {(df['value'].max() - df['value'].min()) / df['value'].max() * 100:.1f}%")

def main():
    """
    Main function to run complete academic analysis demonstration
    """
    print("TRAJECTORY PLANNING PARAMETER OPTIMIZATION")
    print("Academic Analysis Demonstration")
    print("="*60)
    
    # Create output directory
    Path('academic_analysis_output').mkdir(exist_ok=True)
    
    # Create simulated study
    study = create_simulated_study()
    
    # Generate comprehensive analysis
    analyzer = generate_comprehensive_analysis()
    
    # Print summary
    print_summary_statistics(analyzer)
    
    print(f"\n" + "="*60)
    print("ANALYSIS COMPLETE")
    print("="*60)
    print(f"All outputs saved to: academic_analysis_output/")
    print(f"📄 Main report: trajectory_optimization_analysis.md")
    print(f"📊 Figures: parameter_importance.png, convergence_analysis.png, etc.")
    print(f"📈 Interactive plots: correlation_heatmap.png, performance_distribution.png")

if __name__ == "__main__":
    main()
