#!/usr/bin/env python3
"""
Trajectory Planning Parameter Optimization with Hauser Method using RRT Variants

This system optimizes parameters for the Hauser method where different RRT variants
(RRT, RRT*, iRRT*, BiRRT) are used as the underlying planning algorithm within Hauser.
"""

import optuna
import subprocess
import json
import pandas as pd
import numpy as np
from pathlib import Path
import time

class HauserRRTOptimizer:
    """
    Parameter optimizer for Hauser method with RRT variant selection
    """
    
    def __init__(self, study_name="hauser_rrt_optimization"):
        self.study_name = study_name
        self.storage_url = "sqlite:///hauser_rrt_studies.db"
        
    def define_parameter_space(self, trial):
        """
        Define the parameter search space for Hauser method with RRT variants
        
        Args:
            trial: Optuna trial object
            
        Returns:
            dict: Parameter configuration
        """
        params = {
            # Core Hauser method parameters
            'hauser_samples': trial.suggest_int('hauser_samples', 100, 3000),
            'hauser_neighbor_radius': trial.suggest_float('hauser_neighbor_radius', 0.1, 2.5),
            'hauser_max_iterations': trial.suggest_int('hauser_max_iterations', 50, 1500),
            'hauser_collision_check_resolution': trial.suggest_float('hauser_collision_check_resolution', 0.01, 0.1),
            
            # RRT variant selection and parameters
            'rrt_variant': trial.suggest_categorical('rrt_variant', ['RRT', 'RRT_STAR', 'iRRT_STAR', 'BiRRT']),
            'rrt_max_iterations': trial.suggest_int('rrt_max_iterations', 500, 8000),
            'rrt_step_size': trial.suggest_float('rrt_step_size', 0.01, 0.3),
            'rrt_goal_bias': trial.suggest_float('rrt_goal_bias', 0.05, 0.25),
            
            # RRT* specific parameters (used when variant is RRT* or iRRT*)
            'rrt_star_radius': trial.suggest_float('rrt_star_radius', 0.1, 1.5),
            'rrt_star_rewire_factor': trial.suggest_float('rrt_star_rewire_factor', 1.0, 2.0),
            
            # iRRT* specific parameters
            'irrt_star_informed_sampling': trial.suggest_categorical('irrt_star_informed_sampling', [True, False]),
            'irrt_star_pruning_radius': trial.suggest_float('irrt_star_pruning_radius', 0.5, 3.0),
            
            # BiRRT specific parameters
            'birrt_connection_radius': trial.suggest_float('birrt_connection_radius', 0.1, 1.0),
            'birrt_swap_probability': trial.suggest_float('birrt_swap_probability', 0.1, 0.5),
            
            # Hauser-specific RRT integration parameters
            'hauser_rrt_integration_mode': trial.suggest_categorical('hauser_rrt_integration_mode', 
                                                                   ['sequential', 'parallel', 'hybrid']),
            'hauser_path_smoothing': trial.suggest_categorical('hauser_path_smoothing', [True, False]),
            'hauser_dynamic_resampling': trial.suggest_categorical('hauser_dynamic_resampling', [True, False]),
        }
        
        return params
    
    def generate_config_file(self, params, trial_number):
        """
        Generate configuration file for the Hauser-RRT executable
        
        Args:
            params: Parameter dictionary
            trial_number: Trial identifier
            
        Returns:
            str: Path to generated config file
        """
        config_path = f'config_hauser_rrt_trial_{trial_number}.json'
        
        config = {
            "algorithm": "HAUSER_RRT",
            "trial_id": trial_number,
            "hauser_method": {
                "samples": params['hauser_samples'],
                "neighbor_radius": params['hauser_neighbor_radius'],
                "max_iterations": params['hauser_max_iterations'],
                "collision_check_resolution": params['hauser_collision_check_resolution'],
                "path_smoothing": params['hauser_path_smoothing'],
                "dynamic_resampling": params['hauser_dynamic_resampling'],
                "rrt_integration_mode": params['hauser_rrt_integration_mode']
            },
            "rrt_configuration": {
                "variant": params['rrt_variant'],
                "max_iterations": params['rrt_max_iterations'],
                "step_size": params['rrt_step_size'],
                "goal_bias": params['rrt_goal_bias']
            },
            "rrt_star_parameters": {
                "radius": params['rrt_star_radius'],
                "rewire_factor": params['rrt_star_rewire_factor']
            },
            "irrt_star_parameters": {
                "informed_sampling": params['irrt_star_informed_sampling'],
                "pruning_radius": params['irrt_star_pruning_radius']
            },
            "birrt_parameters": {
                "connection_radius": params['birrt_connection_radius'],
                "swap_probability": params['birrt_swap_probability']
            },
            "evaluation": {
                "num_test_poses": 10,
                "timeout_seconds": 30,
                "success_threshold": 0.001
            }
        }
        
        with open(config_path, 'w') as f:
            json.dump(config, f, indent=2)
        
        return config_path
    
    def objective_function(self, trial):
        """
        Objective function for Hauser-RRT optimization
        
        Args:
            trial: Optuna trial object
            
        Returns:
            float: Objective value (lower is better)
        """
        # Define parameter space
        params = self.define_parameter_space(trial)
        
        # Generate config file
        config_path = self.generate_config_file(params, trial.number)
        
        try:
            # For demonstration, simulate the Hauser-RRT execution
            objective_value = self.simulate_hauser_rrt_performance(params)
            
            # Log results
            print(f"Trial {trial.number}: {params['rrt_variant']} in Hauser, "
                  f"samples={params['hauser_samples']}, objective={objective_value:.3f}")
            
            return objective_value
            
        except Exception as e:
            print(f"Trial {trial.number} failed: {e}")
            return 1000.0  # High penalty for failure
            
        finally:
            # Clean up config file
            Path(config_path).unlink(missing_ok=True)
    
    def simulate_hauser_rrt_performance(self, params):
        """
        Simulate Hauser method performance with different RRT variants
        
        Args:
            params: Parameter configuration
            
        Returns:
            float: Simulated objective value
        """
        # Base performance for Hauser method
        base_performance = 8.0
        
        # RRT variant impact on Hauser performance
        rrt_variant_factors = {
            'RRT': 1.0,           # Baseline
            'RRT_STAR': 0.75,     # Better path quality
            'iRRT_STAR': 0.65,    # Best path quality with informed sampling
            'BiRRT': 0.85         # Faster convergence but slightly lower quality
        }
        
        variant_factor = rrt_variant_factors[params['rrt_variant']]
        
        # Hauser-specific performance factors
        sample_factor = max(0.4, 1.0 - (params['hauser_samples'] - 1000) / 2000 * 0.5)
        radius_factor = 1.0 + abs(params['hauser_neighbor_radius'] - 1.0) / 1.0 * 0.3
        iteration_factor = max(0.6, 1.0 - (params['hauser_max_iterations'] - 500) / 1000 * 0.25)
        
        # RRT parameters impact within Hauser
        rrt_iter_factor = max(0.7, 1.0 - (params['rrt_max_iterations'] - 2000) / 6000 * 0.2)
        step_size_factor = 1.0 + abs(params['rrt_step_size'] - 0.1) / 0.1 * 0.15
        
        # Variant-specific optimizations
        if params['rrt_variant'] == 'RRT_STAR':
            radius_opt = 1.0 + abs(params['rrt_star_radius'] - 0.5) / 0.5 * 0.2
            variant_factor *= radius_opt
            
        elif params['rrt_variant'] == 'iRRT_STAR':
            if params['irrt_star_informed_sampling']:
                variant_factor *= 0.9  # Informed sampling improvement
            pruning_opt = 1.0 + abs(params['irrt_star_pruning_radius'] - 1.5) / 1.5 * 0.15
            variant_factor *= pruning_opt
            
        elif params['rrt_variant'] == 'BiRRT':
            connection_opt = 1.0 + abs(params['birrt_connection_radius'] - 0.3) / 0.3 * 0.1
            variant_factor *= connection_opt
        
        # Integration mode impact
        integration_factors = {
            'sequential': 1.0,
            'parallel': 0.85,    # Better performance with parallel execution
            'hybrid': 0.90       # Good balance
        }
        integration_factor = integration_factors[params['hauser_rrt_integration_mode']]
        
        # Additional optimizations
        smoothing_factor = 0.95 if params['hauser_path_smoothing'] else 1.0
        resampling_factor = 0.92 if params['hauser_dynamic_resampling'] else 1.0
        
        # Compute final performance
        performance = (base_performance * variant_factor * sample_factor * 
                      radius_factor * iteration_factor * rrt_iter_factor * 
                      step_size_factor * integration_factor * smoothing_factor * 
                      resampling_factor)
        
        # Add realistic noise
        noise = np.random.normal(0, performance * 0.08)
        performance += noise
        
        return max(1.0, performance)
    
    def run_optimization(self, n_trials=200):
        """
        Run the Hauser-RRT parameter optimization
        
        Args:
            n_trials: Number of optimization trials
            
        Returns:
            optuna.Study: Completed optimization study
        """
        print(f"Starting Hauser-RRT parameter optimization with {n_trials} trials...")
        
        # Create storage
        storage = optuna.storages.RDBStorage(self.storage_url)
        
        # Delete existing study if it exists
        try:
            optuna.delete_study(study_name=self.study_name, storage=storage)
            print('Deleted existing study')
        except:
            pass
        
        # Create new study
        study = optuna.create_study(
            study_name=self.study_name,
            direction='minimize',
            storage=storage,
            load_if_exists=False
        )
        
        # Run optimization
        study.optimize(self.objective_function, n_trials=n_trials, show_progress_bar=True)
        
        return study
    
    def analyze_results(self, study):
        """
        Analyze optimization results
        
        Args:
            study: Completed Optuna study
        """
        print(f"\n{'='*60}")
        print("HAUSER-RRT OPTIMIZATION RESULTS")
        print(f"{'='*60}")
        
        print(f"Best trial: {study.best_trial.number}")
        print(f"Best objective value: {study.best_value:.4f}")
        
        print(f"\nOptimal Configuration:")
        print(f"{'─'*40}")
        best_params = study.best_params
        
        print(f"RRT Variant: {best_params['rrt_variant']}")
        print(f"Hauser Samples: {best_params['hauser_samples']}")
        print(f"Hauser Neighbor Radius: {best_params['hauser_neighbor_radius']:.4f}")
        print(f"Hauser Max Iterations: {best_params['hauser_max_iterations']}")
        print(f"RRT Max Iterations: {best_params['rrt_max_iterations']}")
        print(f"RRT Step Size: {best_params['rrt_step_size']:.4f}")
        print(f"Integration Mode: {best_params['hauser_rrt_integration_mode']}")
        
        # Analyze RRT variant performance
        df = study.trials_dataframe()
        
        print(f"\nRRT Variant Performance Analysis:")
        print(f"{'─'*40}")
        variant_stats = df.groupby('params_rrt_variant')['value'].agg(['mean', 'std', 'count'])
        for variant, stats in variant_stats.iterrows():
            print(f"{variant:12s}: μ={stats['mean']:6.2f} ± {stats['std']:5.2f} (n={int(stats['count']):3d})")
        
        # Parameter importance
        importance = optuna.importance.get_param_importances(study)
        print(f"\nParameter Importance (Top 10):")
        print(f"{'─'*40}")
        for i, (param, imp) in enumerate(sorted(importance.items(), key=lambda x: x[1], reverse=True)[:10]):
            print(f"{i+1:2d}. {param:25s}: {imp:.3f}")

def main():
    """
    Main function to run Hauser-RRT optimization
    """
    print("HAUSER METHOD WITH RRT VARIANTS OPTIMIZATION")
    print("="*60)
    print("Optimizing parameters for Hauser method using different RRT variants:")
    print("- RRT (basic)")
    print("- RRT* (optimal)")
    print("- iRRT* (informed RRT*)")
    print("- BiRRT (bidirectional)")
    print()
    
    # Initialize optimizer
    optimizer = HauserRRTOptimizer()
    
    # Run optimization
    study = optimizer.run_optimization(n_trials=200)
    
    # Analyze results
    optimizer.analyze_results(study)
    
    # Generate detailed configuration
    print(f"\n{'='*60}")
    print("RECOMMENDED IMPLEMENTATION")
    print(f"{'='*60}")
    
    best_params = study.best_params
    
    config_template = f"""
# Optimal Hauser-RRT Configuration
# Objective Value: {study.best_value:.4f}

hauser_method:
  samples: {best_params['hauser_samples']}
  neighbor_radius: {best_params['hauser_neighbor_radius']:.4f}
  max_iterations: {best_params['hauser_max_iterations']}
  collision_check_resolution: {best_params['hauser_collision_check_resolution']:.4f}
  path_smoothing: {best_params['hauser_path_smoothing']}
  dynamic_resampling: {best_params['hauser_dynamic_resampling']}
  rrt_integration_mode: {best_params['hauser_rrt_integration_mode']}

rrt_configuration:
  variant: {best_params['rrt_variant']}
  max_iterations: {best_params['rrt_max_iterations']}
  step_size: {best_params['rrt_step_size']:.4f}
  goal_bias: {best_params['rrt_goal_bias']:.4f}"""
    
    if best_params['rrt_variant'] in ['RRT_STAR', 'iRRT_STAR']:
        config_template += f"""
  
rrt_star_parameters:
  radius: {best_params['rrt_star_radius']:.4f}
  rewire_factor: {best_params['rrt_star_rewire_factor']:.4f}"""
    
    if best_params['rrt_variant'] == 'iRRT_STAR':
        config_template += f"""
        
irrt_star_parameters:
  informed_sampling: {best_params['irrt_star_informed_sampling']}
  pruning_radius: {best_params['irrt_star_pruning_radius']:.4f}"""
    
    if best_params['rrt_variant'] == 'BiRRT':
        config_template += f"""
        
birrt_parameters:
  connection_radius: {best_params['birrt_connection_radius']:.4f}
  swap_probability: {best_params['birrt_swap_probability']:.4f}"""
    
    print(config_template)
    
    # Save configuration
    with open('optimal_hauser_rrt_config.yaml', 'w') as f:
        f.write(config_template)
    
    print(f"\n✓ Optimal configuration saved to: optimal_hauser_rrt_config.yaml")
    print(f"✓ Study database saved to: hauser_rrt_studies.db")

if __name__ == "__main__":
    main()
