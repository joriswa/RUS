#!/usr/bin/env python3
"""
Advanced Multi-Objective STOMP Parameter Optimization using Optuna
Implements research-grade Bayesian optimization with:
- Multi-objective optimization (Pareto front analysis)
- Advanced sampling strategies (NSGA-II)
- Logarithmic parameter scaling
- Safety constraints and early stopping
- Trade-off analysis between speed, success rate, and safety
"""

import json
import subprocess
import time
import logging
import numpy as np
from pathlib import Path
from typing import Dict, Any, Tuple, List
import optuna
from optuna.visualization import plot_optimization_history, plot_param_importances, plot_pareto_front
from optuna.samplers import NSGAIISampler
from optuna.pruners import MedianPruner
from optuna.trial import TrialState

# Setup logging
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(message)s')
logger = logging.getLogger(__name__)


class AdvancedSTOMPOptimizer:
    def __init__(self, scenario_config: Dict[str, str], n_startup_trials: int = 15):
        """
        Initialize advanced multi-objective STOMP parameter optimizer
        
        Args:
            scenario_config: Dictionary with csv_file, obstacles_file, urdf_file paths
            n_startup_trials: Number of random trials before Bayesian optimization
        """
        self.scenario_config = scenario_config
        self.n_startup_trials = n_startup_trials
        self.results_dir = Path("advanced_results")
        self.results_dir.mkdir(exist_ok=True)
        
        # Safety constraints for robotics applications
        self.safety_constraints = {
            'max_temperature': 50.0,  # Prevent excessive exploration
            'max_iterations': 400,    # Computational budget limit
            'max_N': 200,            # Trajectory resolution limit
            'min_success_rate': 0.1   # Minimum acceptable success rate
        }
        
        # Create multi-objective study with NSGA-II sampler
        storage_name = f"sqlite:///{Path.cwd()}/optuna_advanced_study.db"
        study_name = f"advanced_stomp_optimization_{int(time.time())}"
        
        # Advanced sampler configuration for multi-objective optimization
        sampler = NSGAIISampler(
            population_size=20,  # Larger population for better Pareto front
            mutation_prob=0.1,   # Mutation probability for genetic algorithm
            crossover_prob=0.9   # Crossover probability for genetic algorithm
        )
        
        # Pruner for early stopping of unpromising trials
        pruner = MedianPruner(
            n_startup_trials=5,
            n_warmup_steps=2,
            interval_steps=1
        )
        
        self.study = optuna.create_study(
            directions=["minimize", "minimize", "minimize", "minimize"],  # 4 objectives to minimize
            storage=storage_name,
            study_name=study_name,
            sampler=sampler,
            pruner=pruner
        )
        
        logger.info(f"Created advanced multi-objective study: {study_name}")
        logger.info(f"Storage: {storage_name}")
        logger.info("Objectives: [planning_time, failure_rate, constraint_violations, collision_risk]")

    def suggest_parameters(self, trial: optuna.Trial) -> Dict[str, Any]:
        """
        Suggest parameters with advanced strategies:
        - Logarithmic scaling for appropriate parameters
        - Safety constraints
        - Parameter relationships
        """
        
        # Core STOMP parameters with logarithmic scaling where appropriate
        params = {
            # Exploration/exploitation balance (log scale - multiplicative effect)
            'temperature': trial.suggest_float('temperature', 1.0, self.safety_constraints['max_temperature'], log=True),
            
            # Convergence rate (log scale - multiplicative effect) 
            'learning_rate': trial.suggest_float('learning_rate', 0.01, 1.0, log=True),
            
            # Computational budget (linear scale)
            'max_iterations': trial.suggest_int('max_iterations', 30, self.safety_constraints['max_iterations'], step=10),
            
            # Trajectory resolution (linear scale, but constrained)
            'N': trial.suggest_int('N', 30, min(self.safety_constraints['max_N'], 150), step=5),
            
            # Exploration samples (related to temperature)
            'num_noisy_trajectories': trial.suggest_int('num_noisy_trajectories', 8, 80, step=4),
            
            # Exploitation samples (should be < num_noisy_trajectories)
            'num_best_samples': trial.suggest_int('num_best_samples', 2, 25, step=1),
            
            # Cost weights (log scale - relative importance)
            'obstacle_cost_weight': trial.suggest_float('obstacle_cost_weight', 0.1, 10.0, log=True),
            'constraint_cost_weight': trial.suggest_float('constraint_cost_weight', 0.1, 10.0, log=True),
        }
        
        # Enforce parameter relationships and safety constraints
        params = self._enforce_parameter_constraints(params, trial)
        
        return params

    def _enforce_parameter_constraints(self, params: Dict[str, Any], trial: optuna.Trial) -> Dict[str, Any]:
        """Enforce safety constraints and parameter relationships"""
        
        # Ensure num_best_samples <= num_noisy_trajectories
        if params['num_best_samples'] > params['num_noisy_trajectories']:
            params['num_best_samples'] = max(2, params['num_noisy_trajectories'] // 4)
        
        # Scale exploration with temperature (higher temp = more exploration)
        temp_factor = params['temperature'] / 20.0  # Normalize around typical values
        min_trajectories = max(8, int(16 * temp_factor))
        if params['num_noisy_trajectories'] < min_trajectories:
            params['num_noisy_trajectories'] = min_trajectories
        
        # Computational safety: limit total work
        total_work = params['max_iterations'] * params['num_noisy_trajectories'] * params['N']
        max_work = 2000000  # Reasonable computational budget
        if total_work > max_work:
            # Scale down iterations to stay within budget
            params['max_iterations'] = min(params['max_iterations'], 
                                         max_work // (params['num_noisy_trajectories'] * params['N']))
            params['max_iterations'] = max(30, params['max_iterations'])  # Minimum iterations
        
        return params

    def create_config_file(self, trial: optuna.Trial) -> str:
        """Create YAML config file with advanced parameter suggestions"""
        
        params = self.suggest_parameters(trial)
        
        # Store parameters for analysis
        for key, value in params.items():
            trial.set_user_attr(f"param_{key}", value)
        
        # Use more trajectory pairs for better statistical significance
        trajectory_pairs = 12  # Increased for more robust evaluation
        
        # Create unique output filename
        timestamp = int(time.time() * 1000) % 1000000
        output_file = self.results_dir / f"trial_{trial.number}_{timestamp}.json"
        
        config_content = f"""csv_file: "{self.scenario_config['csv_file']}"
obstacles_file: "{self.scenario_config['obstacles_file']}"
urdf_file: "{self.scenario_config['urdf_file']}"
output_file: "{output_file}"

stomp:
  temperature: {params['temperature']:.6f}
  learning_rate: {params['learning_rate']:.6f}
  max_iterations: {params['max_iterations']}
  N: {params['N']}
  num_noisy_trajectories: {params['num_noisy_trajectories']}
  num_best_samples: {params['num_best_samples']}
  obstacle_cost_weight: {params['obstacle_cost_weight']:.6f}
  constraint_cost_weight: {params['constraint_cost_weight']:.6f}

evaluation:
  trajectory_pairs: {trajectory_pairs}
"""
        
        config_file = Path(f'/tmp/advanced_config_{trial.number}_{timestamp}.yaml')
        config_file.write_text(config_content)
        
        # Store paths for cleanup
        trial.set_user_attr('output_file', str(output_file))
        trial.set_user_attr('config_file', str(config_file))
        
        return str(config_file)

    def evaluate_parameters(self, trial: optuna.Trial) -> Tuple[float, float, float, float]:
        """
        Multi-objective evaluation returning 4 objectives:
        1. Planning time (minimize)
        2. Failure rate (minimize) 
        3. Constraint violations (minimize)
        4. Collision risk score (minimize)
        """
        config_file = self.create_config_file(trial)
        output_file = trial.user_attrs['output_file']
        
        try:
            # Execute parameter_evaluator with extended timeout for higher parameters
            evaluator_path = Path.cwd() / 'apps' / 'ParameterTuning' / 'parameter_evaluator'
            
            start_time = time.time()
            result = subprocess.run([str(evaluator_path), config_file], 
                                  text=True, 
                                  capture_output=True, 
                                  timeout=300)  # 5 minute timeout
            
            execution_time = time.time() - start_time
            
            if result.returncode != 0:
                logger.error(f"Trial {trial.number}: Execution failed - {result.stderr}")
                # Return poor values for all objectives
                return (300.0, 1.0, 10.0, 10.0)  # Max time, 100% failure, high violations/risk
            
            # Parse results
            if not Path(output_file).exists():
                logger.error(f"Trial {trial.number}: Output file not found")
                return (300.0, 1.0, 10.0, 10.0)
            
            with open(output_file, 'r') as f:
                results = json.load(f)
            
            # Extract multi-objective metrics
            success_rate = results.get('success_rate', 0.0)
            avg_planning_time = results.get('avg_planning_time_ms', 300000.0) / 1000.0  # Convert to seconds
            
            # Compute individual objectives
            planning_time_obj = avg_planning_time  # Minimize planning time
            failure_rate_obj = 1.0 - success_rate  # Minimize failure rate (maximize success)
            constraint_violations_obj = results.get('avg_constraint_violations', 10.0)  # Minimize violations
            collision_risk_obj = results.get('collision_risk_score', 10.0)  # Minimize collision risk
            
            # Safety constraint: penalize if success rate too low
            if success_rate < self.safety_constraints['min_success_rate']:
                failure_rate_obj += 5.0  # Heavy penalty for unacceptable success rates
            
            # Store detailed metrics as user attributes
            trial.set_user_attr('success_rate', success_rate)
            trial.set_user_attr('avg_planning_time_s', avg_planning_time)
            trial.set_user_attr('execution_time_s', execution_time)
            trial.set_user_attr('constraint_violations', constraint_violations_obj)
            trial.set_user_attr('collision_risk', collision_risk_obj)
            
            # Report intermediate values for pruning
            trial.report(failure_rate_obj, step=0)
            if trial.should_prune():
                logger.info(f"Trial {trial.number}: Pruned due to poor performance")
                raise optuna.TrialPruned()
            
            logger.info(f"Trial {trial.number}: Time={avg_planning_time:.2f}s, Success={success_rate:.3f}, "
                       f"Violations={constraint_violations_obj:.3f}, Risk={collision_risk_obj:.3f}")
            
            return (planning_time_obj, failure_rate_obj, constraint_violations_obj, collision_risk_obj)
            
        except subprocess.TimeoutExpired:
            logger.error(f"Trial {trial.number}: Timeout after 300 seconds")
            return (300.0, 1.0, 10.0, 10.0)
        except Exception as e:
            logger.error(f"Trial {trial.number}: Error - {e}")
            return (300.0, 1.0, 10.0, 10.0)
        finally:
            # Cleanup config file
            config_file_path = Path(trial.user_attrs.get('config_file', ''))
            if config_file_path.exists():
                config_file_path.unlink()

    def optimize(self, n_trials: int = 50) -> Dict[str, Any]:
        """
        Run multi-objective optimization and return Pareto-optimal solutions
        """
        logger.info(f"Starting advanced multi-objective optimization with {n_trials} trials")
        
        self.study.optimize(self.evaluate_parameters, n_trials=n_trials)
        
        # Analyze Pareto front
        pareto_trials = self.study.best_trials
        logger.info(f"Found {len(pareto_trials)} Pareto-optimal solutions")
        
        # Save detailed results
        results = {
            'pareto_solutions': [],
            'study_summary': {
                'n_trials': len(self.study.trials),
                'n_pareto_optimal': len(pareto_trials),
                'objectives': ['planning_time', 'failure_rate', 'constraint_violations', 'collision_risk']
            }
        }
        
        for i, trial in enumerate(pareto_trials):
            solution = {
                'rank': i + 1,
                'trial_number': trial.number,
                'objectives': trial.values,
                'parameters': trial.params,
                'metrics': {
                    'planning_time_s': trial.user_attrs.get('avg_planning_time_s', 0),
                    'success_rate': trial.user_attrs.get('success_rate', 0),
                    'constraint_violations': trial.user_attrs.get('constraint_violations', 0),
                    'collision_risk': trial.user_attrs.get('collision_risk', 0)
                }
            }
            results['pareto_solutions'].append(solution)
        
        return results

    def save_pareto_configs(self, results: Dict[str, Any]):
        """Save top Pareto-optimal configurations for production use"""
        
        pareto_configs_dir = Path("pareto_configs")
        pareto_configs_dir.mkdir(exist_ok=True)
        
        for solution in results['pareto_solutions'][:5]:  # Save top 5 solutions
            rank = solution['rank']
            params = solution['parameters']
            metrics = solution['metrics']
            
            config_content = f"""# Pareto-Optimal STOMP Configuration #{rank}
# Planning Time: {metrics['planning_time_s']:.2f}s
# Success Rate: {metrics['success_rate']:.3f}
# Constraint Violations: {metrics['constraint_violations']:.3f}
# Collision Risk: {metrics['collision_risk']:.3f}

csv_file: "{self.scenario_config['csv_file']}"
obstacles_file: "{self.scenario_config['obstacles_file']}"
urdf_file: "{self.scenario_config['urdf_file']}"
output_file: "pareto_results_{rank}.json"

stomp:
  temperature: {params['temperature']:.6f}
  learning_rate: {params['learning_rate']:.6f}
  max_iterations: {params['max_iterations']}
  N: {params['N']}
  num_noisy_trajectories: {params['num_noisy_trajectories']}
  num_best_samples: {params['num_best_samples']}
  obstacle_cost_weight: {params['obstacle_cost_weight']:.6f}
  constraint_cost_weight: {params['constraint_cost_weight']:.6f}

evaluation:
  trajectory_pairs: 20  # Extended evaluation for production
"""
            
            config_path = pareto_configs_dir / f"pareto_config_rank_{rank}.yaml"
            config_path.write_text(config_content)
            logger.info(f"Saved Pareto-optimal config rank {rank} to: {config_path}")

    def generate_advanced_plots(self):
        """Generate advanced visualization plots"""
        try:
            # Pareto front visualization
            fig_pareto = plot_pareto_front(
                self.study, 
                target_names=['Planning Time', 'Failure Rate', 'Constraint Violations', 'Collision Risk']
            )
            fig_pareto.write_html(self.results_dir / "pareto_front.html")
            
            # Parameter importance
            fig_importance = plot_param_importances(self.study)
            fig_importance.write_html(self.results_dir / "parameter_importance.html")
            
            # Optimization history
            fig_history = plot_optimization_history(self.study)
            fig_history.write_html(self.results_dir / "optimization_history.html")
            
            logger.info("Advanced visualization plots saved to advanced_results/")
            
        except Exception as e:
            logger.warning(f"Could not generate plots: {e}")


def main():
    """Main execution function"""
    print("🚀 ADVANCED MULTI-OBJECTIVE STOMP PARAMETER OPTIMIZATION")
    print("=" * 65)
    print("Objectives: Planning Time | Success Rate | Constraints | Safety")
    print("Strategy: NSGA-II Multi-Objective Bayesian Optimization")
    print("=" * 65)
    
    # Configuration for external anterior scan case
    scenario_config = {
        'csv_file': 'apps/ParameterTuning/Data/Ext_Ant_Scan/scan_pose_list_ee.csv',
        'obstacles_file': 'apps/ParameterTuning/Data/Ext_Ant_Scan/obstacles.xml',
        'urdf_file': '/Users/joris/Uni/MA/Code/PathPlanner_US_wip/res/scenario_1/panda_US.urdf'
    }
    
    # Initialize advanced optimizer
    optimizer = AdvancedSTOMPOptimizer(scenario_config, n_startup_trials=15)
    
    # Run multi-objective optimization
    results = optimizer.optimize(n_trials=40)  # Fewer trials but multi-objective
    
    # Print Pareto front summary
    print(f"\n🎯 PARETO-OPTIMAL SOLUTIONS FOUND: {len(results['pareto_solutions'])}")
    print("=" * 65)
    
    for solution in results['pareto_solutions'][:3]:  # Show top 3
        rank = solution['rank']
        metrics = solution['metrics']
        params = solution['parameters']
        
        print(f"\n📊 RANK {rank} SOLUTION:")
        print(f"   ⏱️  Planning Time: {metrics['planning_time_s']:.2f}s")
        print(f"   ✅ Success Rate: {metrics['success_rate']:.1%}")
        print(f"   ⚠️  Constraint Violations: {metrics['constraint_violations']:.3f}")
        print(f"   🛡️  Collision Risk: {metrics['collision_risk']:.3f}")
        print(f"   🔧 Key Parameters:")
        print(f"      Temperature: {params['temperature']:.2f}")
        print(f"      Learning Rate: {params['learning_rate']:.3f}")
        print(f"      Iterations: {params['max_iterations']}")
        print(f"      N (points): {params['N']}")
    
    # Save Pareto-optimal configurations
    optimizer.save_pareto_configs(results)
    
    # Generate advanced visualizations
    optimizer.generate_advanced_plots()
    
    print(f"\n🎉 OPTIMIZATION COMPLETE!")
    print(f"📁 Results saved to: advanced_results/")
    print(f"⚙️  Configs saved to: pareto_configs/")
    print(f"📊 Visualizations: advanced_results/*.html")


if __name__ == "__main__":
    main()
