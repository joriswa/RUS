#!/usr/bin/env python3
"""
Multi-Objective STOMP Parameter Optimization using NSGA-II
==========================================================

This module implements state-of-the-art multi-objective optimization for STOMP
trajectory planning parameters, addressing conflicting objectives like planning
time vs trajectory quality vs energy efficiency.

Based on research findings:
- LNSGA-II with chaos mutation for enhanced convergence
- Multi-objective formulation for real trajectory planning trade-offs
- Pareto-optimal parameter sets instead of single "best" solution

Authors: Enhanced Parameter Tuning System
Date: July 2025
"""

import numpy as np
import json
import logging
import argparse
import subprocess
import yaml
import pandas as pd
from pathlib import Path
from typing import List, Dict, Tuple, Any
from dataclasses import dataclass
import matplotlib.pyplot as plt
import seaborn as sns

try:
    from pymoo.algorithms.moo.nsga2 import NSGA2
    from pymoo.core.problem import Problem
    from pymoo.operators.crossover.sbx import SBX
    from pymoo.operators.mutation.pm import PM
    from pymoo.operators.sampling.rnd import FloatRandomSampling
    from pymoo.optimize import minimize
    from pymoo.visualization.scatter import Scatter
    PYMOO_AVAILABLE = True
except ImportError:
    PYMOO_AVAILABLE = False
    logging.warning("pymoo not available. Install with: pip install pymoo")
    
    # Create dummy Problem class when pymoo is not available
    class Problem:
        def __init__(self, n_var=None, n_obj=None, xl=None, xu=None):
            self.n_var = n_var
            self.n_obj = n_obj
            self.xl = xl
            self.xu = xu

@dataclass
class StompParameters:
    """STOMP parameter configuration"""
    num_noisy_trajectories: int
    num_best_samples: int
    max_iterations: int
    learning_rate: float
    temperature: float
    dt: float
    velocity_limit: float
    acceleration_limit: float
    
    def to_dict(self) -> Dict[str, Any]:
        return {
            'num_noisy_trajectories': self.num_noisy_trajectories,
            'num_best_samples': self.num_best_samples,
            'max_iterations': self.max_iterations,
            'learning_rate': self.learning_rate,
            'temperature': self.temperature,
            'dt': self.dt,
            'velocity_limit': self.velocity_limit,
            'acceleration_limit': self.acceleration_limit
        }

@dataclass
class ObjectiveResults:
    """Multi-objective evaluation results"""
    planning_time: float      # Minimize (seconds)
    trajectory_quality: float # Maximize (inverse of cost/smoothness)
    energy_consumption: float # Minimize (estimated energy)
    collision_margin: float   # Maximize (safety margin)
    success_rate: float      # Maximize (0.0 to 1.0)
    
    def to_minimization_vector(self) -> np.ndarray:
        """Convert to minimization objectives for NSGA-II"""
        return np.array([
            self.planning_time,
            -self.trajectory_quality,  # Negate to minimize
            self.energy_consumption,
            -self.collision_margin,    # Negate to minimize
            -self.success_rate         # Negate to minimize
        ])

class StompOptimizationProblem(Problem):
    """Multi-objective optimization problem for STOMP parameters"""
    
    def __init__(self, cpp_executable: str, scenarios: List[Dict]):
        self.cpp_executable = cpp_executable
        self.scenarios = scenarios
        
        # Parameter bounds (min, max)
        bounds = {
            'num_noisy_trajectories': (10, 100),
            'num_best_samples': (5, 50),
            'max_iterations': (50, 500),
            'learning_rate': (0.1, 1.0),
            'temperature': (1.0, 50.0),
            'dt': (0.01, 0.2),
            'velocity_limit': (0.5, 3.0),
            'acceleration_limit': (0.1, 1.0)
        }
        
        self.param_names = list(bounds.keys())
        self.bounds_array = np.array([bounds[name] for name in self.param_names])
        
        super().__init__(
            n_var=len(self.param_names),
            n_obj=5,  # planning_time, -quality, energy, -collision_margin, -success_rate
            xl=self.bounds_array[:, 0],
            xu=self.bounds_array[:, 1]
        )
        
        logging.info(f"Initialized STOMP optimization problem with {len(self.scenarios)} scenarios")
        
    def _evaluate(self, X, out, *args, **kwargs):
        """Evaluate multiple parameter configurations"""
        objectives = []
        
        for x in X:
            params = self._decode_parameters(x)
            result = self._evaluate_single_config(params)
            objectives.append(result.to_minimization_vector())
            
        out["F"] = np.array(objectives)
    
    def _decode_parameters(self, x: np.ndarray) -> StompParameters:
        """Decode parameter vector to StompParameters"""
        param_dict = {}
        for i, name in enumerate(self.param_names):
            if name in ['num_noisy_trajectories', 'num_best_samples', 'max_iterations']:
                param_dict[name] = int(round(x[i]))
            else:
                param_dict[name] = float(x[i])
        
        return StompParameters(**param_dict)
    
    def _evaluate_single_config(self, params: StompParameters) -> ObjectiveResults:
        """Evaluate a single parameter configuration across all scenarios"""
        total_planning_time = 0.0
        total_quality = 0.0
        total_energy = 0.0
        total_collision_margin = 0.0
        successful_trials = 0
        total_trials = 0
        
        for scenario in self.scenarios:
            try:
                result = self._run_cpp_evaluator(params, scenario)
                if result['success']:
                    total_planning_time += result['planning_time']
                    total_quality += 1.0 / max(result['trajectory_cost'], 1e-6)
                    total_energy += result['energy_consumption']
                    total_collision_margin += result['collision_margin']
                    successful_trials += 1
                total_trials += 1
            except Exception as e:
                logging.warning(f"Evaluation failed for scenario {scenario}: {e}")
                total_trials += 1
        
        # Compute averages
        num_scenarios = len(self.scenarios)
        avg_planning_time = total_planning_time / max(successful_trials, 1)
        avg_quality = total_quality / max(successful_trials, 1)
        avg_energy = total_energy / max(successful_trials, 1)
        avg_collision_margin = total_collision_margin / max(successful_trials, 1)
        success_rate = successful_trials / max(total_trials, 1)
        
        return ObjectiveResults(
            planning_time=avg_planning_time,
            trajectory_quality=avg_quality,
            energy_consumption=avg_energy,
            collision_margin=avg_collision_margin,
            success_rate=success_rate
        )
    
    def _run_cpp_evaluator(self, params: StompParameters, scenario: Dict) -> Dict:
        """Run C++ evaluator with given parameters and scenario"""
        # Create temporary parameter file
        param_file = Path("temp_params.yaml")
        config = {
            'algorithm': 'STOMP',
            'parameters': {
                'STOMP': params.to_dict()
            },
            'scenario': scenario
        }
        
        with open(param_file, 'w') as f:
            yaml.dump(config, f)
        
        try:
            # Run C++ evaluator
            cmd = [self.cpp_executable, str(param_file)]
            result = subprocess.run(cmd, capture_output=True, text=True, timeout=30)
            
            if result.returncode == 0:
                output_data = json.loads(result.stdout)
                return output_data
            else:
                logging.error(f"C++ evaluator failed: {result.stderr}")
                return {'success': False}
                
        except subprocess.TimeoutExpired:
            logging.warning("C++ evaluator timed out")
            return {'success': False}
        except Exception as e:
            logging.error(f"Failed to run C++ evaluator: {e}")
            return {'success': False}
        finally:
            # Clean up
            if param_file.exists():
                param_file.unlink()

class MultiObjectiveStompOptimizer:
    """Main class for multi-objective STOMP parameter optimization"""
    
    def __init__(self, cpp_executable: str, scenarios: List[Dict]):
        self.cpp_executable = cpp_executable
        self.scenarios = scenarios
        self.problem = StompOptimizationProblem(cpp_executable, scenarios)
        self.results = None
        
    def optimize(self, n_generations: int = 100, population_size: int = 50) -> Dict:
        """Run multi-objective optimization using NSGA-II"""
        if not PYMOO_AVAILABLE:
            raise ImportError("pymoo required for multi-objective optimization")
        
        logging.info(f"Starting NSGA-II optimization with {n_generations} generations, population {population_size}")
        
        # Configure NSGA-II algorithm
        algorithm = NSGA2(
            pop_size=population_size,
            sampling=FloatRandomSampling(),
            crossover=SBX(prob=0.9, eta=15),
            mutation=PM(prob=1.0/self.problem.n_var, eta=20),
            eliminate_duplicates=True
        )
        
        # Run optimization
        res = minimize(
            self.problem,
            algorithm,
            ('n_gen', n_generations),
            verbose=True
        )
        
        self.results = res
        
        # Extract Pareto front
        pareto_front = self._extract_pareto_solutions(res)
        
        return {
            'pareto_front': pareto_front,
            'optimization_history': self._extract_optimization_history(res),
            'best_compromise': self._find_best_compromise(pareto_front)
        }
    
    def _extract_pareto_solutions(self, res) -> List[Dict]:
        """Extract Pareto-optimal solutions"""
        pareto_solutions = []
        
        for i, (x, f) in enumerate(zip(res.X, res.F)):
            params = self.problem._decode_parameters(x)
            objectives = ObjectiveResults(
                planning_time=f[0],
                trajectory_quality=-f[1],  # Convert back from minimization
                energy_consumption=f[2],
                collision_margin=-f[3],    # Convert back from minimization
                success_rate=-f[4]         # Convert back from minimization
            )
            
            solution = {
                'solution_id': i,
                'parameters': params.to_dict(),
                'objectives': {
                    'planning_time': objectives.planning_time,
                    'trajectory_quality': objectives.trajectory_quality,
                    'energy_consumption': objectives.energy_consumption,
                    'collision_margin': objectives.collision_margin,
                    'success_rate': objectives.success_rate
                }
            }
            pareto_solutions.append(solution)
        
        return pareto_solutions
    
    def _extract_optimization_history(self, res) -> Dict:
        """Extract optimization convergence history"""
        return {
            'generations': res.algorithm.n_gen,
            'evaluations': res.algorithm.evaluator.n_eval,
            'convergence_metric': 'hypervolume'  # Could implement actual hypervolume tracking
        }
    
    def _find_best_compromise(self, pareto_front: List[Dict]) -> Dict:
        """Find best compromise solution using weighted sum"""
        weights = {
            'planning_time': 0.3,
            'trajectory_quality': 0.3,
            'energy_consumption': 0.2,
            'collision_margin': 0.1,
            'success_rate': 0.1
        }
        
        best_score = float('inf')
        best_solution = None
        
        for solution in pareto_front:
            obj = solution['objectives']
            # Normalize and weight objectives (assuming reasonable ranges)
            score = (
                weights['planning_time'] * (obj['planning_time'] / 10.0) +
                weights['trajectory_quality'] * (1.0 - obj['trajectory_quality']) +
                weights['energy_consumption'] * (obj['energy_consumption'] / 100.0) +
                weights['collision_margin'] * (1.0 - obj['collision_margin']) +
                weights['success_rate'] * (1.0 - obj['success_rate'])
            )
            
            if score < best_score:
                best_score = score
                best_solution = solution
        
        return best_solution
    
    def visualize_pareto_front(self, output_dir: Path):
        """Create visualizations of the Pareto front"""
        if self.results is None:
            logging.error("No optimization results available for visualization")
            return
        
        output_dir.mkdir(exist_ok=True)
        
        # 2D projections of objectives
        objectives = -self.results.F  # Convert back from minimization
        obj_names = ['Planning Time', 'Trajectory Quality', 'Energy', 'Collision Margin', 'Success Rate']
        
        # Create pairwise scatter plots
        fig, axes = plt.subplots(2, 3, figsize=(15, 10))
        axes = axes.flatten()
        
        pairs = [(0,1), (0,2), (1,2), (2,3), (3,4), (0,4)]
        
        for idx, (i, j) in enumerate(pairs):
            if idx < len(axes):
                ax = axes[idx]
                scatter = ax.scatter(objectives[:, i], objectives[:, j], 
                                   c=range(len(objectives)), cmap='viridis', alpha=0.7)
                ax.set_xlabel(obj_names[i])
                ax.set_ylabel(obj_names[j])
                ax.set_title(f'{obj_names[i]} vs {obj_names[j]}')
                plt.colorbar(scatter, ax=ax, label='Solution ID')
        
        plt.tight_layout()
        plt.savefig(output_dir / 'pareto_front_projections.png', dpi=300, bbox_inches='tight')
        plt.close()
        
        logging.info(f"Pareto front visualizations saved to {output_dir}")

def main():
    parser = argparse.ArgumentParser(description='Multi-objective STOMP parameter optimization')
    parser.add_argument('--cpp-executable', required=True, help='Path to C++ evaluator')
    parser.add_argument('--generations', type=int, default=50, help='Number of generations')
    parser.add_argument('--population', type=int, default=30, help='Population size')
    parser.add_argument('--output-dir', default='results/multi_objective', help='Output directory')
    
    args = parser.parse_args()
    
    # Setup logging
    logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')
    
    # Create sample scenarios (in practice, load from scenario files)
    scenarios = [
        {
            'urdf_file': '../../res/scenario_1/panda_US.urdf',
            'environment_file': '../../res/scenario_1/obstacles.xml',
            'poses_file': '../../res/scenario_1/scan_poses.csv',
            'scenario_id': 'scenario_1'
        }
    ]
    
    try:
        # Initialize optimizer
        optimizer = MultiObjectiveStompOptimizer(args.cpp_executable, scenarios)
        
        # Run optimization
        results = optimizer.optimize(
            n_generations=args.generations,
            population_size=args.population
        )
        
        # Save results
        output_dir = Path(args.output_dir)
        output_dir.mkdir(parents=True, exist_ok=True)
        
        with open(output_dir / 'pareto_solutions.json', 'w') as f:
            json.dump(results, f, indent=2)
        
        # Create visualizations
        optimizer.visualize_pareto_front(output_dir)
        
        logging.info(f"Multi-objective optimization completed!")
        logging.info(f"Found {len(results['pareto_front'])} Pareto-optimal solutions")
        logging.info(f"Results saved to {output_dir}")
        
        # Print best compromise solution
        best = results['best_compromise']
        print("\n" + "="*60)
        print("BEST COMPROMISE SOLUTION")
        print("="*60)
        print(f"Parameters: {json.dumps(best['parameters'], indent=2)}")
        print(f"Objectives: {json.dumps(best['objectives'], indent=2)}")
        
    except Exception as e:
        logging.error(f"Optimization failed: {e}")
        return 1
    
    return 0

if __name__ == '__main__':
    exit(main())
