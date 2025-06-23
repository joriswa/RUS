#!/usr/bin/env python3
"""
Core algorithm abstraction layer for trajectory planning optimization

This module provides the base classes and interfaces for different trajectory
planning algorithms, enabling unified optimization and comparison.
"""

from abc import ABC, abstractmethod
from typing import Dict, Any, Tuple, List
import optuna
import numpy as np
from dataclasses import dataclass
from enum import Enum


class AlgorithmType(Enum):
    """Supported trajectory planning algorithm types"""
    STOMP = "stomp"
    HAUSER_RRT = "hauser_rrt"
    HAUSER_RRT_STAR = "hauser_rrt_star"
    HAUSER_IRRT_STAR = "hauser_irrt_star"
    HAUSER_BIRRT = "hauser_birrt"


@dataclass
class PerformanceMetrics:
    """Comprehensive performance metrics for trajectory planning"""
    planning_time: float           # Planning time in seconds
    success_rate: float            # Success probability [0,1]
    trajectory_length: float       # Path length in joint space
    smoothness: float             # Trajectory smoothness measure
    safety_clearance: float       # Minimum obstacle clearance
    energy_consumption: float     # Estimated energy cost
    execution_time: float         # Trajectory execution time
    computational_cost: float     # Computational resource usage
    
    def get_composite_score(self, weights: Dict[str, float] = None) -> float:
        """Calculate weighted composite performance score"""
        if weights is None:
            # Default weights favoring clinical requirements
            weights = {
                'planning_time': 0.25,
                'success_rate': 0.30,
                'safety_clearance': 0.20,
                'trajectory_length': 0.15,
                'smoothness': 0.10
            }
        
        # Normalize metrics (lower is better except success_rate and safety_clearance)
        normalized_score = (
            weights.get('planning_time', 0) * (1.0 / max(0.1, self.planning_time)) +
            weights.get('success_rate', 0) * self.success_rate +
            weights.get('trajectory_length', 0) * (1.0 / max(0.1, self.trajectory_length)) +
            weights.get('smoothness', 0) * (1.0 / max(0.1, self.smoothness)) +
            weights.get('safety_clearance', 0) * self.safety_clearance +
            weights.get('energy_consumption', 0) * (1.0 / max(0.1, self.energy_consumption)) +
            weights.get('execution_time', 0) * (1.0 / max(0.1, self.execution_time)) +
            weights.get('computational_cost', 0) * (1.0 / max(0.1, self.computational_cost))
        )
        
        return normalized_score


class TrajectoryAlgorithm(ABC):
    """Abstract base class for trajectory planning algorithms"""
    
    def __init__(self, algorithm_type: AlgorithmType):
        self.algorithm_type = algorithm_type
        self.name = algorithm_type.value
        
    @abstractmethod
    def define_parameter_space(self, trial: optuna.Trial) -> Dict[str, Any]:
        """
        Define the parameter search space for this algorithm
        
        Args:
            trial: Optuna trial object for parameter suggestion
            
        Returns:
            Dictionary of parameter name -> value mappings
        """
        pass
    
    @abstractmethod
    def simulate_performance(self, params: Dict[str, Any]) -> PerformanceMetrics:
        """
        Simulate algorithm performance with given parameters
        
        Args:
            params: Parameter configuration
            
        Returns:
            Performance metrics for this configuration
        """
        pass
    
    @abstractmethod
    def get_parameter_bounds(self) -> Dict[str, Tuple[float, float]]:
        """
        Get parameter bounds for visualization and analysis
        
        Returns:
            Dictionary mapping parameter names to (min, max) bounds
        """
        pass
    
    def validate_parameters(self, params: Dict[str, Any]) -> bool:
        """
        Validate parameter configuration for this algorithm
        
        Args:
            params: Parameter configuration to validate
            
        Returns:
            True if parameters are valid, False otherwise
        """
        bounds = self.get_parameter_bounds()
        for param_name, value in params.items():
            if param_name in bounds:
                min_val, max_val = bounds[param_name]
                if not (min_val <= value <= max_val):
                    return False
        return True
    
    def get_algorithm_info(self) -> Dict[str, Any]:
        """Get algorithm metadata and description"""
        return {
            'name': self.name,
            'type': self.algorithm_type.value,
            'description': self.__doc__ or "No description available",
            'parameter_count': len(self.get_parameter_bounds()),
            'supports_real_time': getattr(self, 'supports_real_time', False),
            'computational_complexity': getattr(self, 'computational_complexity', 'Unknown')
        }


class STOMPAlgorithm(TrajectoryAlgorithm):
    """
    Stochastic Trajectory Optimization for Motion Planning (STOMP)
    
    STOMP optimizes trajectories through iterative stochastic sampling
    and cost-based trajectory refinement without explicit path planning.
    """
    
    def __init__(self):
        super().__init__(AlgorithmType.STOMP)
        self.supports_real_time = False
        self.computational_complexity = "O(I * N * T)" # Iterations * Noisy trajectories * Trajectory points
        
    def define_parameter_space(self, trial: optuna.Trial) -> Dict[str, Any]:
        """Define STOMP parameter space"""
        return {
            # Core STOMP parameters
            'max_iterations': trial.suggest_int('stomp_max_iterations', 50, 500),
            'num_noisy_trajectories': trial.suggest_int('stomp_num_noisy_trajectories', 10, 100),
            'num_best_samples': trial.suggest_int('stomp_num_best_samples', 2, 20),
            'learning_rate': trial.suggest_float('stomp_learning_rate', 0.01, 0.5, log=True),
            'temperature': trial.suggest_float('stomp_temperature', 1.0, 50.0),
            'dt': trial.suggest_float('stomp_dt', 0.01, 0.2),
            
            # Noise parameters
            'noise_stddev': trial.suggest_float('stomp_noise_stddev', 0.05, 0.3),
            'control_cost_weight': trial.suggest_float('stomp_control_cost_weight', 1.0, 10.0),
            'smoothness_cost_weight': trial.suggest_float('stomp_smoothness_cost_weight', 1.0, 10.0),
            'collision_cost_weight': trial.suggest_float('stomp_collision_cost_weight', 10.0, 100.0),
            
            # Advanced parameters
            'use_finite_differences': trial.suggest_categorical('stomp_use_finite_differences', [True, False]),
            'trajectory_length': trial.suggest_int('stomp_trajectory_length', 20, 100),
            'convergence_threshold': trial.suggest_float('stomp_convergence_threshold', 1e-6, 1e-3, log=True)
        }
    
    def simulate_performance(self, params: Dict[str, Any]) -> PerformanceMetrics:
        """Simulate STOMP performance with realistic modeling"""
        
        # Base performance characteristics for STOMP
        base_planning_time = 15.0  # seconds
        base_success_rate = 0.85
        
        # Parameter impact factors
        iteration_factor = np.clip(params['max_iterations'] / 200.0, 0.5, 2.5)
        noise_traj_factor = np.clip(params['num_noisy_trajectories'] / 50.0, 0.7, 1.8)
        learning_rate_factor = 1.0 + abs(params['learning_rate'] - 0.1) / 0.1 * 0.3
        temperature_factor = 1.0 + abs(params['temperature'] - 10.0) / 10.0 * 0.2
        
        # Advanced parameter impacts
        trajectory_length_factor = np.clip(params['trajectory_length'] / 50.0, 0.8, 1.5)
        control_cost_factor = 1.0 + (params['control_cost_weight'] - 5.0) / 5.0 * 0.1
        
        # Calculate performance metrics
        planning_time = base_planning_time * iteration_factor * noise_traj_factor * learning_rate_factor
        
        # Success rate depends on exploration vs exploitation balance
        exploration_quality = min(params['num_noisy_trajectories'] / 50.0, 1.0)
        convergence_quality = 1.0 / max(1.0, params['convergence_threshold'] * 1000)
        success_rate = base_success_rate * exploration_quality * convergence_quality
        
        # Trajectory quality metrics
        trajectory_length = 2.5 * trajectory_length_factor * (1.0 / control_cost_factor)
        smoothness = 0.15 * (1.0 / params['smoothness_cost_weight']) * temperature_factor
        safety_clearance = 0.12 + 0.05 * (params['collision_cost_weight'] / 50.0)
        
        # Computational and execution metrics
        energy_consumption = 3.5 * control_cost_factor * trajectory_length_factor
        execution_time = trajectory_length * params['dt'] * 1000  # Convert to ms
        computational_cost = iteration_factor * noise_traj_factor
        
        # Add realistic noise
        noise_scale = 0.1
        planning_time += np.random.normal(0, planning_time * noise_scale)
        success_rate = np.clip(success_rate + np.random.normal(0, 0.05), 0.0, 1.0)
        trajectory_length += np.random.normal(0, trajectory_length * noise_scale)
        
        return PerformanceMetrics(
            planning_time=max(1.0, planning_time),
            success_rate=success_rate,
            trajectory_length=max(0.5, trajectory_length),
            smoothness=max(0.01, smoothness),
            safety_clearance=max(0.02, safety_clearance),
            energy_consumption=max(1.0, energy_consumption),
            execution_time=max(100.0, execution_time),
            computational_cost=max(0.5, computational_cost)
        )
    
    def get_parameter_bounds(self) -> Dict[str, Tuple[float, float]]:
        """Get STOMP parameter bounds"""
        return {
            'max_iterations': (50, 500),
            'num_noisy_trajectories': (10, 100),
            'num_best_samples': (2, 20),
            'learning_rate': (0.01, 0.5),
            'temperature': (1.0, 50.0),
            'dt': (0.01, 0.2),
            'noise_stddev': (0.05, 0.3),
            'control_cost_weight': (1.0, 10.0),
            'smoothness_cost_weight': (1.0, 10.0),
            'collision_cost_weight': (10.0, 100.0),
            'trajectory_length': (20, 100),
            'convergence_threshold': (1e-6, 1e-3)
        }


class HauserRRTAlgorithm(TrajectoryAlgorithm):
    """
    Hauser method with RRT variants for path planning
    
    Combines Hauser's roadmap-based approach with different RRT variants
    for comprehensive trajectory planning in complex environments.
    """
    
    def __init__(self, rrt_variant: str = "iRRT_STAR"):
        # Map variant to algorithm type
        variant_map = {
            "RRT": AlgorithmType.HAUSER_RRT,
            "RRT_STAR": AlgorithmType.HAUSER_RRT_STAR,
            "iRRT_STAR": AlgorithmType.HAUSER_IRRT_STAR,
            "BiRRT": AlgorithmType.HAUSER_BIRRT
        }
        
        super().__init__(variant_map.get(rrt_variant, AlgorithmType.HAUSER_IRRT_STAR))
        self.rrt_variant = rrt_variant
        self.supports_real_time = True
        self.computational_complexity = "O(n log n + k * m)" # Roadmap + RRT expansion
        
    def define_parameter_space(self, trial: optuna.Trial) -> Dict[str, Any]:
        """Define Hauser+RRT parameter space"""
        params = {
            # Core Hauser parameters
            'hauser_samples': trial.suggest_int('hauser_samples', 100, 3000),
            'hauser_neighbor_radius': trial.suggest_float('hauser_neighbor_radius', 0.1, 2.5),
            'hauser_max_iterations': trial.suggest_int('hauser_max_iterations', 50, 1500),
            'hauser_collision_check_resolution': trial.suggest_float('hauser_collision_check_resolution', 0.01, 0.1),
            
            # RRT parameters
            'rrt_max_iterations': trial.suggest_int('rrt_max_iterations', 500, 8000),
            'rrt_step_size': trial.suggest_float('rrt_step_size', 0.01, 0.3),
            'rrt_goal_bias': trial.suggest_float('rrt_goal_bias', 0.05, 0.25),
            
            # Integration parameters
            'hauser_rrt_integration_mode': trial.suggest_categorical('hauser_rrt_integration_mode', 
                                                                   ['sequential', 'parallel', 'hybrid']),
            'hauser_path_smoothing': trial.suggest_categorical('hauser_path_smoothing', [True, False]),
            'hauser_dynamic_resampling': trial.suggest_categorical('hauser_dynamic_resampling', [True, False]),
        }
        
        # Add variant-specific parameters
        if self.rrt_variant in ['RRT_STAR', 'iRRT_STAR']:
            params.update({
                'rrt_star_radius': trial.suggest_float('rrt_star_radius', 0.1, 1.5),
                'rrt_star_rewire_factor': trial.suggest_float('rrt_star_rewire_factor', 1.0, 2.0)
            })
        
        if self.rrt_variant == 'iRRT_STAR':
            params.update({
                'irrt_star_informed_sampling': trial.suggest_categorical('irrt_star_informed_sampling', [True, False]),
                'irrt_star_pruning_radius': trial.suggest_float('irrt_star_pruning_radius', 0.5, 3.0)
            })
        
        if self.rrt_variant == 'BiRRT':
            params.update({
                'birrt_connection_radius': trial.suggest_float('birrt_connection_radius', 0.1, 1.0),
                'birrt_swap_probability': trial.suggest_float('birrt_swap_probability', 0.1, 0.5)
            })
        
        return params
    
    def simulate_performance(self, params: Dict[str, Any]) -> PerformanceMetrics:
        """Simulate Hauser+RRT performance with realistic modeling"""
        
        # Base performance for Hauser method
        base_planning_time = 8.0
        
        # RRT variant performance factors
        variant_factors = {
            'RRT': {'time_factor': 1.0, 'success_factor': 0.75, 'quality_factor': 1.0},
            'RRT_STAR': {'time_factor': 1.3, 'success_factor': 0.85, 'quality_factor': 0.8},
            'iRRT_STAR': {'time_factor': 1.1, 'success_factor': 0.92, 'quality_factor': 0.65},
            'BiRRT': {'time_factor': 0.9, 'success_factor': 0.82, 'quality_factor': 0.85}
        }
        
        variant_info = variant_factors.get(self.rrt_variant, variant_factors['iRRT_STAR'])
        
        # Hauser parameter impacts
        sample_factor = max(0.4, 1.0 - (params['hauser_samples'] - 1000) / 2000 * 0.5)
        radius_factor = 1.0 + abs(params['hauser_neighbor_radius'] - 1.0) / 1.0 * 0.3
        iteration_factor = max(0.6, 1.0 - (params['hauser_max_iterations'] - 500) / 1000 * 0.25)
        
        # RRT parameter impacts
        rrt_iter_factor = max(0.7, 1.0 - (params['rrt_max_iterations'] - 2000) / 6000 * 0.2)
        step_size_factor = 1.0 + abs(params['rrt_step_size'] - 0.1) / 0.1 * 0.15
        
        # Integration mode impacts
        integration_factors = {
            'sequential': 1.0,
            'parallel': 0.85,
            'hybrid': 0.90
        }
        integration_factor = integration_factors[params['hauser_rrt_integration_mode']]
        
        # Optimization factors
        smoothing_factor = 0.95 if params['hauser_path_smoothing'] else 1.0
        resampling_factor = 0.92 if params['hauser_dynamic_resampling'] else 1.0
        
        # Calculate performance metrics
        planning_time = (base_planning_time * variant_info['time_factor'] * sample_factor * 
                        radius_factor * iteration_factor * rrt_iter_factor * 
                        step_size_factor * integration_factor * smoothing_factor * 
                        resampling_factor)
        
        success_rate = variant_info['success_factor'] * (1.0 / radius_factor) * (1.0 / step_size_factor)
        trajectory_length = 2.8 * variant_info['quality_factor'] * sample_factor
        smoothness = 0.12 * smoothing_factor * variant_info['quality_factor']
        safety_clearance = 0.15 + 0.05 * (params['hauser_collision_check_resolution'] * 10)
        
        # Add realistic noise
        noise_scale = 0.08
        planning_time += np.random.normal(0, planning_time * noise_scale)
        success_rate = np.clip(success_rate + np.random.normal(0, 0.03), 0.0, 1.0)
        
        return PerformanceMetrics(
            planning_time=max(1.0, planning_time),
            success_rate=success_rate,
            trajectory_length=max(0.5, trajectory_length),
            smoothness=max(0.01, smoothness),
            safety_clearance=max(0.02, safety_clearance),
            energy_consumption=max(1.0, trajectory_length * 1.2),
            execution_time=max(100.0, trajectory_length * 300),
            computational_cost=max(0.5, sample_factor * iteration_factor)
        )
    
    def get_parameter_bounds(self) -> Dict[str, Tuple[float, float]]:
        """Get Hauser+RRT parameter bounds"""
        bounds = {
            'hauser_samples': (100, 3000),
            'hauser_neighbor_radius': (0.1, 2.5),
            'hauser_max_iterations': (50, 1500),
            'hauser_collision_check_resolution': (0.01, 0.1),
            'rrt_max_iterations': (500, 8000),
            'rrt_step_size': (0.01, 0.3),
            'rrt_goal_bias': (0.05, 0.25)
        }
        
        # Add variant-specific bounds
        if self.rrt_variant in ['RRT_STAR', 'iRRT_STAR']:
            bounds.update({
                'rrt_star_radius': (0.1, 1.5),
                'rrt_star_rewire_factor': (1.0, 2.0)
            })
        
        if self.rrt_variant == 'iRRT_STAR':
            bounds.update({
                'irrt_star_pruning_radius': (0.5, 3.0)
            })
        
        if self.rrt_variant == 'BiRRT':
            bounds.update({
                'birrt_connection_radius': (0.1, 1.0),
                'birrt_swap_probability': (0.1, 0.5)
            })
        
        return bounds


# Factory functions for easy algorithm creation
def create_stomp_algorithm() -> STOMPAlgorithm:
    """Create a STOMP algorithm instance"""
    return STOMPAlgorithm()

def create_hauser_rrt_algorithms() -> List[HauserRRTAlgorithm]:
    """Create all Hauser+RRT variant algorithms"""
    return [
        HauserRRTAlgorithm("RRT"),
        HauserRRTAlgorithm("RRT_STAR"),
        HauserRRTAlgorithm("iRRT_STAR"),
        HauserRRTAlgorithm("BiRRT")
    ]

def create_all_algorithms() -> List[TrajectoryAlgorithm]:
    """Create complete algorithm suite for comparison"""
    algorithms = [create_stomp_algorithm()]
    algorithms.extend(create_hauser_rrt_algorithms())
    return algorithms
