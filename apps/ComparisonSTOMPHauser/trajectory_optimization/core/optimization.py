#!/usr/bin/env python3
"""
Multi-algorithm trajectory planning optimization framework

This module provides a unified optimization interface that can compare
and optimize across different trajectory planning algorithms including
STOMP and Hauser+RRT variants.
"""

import optuna
import pandas as pd
import numpy as np
from typing import List, Dict, Any, Optional, Union
from pathlib import Path
import json
import sqlite3
from datetime import datetime
import logging

from .algorithms import (
    TrajectoryAlgorithm, STOMPAlgorithm, HauserRRTAlgorithm, 
    AlgorithmType, PerformanceMetrics
)

logger = logging.getLogger(__name__)


class MultiAlgorithmOptimizer:
    """
    Unified optimization framework for trajectory planning algorithms
    
    Supports optimization of individual algorithms and head-to-head comparisons
    across different algorithm types (STOMP vs Hauser+RRT variants).
    """
    
    def __init__(self, 
                 study_name: str = "trajectory_planning_optimization",
                 storage_url: Optional[str] = None,
                 algorithms: Optional[List[TrajectoryAlgorithm]] = None):
        """
        Initialize the multi-algorithm optimizer
        
        Args:
            study_name: Name for the optimization study
            storage_url: Database URL for study persistence
            algorithms: List of algorithms to optimize/compare
        """
        self.study_name = study_name
        self.storage_url = storage_url or f"sqlite:///{study_name}.db"
        self.algorithms = algorithms or self._create_default_algorithms()
        self.studies = {}
        self.results = {}
        
        # Create data directory
        self.data_dir = Path("trajectory_optimization/data")
        self.data_dir.mkdir(parents=True, exist_ok=True)
        
    def _create_default_algorithms(self) -> List[TrajectoryAlgorithm]:
        """Create default set of algorithms for comparison"""
        return [
            STOMPAlgorithm(),
            HauserRRTAlgorithm("RRT"),
            HauserRRTAlgorithm("RRT_STAR"),
            HauserRRTAlgorithm("iRRT_STAR"),
            HauserRRTAlgorithm("BiRRT")
        ]
    
    def add_algorithm(self, algorithm: TrajectoryAlgorithm):
        """Add an algorithm to the optimization suite"""
        self.algorithms.append(algorithm)
        logger.info(f"Added algorithm: {algorithm.name}")
    
    def optimize_single_algorithm(self, 
                                 algorithm: TrajectoryAlgorithm,
                                 n_trials: int = 200,
                                 timeout: Optional[float] = None,
                                 direction: str = "minimize") -> optuna.Study:
        """
        Optimize parameters for a single algorithm
        
        Args:
            algorithm: Algorithm to optimize
            n_trials: Number of optimization trials
            timeout: Maximum optimization time in seconds
            direction: Optimization direction ("minimize" or "maximize")
            
        Returns:
            Completed Optuna study
        """
        study_name = f"{self.study_name}_{algorithm.name}"
        
        # Create objective function for this algorithm
        def objective(trial):
            try:
                # Get parameter configuration
                params = algorithm.define_parameter_space(trial)
                
                # Validate parameters
                if not algorithm.validate_parameters(params):
                    logger.warning(f"Invalid parameters for {algorithm.name}: {params}")
                    return float('inf')
                
                # Simulate performance
                metrics = algorithm.simulate_performance(params)
                
                # Use composite score as objective
                objective_value = metrics.get_composite_score()
                
                # Log trial information
                logger.debug(f"Trial {trial.number} ({algorithm.name}): "
                           f"objective={objective_value:.4f}, "
                           f"planning_time={metrics.planning_time:.2f}s, "
                           f"success_rate={metrics.success_rate:.3f}")
                
                # Store additional metrics in trial user attributes
                trial.set_user_attr("planning_time", metrics.planning_time)
                trial.set_user_attr("success_rate", metrics.success_rate)
                trial.set_user_attr("trajectory_length", metrics.trajectory_length)
                trial.set_user_attr("safety_clearance", metrics.safety_clearance)
                trial.set_user_attr("algorithm_type", algorithm.algorithm_type.value)
                
                return objective_value
                
            except Exception as e:
                logger.error(f"Trial {trial.number} failed for {algorithm.name}: {e}")
                return float('inf')
        
        # Create or load study
        storage = optuna.storages.RDBStorage(self.storage_url)
        
        try:
            study = optuna.load_study(study_name=study_name, storage=storage)
            logger.info(f"Loaded existing study: {study_name}")
        except:
            study = optuna.create_study(
                study_name=study_name,
                direction=direction,
                storage=storage,
                load_if_exists=False
            )
            logger.info(f"Created new study: {study_name}")
        
        # Run optimization
        logger.info(f"Starting optimization for {algorithm.name} with {n_trials} trials...")
        study.optimize(objective, n_trials=n_trials, timeout=timeout, show_progress_bar=True)
        
        # Store study results
        self.studies[algorithm.name] = study
        self._save_study_results(algorithm.name, study)
        
        logger.info(f"Optimization completed for {algorithm.name}. "
                   f"Best value: {study.best_value:.4f}")
        
        return study
    
    def optimize_all_algorithms(self, 
                               n_trials_per_algorithm: int = 200,
                               timeout_per_algorithm: Optional[float] = None) -> Dict[str, optuna.Study]:
        """
        Optimize all algorithms in the suite
        
        Args:
            n_trials_per_algorithm: Number of trials per algorithm
            timeout_per_algorithm: Timeout per algorithm in seconds
            
        Returns:
            Dictionary mapping algorithm names to their studies
        """
        logger.info(f"Starting multi-algorithm optimization with {len(self.algorithms)} algorithms")
        
        for algorithm in self.algorithms:
            try:
                study = self.optimize_single_algorithm(
                    algorithm, 
                    n_trials=n_trials_per_algorithm,
                    timeout=timeout_per_algorithm
                )
                logger.info(f"✓ Completed optimization for {algorithm.name}")
                
            except Exception as e:
                logger.error(f"✗ Failed optimization for {algorithm.name}: {e}")
                continue
        
        logger.info("Multi-algorithm optimization completed")
        return self.studies
    
    def compare_algorithms(self, 
                          algorithms: Optional[List[str]] = None,
                          metric: str = "composite_score") -> pd.DataFrame:
        """
        Compare performance across algorithms
        
        Args:
            algorithms: List of algorithm names to compare (None for all)
            metric: Metric to use for comparison
            
        Returns:
            DataFrame with comparison results
        """
        if algorithms is None:
            algorithms = [alg.name for alg in self.algorithms]
        
        comparison_data = []
        
        for alg_name in algorithms:
            if alg_name not in self.studies:
                logger.warning(f"No study found for algorithm: {alg_name}")
                continue
                
            study = self.studies[alg_name]
            df = study.trials_dataframe()
            
            # Calculate statistics
            if metric == "composite_score":
                values = df['value']
            else:
                values = df[f'user_attrs_{metric}'] if f'user_attrs_{metric}' in df.columns else df['value']
            
            comparison_data.append({
                'algorithm': alg_name,
                'best_value': values.min(),
                'mean_value': values.mean(),
                'std_value': values.std(),
                'median_value': values.median(),
                'q25_value': values.quantile(0.25),
                'q75_value': values.quantile(0.75),
                'n_trials': len(values),
                'success_rate': len(values[values != float('inf')]) / len(values)
            })
        
        comparison_df = pd.DataFrame(comparison_data)
        comparison_df = comparison_df.sort_values('best_value')
        
        return comparison_df
    
    def get_best_configurations(self) -> Dict[str, Dict[str, Any]]:
        """
        Get best parameter configurations for each algorithm
        
        Returns:
            Dictionary mapping algorithm names to their best configurations
        """
        best_configs = {}
        
        for alg_name, study in self.studies.items():
            best_configs[alg_name] = {
                'parameters': study.best_params,
                'objective_value': study.best_value,
                'trial_number': study.best_trial.number,
                'algorithm_info': next(
                    (alg.get_algorithm_info() for alg in self.algorithms if alg.name == alg_name),
                    {}
                )
            }
        
        return best_configs
    
    def _save_study_results(self, algorithm_name: str, study: optuna.Study):
        """Save study results to files"""
        results_dir = self.data_dir / "results" / algorithm_name
        results_dir.mkdir(parents=True, exist_ok=True)
        
        # Save study dataframe
        df = study.trials_dataframe()
        df.to_csv(results_dir / "trials.csv", index=False)
        
        # Save best configuration
        best_config = {
            'algorithm': algorithm_name,
            'best_value': study.best_value,
            'best_params': study.best_params,
            'n_trials': len(study.trials),
            'timestamp': datetime.now().isoformat()
        }
        
        with open(results_dir / "best_config.json", 'w') as f:
            json.dump(best_config, f, indent=2)
        
        logger.info(f"Saved results for {algorithm_name} to {results_dir}")
    
    def generate_optimization_report(self, output_path: Optional[Path] = None) -> str:
        """
        Generate comprehensive optimization report
        
        Args:
            output_path: Path to save the report
            
        Returns:
            Report content as string
        """
        if output_path is None:
            output_path = self.data_dir / "reports" / f"optimization_report_{datetime.now().strftime('%Y%m%d_%H%M%S')}.md"
        
        output_path.parent.mkdir(parents=True, exist_ok=True)
        
        # Generate report content
        report_lines = [
            "# Trajectory Planning Algorithm Optimization Report",
            "",
            f"**Generated**: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}",
            f"**Study**: {self.study_name}",
            f"**Algorithms**: {len(self.algorithms)}",
            "",
            "## Executive Summary",
            ""
        ]
        
        # Algorithm comparison
        comparison_df = self.compare_algorithms()
        if not comparison_df.empty:
            best_algorithm = comparison_df.iloc[0]['algorithm']
            best_value = comparison_df.iloc[0]['best_value']
            
            report_lines.extend([
                f"**Best Performing Algorithm**: {best_algorithm}",
                f"**Best Objective Value**: {best_value:.4f}",
                "",
                "## Algorithm Performance Comparison",
                "",
                "| Algorithm | Best Value | Mean ± Std | Median | Trials |",
                "|-----------|------------|-------------|---------|--------|"
            ])
            
            for _, row in comparison_df.iterrows():
                report_lines.append(
                    f"| {row['algorithm']} | {row['best_value']:.4f} | "
                    f"{row['mean_value']:.4f} ± {row['std_value']:.4f} | "
                    f"{row['median_value']:.4f} | {row['n_trials']} |"
                )
        
        # Best configurations
        best_configs = self.get_best_configurations()
        report_lines.extend([
            "",
            "## Optimal Configurations",
            ""
        ])
        
        for alg_name, config in best_configs.items():
            report_lines.extend([
                f"### {alg_name}",
                "",
                f"**Objective Value**: {config['objective_value']:.4f}",
                f"**Trial Number**: {config['trial_number']}",
                "",
                "**Parameters**:",
                ""
            ])
            
            for param, value in config['parameters'].items():
                if isinstance(value, float):
                    report_lines.append(f"- {param}: {value:.4f}")
                else:
                    report_lines.append(f"- {param}: {value}")
            
            report_lines.append("")
        
        # Statistical analysis
        report_lines.extend([
            "## Statistical Analysis",
            "",
            "### Parameter Importance",
            ""
        ])
        
        for alg_name, study in self.studies.items():
            if len(study.trials) > 10:  # Only analyze if sufficient data
                try:
                    importance = optuna.importance.get_param_importances(study)
                    top_params = sorted(importance.items(), key=lambda x: x[1], reverse=True)[:5]
                    
                    report_lines.extend([
                        f"**{alg_name} - Top 5 Parameters**:",
                        ""
                    ])
                    
                    for i, (param, importance_val) in enumerate(top_params, 1):
                        report_lines.append(f"{i}. {param}: {importance_val:.4f}")
                    
                    report_lines.append("")
                    
                except Exception as e:
                    logger.warning(f"Could not calculate importance for {alg_name}: {e}")
        
        # Write report
        report_content = "\n".join(report_lines)
        with open(output_path, 'w') as f:
            f.write(report_content)
        
        logger.info(f"Generated optimization report: {output_path}")
        return report_content
    
    def export_results(self, format: str = "json", output_dir: Optional[Path] = None):
        """
        Export optimization results in various formats
        
        Args:
            format: Export format ("json", "csv", "yaml")
            output_dir: Output directory for exports
        """
        if output_dir is None:
            output_dir = self.data_dir / "exports"
        
        output_dir.mkdir(parents=True, exist_ok=True)
        
        # Export best configurations
        best_configs = self.get_best_configurations()
        
        if format == "json":
            with open(output_dir / "best_configurations.json", 'w') as f:
                json.dump(best_configs, f, indent=2)
        elif format == "csv":
            # Flatten configurations for CSV
            flattened_data = []
            for alg_name, config in best_configs.items():
                row = {'algorithm': alg_name, 'objective_value': config['objective_value']}
                row.update(config['parameters'])
                flattened_data.append(row)
            
            pd.DataFrame(flattened_data).to_csv(output_dir / "best_configurations.csv", index=False)
        
        # Export comparison data
        comparison_df = self.compare_algorithms()
        comparison_df.to_csv(output_dir / "algorithm_comparison.csv", index=False)
        
        logger.info(f"Exported results to {output_dir} in {format} format")


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
