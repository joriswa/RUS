#!/usr/bin/env python3
"""
Parameter Sensitivity Diagnostic Tool

This script investigates why parameter variations appear to have minimal impact
on the optimization objective. It performs systematic tests to identify issues
with parameter sensitivity, objective function computation, or C++ evaluator behavior.
"""

import json
import yaml
import subprocess
import tempfile
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import seaborn as sns
from pathlib import Path
import logging
import time
from typing import Dict, List, Tuple, Optional

def setup_logging():
    """Setup logging for diagnostic output."""
    logging.basicConfig(
        level=logging.INFO,
        format='%(asctime)s - %(levelname)s - %(message)s',
        handlers=[
            logging.FileHandler('parameter_sensitivity_diagnosis.log'),
            logging.StreamHandler()
        ]
    )
    return logging.getLogger(__name__)

class ParameterSensitivityDiagnostic:
    """Diagnostic tool for parameter sensitivity analysis."""
    
    def __init__(self, cpp_executable_path: str):
        self.logger = logging.getLogger(__name__)
        self.cpp_executable = Path(cpp_executable_path)
        self.temp_dir = Path(tempfile.mkdtemp())
        
        # Resource paths
        scenario_1_dir = Path(__file__).parent.parent.parent / "res" / "scenario_1"
        self.obstacles_file = scenario_1_dir / "obstacles.xml"
        self.urdf_file = scenario_1_dir / "panda_US.urdf"
        self.poses_file = scenario_1_dir / "scan_poses.csv"
        
        if not self.cpp_executable.exists():
            raise FileNotFoundError(f"C++ executable not found: {cpp_executable_path}")
    
    def create_simple_scenario(self) -> List[Dict]:
        """Create a minimal test scenario for sensitivity analysis."""
        start_config = [-0.785398, 0.196349, -0.196349, -1.047197, 0.0, 1.570796, 0.785398]
        
        # Single simple pose for testing
        simple_pose = {
            'position': [0.55, -0.32, 0.45],
            'orientation': [0.32, 0.051, 0.317, 0.891],  # w,x,y,z
            'contact': False,
            'distance': 0.06,
            'index': 0
        }
        
        scenario = {
            'name': 'sensitivity_test',
            'description': 'Simple scenario for parameter sensitivity testing',
            'difficulty': 1,
            'start_config': start_config,
            'target_poses': [simple_pose],
            'environment': '../../res/scenario_1/obstacles.xml',
            'urdf': '../../res/scenario_1/panda_US.urdf'
        }
        
        return [scenario]
    
    def evaluate_with_raw_output(self, algorithm: str, parameters: Dict, scenarios: List[Dict]) -> Dict:
        """Evaluate parameters and return raw C++ output for analysis."""
        config_file = self.temp_dir / f"diag_{algorithm}_{int(time.time() * 1000000)}.yaml"
        
        config_data = {
            'algorithm': algorithm,
            'parameters': parameters,
            'scenarios': scenarios,
            'evaluation_settings': {
                'num_runs_per_scenario': 5,  # More runs for better statistics
                'timeout_seconds': 30,
                'output_trajectories': False
            },
            'resources': {
                'obstacles_file': str(self.obstacles_file),
                'urdf_file': str(self.urdf_file),
                'poses_file': str(self.poses_file)
            }
        }
        
        try:
            with open(config_file, 'w') as f:
                yaml.dump(config_data, f, default_flow_style=False)
            
            result = subprocess.run([
                str(self.cpp_executable),
                '--config', str(config_file),
                '--output-format', 'json'
            ], capture_output=True, text=True, timeout=60)
            
            raw_output = {
                'returncode': result.returncode,
                'stdout': result.stdout,
                'stderr': result.stderr,
                'parameters': parameters
            }
            
            if result.returncode == 0:
                try:
                    metrics = json.loads(result.stdout)
                    raw_output['parsed_metrics'] = metrics
                except json.JSONDecodeError as e:
                    raw_output['parse_error'] = str(e)
            
            return raw_output
            
        except Exception as e:
            return {
                'error': str(e),
                'parameters': parameters
            }
        finally:
            if config_file.exists():
                config_file.unlink()
    
    def test_extreme_parameters(self, algorithm: str) -> List[Dict]:
        """Test with extreme parameter values to see if there's any sensitivity."""
        scenarios = self.create_simple_scenario()
        
        if algorithm == 'STOMP':
            # Test extreme parameter combinations
            test_configs = [
                {
                    'name': 'minimal_exploration',
                    'params': {
                        'exploration_constant': 0.001,
                        'num_noisy_trajectories': 5,
                        'num_best_samples': 3,
                        'max_iterations': 50,
                        'learning_rate': 0.1,
                        'temperature': 5.0,
                        'dt': 0.01,
                        'adaptive_sampling': False,
                        'early_termination': False
                    }
                },
                {
                    'name': 'maximal_exploration',
                    'params': {
                        'exploration_constant': 0.5,
                        'num_noisy_trajectories': 100,
                        'num_best_samples': 20,
                        'max_iterations': 1000,
                        'learning_rate': 0.7,
                        'temperature': 50.0,
                        'dt': 0.2,
                        'adaptive_sampling': True,
                        'early_termination': True
                    }
                },
                {
                    'name': 'ultra_conservative',
                    'params': {
                        'exploration_constant': 0.001,
                        'num_noisy_trajectories': 5,
                        'num_best_samples': 3,
                        'max_iterations': 50,
                        'learning_rate': 0.1,
                        'temperature': 5.0,
                        'dt': 0.01,
                        'adaptive_sampling': False,
                        'early_termination': True
                    }
                },
                {
                    'name': 'ultra_aggressive',
                    'params': {
                        'exploration_constant': 0.5,
                        'num_noisy_trajectories': 100,
                        'num_best_samples': 20,
                        'max_iterations': 1000,
                        'learning_rate': 0.7,
                        'temperature': 50.0,
                        'dt': 0.2,
                        'adaptive_sampling': True,
                        'early_termination': False
                    }
                }
            ]
        else:  # Hauser
            test_configs = [
                {
                    'name': 'minimal_deviation',
                    'params': {
                        'max_deviation': 0.1,
                        'time_step': 0.01,
                        'max_iterations': 100,
                        'tolerance': 1e-6,
                        'acceleration_limit': 0.5,
                        'velocity_limit': 0.5,
                        'interpolation_dt': 0.01
                    }
                },
                {
                    'name': 'maximal_deviation',
                    'params': {
                        'max_deviation': 2.0,
                        'time_step': 0.5,
                        'max_iterations': 2000,
                        'tolerance': 1e-3,
                        'acceleration_limit': 5.0,
                        'velocity_limit': 3.0,
                        'interpolation_dt': 0.1
                    }
                }
            ]
        
        results = []
        for config in test_configs:
            self.logger.info(f"Testing {algorithm} with {config['name']} parameters...")
            result = self.evaluate_with_raw_output(algorithm, config['params'], scenarios)
            result['config_name'] = config['name']
            results.append(result)
        
        return results
    
    def test_single_parameter_sweep(self, algorithm: str, param_name: str) -> List[Dict]:
        """Test sweeping a single parameter while keeping others fixed."""
        scenarios = self.create_simple_scenario()
        
        # Base parameters (reasonable defaults)
        if algorithm == 'STOMP':
            base_params = {
                'exploration_constant': 0.1,
                'num_noisy_trajectories': 20,
                'num_best_samples': 10,
                'max_iterations': 100,
                'learning_rate': 0.3,
                'temperature': 15.0,
                'dt': 0.1,
                'adaptive_sampling': True,
                'early_termination': True
            }
            
            # Parameter sweep ranges
            sweep_ranges = {
                'exploration_constant': np.linspace(0.001, 0.5, 10),
                'num_noisy_trajectories': np.linspace(5, 100, 10, dtype=int),
                'num_best_samples': np.linspace(3, 20, 8, dtype=int),
                'max_iterations': np.linspace(50, 1000, 10, dtype=int),
                'learning_rate': np.linspace(0.1, 0.7, 10),
                'temperature': np.linspace(5.0, 50.0, 10),
                'dt': np.linspace(0.01, 0.2, 10)
            }
        else:  # Hauser
            base_params = {
                'max_deviation': 1.0,
                'time_step': 0.1,
                'max_iterations': 500,
                'tolerance': 1e-4,
                'acceleration_limit': 2.0,
                'velocity_limit': 1.5,
                'interpolation_dt': 0.05
            }
            
            sweep_ranges = {
                'max_deviation': np.linspace(0.1, 2.0, 10),
                'time_step': np.linspace(0.01, 0.5, 10),
                'max_iterations': np.linspace(100, 2000, 10, dtype=int),
                'tolerance': np.logspace(-6, -3, 10),
                'acceleration_limit': np.linspace(0.5, 5.0, 10),
                'velocity_limit': np.linspace(0.5, 3.0, 10),
                'interpolation_dt': np.linspace(0.01, 0.1, 10)
            }
        
        if param_name not in sweep_ranges:
            self.logger.error(f"Parameter {param_name} not available for {algorithm}")
            return []
        
        results = []
        values = sweep_ranges[param_name]
        
        for i, value in enumerate(values):
            params = base_params.copy()
            params[param_name] = value
            
            self.logger.info(f"Testing {param_name}={value} ({i+1}/{len(values)})")
            result = self.evaluate_with_raw_output(algorithm, params, scenarios)
            result['swept_param'] = param_name
            result['swept_value'] = value
            results.append(result)
        
        return results
    
    def analyze_raw_metrics(self, results: List[Dict]) -> Dict:
        """Analyze raw metrics to understand where the lack of sensitivity comes from."""
        analysis = {
            'success_rates': [],
            'planning_times': [],
            'path_lengths': [],
            'smoothness_scores': [],
            'safety_scores': [],
            'composite_objectives': [],
            'raw_outputs': []
        }
        
        for result in results:
            if 'parsed_metrics' in result:
                metrics = result['parsed_metrics']
                analysis['success_rates'].append(metrics.get('success_rate', 0))
                analysis['planning_times'].append(metrics.get('avg_planning_time_ms', 0))
                analysis['path_lengths'].append(metrics.get('avg_path_length', 0))
                analysis['smoothness_scores'].append(metrics.get('avg_smoothness_score', 0))
                analysis['safety_scores'].append(metrics.get('avg_safety_score', 0))
                
                # Compute composite objective manually
                composite = self.compute_composite_objective(metrics)
                analysis['composite_objectives'].append(composite)
                
                analysis['raw_outputs'].append({
                    'params': result['parameters'],
                    'metrics': metrics,
                    'composite': composite
                })
        
        # Compute statistics
        for key in ['success_rates', 'planning_times', 'path_lengths', 'smoothness_scores', 
                   'safety_scores', 'composite_objectives']:
            values = analysis[key]
            if values:
                analysis[f'{key}_stats'] = {
                    'mean': np.mean(values),
                    'std': np.std(values),
                    'min': np.min(values),
                    'max': np.max(values),
                    'range': np.max(values) - np.min(values),
                    'cv': np.std(values) / np.mean(values) if np.mean(values) > 0 else 0
                }
        
        return analysis
    
    def compute_composite_objective(self, metrics: Dict) -> float:
        """Compute composite objective to verify calculation."""
        weights = {
            'time_weight': 0.3,
            'success_weight': 0.25,
            'path_weight': 0.2,
            'safety_weight': 0.15,
            'smoothness_weight': 0.1
        }
        
        time_penalty = min(metrics.get('avg_planning_time_ms', 10000) / 1000.0, 10.0)
        success_bonus = metrics.get('success_rate', 0.0)
        path_penalty = min(metrics.get('avg_path_length', float('inf')) / 10.0, 5.0)
        safety_bonus = metrics.get('avg_safety_score', 0.0)
        smoothness_bonus = metrics.get('avg_smoothness_score', 0.0)
        
        if metrics.get('avg_path_length', 0) == float('inf'):
            path_penalty = 5.0
        
        composite = (
            weights['time_weight'] * time_penalty +
            weights['success_weight'] * (1.0 - success_bonus) +
            weights['path_weight'] * path_penalty +
            weights['safety_weight'] * (1.0 - safety_bonus) +
            weights['smoothness_weight'] * (1.0 - smoothness_bonus)
        )
        
        return composite
    
    def create_diagnostic_plots(self, analysis: Dict, output_dir: Path, test_name: str):
        """Create diagnostic plots for sensitivity analysis."""
        output_dir.mkdir(parents=True, exist_ok=True)
        
        # Individual metric distributions
        fig, axes = plt.subplots(2, 3, figsize=(18, 12))
        axes = axes.flatten()
        
        metrics = ['success_rates', 'planning_times', 'path_lengths', 
                  'smoothness_scores', 'safety_scores', 'composite_objectives']
        titles = ['Success Rate', 'Planning Time (ms)', 'Path Length', 
                 'Smoothness Score', 'Safety Score', 'Composite Objective']
        
        for i, (metric, title) in enumerate(zip(metrics, titles)):
            if i < len(axes) and metric in analysis:
                values = analysis[metric]
                if values:
                    axes[i].hist(values, bins=20, alpha=0.7, edgecolor='black')
                    axes[i].set_title(f'{title}\nRange: {np.max(values) - np.min(values):.6f}')
                    axes[i].set_xlabel(title)
                    axes[i].set_ylabel('Frequency')
                    axes[i].grid(True, alpha=0.3)
                    
                    # Add statistics
                    stats = analysis.get(f'{metric}_stats', {})
                    stats_text = f"Mean: {stats.get('mean', 0):.6f}\nStd: {stats.get('std', 0):.6f}\nCV: {stats.get('cv', 0):.6f}"
                    axes[i].text(0.02, 0.98, stats_text, transform=axes[i].transAxes, 
                               verticalalignment='top', bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.5))
        
        plt.suptitle(f'Parameter Sensitivity Diagnostic: {test_name}', fontsize=16)
        plt.tight_layout()
        plt.savefig(output_dir / f'{test_name}_distributions.png', dpi=300, bbox_inches='tight')
        plt.close()
        
        # Create correlation matrix if we have parameter sweep data
        if 'raw_outputs' in analysis and len(analysis['raw_outputs']) > 5:
            self.create_correlation_analysis(analysis['raw_outputs'], output_dir, test_name)
    
    def create_correlation_analysis(self, raw_outputs: List[Dict], output_dir: Path, test_name: str):
        """Create correlation analysis between parameters and metrics."""
        data = []
        for output in raw_outputs:
            row = {}
            row.update(output['params'])
            row.update(output['metrics'])
            row['composite_objective'] = output['composite']
            data.append(row)
        
        df = pd.DataFrame(data)
        
        # Select numeric columns
        numeric_cols = df.select_dtypes(include=[np.number]).columns
        correlation_matrix = df[numeric_cols].corr()
        
        # Plot correlation heatmap
        plt.figure(figsize=(12, 10))
        sns.heatmap(correlation_matrix, annot=True, cmap='coolwarm', center=0,
                   square=True, fmt='.3f')
        plt.title(f'Parameter-Metric Correlation Matrix: {test_name}')
        plt.tight_layout()
        plt.savefig(output_dir / f'{test_name}_correlations.png', dpi=300, bbox_inches='tight')
        plt.close()
    
    def run_comprehensive_diagnostic(self) -> Dict:
        """Run comprehensive diagnostic analysis."""
        self.logger.info("Starting comprehensive parameter sensitivity diagnostic...")
        
        output_dir = Path("diagnostic_results")
        output_dir.mkdir(exist_ok=True)
        
        results = {}
        
        # Test 1: Extreme parameter configurations
        self.logger.info("=" * 50)
        self.logger.info("TEST 1: Extreme Parameter Configurations")
        self.logger.info("=" * 50)
        
        for algorithm in ['STOMP', 'Hauser']:
            self.logger.info(f"Testing {algorithm} with extreme parameters...")
            extreme_results = self.test_extreme_parameters(algorithm)
            analysis = self.analyze_raw_metrics(extreme_results)
            
            self.logger.info(f"{algorithm} Extreme Test Results:")
            self.logger.info(f"  Composite objective range: {analysis.get('composite_objectives_stats', {}).get('range', 0):.8f}")
            self.logger.info(f"  Success rate range: {analysis.get('success_rates_stats', {}).get('range', 0):.8f}")
            self.logger.info(f"  Planning time range: {analysis.get('planning_times_stats', {}).get('range', 0):.8f}")
            
            self.create_diagnostic_plots(analysis, output_dir, f"{algorithm}_extreme")
            results[f'{algorithm}_extreme'] = {
                'raw_results': extreme_results,
                'analysis': analysis
            }
        
        # Test 2: Single parameter sweeps
        self.logger.info("=" * 50)
        self.logger.info("TEST 2: Single Parameter Sweeps")
        self.logger.info("=" * 50)
        
        # Test most important parameters
        important_params = {
            'STOMP': ['exploration_constant', 'num_noisy_trajectories', 'learning_rate'],
            'Hauser': ['max_deviation', 'time_step', 'tolerance']
        }
        
        for algorithm in ['STOMP', 'Hauser']:
            for param in important_params[algorithm]:
                self.logger.info(f"Testing {algorithm} parameter sweep: {param}")
                sweep_results = self.test_single_parameter_sweep(algorithm, param)
                analysis = self.analyze_raw_metrics(sweep_results)
                
                self.logger.info(f"{algorithm} {param} Sweep Results:")
                self.logger.info(f"  Composite objective range: {analysis.get('composite_objectives_stats', {}).get('range', 0):.8f}")
                self.logger.info(f"  Coefficient of variation: {analysis.get('composite_objectives_stats', {}).get('cv', 0):.8f}")
                
                self.create_diagnostic_plots(analysis, output_dir, f"{algorithm}_{param}_sweep")
                results[f'{algorithm}_{param}_sweep'] = {
                    'raw_results': sweep_results,
                    'analysis': analysis
                }
        
        # Generate summary report
        self.generate_diagnostic_report(results, output_dir)
        
        return results
    
    def generate_diagnostic_report(self, results: Dict, output_dir: Path):
        """Generate comprehensive diagnostic report."""
        report_content = """# Parameter Sensitivity Diagnostic Report

## Executive Summary

This report analyzes the parameter sensitivity of trajectory planning algorithms
to determine why parameter variations appear to have minimal impact on optimization objectives.

## Test Results Summary

"""
        
        # Analyze all test results
        for test_name, test_data in results.items():
            analysis = test_data['analysis']
            obj_stats = analysis.get('composite_objectives_stats', {})
            
            report_content += f"### {test_name.replace('_', ' ').title()}\n\n"
            report_content += f"- **Objective Range**: {obj_stats.get('range', 0):.8f}\n"
            report_content += f"- **Coefficient of Variation**: {obj_stats.get('cv', 0):.8f}\n"
            report_content += f"- **Mean Objective**: {obj_stats.get('mean', 0):.6f}\n"
            report_content += f"- **Standard Deviation**: {obj_stats.get('std', 0):.8f}\n\n"
        
        # Add diagnostic conclusions
        report_content += """## Diagnostic Conclusions

### Potential Issues Identified:

1. **Low Parameter Sensitivity**: If objective ranges are < 0.001, parameters have minimal impact
2. **Metric Saturation**: If success rates are consistently 0 or planning always times out
3. **Objective Function Issues**: If individual metrics don't vary meaningfully
4. **C++ Evaluator Problems**: If raw outputs show unexpected patterns

### Recommendations:

1. **If success rates are consistently 0**: 
   - Scenarios may be too difficult or impossible
   - Robot kinematics or obstacle constraints may be preventing solutions
   - Consider simpler test scenarios

2. **If planning times are consistently at timeout**:
   - Reduce scenario complexity
   - Increase timeout values
   - Check for fundamental planning issues

3. **If objective variations are minimal but metrics vary**:
   - Objective function weights may need adjustment
   - Consider different metric combinations
   - Evaluate metric scaling

4. **If no variation in any metrics**:
   - C++ evaluator may not be properly using parameters
   - Algorithm implementations may ignore certain parameters
   - Configuration parsing may have issues

### Next Steps:

1. Review this diagnostic report and identify the primary issue
2. Implement targeted fixes based on findings
3. Re-run parameter optimization with corrected system
4. Validate that parameters now have meaningful impact

---
*Generated automatically by Parameter Sensitivity Diagnostic Tool*
"""
        
        with open(output_dir / 'diagnostic_report.md', 'w') as f:
            f.write(report_content)
        
        self.logger.info(f"Diagnostic report generated: {output_dir / 'diagnostic_report.md'}")

def main():
    """Main diagnostic function."""
    logger = setup_logging()
    logger.info("Parameter Sensitivity Diagnostic Tool")
    logger.info("=" * 50)
    
    # Find C++ evaluator
    script_dir = Path(__file__).parent
    project_root = script_dir.parent.parent
    cpp_executable = project_root / "build" / "apps" / "ParameterTuning" / "EnhancedParameterEvaluator"
    
    if not cpp_executable.exists():
        logger.error("C++ evaluator not found! Please build it first:")
        logger.error("  cd PathPlanner_US_wip/build && make EnhancedParameterEvaluator")
        return 1
    
    # Run diagnostic
    try:
        diagnostic = ParameterSensitivityDiagnostic(str(cpp_executable))
        results = diagnostic.run_comprehensive_diagnostic()
        
        logger.info("=" * 50)
        logger.info("DIAGNOSTIC COMPLETE!")
        logger.info("=" * 50)
        logger.info("Check diagnostic_results/ folder for detailed analysis")
        logger.info("Review diagnostic_report.md for conclusions and recommendations")
        
        return 0
        
    except Exception as e:
        logger.error(f"Diagnostic failed: {e}")
        return 1

if __name__ == "__main__":
    exit(main())