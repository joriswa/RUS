#!/usr/bin/env python3
"""
Smart Parameter Optimization Demo
=================================

This script demonstrates the complete smart multi-objective parameter
optimization pipeline for STOMP trajectory planning.

Usage:
    python3 demo_smart_optimization.py [--quick-demo]
"""

import argparse
import logging
import json
import yaml
import numpy as np
from pathlib import Path
import matplotlib.pyplot as plt

# Import our optimization components
try:
    from scenario_analyzer import ScenarioAnalyzer, ScenarioFeatures
    from adaptive_parameter_selector import AdaptiveParameterSelector
    # Import smart_optimizer conditionally to handle missing dependencies
    try:
        from smart_optimizer import SmartParameterOptimizer
        SMART_OPTIMIZER_AVAILABLE = True
    except ImportError as e:
        SMART_OPTIMIZER_AVAILABLE = False
        print(f"Note: Full smart optimizer not available: {e}")
        print("Demo will run with reduced functionality")
except ImportError as e:
    print(f"Warning: Some components not available: {e}")
    # Create dummy classes for demo purposes
    class ScenarioFeatures:
        def __init__(self, **kwargs):
            for k, v in kwargs.items():
                setattr(self, k, v)
        def to_vector(self):
            return np.array([0.3, 0.6, 0.25, 0.8, 0.4, 2.5, 1.2, 1.8, 0.7, 8.5, 0.8, 2.1, 1.5, 0.4, 0.3])
    
    class ScenarioAnalyzer:
        def analyze_scenario(self, *args):
            return create_demo_features()
    
    class AdaptiveParameterSelector:
        def predict_parameters(self, features):
            class DummyPrediction:
                def __init__(self):
                    self.num_noisy_trajectories = 22
                    self.num_best_samples = 17
                    self.max_iterations = 108
                    self.learning_rate = 0.354
                    self.temperature = 28.27
                    self.dt = 0.092
                    self.velocity_limit = 1.93
                    self.acceleration_limit = 0.523
                    self.confidence_scores = {'learning_rate': 0.8, 'temperature': 0.7}
                    self.prediction_method = 'heuristic_default'
                    self.scenario_similarity = 0.5
            return DummyPrediction()
        
        def generate_parameter_candidates(self, features, n_candidates=3):
            return [self.predict_parameters(features) for _ in range(n_candidates)]

def create_demo_scenario():
    """Create a demonstration scenario for testing"""
    
    demo_scenario = {
        'name': 'demo_knee_ultrasound',
        'urdf_file': '../../res/scenario_1/panda_US.urdf',
        'environment_file': '../../res/scenario_1/obstacles.xml', 
        'poses_file': '../../res/scenario_1/scan_poses.csv',
        'description': 'Demonstration knee ultrasound scanning scenario',
        'complexity_level': 'medium'
    }
    
    return demo_scenario

def create_demo_features():
    """Create demonstration scenario features"""
    
    features = ScenarioFeatures(
        obstacle_density=0.3,
        obstacle_clustering=0.6,
        workspace_coverage=0.25,
        avg_obstacle_size=0.8,
        min_corridor_width=0.4,
        path_length=2.5,
        pose_diversity=1.2,
        orientation_complexity=1.8,
        joint_range_utilization=0.7,
        workspace_volume=8.5,
        goal_reachability=0.8,
        kinematic_complexity=2.1,
        start_goal_distance=1.5,
        configuration_space_obstacles=0.4,
        manipulability_variation=0.3
    )
    
    return features

def demo_scenario_analysis():
    """Demonstrate scenario feature extraction"""
    
    print("\n" + "="*60)
    print("DEMO: SCENARIO FEATURE EXTRACTION")
    print("="*60)
    
    # Create demo features (in practice would extract from actual scenario)
    features = create_demo_features()
    
    print(f"Extracted Scenario Features:")
    print(f"- Obstacle density: {features.obstacle_density:.3f}")
    print(f"- Workspace coverage: {features.workspace_coverage:.3f}")
    print(f"- Path length: {features.path_length:.3f}")
    print(f"- Orientation complexity: {features.orientation_complexity:.3f}")
    print(f"- Kinematic complexity: {features.kinematic_complexity:.3f}")
    
    # Show feature vector
    feature_vector = features.to_vector()
    print(f"\nFeature vector (15 dimensions): {feature_vector[:5]}... (truncated)")
    
    return features

def demo_ml_prediction(features):
    """Demonstrate ML-based parameter prediction"""
    
    print("\n" + "="*60)
    print("DEMO: ML-BASED PARAMETER PREDICTION")
    print("="*60)
    
    # Initialize adaptive selector
    selector = AdaptiveParameterSelector(model_dir="demo_models")
    
    # Generate prediction (will use defaults since no trained models)
    prediction = selector.predict_parameters(features)
    
    print(f"ML Predicted Parameters:")
    print(f"- Noisy trajectories: {prediction.num_noisy_trajectories}")
    print(f"- Best samples: {prediction.num_best_samples}")
    print(f"- Max iterations: {prediction.max_iterations}")
    print(f"- Learning rate: {prediction.learning_rate:.3f}")
    print(f"- Temperature: {prediction.temperature:.2f}")
    print(f"- Time step (dt): {prediction.dt:.3f}")
    print(f"- Velocity limit: {prediction.velocity_limit:.2f}")
    print(f"- Acceleration limit: {prediction.acceleration_limit:.3f}")
    
    print(f"\nPrediction confidence: {np.mean(list(prediction.confidence_scores.values())):.3f}")
    print(f"Prediction method: {prediction.prediction_method}")
    print(f"Scenario similarity: {prediction.scenario_similarity:.3f}")
    
    return prediction

def demo_parameter_candidates(features):
    """Demonstrate parameter candidate generation"""
    
    print("\n" + "="*60)
    print("DEMO: PARAMETER CANDIDATE GENERATION")
    print("="*60)
    
    selector = AdaptiveParameterSelector()
    candidates = selector.generate_parameter_candidates(features, n_candidates=3)
    
    print(f"Generated {len(candidates)} parameter candidates:")
    
    for i, candidate in enumerate(candidates):
        print(f"\nCandidate {i+1} ({candidate.prediction_method}):")
        print(f"  Noisy trajectories: {candidate.num_noisy_trajectories}")
        print(f"  Learning rate: {candidate.learning_rate:.3f}")
        print(f"  Temperature: {candidate.temperature:.2f}")
    
    return candidates

def demo_multi_objective_analysis():
    """Demonstrate multi-objective analysis concepts"""
    
    print("\n" + "="*60)
    print("DEMO: MULTI-OBJECTIVE ANALYSIS CONCEPTS")
    print("="*60)
    
    # Simulate Pareto front data
    np.random.seed(42)
    n_solutions = 20
    
    # Generate realistic objective values
    planning_times = np.random.uniform(1.0, 10.0, n_solutions)
    trajectory_qualities = np.random.uniform(0.3, 1.0, n_solutions)
    energy_consumptions = np.random.uniform(10, 100, n_solutions)
    collision_margins = np.random.uniform(0.1, 0.8, n_solutions)
    success_rates = np.random.uniform(0.7, 1.0, n_solutions)
    
    print("Multi-Objective Trade-offs:")
    print(f"Planning time range: {planning_times.min():.1f} - {planning_times.max():.1f} seconds")
    print(f"Trajectory quality range: {trajectory_qualities.min():.2f} - {trajectory_qualities.max():.2f}")
    print(f"Energy consumption range: {energy_consumptions.min():.1f} - {energy_consumptions.max():.1f}")
    print(f"Safety margin range: {collision_margins.min():.2f} - {collision_margins.max():.2f}")
    print(f"Success rate range: {success_rates.min():.2f} - {success_rates.max():.2f}")
    
    # Show trade-off analysis
    print(f"\nTrade-off Analysis:")
    time_quality_corr = np.corrcoef(planning_times, trajectory_qualities)[0, 1]
    energy_quality_corr = np.corrcoef(energy_consumptions, trajectory_qualities)[0, 1]
    
    print(f"Planning time vs Quality correlation: {time_quality_corr:.3f}")
    print(f"Energy vs Quality correlation: {energy_quality_corr:.3f}")
    
    if time_quality_corr < -0.3:
        print("→ Faster planning often reduces trajectory quality")
    if energy_quality_corr < -0.3:
        print("→ Lower energy consumption may compromise quality")
    
    return {
        'planning_times': planning_times,
        'trajectory_qualities': trajectory_qualities,
        'energy_consumptions': energy_consumptions,
        'collision_margins': collision_margins,
        'success_rates': success_rates
    }

def demo_adaptive_learning():
    """Demonstrate adaptive learning concepts"""
    
    print("\n" + "="*60)
    print("DEMO: ADAPTIVE LEARNING CONCEPTS")
    print("="*60)
    
    print("Adaptive Learning Pipeline:")
    print("1. Extract scenario features → Characterize optimization problem")
    print("2. Predict parameters with ML → Initialize with smart guesses")
    print("3. Run multi-objective optimization → Find Pareto-optimal solutions")
    print("4. Analyze results → Learn parameter-performance relationships")
    print("5. Update ML models → Improve future predictions")
    
    print("\nKey Benefits:")
    print("✓ Faster convergence by starting with good parameters")
    print("✓ Scenario-specific optimization strategies")
    print("✓ Continuous improvement through experience")
    print("✓ Multi-objective trade-off analysis")
    print("✓ Robust parameter selection across diverse scenarios")
    
    # Simulate learning improvement
    baseline_time = 120  # seconds for random parameter search
    ml_time = 45        # seconds with ML-guided optimization
    
    improvement = (baseline_time - ml_time) / baseline_time * 100
    print(f"\nExample Performance Improvement:")
    print(f"- Baseline (random search): {baseline_time} seconds")
    print(f"- ML-guided optimization: {ml_time} seconds")
    print(f"- Improvement: {improvement:.1f}% faster convergence")

def demo_visualization(objective_data):
    """Create demonstration visualizations"""
    
    print("\n" + "="*60)
    print("DEMO: MULTI-OBJECTIVE VISUALIZATION")
    print("="*60)
    
    try:
        # Create 2D Pareto front visualization
        fig, axes = plt.subplots(1, 2, figsize=(12, 5))
        
        # Plot 1: Planning time vs Quality
        axes[0].scatter(objective_data['planning_times'], 
                       objective_data['trajectory_qualities'],
                       c=objective_data['energy_consumptions'], 
                       cmap='viridis', alpha=0.7)
        axes[0].set_xlabel('Planning Time (seconds)')
        axes[0].set_ylabel('Trajectory Quality')
        axes[0].set_title('Pareto Front: Time vs Quality')
        axes[0].grid(True, alpha=0.3)
        
        # Add colorbar
        cbar = plt.colorbar(axes[0].collections[0], ax=axes[0])
        cbar.set_label('Energy Consumption')
        
        # Plot 2: Energy vs Safety
        axes[1].scatter(objective_data['energy_consumptions'],
                       objective_data['collision_margins'],
                       c=objective_data['success_rates'],
                       cmap='plasma', alpha=0.7)
        axes[1].set_xlabel('Energy Consumption')
        axes[1].set_ylabel('Collision Margin (Safety)')
        axes[1].set_title('Energy vs Safety Trade-off')
        axes[1].grid(True, alpha=0.3)
        
        # Add colorbar
        cbar2 = plt.colorbar(axes[1].collections[0], ax=axes[1])
        cbar2.set_label('Success Rate')
        
        plt.tight_layout()
        
        # Save plot
        output_dir = Path("results/demo_plots")
        output_dir.mkdir(parents=True, exist_ok=True)
        
        plt.savefig(output_dir / "demo_pareto_front.png", dpi=150, bbox_inches='tight')
        print(f"Visualization saved to: {output_dir / 'demo_pareto_front.png'}")
        
        plt.close()
        
    except Exception as e:
        print(f"Visualization failed (optional): {e}")

def quick_demo():
    """Run a quick demonstration of key concepts"""
    
    print("🚀 SMART MULTI-OBJECTIVE STOMP PARAMETER OPTIMIZATION DEMO")
    print("="*65)
    
    # Demo 1: Scenario Analysis
    features = demo_scenario_analysis()
    
    # Demo 2: ML Prediction
    prediction = demo_ml_prediction(features)
    
    # Demo 3: Parameter Candidates
    candidates = demo_parameter_candidates(features)
    
    # Demo 4: Multi-objective Analysis
    objective_data = demo_multi_objective_analysis()
    
    # Demo 5: Adaptive Learning
    demo_adaptive_learning()
    
    # Demo 6: Visualization
    demo_visualization(objective_data)
    
    print("\n" + "="*65)
    print("🎯 DEMO COMPLETED SUCCESSFULLY!")
    print("="*65)
    print("\nNext Steps:")
    print("1. Set up the system: ./setup_smart_optimization.sh")
    print("2. Build C++ evaluator: make -j4 in build directory")
    print("3. Run full optimization: python3 smart_optimizer.py --config config/smart_optimizer_config.yaml --scenarios config/scenarios_config.yaml")
    print("\nKey innovations implemented:")
    print("✓ Multi-objective optimization with NSGA-II")
    print("✓ ML-based adaptive parameter prediction")
    print("✓ Scenario complexity analysis")
    print("✓ Pareto-optimal solution selection")
    print("✓ Continuous learning and improvement")

def full_demo():
    """Run a comprehensive demonstration"""
    
    print("🔬 COMPREHENSIVE SMART OPTIMIZATION DEMO")
    print("="*65)
    
    # Run quick demo first
    quick_demo()
    
    # Additional comprehensive demos
    print("\n" + "="*60)
    print("ADDITIONAL ANALYSIS")
    print("="*60)
    
    # Demonstrate system configuration
    print("\nSystem Configuration:")
    print("- Multi-objective algorithms: NSGA-II, LNSGA-II")
    print("- ML models: Random Forest, Gradient Boosting, Neural Networks")
    print("- Scenario features: 15-dimensional feature space")
    print("- Parameter space: 8-dimensional optimization")
    print("- Objectives: 5 competing optimization goals")
    
    # Show integration benefits
    print("\nIntegration Benefits:")
    print("1. Scenario-aware optimization: Parameters adapt to problem complexity")
    print("2. Multi-objective awareness: Balance competing goals explicitly")
    print("3. Learning from experience: Performance improves over time")
    print("4. Robust parameter selection: Works across diverse scenarios")
    print("5. Interpretable results: Clear trade-off analysis")

def main():
    parser = argparse.ArgumentParser(description='Smart Parameter Optimization Demo')
    parser.add_argument('--quick-demo', action='store_true', 
                       help='Run quick demonstration of key concepts')
    parser.add_argument('--full-demo', action='store_true',
                       help='Run comprehensive demonstration')
    parser.add_argument('--verbose', action='store_true', help='Verbose output')
    
    args = parser.parse_args()
    
    # Setup logging
    level = logging.INFO if args.verbose else logging.WARNING
    logging.basicConfig(level=level, format='%(asctime)s - %(levelname)s - %(message)s')
    
    try:
        if args.full_demo:
            full_demo()
        else:
            quick_demo()
            
        return 0
        
    except Exception as e:
        print(f"Demo failed: {e}")
        return 1

if __name__ == '__main__':
    exit(main())
