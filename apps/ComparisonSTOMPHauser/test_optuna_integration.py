#!/usr/bin/env python3
"""
Test the Optuna integration with the simple configurable comparison tool
"""

import os
import sys
import tempfile
from pathlib import Path

# Add the current directory to Python path to import optuna_optimizer
sys.path.append('/Users/joris/Uni/MA/Code/PathPlanner_US_wip/apps/ComparisonSTOMPHauser')

from optuna_optimizer import Algorithm, OptimizationConfig, TrajectoryOptimizer

def test_simple_integration():
    """Test the basic integration between Optuna and the C++ tool"""
    
    # Path to the built executable
    executable_path = "/Users/joris/Uni/MA/Code/PathPlanner_US_wip/build/apps/ComparisonSTOMPHauser/SimpleConfigurableComparison"
    
    if not Path(executable_path).exists():
        print(f"❌ Executable not found: {executable_path}")
        print("Please build the SimpleConfigurableComparison executable first.")
        return False
    
    print(f"✓ Found executable: {executable_path}")
    
    # Create a simple test configuration
    config = OptimizationConfig(
        algorithm=Algorithm.STOMP,
        n_trials=3,
        study_name="test_integration"
    )
    
    # Create results directory
    results_dir = Path("/tmp/optuna_test")
    results_dir.mkdir(parents=True, exist_ok=True)
    
    print("✓ Configuration created")
    print(f"  Algorithm: {config.algorithm.value}")
    print(f"  Results dir: {results_dir}")
    print(f"  Trials: {config.n_trials}")
    
    # Initialize optimizer
    try:
        optimizer = TrajectoryOptimizer(config)
        print("✓ Optimizer initialized")
        
        # Run optimization
        print("🚀 Starting optimization...")
        study = optimizer.optimize()
        
        print(f"✅ Optimization completed!")
        print(f"  Best trial: {study.best_trial.number}")
        print(f"  Best value: {study.best_value:.4f}")
        print(f"  Best params: {study.best_params}")
        
        return True
        
    except Exception as e:
        print(f"❌ Error during optimization: {e}")
        import traceback
        traceback.print_exc()
        return False

if __name__ == "__main__":
    print("🧪 Testing Optuna integration with Simple Configurable Comparison")
    print("=" * 60)
    
    success = test_simple_integration()
    
    if success:
        print("\n🎉 Test completed successfully!")
        print("You can now run the full optimization with:")
        print("python optuna_optimizer.py --executable /path/to/SimpleConfigurableComparison --algorithm stomp --trials 50")
    else:
        print("\n💥 Test failed. Please check the error messages above.")
