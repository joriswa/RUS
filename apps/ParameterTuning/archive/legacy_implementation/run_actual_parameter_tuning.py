#!/usr/bin/env python3
"""
Run Enhanced Parameter Tuning with Actual C++ Evaluator

This script runs the enhanced parameter tuning system using the compiled
C++ evaluator with real scenario data and motion generation libraries.
"""

import os
import sys
import subprocess
import logging
from pathlib import Path

# Add the current directory to Python path
sys.path.append(str(Path(__file__).parent))

from enhanced_parameter_optimizer import main as optimizer_main

def setup_logging():
    """Setup logging configuration."""
    logging.basicConfig(
        level=logging.INFO,
        format='%(asctime)s - %(name)s - %(levelname)s - %(message)s',
        handlers=[
            logging.FileHandler('actual_parameter_tuning.log'),
            logging.StreamHandler()
        ]
    )
    return logging.getLogger(__name__)

def find_cpp_executable():
    """Find the compiled C++ evaluator executable."""
    script_dir = Path(__file__).parent
    project_root = script_dir.parent.parent
    
    # Look for the executable in build directory
    build_paths = [
        project_root / "build" / "apps" / "ParameterTuning" / "EnhancedParameterEvaluator",
        project_root / "build" / "EnhancedParameterEvaluator",
        script_dir / "EnhancedParameterEvaluator"
    ]
    
    for path in build_paths:
        if path.exists() and path.is_file():
            return str(path.absolute())
    
    return None

def verify_scenario_data():
    """Verify that scenario_1 data files exist."""
    script_dir = Path(__file__).parent
    project_root = script_dir.parent.parent
    scenario_dir = project_root / "res" / "scenario_1"
    
    required_files = [
        "obstacles.xml",
        "panda_US.urdf", 
        "scan_poses.csv"
    ]
    
    missing_files = []
    for file_name in required_files:
        file_path = scenario_dir / file_name
        if not file_path.exists():
            missing_files.append(str(file_path))
    
    return missing_files

def activate_virtual_environment():
    """Activate the virtual environment if it exists."""
    script_dir = Path(__file__).parent
    project_root = script_dir.parent.parent
    venv_path = project_root / "venv_parameter_tuning"
    
    if venv_path.exists():
        # Set environment variables for virtual environment
        venv_bin = venv_path / "bin"
        if venv_bin.exists():
            os.environ["VIRTUAL_ENV"] = str(venv_path)
            os.environ["PATH"] = f"{venv_bin}:{os.environ.get('PATH', '')}"
            return True
    
    return False

def install_dependencies():
    """Install required Python dependencies."""
    dependencies = [
        "numpy",
        "scipy", 
        "scikit-optimize",
        "optuna",
        "hyperopt",
        "ray[tune]",
        "matplotlib",
        "seaborn",
        "pandas",
        "pyyaml",
        "tqdm"
    ]
    
    try:
        for dep in dependencies:
            subprocess.run([sys.executable, "-m", "pip", "install", dep], 
                         check=True, capture_output=True)
        return True
    except subprocess.CalledProcessError as e:
        return False

def run_parameter_tuning(cpp_executable, algorithm="both", n_calls=100, compare_optimizers=False):
    """Run the parameter tuning with specified configuration."""
    logger = logging.getLogger(__name__)
    
    # Prepare arguments for the optimizer
    sys.argv = [
        "enhanced_parameter_optimizer.py",
        "--cpp-executable", cpp_executable,
        "--algorithm", algorithm,
        "--n-calls", str(n_calls),
        "--output-dir", "actual_tuning_results"
    ]
    
    if compare_optimizers:
        sys.argv.append("--compare-optimizers")
    
    logger.info(f"Running parameter tuning with arguments: {sys.argv[1:]}")
    
    try:
        result = optimizer_main()
        return result == 0
    except Exception as e:
        logger.error(f"Parameter tuning failed: {e}")
        return False

def main():
    """Main function."""
    logger = setup_logging()
    logger.info("Starting Actual Parameter Tuning System")
    
    # Step 1: Activate virtual environment
    if activate_virtual_environment():
        logger.info("Virtual environment activated")
    else:
        logger.warning("Virtual environment not found or could not be activated")
    
    # Step 2: Install dependencies
    logger.info("Installing/checking Python dependencies...")
    if not install_dependencies():
        logger.error("Failed to install some dependencies. Continuing anyway...")
    
    # Step 3: Find C++ executable
    logger.info("Looking for compiled C++ evaluator...")
    cpp_executable = find_cpp_executable()
    
    if not cpp_executable:
        logger.error("Could not find compiled C++ evaluator!")
        logger.error("Please make sure to build the project first:")
        logger.error("  cd PathPlanner_US_wip")
        logger.error("  mkdir -p build && cd build")
        logger.error("  cmake .. && make EnhancedParameterEvaluator")
        return 1
    
    logger.info(f"Found C++ evaluator: {cpp_executable}")
    
    # Step 4: Verify scenario data
    logger.info("Verifying scenario data files...")
    missing_files = verify_scenario_data()
    
    if missing_files:
        logger.error("Missing required scenario data files:")
        for file_path in missing_files:
            logger.error(f"  - {file_path}")
        return 1
    
    logger.info("All scenario data files found")
    
    # Step 5: Run parameter tuning
    logger.info("Starting parameter tuning with actual evaluator...")
    
    # Configuration for the tuning run
    configs = [
        {
            "name": "Quick STOMP Test",
            "algorithm": "STOMP",
            "n_calls": 20,
            "compare_optimizers": False
        },
        {
            "name": "Quick Hauser Test", 
            "algorithm": "Hauser",
            "n_calls": 20,
            "compare_optimizers": False
        },
        {
            "name": "Full Comparison",
            "algorithm": "both",
            "n_calls": 50,
            "compare_optimizers": True
        }
    ]
    
    # Start with a quick test
    logger.info("Running quick STOMP test...")
    success = run_parameter_tuning(
        cpp_executable=cpp_executable,
        algorithm="STOMP",
        n_calls=10,
        compare_optimizers=False
    )
    
    if not success:
        logger.error("Quick test failed. Please check the logs for errors.")
        return 1
    
    logger.info("Quick test successful! Running full parameter tuning...")
    
    # Run full parameter tuning
    success = run_parameter_tuning(
        cpp_executable=cpp_executable,
        algorithm="both",
        n_calls=100,
        compare_optimizers=True
    )
    
    if success:
        logger.info("Parameter tuning completed successfully!")
        logger.info("Results saved in: actual_tuning_results/")
        logger.info("Check the comprehensive report for detailed analysis.")
        return 0
    else:
        logger.error("Parameter tuning failed. Check logs for details.")
        return 1

if __name__ == "__main__":
    exit(main())