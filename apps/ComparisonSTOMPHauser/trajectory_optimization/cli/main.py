#!/usr/bin/env python3
"""
Main CLI entry point for trajectory planning optimization

This script provides a unified interface for running STOMP vs Hauser+RRT
algorithm optimization and comparison with professional visualizations.
"""

import argparse
import logging
import sys
from pathlib import Path
from typing import List, Optional

# Add parent directory to path for imports
sys.path.insert(0, str(Path(__file__).parent.parent))

from core.algorithms import create_all_algorithms, STOMPAlgorithm, HauserRRTAlgorithm
from core.optimization import MultiAlgorithmOptimizer
from visualization.plots import TrajectoryOptimizationVisualizer

# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s',
    handlers=[
        logging.StreamHandler(),
        logging.FileHandler('trajectory_optimization.log')
    ]
)
logger = logging.getLogger(__name__)


def run_full_optimization(n_trials: int = 200, timeout_minutes: Optional[int] = None):
    """
    Run complete optimization across all algorithms (STOMP + Hauser+RRT variants)
    
    Args:
        n_trials: Number of optimization trials per algorithm
        timeout_minutes: Timeout per algorithm in minutes
    """
    logger.info("🚀 Starting comprehensive trajectory planning optimization")
    logger.info(f"Configuration: {n_trials} trials per algorithm")
    
    # Create optimizer with all algorithms
    algorithms = create_all_algorithms()
    optimizer = MultiAlgorithmOptimizer(
        study_name="comprehensive_trajectory_optimization",
        algorithms=algorithms
    )
    
    logger.info(f"Algorithms to optimize: {[alg.name for alg in algorithms]}")
    
    # Run optimization
    timeout_seconds = timeout_minutes * 60 if timeout_minutes else None
    studies = optimizer.optimize_all_algorithms(
        n_trials_per_algorithm=n_trials,
        timeout_per_algorithm=timeout_seconds
    )
    
    # Generate comparison report
    logger.info("📊 Generating optimization report...")
    report_content = optimizer.generate_optimization_report()
    
    # Export results
    optimizer.export_results(format="json")
    optimizer.export_results(format="csv")
    
    logger.info("✅ Optimization complete! Check the data/reports directory for results.")
    
    return optimizer, studies


def run_algorithm_comparison():
    """
    Compare existing optimization results across algorithms
    """
    logger.info("📈 Running algorithm comparison analysis")
    
    # Load existing results
    optimizer = MultiAlgorithmOptimizer()
    
    # Try to load existing studies
    try:
        comparison_df = optimizer.compare_algorithms()
        if comparison_df.empty:
            logger.warning("No existing optimization results found. Run optimization first.")
            return
        
        logger.info("Algorithm Performance Comparison:")
        logger.info(f"\n{comparison_df.to_string()}")
        
        # Generate detailed report
        report_content = optimizer.generate_optimization_report()
        logger.info("📄 Detailed report generated")
        
    except Exception as e:
        logger.error(f"Failed to load existing results: {e}")
        logger.info("Try running optimization first with: python -m cli.main optimize")


def generate_professional_visualizations():
    """
    Generate modern, interactive visualizations using Plotly
    """
    logger.info("🎨 Generating professional visualizations...")
    
    try:
        visualizer = TrajectoryOptimizationVisualizer()
        data = visualizer.load_optimization_data()
        
        if not data:
            logger.warning("No optimization data found. Run optimization first.")
            return
        
        logger.info(f"Found data for {len(data)} algorithms")
        
        # Generate dashboard
        dashboard = visualizer.create_algorithm_comparison_dashboard(data)
        
        # Save interactive HTML
        output_file = visualizer.output_dir / "algorithm_comparison_dashboard.html"
        dashboard.write_html(str(output_file))
        
        logger.info(f"✅ Interactive dashboard saved to: {output_file}")
        logger.info("Open the HTML file in your browser to explore the results!")
        
    except ImportError as e:
        logger.error(f"Missing visualization dependencies: {e}")
        logger.info("Install with: pip install plotly pandas")
    except Exception as e:
        logger.error(f"Visualization generation failed: {e}")


def run_stomp_focus():
    """
    Run optimization focused specifically on STOMP algorithm
    """
    logger.info("🎯 Running STOMP-focused optimization")
    
    stomp_alg = STOMPAlgorithm()
    optimizer = MultiAlgorithmOptimizer(
        study_name="stomp_focused_optimization",
        algorithms=[stomp_alg]
    )
    
    # Run extended optimization for STOMP
    study = optimizer.optimize_single_algorithm(stomp_alg, n_trials=400)
    
    logger.info(f"STOMP optimization complete!")
    logger.info(f"Best objective value: {study.best_value:.4f}")
    logger.info(f"Best parameters: {study.best_params}")
    
    # Generate STOMP-specific report
    report_content = optimizer.generate_optimization_report()
    
    return optimizer, study


def run_hauser_rrt_comparison():
    """
    Run comparison across all Hauser+RRT variants
    """
    logger.info("🔄 Running Hauser+RRT variant comparison")
    
    # Create all Hauser+RRT variants
    algorithms = [
        HauserRRTAlgorithm("RRT"),
        HauserRRTAlgorithm("RRT_STAR"),
        HauserRRTAlgorithm("iRRT_STAR"),
        HauserRRTAlgorithm("BiRRT")
    ]
    
    optimizer = MultiAlgorithmOptimizer(
        study_name="hauser_rrt_variants_comparison",
        algorithms=algorithms
    )
    
    # Run optimization
    studies = optimizer.optimize_all_algorithms(n_trials_per_algorithm=200)
    
    # Generate comparison
    comparison_df = optimizer.compare_algorithms()
    logger.info("Hauser+RRT Variant Comparison:")
    logger.info(f"\n{comparison_df.to_string()}")
    
    return optimizer, studies


def main():
    """Main CLI entry point"""
    parser = argparse.ArgumentParser(
        description="Professional Trajectory Planning Optimization Framework",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  python -m cli.main optimize --trials 200          # Run full optimization
  python -m cli.main compare                        # Compare existing results  
  python -m cli.main visualize                      # Generate professional plots
  python -m cli.main stomp --trials 400             # STOMP-focused optimization
  python -m cli.main hauser-rrt                     # Hauser+RRT comparison
        """
    )
    
    subparsers = parser.add_subparsers(dest='command', help='Available commands')
    
    # Optimize command
    optimize_parser = subparsers.add_parser('optimize', help='Run comprehensive optimization')
    optimize_parser.add_argument('--trials', type=int, default=200, 
                                help='Number of trials per algorithm (default: 200)')
    optimize_parser.add_argument('--timeout', type=int, 
                                help='Timeout per algorithm in minutes')
    
    # Compare command
    subparsers.add_parser('compare', help='Compare existing optimization results')
    
    # Visualize command
    subparsers.add_parser('visualize', help='Generate professional visualizations')
    
    # STOMP-focused command
    stomp_parser = subparsers.add_parser('stomp', help='STOMP-focused optimization')
    stomp_parser.add_argument('--trials', type=int, default=400,
                             help='Number of STOMP trials (default: 400)')
    
    # Hauser+RRT comparison command
    subparsers.add_parser('hauser-rrt', help='Compare Hauser+RRT variants')
    
    args = parser.parse_args()
    
    if not args.command:
        parser.print_help()
        return
    
    try:
        if args.command == 'optimize':
            run_full_optimization(args.trials, args.timeout)
        elif args.command == 'compare':
            run_algorithm_comparison()
        elif args.command == 'visualize':
            generate_professional_visualizations()
        elif args.command == 'stomp':
            run_stomp_focus()
        elif args.command == 'hauser-rrt':
            run_hauser_rrt_comparison()
        
    except KeyboardInterrupt:
        logger.info("Optimization interrupted by user")
    except Exception as e:
        logger.error(f"Command failed: {e}")
        raise


if __name__ == "__main__":
    main()
