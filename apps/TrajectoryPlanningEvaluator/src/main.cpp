#include "trajectory_planning_evaluator.h"
#include <iostream>
#include <string>

int main() {
    try {
        std::cout << "=== TRAJECTORY PLANNING EVALUATION SYSTEM ===" << std::endl;
        std::cout << "Using ComparisonIK environment and scan poses" << std::endl;
        std::cout << "===============================================" << std::endl;
        
        // Initialize evaluator
        TrajectoryPlanningEvaluator evaluator;
        
        // Generate parameter sweep configuration with real ComparisonIK poses
        auto config = evaluator.generateParameterSweepConfig();
        
        // Load the same poses as ComparisonIK from scenario_1
        std::string poses_csv = "/Users/joris/Uni/MA/Code/PathPlanner_US_wip/res/scenario_1/scan_poses.csv";
        config.test_poses = evaluator.loadPosesFromComparisonIK(poses_csv);
        
        if (config.test_poses.empty()) {
            std::cerr << "Failed to load poses from ComparisonIK dataset" << std::endl;
            return 1;
        }
        
        std::cout << "Loaded " << config.test_poses.size() << " poses from ComparisonIK scenario" << std::endl;
        
        // Run comprehensive evaluation
        bool success = evaluator.runComprehensiveEvaluation(config);
        
        if (!success) {
            std::cerr << "Evaluation failed" << std::endl;
            return 1;
        }
        
        std::cout << "\n=== EVALUATION COMPLETE ===" << std::endl;
        std::cout << "Ready for visualization with Python scripts" << std::endl;
        std::cout << "Run: make plots" << std::endl;
        
    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        return 1;
    }
    
    return 0;
}
