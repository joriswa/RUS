#include <iostream>
#include <chrono>
#include <vector>
#include <random>
#include "USLib/USTrajectoryPlanner.h"

/**
 * @brief Simple benchmark to test parallelization optimizations
 */
int main() {
    std::cout << "Parallelization Optimization Benchmark" << std::endl;
    std::cout << "=======================================" << std::endl;
    
    // Create a simple environment string (minimal URDF)
    std::string environmentString = R"(
    <?xml version="1.0"?>
    <robot name="benchmark_robot">
        <link name="base_link">
            <visual>
                <geometry>
                    <box size="0.1 0.1 0.1"/>
                </geometry>
            </visual>
        </link>
    </robot>
    )";
    
    try {
        // Initialize the planner
        UltrasoundScanTrajectoryPlanner planner(environmentString);
        
        // Set initial joint configuration
        Eigen::VectorXd currentJoints(7);
        currentJoints << 0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785;
        planner.setCurrentJoints(currentJoints);
        
        // Create test poses for trajectory planning
        std::vector<Eigen::Affine3d> testPoses;
        std::random_device rd;
        std::mt19937 gen(rd());
        std::uniform_real_distribution<double> dis(-0.5, 0.5);
        
        // Generate random target poses
        for (int i = 0; i < 5; ++i) {
            Eigen::Affine3d pose = Eigen::Affine3d::Identity();
            pose.translation() << 0.5 + dis(gen), dis(gen), 0.3 + dis(gen);
            testPoses.push_back(pose);
        }
        
        planner.setPoses(testPoses);
        
        // Benchmark different batch sizes to test workload-aware parallelization
        std::vector<int> batchSizes = {1, 2, 4, 8, 16};
        
        for (int batchSize : batchSizes) {
            std::cout << "\nTesting batch size: " << batchSize << std::endl;
            
            // Create trajectory requests
            std::vector<std::pair<Eigen::VectorXd, Eigen::VectorXd>> requests;
            std::vector<std::string> descriptions;
            
            for (int i = 0; i < batchSize; ++i) {
                Eigen::VectorXd startJoints = currentJoints;
                Eigen::VectorXd targetJoints = currentJoints;
                
                // Add some variation to target joints
                for (int j = 0; j < 7; ++j) {
                    targetJoints[j] += dis(gen) * 0.5;
                }
                
                requests.emplace_back(startJoints, targetJoints);
                descriptions.push_back("Test trajectory " + std::to_string(i));
            }
            
            // Benchmark the planning
            auto startTime = std::chrono::high_resolution_clock::now();
            
            auto results = planner.planTrajectoryBatch(requests, descriptions, true);
            
            auto endTime = std::chrono::high_resolution_clock::now();
            auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
            
            int successCount = 0;
            for (const auto& result : results) {
                if (!result.empty()) {
                    successCount++;
                }
            }
            
            std::cout << "Results: " << successCount << "/" << batchSize 
                      << " successful in " << duration.count() << "ms" << std::endl;
            std::cout << "Throughput: " << std::fixed << std::setprecision(2)
                      << (double)successCount / (duration.count() / 1000.0) 
                      << " trajectories/sec" << std::endl;
        }
        
    } catch (const std::exception& e) {
        std::cerr << "Benchmark error: " << e.what() << std::endl;
        return 1;
    }
    
    std::cout << "\nBenchmark completed successfully!" << std::endl;
    return 0;
}