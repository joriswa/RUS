#include "USLib/USTrajectoryPlanner.h"
#include "TrajectoryLib/Logger.h"
#include <iostream>
#include <iomanip>

int main() {
    // Test configuration for trajectory timing fix
    LOG_INFO << "Testing trajectory timing continuity fix";
    
    try {
        // Initialize trajectory planner with simple environment
        std::string environment = "environment_UR10"; 
        std::vector<double> currentJoints = {0.0, -1.57, 1.57, -1.57, -1.57, 0.0, 0.0};
        
        UltrasoundScanTrajectoryPlanner planner(environment, currentJoints);
        
        // Create test poses that will likely create discontinuities
        std::vector<Eigen::Affine3d> testPoses;
        
        // Pose 1: Initial position
        Eigen::Affine3d pose1 = Eigen::Affine3d::Identity();
        pose1.translation() << 0.5, 0.2, 0.3;
        testPoses.push_back(pose1);
        
        // Pose 2: Slightly different position
        Eigen::Affine3d pose2 = Eigen::Affine3d::Identity(); 
        pose2.translation() << 0.52, 0.22, 0.32;
        testPoses.push_back(pose2);
        
        // Pose 3: Larger jump that might cause discontinuity
        Eigen::Affine3d pose3 = Eigen::Affine3d::Identity();
        pose3.translation() << 0.45, 0.15, 0.35;
        testPoses.push_back(pose3);
        
        planner.setScanPoses(testPoses);
        
        LOG_INFO << "Planning trajectories with " << testPoses.size() << " poses";
        bool success = planner.planTrajectories();
        
        if (success) {
            auto trajectories = planner.getTrajectories();
            LOG_INFO << "Successfully planned " << trajectories.size() << " trajectory segments";
            
            // Check timing continuity
            LOG_INFO << "Analyzing trajectory timing:";
            for (size_t i = 0; i < trajectories.size(); ++i) {
                const auto& segment = trajectories[i].first;
                bool isContactForce = trajectories[i].second;
                
                if (segment.empty()) {
                    LOG_INFO << "Segment " << i << ": EMPTY";
                    continue;
                }
                
                double startTime = segment.front().time;
                double endTime = segment.back().time;
                double duration = endTime - startTime;
                
                LOG_INFO << "Segment " << i << ": " 
                        << std::fixed << std::setprecision(3)
                        << startTime << "s -> " << endTime << "s"
                        << " (duration: " << duration << "s, "
                        << segment.size() << " points, "
                        << (isContactForce ? "Contact" : "Repositioning") << ")";
                
                // Check if this segment starts at 0 (as intended)
                if (std::abs(startTime) < 1e-6) {
                    LOG_INFO << "  ✓ Segment starts at t=0 as intended";
                } else {
                    LOG_WARNING << "  ✗ Segment starts at t=" << startTime << " (not 0)";
                }
            }
            
            // Check for timing gaps between segments
            LOG_INFO << "Checking timing continuity between segments:";
            for (size_t i = 0; i < trajectories.size() - 1; ++i) {
                const auto& currentSegment = trajectories[i].first;
                const auto& nextSegment = trajectories[i + 1].first;
                
                if (currentSegment.empty() || nextSegment.empty()) {
                    continue;
                }
                
                double currentEnd = currentSegment.back().time;
                double nextStart = nextSegment.front().time;
                double gap = nextStart - currentEnd;
                
                LOG_INFO << "Between segments " << i << " and " << (i + 1) 
                        << ": end=" << std::fixed << std::setprecision(3) << currentEnd 
                        << "s, start=" << nextStart << "s, gap=" << gap << "s";
                
                if (std::abs(gap - nextStart) < 1e-6) {
                    LOG_INFO << "  ✓ Next segment correctly starts at t=0 relative to its own timeline";
                } else if (gap >= 0) {
                    LOG_INFO << "  ✓ Positive gap indicates proper time continuation";
                } else {
                    LOG_WARNING << "  ✗ Negative gap indicates timing overlap";
                }
            }
            
        } else {
            LOG_ERROR << "Trajectory planning failed";
            return 1;
        }
        
    } catch (const std::exception& e) {
        LOG_ERROR << "Test failed with exception: " << e.what();
        return 1;
    }
    
    LOG_INFO << "Trajectory timing test completed successfully";
    return 0;
}
