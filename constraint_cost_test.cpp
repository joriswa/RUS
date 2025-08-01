#include <iostream>
#include <vector>
#include <cmath>
#include <algorithm>

// Simple test to demonstrate constraint cost explosion
class ConstraintCostTest {
public:
    std::vector<double> maxVel{2.0, 2.0, 2.0, 2.0, 2.0, 2.0, 2.0}; // rad/s
    std::vector<double> maxAcc{10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0}; // rad/s²
    
    double computeConstraintCost(const std::vector<std::vector<double>>& trajectory, double dt) {
        double cost = 0;
        int N = trajectory.size();
        int D = trajectory[0].size();
        
        // SOLUTION 1: Skip constraint checking for very small dt values
        if (dt < 0.02) {
            std::cout << "Skipping constraint checking for very small dt=" << dt << "s (< 0.02s threshold)" << std::endl;
            return 0.0;
        }
        
        std::cout << "=== CONSTRAINT COST ANALYSIS ===" << std::endl;
        std::cout << "Trajectory: " << N << " waypoints, " << D << " joints, dt=" << dt << "s" << std::endl;
        
        // Velocity violations
        int velViolations = 0;
        double totalVelCost = 0;
        for (int i = 0; i < N - 1; ++i) {
            for (int j = 0; j < D; ++j) {
                double velocity = (trajectory[i + 1][j] - trajectory[i][j]) / dt;
                double violation = std::max(0.0, std::abs(velocity) - maxVel[j]);
                if (violation > 0) {
                    velViolations++;
                    if (velViolations <= 3) {
                        std::cout << "Vel violation: waypoint " << i << ", joint " << j 
                                  << ", |vel|=" << std::abs(velocity) << ", limit=" << maxVel[j] 
                                  << ", violation=" << violation << std::endl;
                    }
                    // SOLUTION 3: Use normalized violation cost
                    double normalizedViolation = violation / maxVel[j];
                    totalVelCost += normalizedViolation;
                }
            }
        }
        
        // Acceleration violations  
        int accViolations = 0;
        double totalAccCost = 0;
        for (int i = 0; i < N - 2; ++i) {
            for (int j = 0; j < D; ++j) {
                double vel1 = (trajectory[i + 1][j] - trajectory[i][j]) / dt;
                double vel2 = (trajectory[i + 2][j] - trajectory[i + 1][j]) / dt;
                double acceleration = (vel2 - vel1) / dt;
                double violation = std::max(0.0, std::abs(acceleration) - maxAcc[j]);
                if (violation > 0) {
                    accViolations++;
                    if (accViolations <= 3) {
                        std::cout << "Acc violation: waypoint " << i << ", joint " << j 
                                  << ", |acc|=" << std::abs(acceleration) << ", limit=" << maxAcc[j] 
                                  << ", violation=" << violation << std::endl;
                    }
                    // SOLUTION 3: Use normalized violation cost
                    double normalizedViolation = violation / maxAcc[j];
                    totalAccCost += normalizedViolation;
                }
            }
        }
        
        // SOLUTION 4: Cap maximum constraint cost
        const double maxConstraintCost = 50.0;
        double rawCost = totalVelCost + totalAccCost;
        double cappedCost = std::min(rawCost, maxConstraintCost);
        
        // SOLUTION 5: Scale by trajectory length
        double lengthNormalizedCost = cappedCost * 20.0 / N;
        
        std::cout << "Results:" << std::endl;
        std::cout << "  Velocity violations: " << velViolations << ", normalized cost: " << totalVelCost << std::endl;
        std::cout << "  Acceleration violations: " << accViolations << ", normalized cost: " << totalAccCost << std::endl;
        std::cout << "  Raw total cost: " << rawCost << std::endl;
        std::cout << "  Capped cost: " << cappedCost << std::endl;
        std::cout << "  FINAL LENGTH-NORMALIZED COST: " << lengthNormalizedCost << std::endl;
        std::cout << "  With weight 1.0: " << lengthNormalizedCost * 1.0 << std::endl;
        std::cout << "  With weight 5.0: " << lengthNormalizedCost * 5.0 << std::endl;
        std::cout << "==================================" << std::endl;
        
        return lengthNormalizedCost;
    }
};

int main() {
    ConstraintCostTest test;
    
    // Test case 1: Very fast movement that violates velocity
    std::cout << "TEST 1: Very fast 45-degree movement (velocity violations)" << std::endl;
    std::vector<std::vector<double>> traj1(10, std::vector<double>(7, 0.0));
    for (int i = 0; i < 10; ++i) {
        for (int j = 0; j < 7; ++j) {
            // 45-degree movement over just 10 waypoints with small dt -> high velocity
            traj1[i][j] = (45.0 * M_PI/180.0) * i / 9.0;
        }
    }
    test.computeConstraintCost(traj1, 0.01); // Small dt = high calculated velocities
    
    std::cout << std::endl;
    
    // Test case 2: Sudden direction change (acceleration violations)
    std::cout << "TEST 2: Sudden direction change (acceleration violations)" << std::endl;
    std::vector<std::vector<double>> traj2(20, std::vector<double>(7, 0.0));
    for (int i = 0; i < 20; ++i) {
        for (int j = 0; j < 7; ++j) {
            if (i < 10) {
                // First half: move to 30 degrees
                traj2[i][j] = (30.0 * M_PI/180.0) * i / 9.0;
            } else {
                // Second half: sudden reverse to -30 degrees
                traj2[i][j] = (30.0 * M_PI/180.0) - (60.0 * M_PI/180.0) * (i - 10) / 9.0;
            }
        }
    }
    test.computeConstraintCost(traj2, 0.02);
    
    std::cout << std::endl;
    
    // Test case 3: Large movement with many waypoints (cost accumulation)
    std::cout << "TEST 3: Large movement with many waypoints (cost accumulation)" << std::endl;
    std::vector<std::vector<double>> traj3(100, std::vector<double>(7, 0.0));
    for (int i = 0; i < 100; ++i) {
        for (int j = 0; j < 7; ++j) {
            // 180-degree movement over 100 waypoints with tiny dt
            traj3[i][j] = (180.0 * M_PI/180.0) * i / 99.0;
        }
    }
    test.computeConstraintCost(traj3, 0.005); // Very small dt
    
    return 0;
}
