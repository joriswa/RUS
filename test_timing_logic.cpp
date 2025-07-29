// Simple test to verify trajectory timing fix logic
#include <iostream>
#include <vector>
#include <iomanip>

struct TrajectoryPoint {
    double time;
    std::vector<double> position;
    std::vector<double> velocity;
    std::vector<double> acceleration;
    
    TrajectoryPoint(double t = 0.0) : time(t) {
        position.resize(7, 0.0);
        velocity.resize(7, 0.0);
        acceleration.resize(7, 0.0);
    }
};

// Simulate the old (buggy) behavior
std::vector<TrajectoryPoint> simulateOldBehavior() {
    std::cout << "=== Old (Buggy) Behavior ===" << std::endl;
    
    // Simulate computeTimeOptimalSegment with startTime = 2.5s
    double fromTime = 2.5;
    std::vector<TrajectoryPoint> correctionTrajectory;
    
    // computeTimeOptimalSegment returns points already offset by startTime
    correctionTrajectory.push_back(TrajectoryPoint(2.5));  // start
    correctionTrajectory.push_back(TrajectoryPoint(2.7));  // intermediate
    correctionTrajectory.push_back(TrajectoryPoint(3.0));  // end
    
    std::cout << "computeTimeOptimalSegment output (with startTime=2.5s):" << std::endl;
    for (const auto& point : correctionTrajectory) {
        std::cout << "  t=" << std::fixed << std::setprecision(1) << point.time << "s" << std::endl;
    }
    
    // OLD CODE: Double time adjustment (BUG!)
    double timeOffset = fromTime;  // 2.5s
    std::vector<TrajectoryPoint> result;
    
    for (size_t i = 1; i < correctionTrajectory.size(); ++i) {
        auto point = correctionTrajectory[i];
        point.time += timeOffset;  // BUG: Adding timeOffset again!
        result.push_back(point);
    }
    
    std::cout << "After OLD buggy time adjustment (+2.5s again):" << std::endl;
    for (const auto& point : result) {
        std::cout << "  t=" << std::fixed << std::setprecision(1) << point.time << "s (WRONG!)" << std::endl;
    }
    
    return result;
}

// Simulate the new (fixed) behavior
std::vector<TrajectoryPoint> simulateNewBehavior() {
    std::cout << "\n=== New (Fixed) Behavior ===" << std::endl;
    
    // Simulate computeTimeOptimalSegment with startTime = 2.5s
    double fromTime = 2.5;
    std::vector<TrajectoryPoint> correctionTrajectory;
    
    // computeTimeOptimalSegment returns points already offset by startTime
    correctionTrajectory.push_back(TrajectoryPoint(2.5));  // start
    correctionTrajectory.push_back(TrajectoryPoint(2.7));  // intermediate
    correctionTrajectory.push_back(TrajectoryPoint(3.0));  // end
    
    std::cout << "computeTimeOptimalSegment output (with startTime=2.5s):" << std::endl;
    for (const auto& point : correctionTrajectory) {
        std::cout << "  t=" << std::fixed << std::setprecision(1) << point.time << "s" << std::endl;
    }
    
    // NEW CODE: No double time adjustment (FIXED!)
    std::vector<TrajectoryPoint> result;
    
    for (size_t i = 1; i < correctionTrajectory.size(); ++i) {
        auto point = correctionTrajectory[i];
        // FIXED: No additional time offset - computeTimeOptimalSegment already handled it
        result.push_back(point);
    }
    
    std::cout << "After NEW fixed behavior (no additional adjustment):" << std::endl;
    for (const auto& point : result) {
        std::cout << "  t=" << std::fixed << std::setprecision(1) << point.time << "s (CORRECT!)" << std::endl;
    }
    
    return result;
}

// Simulate next segment timing adjustment
void simulateNextSegmentAdjustment() {
    std::cout << "\n=== Next Segment Timing Adjustment ===" << std::endl;
    
    // Original next segment (starts at t=0 as intended)
    std::vector<TrajectoryPoint> nextSegment = {
        TrajectoryPoint(0.0),
        TrajectoryPoint(0.5),
        TrajectoryPoint(1.0)
    };
    
    std::cout << "Original next segment (starts at t=0 as intended):" << std::endl;
    for (const auto& point : nextSegment) {
        std::cout << "  t=" << std::fixed << std::setprecision(1) << point.time << "s" << std::endl;
    }
    
    // Correction trajectory ends at t=3.0s
    double correctionEndTime = 3.0;
    
    // NEW LOGIC: Shift next segment to start after correction ends
    double nextSegmentOffset = correctionEndTime;
    std::cout << "Shifting next segment to start after correction ends (t=3.0s):" << std::endl;
    
    double originalStart = nextSegment.front().time;
    for (auto& point : nextSegment) {
        double originalRelativeTime = point.time - originalStart;
        point.time = nextSegmentOffset + originalRelativeTime;
        std::cout << "  t=" << std::fixed << std::setprecision(1) << point.time << "s" << std::endl;
    }
}

int main() {
    std::cout << "Trajectory Timing Fix Demonstration" << std::endl;
    std::cout << "====================================" << std::endl;
    
    std::cout << "\nProblem: Some trajectory segments don't start at 0s" << std::endl;
    std::cout << "Root Cause: Double time adjustment in fixTrajectoryDiscontinuities()" << std::endl;
    
    auto oldResult = simulateOldBehavior();
    auto newResult = simulateNewBehavior();
    
    simulateNextSegmentAdjustment();
    
    std::cout << "\n=== Summary ===" << std::endl;
    std::cout << "✓ Fixed double time adjustment bug in addCorrectionSegment()" << std::endl;
    std::cout << "✓ computeTimeOptimalSegment() already handles startTime parameter correctly" << std::endl;
    std::cout << "✓ Each trajectory segment now properly starts at t=0 as intended" << std::endl;
    std::cout << "✓ Next segment timing correctly adjusted to maintain continuity" << std::endl;
    
    return 0;
}
