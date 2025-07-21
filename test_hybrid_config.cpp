#include <iostream>
#include "libs/TrajectoryLib/include/TrajectoryLib/Motion/MotionGenerator.h"

int main() {
    auto config = StompConfig::hybrid();
    std::cout << "Hybrid STOMP-BiRRT Configuration:" << std::endl;
    std::cout << "  Max iterations: " << config.maxIterations << std::endl;
    std::cout << "  Time limit (ms): " << config.maxComputeTimeMs << std::endl;
    std::cout << "  Early stopping: " << (config.enableEarlyStopping ? "enabled" : "disabled") << std::endl;
    std::cout << "  Early stopping patience: " << config.earlyStoppingPatience << std::endl;
    std::cout << "  Learning rate: " << config.learningRate << std::endl;
    std::cout << "  Temperature: " << config.temperature << std::endl;
    std::cout << "  Noisy trajectories: " << config.numNoisyTrajectories << std::endl;
    return 0;
}
