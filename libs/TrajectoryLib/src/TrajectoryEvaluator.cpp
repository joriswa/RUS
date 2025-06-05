#include "TrajectoryLib/TrajectoryEvaluator.h"
#include <fstream>
#include <sstream>

TrajectoryEvaluator::TrajectoryEvaluator(
    const RobotArm &startArm,
    const std::vector<std::pair<double, std::vector<double>>> &trajectory,
    std::shared_ptr<BVHTree> obstacleTree)
    : _startArm(startArm)
    , _trajectory(trajectory)
    , _obstacleTree(obstacleTree)
{
    
}

std::vector<double> TrajectoryEvaluator::computeJointDistancesOverTime() const
{
    std::vector<double> jointDistances;

    if (_trajectory.size() < 2) {
        return jointDistances;
    }

    for (size_t i = 1; i < _trajectory.size(); ++i) {
        Eigen::VectorXd currentAngles
            = Eigen::Map<const Eigen::VectorXd>(_trajectory[i].second.data(),
                                                _trajectory[i].second.size());
        Eigen::VectorXd previousAngles
            = Eigen::Map<const Eigen::VectorXd>(_trajectory[i - 1].second.data(),
                                                _trajectory[i - 1].second.size());

        double distance = (currentAngles - previousAngles).norm();
        jointDistances.push_back(distance);
    }

    return jointDistances;
}

// Compute displacements over time
std::vector<double> TrajectoryEvaluator::computeDisplacementOverTime()
{
    std::vector<double> displacements;

    if (_trajectory.size() < 2) {
        return displacements;
    }

    RobotArm currentArm = _startArm;
    RobotArm previousArm = _startArm;

    for (size_t i = 1; i < _trajectory.size(); ++i) {
        currentArm.setJointAngles(Eigen::Map<const Eigen::VectorXd>(_trajectory[i].second.data(),
                                                                    _trajectory[i].second.size()));
        previousArm.setJointAngles(
            Eigen::Map<const Eigen::VectorXd>(_trajectory[i - 1].second.data(),
                                              _trajectory[i - 1].second.size()));

        const auto &curBBoxes = currentArm.getLinkBoundingBoxes();
        const auto &prevBBoxes = previousArm.getLinkBoundingBoxes();

        double maxDist = 0.0;

        for (size_t j = 0; j < curBBoxes.size(); ++j) {
            Eigen::Vector3d curCenter = std::get<0>(curBBoxes[j]);
            Eigen::Vector3d prevCenter = std::get<0>(prevBBoxes[j]);
            double dist = (curCenter - prevCenter).norm();
            maxDist = std::max(maxDist, dist);
        }

        displacements.push_back(maxDist);
    }

    return displacements;
}

// Compute distance to the environment over time
std::vector<double> TrajectoryEvaluator::computeDistanceToEnvironmentOverTime()
{
    std::vector<double> distancesToEnvironment;

    if (_trajectory.size() < 2) {
        return distancesToEnvironment;
    }

    RobotArm arm = _startArm;

    for (size_t i = 0; i < _trajectory.size(); ++i) {
        arm.setJointAngles(Eigen::Map<const Eigen::VectorXd>(_trajectory[i].second.data(),
                                                             _trajectory[i].second.size()));
        const auto &bboxes = arm.getLinkBoundingBoxes();

        double minDist = std::numeric_limits<double>::infinity();

        for (const auto &bbox : bboxes) {
            Eigen::Vector3d center = std::get<0>(bbox);
            double dist = std::get<0>(_obstacleTree->getDistanceAndClosestPoint(center));
            minDist = std::min(minDist, dist);
        }

        distancesToEnvironment.push_back(minDist);
    }

    return distancesToEnvironment;
}

// Save the evaluation data (joint distances, displacements, and environment distances) to a CSV file
void TrajectoryEvaluator::saveMetricsToCSV(const std::string &filename)
{
    std::vector<double> jointDistances = computeJointDistancesOverTime();
    std::vector<double> displacements = computeDisplacementOverTime();
    std::vector<double> distancesToEnvironment = computeDistanceToEnvironmentOverTime();

    // Open file for writing
    std::ofstream file(filename);

    if (file.is_open()) {
        // Write CSV header
        file << "Time,JointDistance,Displacement,DistanceToEnvironment\n";

        // Iterate through the trajectory and write each step
        for (size_t i = 0; i < _trajectory.size(); ++i) {
            double time = _trajectory[i].first;
            double jointDistance = i < jointDistances.size() ? jointDistances[i] : 0.0;
            double displacement = i < displacements.size() ? displacements[i] : 0.0;
            double distanceToEnvironment = i < distancesToEnvironment.size()
                                               ? distancesToEnvironment[i]
                                               : 0.0;

            // Write the data for this step
            file << time << "," << jointDistance << "," << displacement << ","
                 << distanceToEnvironment << "\n";
        }

        // Close the file
        file.close();
    } else {
        throw std::runtime_error("Unable to open file for writing");
    }
}
