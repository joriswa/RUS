#include <iostream>

#include "TrajectoryLib/MotionGenerator.h"
#include "TrajectoryLib/PathPlanner.h"
#include "TrajectoryLib/RobotArm.h"
#include "TrajectoryLib/RobotManager.h"

#include <QDebug>

namespace POS1 {
Eigen::Vector3d pos = Eigen::Vector3d(0.3, 0.3, 0.57);
Eigen::Matrix3d Rx = Eigen::AngleAxisd(0, Eigen::Vector3d::UnitX()).toRotationMatrix();
Eigen::Matrix3d Ry = Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitY()).toRotationMatrix();
Eigen::Matrix3d Rz = Eigen::AngleAxisd(M_PI_2, Eigen::Vector3d::UnitZ()).toRotationMatrix();
Eigen::Matrix3d orientation = Rx * Ry * Rz;
} // namespace POS1

namespace POS2 {
Eigen::Vector3d pos = Eigen::Vector3d(0.475, 0.325, 0.38);
Eigen::Matrix3d Rx = Eigen::AngleAxisd(0, Eigen::Vector3d::UnitX()).toRotationMatrix();
Eigen::Matrix3d Ry = Eigen::AngleAxisd(0, Eigen::Vector3d::UnitY()).toRotationMatrix();
Eigen::Matrix3d Rz = Eigen::AngleAxisd(M_PI_2, Eigen::Vector3d::UnitZ()).toRotationMatrix();
Eigen::Matrix3d orientation = Rx * Ry * Rz;
} // namespace POS2

std::vector<double> stompParams{0.01, 0.02, 0.05, 0.1, 0.2, 0.4, 0.8, 1.0};

std::vector<Params> buildParams()
{
    std::vector<double> stepSizes{0.01, 0.05, 0.1, 0.25};
    std::vector<double> goalBiases{0.05, 0.1, 0.25, 0.5};
    std::vector<int> maxIterations{1000, 5000, 10000};
    std::vector<Algorithm> algorithms{Algorithm::RRT,
                                      Algorithm::RRTConnect,
                                      Algorithm::RRTStar,
                                      Algorithm::InformedRRTStar};

    std::vector<Params> params;

    for (const auto &stepSize : stepSizes) {
        for (const auto &goalBias : goalBiases) {
            for (const auto &algorithm : algorithms) {
                for (const auto &maxIter : maxIterations) {
                    Params p;
                    p.stepSize = stepSize;
                    p.goalBiasProbability = goalBias;
                    p.algo = algorithm;
                    p.maxIterations = maxIter;
                    params.push_back(p);
                }
            }
        }
    }

    return params;
}
void profileSTOMP()
{
    RobotManager robotManager;
    robotManager.parseURDF("/Users/joris/Uni/MA/Code/PathPlanner_US/res/test.xml");

    std::shared_ptr<BVHTree> tree = std::make_shared<BVHTree>(
        robotManager.getTransformedObstacles());

    RobotArm arm("/Users/joris/Uni/MA/Code/PathPlanner_US/res/robot/panda.urdf");
    arm.setJointAngle("panda_joint1", -M_PI / 4);
    arm.setJointAngle("panda_joint2", M_PI / 8);
    arm.setJointAngle("panda_joint3", -M_PI / 8);
    arm.setJointAngle("panda_joint4", -M_PI / 3);

    MotionGenerator motionGenerator(arm);
    motionGenerator.setObstacleTree(tree);

    PathPlanner planner;
    planner.setStartPose(arm);
    planner.setObstacleTree(tree);
    Eigen::Affine3d transform;
    transform.translation() = POS2::pos;
    transform.linear() = POS2::orientation;
    RobotArm goal = planner.selectGoalPose(transform).first;
    Eigen::MatrixXd angles(2, 7);
    angles.row(0) = arm.getJointAngles();
    angles.row(1) = goal.getJointAngles();
    motionGenerator.setWaypoints(angles);

    std::vector<double> params = stompParams;
    for (const auto &param : params) {
        // High-Level Output: Print the parameters being used for this set of runs using qDebug
        qDebug() << "\nRunning STOMP with Exploration Constant:" << param;

        // Create the directory for this parameter
        std::string directory_name = "/Users/joris/Uni/MA/Code/Evaluation/output_pos2/STOMP_"
                                     + std::to_string(param);

        if (!std::filesystem::create_directory(directory_name)) {
            qDebug() << "Directory creation failed or already exists:"
                     << QString::fromStdString(directory_name);
            continue; // Skip to the next param set if directory can't be created
        }

        // Open profiling CSV file to log times for all 10 runs
        std::ofstream profiling_file(directory_name + "/profiling.csv");
        if (!profiling_file.is_open()) {
            qDebug() << "Failed to open profiling file:"
                     << QString::fromStdString(directory_name + "/profiling.csv");
            continue; // Skip to the next param set if profiling file can't be created
        }

        // Write the CSV header
        profiling_file << "Run,STOMP(ms),Success\n";

        // Run the process 10 times, creating 10 trajectory files and recording profiling
        for (int i = 1; i <= 10; ++i) {
            // Indented block for each run
            motionGenerator.setExplorationConstant(param);

            // Timing the motionGenerator's performSTOMP
            auto start = std::chrono::high_resolution_clock::now();
            StompConfig config;
            bool res = motionGenerator.performSTOMP(config);
            auto end = std::chrono::high_resolution_clock::now();
            auto stompDuration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);

            // Save the trajectory to a CSV file
            std::string trajectory_file = directory_name + "/trajectory" + std::to_string(i)
                                          + ".csv";
            motionGenerator.saveTrajectoryToCSV(trajectory_file);

            // Write profiling information for this run (success case)
            if (res) {
                profiling_file << i << "," << stompDuration.count() << ",Success\n";

                // Detailed Output: Use qDebug to print the result of the run
                qDebug() << "    Run " << i << ":"
                         << " Exploration Constant: " << param
                         << ", STOMP duration: " << stompDuration.count() << " ms"
                         << ", Status: Success";
            } else {
                // Write profiling information for this run (failure case)
                profiling_file << i << "," << stompDuration.count() << ",Failure\n";

                // Detailed Output: Use qDebug to print the result of the run
                qDebug() << "    Run " << i << ":"
                         << " Exploration Constant: " << param
                         << ", STOMP duration: " << stompDuration.count() << " ms"
                         << ", Status: Failure";
            }
        }

        profiling_file.close();
    }
}

void profileHauser()
{
    RobotManager robotManager;
    robotManager.parseURDF("/Users/joris/Uni/MA/Code/PathPlanner_US/res/test.xml");

    std::shared_ptr<BVHTree> tree = std::make_shared<BVHTree>(
        robotManager.getTransformedObstacles());

    RobotArm arm("/Users/joris/Uni/MA/Code/PathPlanner_US/res/robot/panda.urdf");
    arm.setJointAngle("panda_joint1", -M_PI / 4);
    arm.setJointAngle("panda_joint2", M_PI / 8);
    arm.setJointAngle("panda_joint3", -M_PI / 8);
    arm.setJointAngle("panda_joint4", -M_PI / 3);

    PathPlanner planner;
    planner.setStartPose(arm);
    planner.setObstacleTree(tree);
    Eigen::Affine3d transform;
    transform.translation() = POS2::pos;
    transform.linear() = POS2::orientation;
    RobotArm goal = planner.selectGoalPose(transform).first;
    planner.setGoalConfiguration(goal);

    MotionGenerator motionGenerator(arm);
    motionGenerator.setObstacleTree(tree);

    std::vector<Params> params = buildParams();
    for (const auto &param : params) {
        // Modify the directory name to include the algorithm
        std::string algorithm_name;
        switch (param.algo) {
        case Algorithm::RRT:
            algorithm_name = "RRT";
            break;
        case Algorithm::RRTConnect:
            algorithm_name = "RRTConnect";
            break;
        case Algorithm::RRTStar:
            algorithm_name = "RRTStar";
            break;
        case Algorithm::InformedRRTStar:
            algorithm_name = "InformedRRTStar";
            break;
        default:
            algorithm_name = "UnknownAlgo";
        }

        qDebug() << "\nRunning planner with the following parameters:";
        qDebug() << "Algorithm:" << QString::fromStdString(algorithm_name);
        qDebug() << "Step Size:" << param.stepSize;
        qDebug() << "Goal Bias:" << param.goalBiasProbability;
        qDebug() << "Max Iterations:" << param.maxIterations;

        std::string directory_name = "/Users/joris/Uni/MA/Code/Evaluation/output_pos2/"
                                     + algorithm_name + "_" + std::to_string(param.stepSize) + "_"
                                     + std::to_string(param.goalBiasProbability) + "_"
                                     + std::to_string(param.maxIterations);

        if (!std::filesystem::create_directory(directory_name)) {
            qDebug() << "Directory creation failed or already exists:"
                     << QString::fromStdString(directory_name);
            continue;
        }

        std::ofstream profiling_file(directory_name + "/profiling.csv");
        if (!profiling_file.is_open()) {
            qDebug() << "Failed to open profiling file:"
                     << QString::fromStdString(directory_name + "/profiling.csv");
            continue; // Skip to the next param set if profiling file can't be created
        }

        profiling_file << "Run,RunPathFinding(ms),Hauser(ms),Success\n";

        // Run the process 10 times, creating 10 angles files and recording profiling
        for (int i = 1; i <= 10; ++i) {
            // Indented block for each run
            planner.setParams(param);

            // Timing the planner's runPathFinding
            auto start = std::chrono::high_resolution_clock::now();
            bool res = planner.runPathFinding();
            auto end = std::chrono::high_resolution_clock::now();
            auto runPathFindingDuration = std::chrono::duration_cast<std::chrono::milliseconds>(
                end - start);

            // Log the result to the profiling file (include success/failure)
            if (res) {
                // Save joint angles to file (angles1.csv, angles2.csv, ..., angles10.csv)
                std::string angles_file = directory_name + "/angles" + std::to_string(i) + ".csv";
                planner.saveJointAnglesToFile(angles_file);

                // Set waypoints for motion generator
                motionGenerator.setWaypoints(planner.getAnglesPath());

                // Timing the motionGenerator's performHauser
                start = std::chrono::high_resolution_clock::now();
                motionGenerator.performHauser(500,
                                              directory_name + "/time_" + std::to_string(i)
                                                  + ".csv");
                end = std::chrono::high_resolution_clock::now();
                auto hauserDuration = std::chrono::duration_cast<std::chrono::milliseconds>(
                    end - start);

                // Save the trajectory to a CSV file
                std::string trajectory_file = directory_name + "/trajectory" + std::to_string(i)
                                              + ".csv";
                motionGenerator.saveTrajectoryToCSV(trajectory_file);

                // Write profiling information for this run (success case)
                profiling_file << i << "," << runPathFindingDuration.count() << ","
                               << hauserDuration.count() << ",Success\n";

                // Detailed Output: Use qDebug to print the result of the run
                qDebug() << "    Run " << i << ":"
                         << " Step size: " << param.stepSize
                         << ", Goal bias: " << param.goalBiasProbability
                         << ", Max iterations: " << param.maxIterations
                         << ", RunPathFinding duration: " << runPathFindingDuration.count() << " ms"
                         << ", Hauser duration: " << hauserDuration.count() << " ms"
                         << ", Status: Success";
            } else {
                // Write profiling information for this run (failure case)
                profiling_file << i << "," << runPathFindingDuration.count() << ",N/A,Failure\n";

                // Detailed Output: Use qDebug to print the result of the run
                qDebug() << "    Run " << i << ":"
                         << " Step size: " << param.stepSize
                         << ", Goal bias: " << param.goalBiasProbability
                         << ", Max iterations: " << param.maxIterations
                         << ", RunPathFinding duration: " << runPathFindingDuration.count() << " ms"
                         << ", Status: Failure";
            }
        }

        profiling_file.close();
    }
}

int main(int argc, char *argv[])
{
    profileHauser();
}
