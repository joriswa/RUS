// Standalone test application to automate the scan process as performed in the PathPlanner main window.
// Loads URDF, environment XML, and scan poses CSV, sets up the planner, and runs planning between scan poses.
// Outputs results to the console.

#include <iostream>
#include <vector>
#include <string>
#include <fstream>
#include <sstream>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <USLib/USTrajectoryPlanner.h>
#include <algorithm> // for std::remove_if
#include <chrono>
#include <random>
#include <iomanip>

/**
 * @brief Structure to hold test configuration parameters
 */
struct TestConfig {
    std::string urdf_path = "/Users/joris/Downloads/panda_US.urdf";
    std::string env_xml_path = "/Users/joris/Downloads/3/obstacles.xml";
    std::string scan_poses_csv = "/Users/joris/Downloads/3/scan_poses.csv";
    std::string output_csv = "scan_test_results.csv";
    double noise_level = 0.0;  // Standard deviation for orientation noise in radians
    int num_trials = 1;        // Number of test trials to run
    bool verbose = true;       // Enable detailed output
};

/**
 * @brief Structure to hold test results for a single trial
 */
struct TestResult {
    int trial_number;
    double noise_level;
    int num_poses;
    int num_trajectories;
    double total_time_ms;
    double csv_load_time_ms;
    double planner_init_time_ms;
    double trajectory_planning_time_ms;
    bool success;
    std::string error_message;
    std::vector<int> trajectory_sizes;
    std::vector<bool> trajectory_types; // true = contact, false = free
};

/**
 * @brief Applies random noise to the orientation of poses
 * @param poses Vector of poses to modify
 * @param noise_std_dev Standard deviation of noise in radians
 * @param seed Random seed for reproducibility
 * @return Vector of poses with noise applied
 */
std::vector<Eigen::Affine3d> addOrientationNoise(const std::vector<Eigen::Affine3d>& poses, 
                                                  double noise_std_dev, 
                                                  unsigned int seed = 0) {
    if (noise_std_dev <= 0.0) {
        return poses; // No noise to apply
    }
    
    std::vector<Eigen::Affine3d> noisy_poses = poses;
    std::mt19937 gen(seed);
    std::normal_distribution<double> noise_dist(0.0, noise_std_dev);
    
    for (auto& pose : noisy_poses) {
        // Generate small random rotations around x, y, z axes
        double roll_noise = noise_dist(gen);
        double pitch_noise = noise_dist(gen);
        double yaw_noise = noise_dist(gen);
        
        // Create noise rotation matrix
        Eigen::AngleAxisd roll_noise_rot(roll_noise, Eigen::Vector3d::UnitX());
        Eigen::AngleAxisd pitch_noise_rot(pitch_noise, Eigen::Vector3d::UnitY());
        Eigen::AngleAxisd yaw_noise_rot(yaw_noise, Eigen::Vector3d::UnitZ());
        Eigen::Matrix3d noise_rotation = (yaw_noise_rot * pitch_noise_rot * roll_noise_rot).toRotationMatrix();
        
        // Apply noise to the existing orientation
        pose.linear() = pose.linear() * noise_rotation;
    }
    
    return noisy_poses;
}

/**
 * @brief Exports test results to CSV file
 * @param results Vector of test results
 * @param filename Output CSV filename
 * @return True if export succeeded, false otherwise
 */
bool exportResultsToCSV(const std::vector<TestResult>& results, const std::string& filename) {
    std::ofstream csv(filename);
    if (!csv.is_open()) {
        std::cerr << "Failed to open output CSV: " << filename << std::endl;
        return false;
    }
    
    // Write CSV header
    csv << "trial_number,noise_level,num_poses,num_trajectories,total_time_ms,"
        << "csv_load_time_ms,planner_init_time_ms,trajectory_planning_time_ms,"
        << "success,error_message,trajectory_sizes,trajectory_types\n";
    
    // Write data
    for (const auto& result : results) {
        csv << result.trial_number << ","
            << std::fixed << std::setprecision(6) << result.noise_level << ","
            << result.num_poses << ","
            << result.num_trajectories << ","
            << std::fixed << std::setprecision(3) << result.total_time_ms << ","
            << result.csv_load_time_ms << ","
            << result.planner_init_time_ms << ","
            << result.trajectory_planning_time_ms << ","
            << (result.success ? "true" : "false") << ","
            << "\"" << result.error_message << "\",";
        
        // Write trajectory sizes as semicolon-separated list
        csv << "\"";
        for (size_t i = 0; i < result.trajectory_sizes.size(); ++i) {
            if (i > 0) csv << ";";
            csv << result.trajectory_sizes[i];
        }
        csv << "\",";
        
        // Write trajectory types as semicolon-separated list
        csv << "\"";
        for (size_t i = 0; i < result.trajectory_types.size(); ++i) {
            if (i > 0) csv << ";";
            csv << (result.trajectory_types[i] ? "contact" : "free");
        }
        csv << "\"\n";
    }
    
    csv.close();
    return true;
}

/**
 * @brief Loads scan poses from a CSV file and converts them to Eigen::Affine3d transforms
 * @param csv_path Path to the CSV file containing scan poses
 * @param verbose Enable verbose output
 * @return Vector of Affine3d transforms representing the scan poses
 */
std::vector<Eigen::Affine3d> loadScanPosesFromCSV(const std::string& csv_path, bool verbose = true) {
    std::vector<Eigen::Affine3d> scan_poses;
    std::ifstream csv(csv_path);
    
    if (!csv.is_open()) {
        std::cerr << "Failed to open scan poses CSV: " << csv_path << std::endl;
        return scan_poses;
    }
    
    std::string line;
    int line_num = 0;
    bool header_skipped = false;
    
    while (std::getline(csv, line)) {
        ++line_num;
        
        // Trim whitespace and carriage returns
        line.erase(std::remove(line.begin(), line.end(), '\r'), line.end());
        line.erase(std::remove(line.begin(), line.end(), '\n'), line.end());
        if (line.empty()) continue;
        
        if (verbose) {
            std::cout << "Raw line " << line_num << ": '" << line << "'" << std::endl;
        }
        
        // Parse CSV values
        std::vector<double> values;
        std::stringstream ss(line);
        std::string token;
        bool parse_error = false;
        
        while (std::getline(ss, token, ',')) {
            // Strip whitespace from token
            token.erase(0, token.find_first_not_of(" \t"));
            token.erase(token.find_last_not_of(" \t") + 1);
            if (token.empty()) continue;
            
            try {
                values.push_back(std::stod(token));
            } catch (...) {
                parse_error = true;
                break;
            }
            if (values.size() == 7) break; // Only need first 7 values (x,y,z,qw,qx,qy,qz)
        }
        
        if (parse_error) {
            if (verbose) {
                std::cout << "  Skipping line due to parse error: '" << line << "'" << std::endl;
            }
            // Try to skip header only once
            if (!header_skipped && line_num == 1) {
                if (verbose) {
                    std::cout << "  Header detected, skipping." << std::endl;
                }
                header_skipped = true;
                continue;
            }
            continue;
        }
        
        if (verbose) {
            std::cout << "Line " << line_num << ": " << values.size() << " values: ";
            for (size_t i = 0; i < values.size(); ++i) std::cout << values[i] << " ";
            std::cout << std::endl;
        }
        
        if (values.size() < 7) continue;
        
        // Create pose from parsed values
        double x = values[0];
        double y = values[1];
        double z = values[2];
        Eigen::Quaterniond q(values[3], values[4], values[5], values[6]);
        q.normalize();
        
        Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
        T.block<3, 3>(0, 0) = q.toRotationMatrix();
        T.block<3, 1>(0, 3) = Eigen::Vector3d(x, y, z);
        
        // Apply local offset (move 2cm back along local Z-axis)
        const Eigen::Vector3d local_move(0.0, 0.0, -0.02);
        T.block<3, 1>(0, 3) += T.block<3, 3>(0, 0) * local_move;
        
        scan_poses.emplace_back(T);
    }
    
    csv.close();
    if (verbose) {
        std::cout << "Parsed " << scan_poses.size() << " scan poses from " << line_num << " lines." << std::endl;
        
        if (!scan_poses.empty()) {
            std::cout << "First pose translation: " << scan_poses[0].translation().transpose() << std::endl;
        }
    }
    
    return scan_poses;
}

/**
 * @brief Initializes the trajectory planner with URDF and environment
 * @param urdf_path Path to the URDF file
 * @param env_xml_path Path to the environment XML file
 * @param verbose Enable verbose output
 * @return Configured UltrasoundScanTrajectoryPlanner
 */
UltrasoundScanTrajectoryPlanner initializePlanner(const std::string& urdf_path, 
                                                   const std::string& env_xml_path, 
                                                   bool verbose = true) {
    if (verbose) {
        std::cout << "Initializing planner with URDF: " << urdf_path << std::endl;
    }
    UltrasoundScanTrajectoryPlanner planner(urdf_path);
    
    if (verbose) {
        std::cout << "Setting environment: " << env_xml_path << std::endl;
    }
    planner.setEnvironment(env_xml_path);
    
    return planner;
}

/**
 * @brief Sets the current joint configuration for the robot
 * @param planner Reference to the trajectory planner
 * @param verbose Enable verbose output
 */
void setInitialJointConfiguration(UltrasoundScanTrajectoryPlanner& planner, bool verbose = true) {
    // Set current joint positions (required before planning)
    Eigen::VectorXd current_joints(7);
    current_joints << 0.0, 0.0, 0.0, -1.5708, 0.0, 1.5708, 0.7854; // Home position for Panda robot
    
    planner.setCurrentJoints(current_joints);
    if (verbose) {
        std::cout << "Set current joints: " << current_joints.transpose() << std::endl;
    }
}

/**
 * @brief Plans trajectories between scan poses and outputs results
 * @param planner Reference to the trajectory planner
 * @param verbose Enable verbose output
 * @return Pair of (success, trajectories) where trajectories contains size and type info
 */
std::pair<bool, std::vector<std::pair<int, bool>>> planAndOutputTrajectories(UltrasoundScanTrajectoryPlanner& planner, 
                                                                            bool verbose = true) {
    if (verbose) {
        std::cout << "Starting trajectory planning..." << std::endl;
    }
    
    try {
        if (!planner.planTrajectories()) {
            if (verbose) {
                std::cerr << "Trajectory planning failed." << std::endl;
            }
            return {false, {}};
        }
        
        auto trajectories = planner.getTrajectories();
        if (verbose) {
            std::cout << "Planned " << trajectories.size() << " trajectories." << std::endl;
            
            for (size_t i = 0; i < trajectories.size(); ++i) {
                std::cout << "Trajectory " << i << ": " << trajectories[i].first.size() << " points, "
                          << (trajectories[i].second ? "contact" : "free") << std::endl;
            }
        }
        
        // Extract trajectory information
        std::vector<std::pair<int, bool>> traj_info;
        for (const auto& traj : trajectories) {
            traj_info.emplace_back(static_cast<int>(traj.first.size()), traj.second);
        }
        
        return {true, traj_info};
        
    } catch (const std::exception& e) {
        if (verbose) {
            std::cerr << "Exception during trajectory planning: " << e.what() << std::endl;
        }
        return {false, {}};
    } catch (...) {
        if (verbose) {
            std::cerr << "Unknown exception during trajectory planning." << std::endl;
        }
        return {false, {}};
    }
}

/**
 * @brief Runs a single test trial with specified configuration
 * @param config Test configuration
 * @param trial_number Current trial number
 * @param seed Random seed for this trial
 * @return TestResult structure with timing and success information
 */
TestResult runSingleTrial(const TestConfig& config, int trial_number, unsigned int seed) {
    TestResult result;
    result.trial_number = trial_number;
    result.noise_level = config.noise_level;
    result.success = false;
    result.error_message = "";
    
    auto trial_start = std::chrono::high_resolution_clock::now();
    
    try {
        // 1. Load scan poses from CSV
        if (config.verbose) {
            std::cout << "\n--- Trial " << trial_number << ": Loading Scan Poses ---" << std::endl;
        }
        
        auto csv_start = std::chrono::high_resolution_clock::now();
        auto scan_poses = loadScanPosesFromCSV(config.scan_poses_csv, config.verbose);
        auto csv_end = std::chrono::high_resolution_clock::now();
        result.csv_load_time_ms = std::chrono::duration<double, std::milli>(csv_end - csv_start).count();
        
        if (scan_poses.size() < 2) {
            result.error_message = "Not enough scan poses loaded (need at least 2, got " + std::to_string(scan_poses.size()) + ")";
            return result;
        }

        // For robustness, limit to reasonable number of poses
        if (scan_poses.size() > 10) {
            scan_poses.resize(10);
            if (config.verbose) {
                std::cout << "Limited to 10 poses for computational efficiency." << std::endl;
            }
        }
        
        result.num_poses = static_cast<int>(scan_poses.size());
        
        // 2. Apply orientation noise if specified
        if (config.noise_level > 0.0) {
            if (config.verbose) {
                std::cout << "Applying orientation noise (std_dev = " << config.noise_level << " rad)" << std::endl;
            }
            scan_poses = addOrientationNoise(scan_poses, config.noise_level, seed);
        }

        // 3. Initialize trajectory planner
        if (config.verbose) {
            std::cout << "\n--- Trial " << trial_number << ": Initializing Planner ---" << std::endl;
        }
        
        auto planner_start = std::chrono::high_resolution_clock::now();
        auto planner = initializePlanner(config.urdf_path, config.env_xml_path, config.verbose);
        auto planner_end = std::chrono::high_resolution_clock::now();
        result.planner_init_time_ms = std::chrono::duration<double, std::milli>(planner_end - planner_start).count();
        
        // 4. Set scan poses in planner
        if (config.verbose) {
            std::cout << "\n--- Trial " << trial_number << ": Setting Scan Poses ---" << std::endl;
        }
        planner.setPoses(scan_poses);
        if (config.verbose) {
            std::cout << "Set " << scan_poses.size() << " scan poses in planner." << std::endl;
        }

        // 5. Set initial joint configuration
        if (config.verbose) {
            std::cout << "\n--- Trial " << trial_number << ": Setting Initial Joint Configuration ---" << std::endl;
        }
        setInitialJointConfiguration(planner, config.verbose);

        // 6. Plan trajectories
        if (config.verbose) {
            std::cout << "\n--- Trial " << trial_number << ": Planning Trajectories ---" << std::endl;
        }
        
        auto planning_start = std::chrono::high_resolution_clock::now();
        auto [planning_success, traj_info] = planAndOutputTrajectories(planner, config.verbose);
        auto planning_end = std::chrono::high_resolution_clock::now();
        result.trajectory_planning_time_ms = std::chrono::duration<double, std::milli>(planning_end - planning_start).count();
        
        if (!planning_success) {
            result.error_message = "Trajectory planning failed";
            return result;
        }
        
        result.num_trajectories = static_cast<int>(traj_info.size());
        for (const auto& info : traj_info) {
            result.trajectory_sizes.push_back(info.first);
            result.trajectory_types.push_back(info.second);
        }
        
        result.success = true;
        
    } catch (const std::exception& e) {
        result.error_message = std::string("Exception: ") + e.what();
    } catch (...) {
        result.error_message = "Unknown exception";
    }
    
    auto trial_end = std::chrono::high_resolution_clock::now();
    result.total_time_ms = std::chrono::duration<double, std::milli>(trial_end - trial_start).count();
    
    return result;
}

/**
 * @brief Runs multiple test trials with different noise levels and exports results
 * @param config Base test configuration
 * @param noise_levels Vector of noise levels to test
 * @return True if all tests completed (regardless of individual success), false on critical error
 */
bool runModularTests(const TestConfig& config, const std::vector<double>& noise_levels) {
    std::vector<TestResult> all_results;
    
    std::cout << "=== Starting Modular Scan Process Tests ===" << std::endl;
    std::cout << "Configuration:" << std::endl;
    std::cout << "  URDF: " << config.urdf_path << std::endl;
    std::cout << "  Environment: " << config.env_xml_path << std::endl;
    std::cout << "  Scan poses: " << config.scan_poses_csv << std::endl;
    std::cout << "  Trials per noise level: " << config.num_trials << std::endl;
    std::cout << "  Output CSV: " << config.output_csv << std::endl;
    std::cout << "  Noise levels: ";
    for (size_t i = 0; i < noise_levels.size(); ++i) {
        if (i > 0) std::cout << ", ";
        std::cout << noise_levels[i];
    }
    std::cout << std::endl;
    
    int total_trials = static_cast<int>(noise_levels.size() * config.num_trials);
    int current_trial = 0;
    
    for (double noise_level : noise_levels) {
        std::cout << "\n=== Testing noise level: " << noise_level << " rad ===" << std::endl;
        
        TestConfig trial_config = config;
        trial_config.noise_level = noise_level;
        
        for (int trial = 1; trial <= config.num_trials; ++trial) {
            ++current_trial;
            std::cout << "\n--- Running trial " << current_trial << "/" << total_trials 
                      << " (noise=" << noise_level << ", trial=" << trial << ") ---" << std::endl;
            
            // Use different seed for each trial to ensure different noise patterns
            unsigned int seed = static_cast<unsigned int>(std::hash<std::string>{}(
                std::to_string(noise_level) + "_" + std::to_string(trial)));
            
            auto result = runSingleTrial(trial_config, current_trial, seed);
            all_results.push_back(result);
            
            // Print summary
            std::cout << "Trial " << current_trial << " summary: " 
                      << (result.success ? "SUCCESS" : "FAILED")
                      << " (total: " << std::fixed << std::setprecision(1) << result.total_time_ms << "ms";
            if (result.success) {
                std::cout << ", " << result.num_trajectories << " trajectories";
            } else {
                std::cout << ", error: " << result.error_message;
            }
            std::cout << ")" << std::endl;
        }
    }
    
    // Export results to CSV
    std::cout << "\n=== Exporting Results ===" << std::endl;
    if (exportResultsToCSV(all_results, config.output_csv)) {
        std::cout << "Results exported to: " << config.output_csv << std::endl;
    } else {
        std::cerr << "Failed to export results to CSV" << std::endl;
        return false;
    }
    
    // Print summary statistics
    std::cout << "\n=== Test Summary ===" << std::endl;
    int successful_trials = 0;
    double total_planning_time = 0.0;
    
    for (const auto& result : all_results) {
        if (result.success) {
            ++successful_trials;
            total_planning_time += result.trajectory_planning_time_ms;
        }
    }
    
    std::cout << "Total trials: " << all_results.size() << std::endl;
    std::cout << "Successful trials: " << successful_trials << " (" 
              << std::fixed << std::setprecision(1) 
              << (100.0 * successful_trials / all_results.size()) << "%)" << std::endl;
    
    if (successful_trials > 0) {
        std::cout << "Average planning time: " 
                  << std::fixed << std::setprecision(1) 
                  << (total_planning_time / successful_trials) << "ms" << std::endl;
    }
    
    return true;
}

/**
 * @brief Tests the IK solver with a simple, known reachable pose
 * @param planner Reference to the trajectory planner
 * @param verbose Enable verbose output
 * @return true if IK test passes, false otherwise
 */
bool testSimpleIKSolver(UltrasoundScanTrajectoryPlanner& planner, bool verbose = true) {
    if (verbose) {
        std::cout << "\n=== Testing IK Solver with Simple Pose ===" << std::endl;
    }
    
    try {
        auto pathPlanner = planner.getPathPlanner();
        
        // Create a simple reachable pose in front of the robot
        // This should be within the robot's workspace
        Eigen::Affine3d simple_pose = Eigen::Affine3d::Identity();
        simple_pose.translation() = Eigen::Vector3d(0.5, 0.0, 0.5);  // 50cm forward, 50cm height
        
        // Keep orientation aligned with base frame (no rotation)
        // This should be a very simple pose to solve
        
        if (verbose) {
            std::cout << "Testing simple pose:" << std::endl;
            std::cout << "  Translation: " << simple_pose.translation().transpose() << std::endl;
            std::cout << "  Rotation matrix:" << std::endl << simple_pose.rotation() << std::endl;
        }
        
        // Test the selectGoalPose method
        auto ik_result = pathPlanner->selectGoalPose(simple_pose);
        bool success = ik_result.second;
        
        if (verbose) {
            std::cout << "IK result: " << (success ? "SUCCESS" : "FAILED") << std::endl;
            if (success) {
                auto joint_angles = ik_result.first.getJointAngles();
                std::cout << "Joint angles: " << joint_angles.transpose() << std::endl;
            }
        }
        
        return success;
        
    } catch (const std::exception& e) {
        if (verbose) {
            std::cerr << "Exception during IK test: " << e.what() << std::endl;
        }
        return false;
    } catch (...) {
        if (verbose) {
            std::cerr << "Unknown exception during IK test." << std::endl;
        }
        return false;
    }
}

/**
 * @brief Main function - Entry point for the scan process test application
 * @param argc Number of command line arguments
 * @param argv Command line arguments
 * @return 0 on success, 1 on failure
 */
int main(int argc, char** argv) {
    // Default test configuration
    TestConfig config;
    
    // Parse command line arguments for basic customization
    bool run_single_test = false;
    bool run_modular_tests = true;
    
    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];
        if (arg == "--single") {
            run_single_test = true;
            run_modular_tests = false;
        } else if (arg == "--quiet") {
            config.verbose = false;
        } else if (arg == "--help" || arg == "-h") {
            std::cout << "Usage: " << argv[0] << " [options]" << std::endl;
            std::cout << "Options:" << std::endl;
            std::cout << "  --single    Run single test (original behavior)" << std::endl;
            std::cout << "  --quiet     Reduce output verbosity" << std::endl;
            std::cout << "  --help      Show this help message" << std::endl;
            return 0;
        }
    }
    
    try {
        if (run_single_test) {
            // Run original single test for backward compatibility
            std::cout << "=== Single Scan Process Test ===" << std::endl;
            
            auto result = runSingleTrial(config, 1, 0);
            
            if (result.success) {
                std::cout << "\n=== Test Completed Successfully ===" << std::endl;
                std::cout << "Total time: " << std::fixed << std::setprecision(1) 
                          << result.total_time_ms << "ms" << std::endl;
                std::cout << "Planning time: " << result.trajectory_planning_time_ms << "ms" << std::endl;
                std::cout << "Trajectories: " << result.num_trajectories << std::endl;
                return 0;
            } else {
                std::cerr << "Test failed: " << result.error_message << std::endl;
                return 1;
            }
        } else if (run_modular_tests) {
            // Run modular tests with different noise levels
            std::vector<double> noise_levels = {
                0.0,      // No noise (baseline)
                0.001,    // 0.001 rad ≈ 0.057 degrees
                0.005,    // 0.005 rad ≈ 0.286 degrees  
                0.01,     // 0.01 rad ≈ 0.573 degrees
                0.02,     // 0.02 rad ≈ 1.146 degrees
                0.05,     // 0.05 rad ≈ 2.865 degrees
                0.1       // 0.1 rad ≈ 5.73 degrees
            };
            
            config.num_trials = 3;  // Run 3 trials per noise level
            config.output_csv = "scan_test_results_" + 
                               std::to_string(std::chrono::duration_cast<std::chrono::seconds>(
                                   std::chrono::system_clock::now().time_since_epoch()).count()) + ".csv";
            
            if (runModularTests(config, noise_levels)) {
                std::cout << "\n=== All Tests Completed ===" << std::endl;
                return 0;
            } else {
                std::cerr << "Tests failed or incomplete" << std::endl;
                return 1;
            }
        }
        
    } catch (const std::exception& e) {
        std::cerr << "Error: Exception caught in main: " << e.what() << std::endl;
        return 1;
    } catch (...) {
        std::cerr << "Error: Unknown exception caught in main." << std::endl;
        return 1;
    }
    
    return 0;
}
