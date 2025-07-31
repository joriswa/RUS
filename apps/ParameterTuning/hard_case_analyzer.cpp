#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include <sstream>
#include <chrono>
#include <yaml-cpp/yaml.h>
#include <random>
#include <algorithm>
#include <iomanip>
#include <set>
#include <Eigen/Dense>

// TrajectoryLib includes
#include "TrajectoryLib/Motion/MotionGenerator.h"
#include "TrajectoryLib/Robot/RobotArm.h"
#include "TrajectoryLib/Robot/RobotManager.h"
#include "TrajectoryLib/Planning/PathPlanner.h"
#include "GeometryLib/BVHTree.h"
#include "USLib/USTrajectoryPlanner.h"

#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include <sstream>
#include <chrono>
#include <yaml-cpp/yaml.h>
#include <random>
#include <algorithm>
#include <iomanip>
#include <set>
#include <Eigen/Dense>
#include <json/json.h>

// TrajectoryLib includes
#include "TrajectoryLib/Motion/MotionGenerator.h"
#include "TrajectoryLib/Robot/RobotArm.h"
#include "TrajectoryLib/Robot/RobotManager.h"
#include "TrajectoryLib/Planning/PathPlanner.h"
#include "GeometryLib/BVHTree.h"
#include "USLib/USTrajectoryPlanner.h"

/**
 * @brief Pose data structure for CSV loading
 */
struct PoseData {
    Eigen::Vector3d position;
    Eigen::Quaterniond quaternion;
    bool contact;
};

struct CaseResult {
    int pose_index;
    std::string pose_name;
    Eigen::VectorXd start_config;
    Eigen::VectorXd target_config;
    bool baseline_success;
    double baseline_time_ms;
    std::string baseline_failure_reason;
    bool optimized_success;
    double optimized_time_ms;  
    std::string optimized_failure_reason;
    double improvement_factor;  // baseline_time / optimized_time
};

class HardCaseAnalyzer {
private:
    std::vector<PoseData> poses;
    std::vector<std::pair<int, int>> pose_pairs;
    std::vector<std::string> pose_names;
    
    YAML::Node baseline_config;
    YAML::Node optimized_config;
    
    // Hard case criteria
    double failure_time_threshold_ms = 3000.0;  // Cases taking > 3 seconds
    double slow_time_threshold_ms = 1500.0;     // Cases taking > 1.5 seconds
    
public:
    HardCaseAnalyzer(const std::string& baseline_config_file, 
                     const std::string& optimized_config_file) {
        
        baseline_config = YAML::LoadFile(baseline_config_file);
        optimized_config = YAML::LoadFile(optimized_config_file);
        
        // Load pose data
        loadPoseData();
        
        std::cout << "Hard Case Analyzer initialized with:" << std::endl;
        std::cout << "  - Pose pairs: " << pose_pairs.size() << std::endl;
        std::cout << "  - Slow threshold: " << slow_time_threshold_ms << " ms" << std::endl;
        std::cout << "  - Failure threshold: " << failure_time_threshold_ms << " ms" << std::endl;
    }
    
    void loadPoseData() {
        std::string csv_path = baseline_config["csv_file"].as<std::string>();
        
        // Load CSV data (simplified version)
        std::ifstream file(csv_path);
        std::string line;
        
        // Skip header
        std::getline(file, line);
        
        while (std::getline(file, line)) {
            std::stringstream ss(line);
            std::string cell;
            
            PoseData pose;
            
            // Parse CSV line - simplified
            std::vector<std::string> values;
            while (std::getline(ss, cell, ',')) {
                values.push_back(cell);
            }
            
            if (values.size() >= 7) {
                pose.position.x() = std::stod(values[1]);
                pose.position.y() = std::stod(values[2]);
                pose.position.z() = std::stod(values[3]);
                
                // Quaternion (x, y, z, w)
                pose.quaternion.x() = std::stod(values[4]);
                pose.quaternion.y() = std::stod(values[5]);
                pose.quaternion.z() = std::stod(values[6]);
                pose.quaternion.w() = std::stod(values[7]);
                
                pose.contact = (values[8] == "True");
                
                poses.push_back(pose);
            }
        }
        
        std::cout << "Loaded " << poses.size() << " poses from CSV" << std::endl;
        
        // Generate random pose pairs for testing
        std::random_device rd;
        std::mt19937 gen(42); // Fixed seed for reproducibility
        std::uniform_int_distribution<> dis(0, poses.size() - 1);
        
        int num_pairs = 100; // Test set
        std::set<std::pair<int, int>> used_pairs;
        
        while (pose_pairs.size() < num_pairs && used_pairs.size() < poses.size() * poses.size()) {
            int start_idx = dis(gen);
            int target_idx = dis(gen);
            
            if (start_idx == target_idx) continue;
            if (used_pairs.count({start_idx, target_idx})) continue;
            
            used_pairs.insert({start_idx, target_idx});
            pose_pairs.push_back({start_idx, target_idx});
            pose_names.push_back("Pair_" + std::to_string(start_idx) + "_to_" + std::to_string(target_idx));
        }
        
        std::cout << "Generated " << pose_pairs.size() << " pose pairs for analysis" << std::endl;
    }
    
    CaseResult evaluateSingleCase(int case_index, const YAML::Node& config, const std::string& config_name) {
        CaseResult result;
        result.pose_index = case_index;
        result.pose_name = pose_names[case_index];
        
        // Create temporary config file
        std::string temp_config_file = "temp_hard_case_config_" + config_name + ".yaml";
        
        // Modify config for single case evaluation
        YAML::Node temp_config = config;
        temp_config["evaluation"]["trajectory_pairs"] = 1;  // Single pair
        
        std::ofstream config_file(temp_config_file);
        config_file << temp_config;
        config_file.close();
        
        // Time the evaluation
        auto start_time = std::chrono::high_resolution_clock::now();
        
        // Run parameter evaluator as subprocess
        std::string cmd = "./build/apps/ParameterTuning/parameter_evaluator " + temp_config_file;
        
        bool success = false;
        std::string failure_reason = "";
        
        try {
            int return_code = std::system(cmd.c_str());
            success = (return_code == 0);
            
            if (!success) {
                failure_reason = "Parameter evaluator failed with return code " + std::to_string(return_code);
            }
        } catch (const std::exception& e) {
            success = false;
            failure_reason = std::string("Exception: ") + e.what();
        }
        
        auto end_time = std::chrono::high_resolution_clock::now();
        double elapsed_ms = std::chrono::duration<double, std::milli>(end_time - start_time).count();
        
        // Clean up temp file
        std::remove(temp_config_file.c_str());
        
        if (config_name == "baseline") {
            result.baseline_success = success;
            result.baseline_time_ms = elapsed_ms;
            result.baseline_failure_reason = failure_reason;
        } else {
            result.optimized_success = success;
            result.optimized_time_ms = elapsed_ms;
            result.optimized_failure_reason = failure_reason;
        }
        
        return result;
    }
    
    std::vector<CaseResult> identifyHardCases() {
        std::cout << "\n=== PHASE 1: Identifying Hard Cases with Baseline Parameters ===" << std::endl;
        
        std::vector<CaseResult> all_results;
        std::vector<CaseResult> hard_cases;
        
        for (int i = 0; i < std::min(50, (int)pose_pairs.size()); i++) {  // Test first 50 pairs
            std::cout << "Testing case " << (i+1) << "/50: " << pose_names[i] << "...";
            
            CaseResult result = evaluateSingleCase(i, baseline_config, "baseline");
            all_results.push_back(result);
            
            // Identify hard cases
            bool is_hard = false;
            std::string reason = "";
            
            if (!result.baseline_success) {
                is_hard = true;
                reason = "FAILED - " + result.baseline_failure_reason;
            } else if (result.baseline_time_ms > failure_time_threshold_ms) {
                is_hard = true;
                reason = "VERY_SLOW (" + std::to_string(int(result.baseline_time_ms)) + " ms)";
            } else if (result.baseline_time_ms > slow_time_threshold_ms) {
                is_hard = true;
                reason = "SLOW (" + std::to_string(int(result.baseline_time_ms)) + " ms)";
            }
            
            if (is_hard) {
                hard_cases.push_back(result);
                std::cout << " HARD CASE - " << reason << std::endl;
            } else {
                std::cout << " OK (" << std::to_string(int(result.baseline_time_ms)) << " ms)" << std::endl;
            }
        }
        
        std::cout << "\nHard case identification complete:" << std::endl;
        std::cout << "  Total cases tested: " << all_results.size() << std::endl;
        std::cout << "  Hard cases found: " << hard_cases.size() << std::endl;
        std::cout << "  Hard case rate: " << (100.0 * hard_cases.size() / all_results.size()) << "%" << std::endl;
        
        return hard_cases;
    }
    
    void analyzeHardCases(std::vector<CaseResult>& hard_cases) {
        std::cout << "\n=== PHASE 2: Re-testing Hard Cases with Optimized Parameters ===" << std::endl;
        
        for (auto& result : hard_cases) {
            std::cout << "Re-testing hard case: " << result.pose_name << "...";
            
            CaseResult optimized_result = evaluateSingleCase(result.pose_index, optimized_config, "optimized");
            
            // Merge results
            result.optimized_success = optimized_result.optimized_success;
            result.optimized_time_ms = optimized_result.optimized_time_ms;
            result.optimized_failure_reason = optimized_result.optimized_failure_reason;
            
            // Calculate improvement
            if (result.baseline_success && result.optimized_success) {
                result.improvement_factor = result.baseline_time_ms / result.optimized_time_ms;
            } else if (!result.baseline_success && result.optimized_success) {
                result.improvement_factor = -1; // Special case: failure -> success
            } else {
                result.improvement_factor = 0; // No improvement or both failed
            }
            
            std::string status;
            if (!result.baseline_success && result.optimized_success) {
                status = "FIXED! (Failure -> Success in " + std::to_string(int(result.optimized_time_ms)) + " ms)";
            } else if (result.baseline_success && result.optimized_success) {
                status = "IMPROVED " + std::to_string(result.improvement_factor) + "x (" + 
                         std::to_string(int(result.baseline_time_ms)) + " -> " + 
                         std::to_string(int(result.optimized_time_ms)) + " ms)";
            } else if (result.baseline_success && !result.optimized_success) {
                status = "REGRESSED (Success -> Failure)";
            } else {
                status = "STILL FAILING";
            }
            
            std::cout << " " << status << std::endl;
        }
    }
    
    void generateReport(const std::vector<CaseResult>& hard_cases) {
        std::cout << "\n=== HARD CASE ANALYSIS REPORT ===" << std::endl;
        
        int fixed_cases = 0;
        int improved_cases = 0;
        int regressed_cases = 0;
        int still_failing = 0;
        double total_speedup = 0;
        int speedup_count = 0;
        
        std::ofstream report_file("hard_case_analysis_report.csv");
        report_file << "Case_Name,Baseline_Success,Baseline_Time_ms,Baseline_Failure,"
                   << "Optimized_Success,Optimized_Time_ms,Optimized_Failure,"
                   << "Improvement_Factor,Status\n";
        
        for (const auto& result : hard_cases) {
            std::string status;
            if (!result.baseline_success && result.optimized_success) {
                fixed_cases++;
                status = "FIXED";
            } else if (result.baseline_success && result.optimized_success) {
                improved_cases++;
                total_speedup += result.improvement_factor;
                speedup_count++;
                status = "IMPROVED_" + std::to_string(result.improvement_factor) + "x";
            } else if (result.baseline_success && !result.optimized_success) {
                regressed_cases++;
                status = "REGRESSED";
            } else {
                still_failing++;
                status = "STILL_FAILING";
            }
            
            report_file << result.pose_name << ","
                       << (result.baseline_success ? "TRUE" : "FALSE") << ","
                       << result.baseline_time_ms << ","
                       << "\"" << result.baseline_failure_reason << "\","
                       << (result.optimized_success ? "TRUE" : "FALSE") << ","
                       << result.optimized_time_ms << ","
                       << "\"" << result.optimized_failure_reason << "\","
                       << result.improvement_factor << ","
                       << status << "\n";
        }
        
        report_file.close();
        
        std::cout << "\nSUMMARY:" << std::endl;
        std::cout << "  Hard cases analyzed: " << hard_cases.size() << std::endl;
        std::cout << "  Cases fixed (failure -> success): " << fixed_cases << std::endl;
        std::cout << "  Cases improved (faster): " << improved_cases << std::endl;
        std::cout << "  Cases regressed: " << regressed_cases << std::endl;
        std::cout << "  Cases still failing: " << still_failing << std::endl;
        
        if (speedup_count > 0) {
            double avg_speedup = total_speedup / speedup_count;
            std::cout << "  Average speedup on improved cases: " << std::fixed << std::setprecision(2) 
                      << avg_speedup << "x" << std::endl;
        }
        
        double success_rate = 100.0 * (fixed_cases + improved_cases) / hard_cases.size();
        std::cout << "  Overall improvement rate: " << std::fixed << std::setprecision(1) 
                  << success_rate << "%" << std::endl;
        
        std::cout << "\nDetailed report saved to: hard_case_analysis_report.csv" << std::endl;
        
        // Display top improvements
        auto sorted_cases = hard_cases;
        std::sort(sorted_cases.begin(), sorted_cases.end(), 
                  [](const CaseResult& a, const CaseResult& b) {
                      return a.improvement_factor > b.improvement_factor;
                  });
        
        std::cout << "\nTOP 10 IMPROVEMENTS:" << std::endl;
        for (int i = 0; i < std::min(10, (int)sorted_cases.size()); i++) {
            const auto& result = sorted_cases[i];
            if (result.improvement_factor > 0) {
                std::cout << "  " << (i+1) << ". " << result.pose_name 
                          << ": " << std::fixed << std::setprecision(1) << result.improvement_factor 
                          << "x speedup (" << int(result.baseline_time_ms) 
                          << " -> " << int(result.optimized_time_ms) << " ms)" << std::endl;
            }
        }
    }
    
    void run() {
        std::cout << "Starting Hard Case Analysis..." << std::endl;
        std::cout << "This will identify challenging pose pairs and test optimized parameters on them." << std::endl;
        
        // Phase 1: Identify hard cases with baseline parameters
        auto hard_cases = identifyHardCases();
        
        if (hard_cases.empty()) {
            std::cout << "No hard cases found! All pose pairs solved quickly." << std::endl;
            return;
        }
        
        // Phase 2: Re-test hard cases with optimized parameters
        analyzeHardCases(hard_cases);
        
        // Phase 3: Generate comprehensive report
        generateReport(hard_cases);
    }
};

int main(int argc, char* argv[]) {
    if (argc != 3) {
        std::cerr << "Usage: " << argv[0] << " <baseline_config.yaml> <optimized_config.yaml>" << std::endl;
        std::cerr << "Example: " << argv[0] << " baseline_stomp_config.yaml optimized_heavy_penalty_config.yaml" << std::endl;
        return 1;
    }
    
    try {
        HardCaseAnalyzer analyzer(argv[1], argv[2]);
        analyzer.run();
        
        std::cout << "\nHard case analysis completed successfully!" << std::endl;
        return 0;
        
    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        return 1;
    }
}
