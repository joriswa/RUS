#include <iostream>
#include <fstream>
#include <sstream>
#include <map>
#include <vector>
#include <string>
#include <chrono>
#include <thread>
#include <cstdlib>

/**
 * Simple Configurable Comparison Tool for Optuna Integration
 * 
 * This tool provides a minimal interface for running trajectory planning
 * algorithms with configurable parameters. It accepts a config file and
 * outputs results in CSV format suitable for Optuna optimization.
 * 
 * Usage: ./SimpleConfigurableComparison config_file output_dir num_poses trials_per_pose
 */

// Simple configuration parser
class ConfigParser {
private:
    std::map<std::string, std::string> config_;
    
public:
    bool loadFromFile(const std::string& filename) {
        std::ifstream file(filename);
        if (!file.is_open()) {
            std::cerr << "Error: Could not open config file: " << filename << std::endl;
            return false;
        }
        
        std::string line;
        while (std::getline(file, line)) {
            // Skip empty lines and comments
            if (line.empty() || line[0] == '#') continue;
            
            size_t pos = line.find('=');
            if (pos != std::string::npos) {
                std::string key = line.substr(0, pos);
                std::string value = line.substr(pos + 1);
                // Trim whitespace
                key.erase(0, key.find_first_not_of(" \t"));
                key.erase(key.find_last_not_of(" \t") + 1);
                value.erase(0, value.find_first_not_of(" \t"));
                value.erase(value.find_last_not_of(" \t") + 1);
                config_[key] = value;
            }
        }
        return true;
    }
    
    std::string getString(const std::string& key, const std::string& defaultValue = "") const {
        auto it = config_.find(key);
        return (it != config_.end()) ? it->second : defaultValue;
    }
    
    int getInt(const std::string& key, int defaultValue = 0) const {
        auto it = config_.find(key);
        return (it != config_.end()) ? std::stoi(it->second) : defaultValue;
    }
    
    double getDouble(const std::string& key, double defaultValue = 0.0) const {
        auto it = config_.find(key);
        return (it != config_.end()) ? std::stod(it->second) : defaultValue;
    }
};

// Mock trial result for testing
struct TrialResult {
    int pose_index;
    std::string pose_name;
    int trial_number;
    std::string algorithm;
    bool success;
    double planning_time_ms;
    double trajectory_duration_s;
    int trajectory_points;
    
    TrialResult() : pose_index(-1), trial_number(-1), success(false),
                    planning_time_ms(0.0), trajectory_duration_s(0.0), trajectory_points(0) {}
};

class SimpleComparison {
private:
    std::string algorithm_;
    std::map<std::string, double> parameters_;
    
public:
    bool loadConfiguration(const std::string& config_file) {
        ConfigParser parser;
        if (!parser.loadFromFile(config_file)) {
            return false;
        }
        
        algorithm_ = parser.getString("algorithm", "stomp");
        std::cerr << "✓ Algorithm: " << algorithm_ << std::endl;
        
        // Load algorithm-specific parameters
        if (algorithm_ == "stomp") {
            parameters_["max_iterations"] = parser.getDouble("stomp_max_iterations", 100);
            parameters_["num_noisy_trajectories"] = parser.getDouble("stomp_num_noisy_trajectories", 10);
            parameters_["learning_rate"] = parser.getDouble("stomp_learning_rate", 0.1);
            parameters_["temperature"] = parser.getDouble("stomp_temperature", 10.0);
        } else if (algorithm_ == "rrt" || algorithm_ == "hauser") {
            parameters_["step_size"] = parser.getDouble("rrt_step_size", 0.1);
            parameters_["goal_bias_probability"] = parser.getDouble("rrt_goal_bias_probability", 0.05);
            parameters_["max_iterations"] = parser.getDouble("rrt_max_iterations", 5000);
        }
        
        return true;
    }
    
    TrialResult runSingleTrial(int pose_index, int trial_number) {
        TrialResult result;
        result.pose_index = pose_index;
        result.pose_name = "Test_Pose_" + std::to_string(pose_index);
        result.trial_number = trial_number;
        result.algorithm = algorithm_;
        
        // Simulate algorithm execution
        auto start_time = std::chrono::high_resolution_clock::now();
        
        // Mock planning with parameter-dependent behavior
        double complexity_factor = 1.0;
        if (algorithm_ == "stomp") {
            complexity_factor = parameters_["max_iterations"] / 100.0;
        } else {
            complexity_factor = parameters_["max_iterations"] / 5000.0;
        }
        
        // Simulate planning time (varies with complexity)
        std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<int>(10 + 20 * complexity_factor)));
        
        auto end_time = std::chrono::high_resolution_clock::now();
        result.planning_time_ms = std::chrono::duration<double, std::milli>(end_time - start_time).count();
        
        // Simulate success probability (parameter-dependent)
        double success_probability = 0.7; // Base success rate
        if (algorithm_ == "stomp") {
            success_probability += (parameters_["learning_rate"] - 0.1) * 2.0; // Better with higher learning rate
            success_probability += (parameters_["temperature"] - 10.0) * 0.01; // Slight improvement with higher temp
        } else {
            success_probability += (0.1 - parameters_["step_size"]) * 2.0; // Better with smaller steps
            success_probability += parameters_["goal_bias_probability"] * 2.0; // Better with more goal bias
        }
        
        // Add some randomness
        double random_factor = (rand() % 100) / 100.0;
        result.success = (random_factor < success_probability);
        
        if (result.success) {
            result.trajectory_duration_s = 2.0 + (rand() % 100) / 50.0; // 2-4 seconds
            result.trajectory_points = 200 + rand() % 100; // 200-300 points
        }
        
        return result;
    }
    
    void writeCSVResults(const std::vector<TrialResult>& results, const std::string& output_file) {
        std::ofstream file(output_file);
        if (!file.is_open()) {
            std::cerr << "Error: Could not open output file: " << output_file << std::endl;
            return;
        }
        
        // Write header
        file << "pose_index,pose_name,trial_number,algorithm,success,planning_time_ms,trajectory_duration_s,trajectory_points\n";
        
        // Write data
        for (const auto& result : results) {
            file << result.pose_index << ","
                 << result.pose_name << ","
                 << result.trial_number << ","
                 << result.algorithm << ","
                 << (result.success ? 1 : 0) << ","
                 << result.planning_time_ms << ","
                 << result.trajectory_duration_s << ","
                 << result.trajectory_points << "\n";
        }
        
        std::cerr << "✓ Results written to: " << output_file << std::endl;
    }
};

int main(int argc, char* argv[]) {
    if (argc != 5) {
        std::cerr << "Usage: " << argv[0] << " <config_file> <output_dir> <num_poses> <trials_per_pose>" << std::endl;
        return 1;
    }
    
    std::string config_file = argv[1];
    std::string output_dir = argv[2];
    int num_poses = std::stoi(argv[3]);
    int trials_per_pose = std::stoi(argv[4]);
    
    // Create output directory
    std::string mkdir_cmd = "mkdir -p " + output_dir;
    std::system(mkdir_cmd.c_str());
    
    SimpleComparison comparison;
    if (!comparison.loadConfiguration(config_file)) {
        std::cerr << "❌ Failed to load configuration" << std::endl;
        return 1;
    }
    
    std::vector<TrialResult> all_results;
    
    // Run trials
    for (int pose = 0; pose < num_poses; ++pose) {
        for (int trial = 0; trial < trials_per_pose; ++trial) {
            TrialResult result = comparison.runSingleTrial(pose, trial);
            all_results.push_back(result);
        }
    }
    
    // Write results
    std::string output_file = output_dir + "/results.csv";
    comparison.writeCSVResults(all_results, output_file);
    
    return 0;
}
