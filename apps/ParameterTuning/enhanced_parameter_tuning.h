#ifndef ENHANCED_PARAMETER_TUNING_H
#define ENHANCED_PARAMETER_TUNING_H

#include <vector>
#include <string>
#include <memory>
#include <functional>
#include <map>
#include <chrono>
#include <random>
#include <mutex>
#include <future>
#include <atomic>

#include "TrajectoryLib/Motion/MotionGenerator.h"
#include "TrajectoryLib/Planning/PathPlanner.h"
#include "TrajectoryLib/Robot/RobotArm.h"
#include "GeometryLib/BVHTree.h"

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>

namespace ParameterTuning {

// Forward declarations
class BayesianOptimizer;
class ParameterSpace;
class ObjectiveFunction;

// Enhanced parameter definitions with bounds and constraints
struct ParameterDefinition {
    std::string name;
    double min_value;
    double max_value;
    double default_value;
    bool is_integer = false;
    bool is_log_scale = false;
    std::vector<double> discrete_values;  // For categorical parameters
    std::string description;
    
    ParameterDefinition(const std::string& n, double min_val, double max_val, 
                       double default_val, const std::string& desc = "")
        : name(n), min_value(min_val), max_value(max_val), 
          default_value(default_val), description(desc) {}
    
    ParameterDefinition(const std::string& n, const std::vector<double>& values, 
                       double default_val, const std::string& desc = "")
        : name(n), min_value(0), max_value(values.size()-1), 
          default_value(default_val), is_integer(true), 
          discrete_values(values), description(desc) {}
};

// Parameter configuration for algorithms
struct ParameterConfiguration {
    std::string algorithm_name;
    std::map<std::string, double> parameters;
    std::string configuration_id;
    double objective_value = std::numeric_limits<double>::quiet_NaN();
    double uncertainty = std::numeric_limits<double>::quiet_NaN();
    std::chrono::system_clock::time_point evaluation_time;
    
    std::string toString() const;
    bool isValid() const;
    double getParameter(const std::string& name, double default_val = 0.0) const;
};

// Comprehensive evaluation metrics
struct EvaluationMetrics {
    // Primary metrics
    double planning_time_ms = 0.0;
    double success_rate = 0.0;
    double path_length_rad = 0.0;
    double smoothness_score = 0.0;
    double safety_score = 0.0;
    
    // Detailed quality metrics
    double max_velocity = 0.0;
    double max_acceleration = 0.0;
    double max_jerk = 0.0;
    double avg_jerk = 0.0;
    double velocity_variance = 0.0;
    double acceleration_variance = 0.0;
    
    // Safety and clearance metrics
    double min_clearance = 0.0;
    double avg_clearance = 0.0;
    double clearance_variance = 0.0;
    bool collision_free = true;
    
    // Computational efficiency
    double memory_usage_mb = 0.0;
    double cpu_utilization = 0.0;
    int iterations_used = 0;
    double convergence_rate = 0.0;
    
    // Reliability metrics
    double consistency_score = 0.0;
    double repeatability_score = 0.0;
    double robustness_score = 0.0;
    
    // Clinical relevance (if applicable)
    double clinical_accuracy = 0.0;
    double patient_comfort_score = 0.0;
    double workflow_efficiency = 0.0;
    
    // Statistical aggregation
    int sample_size = 0;
    std::vector<double> raw_planning_times;
    std::vector<double> raw_path_lengths;
    std::vector<bool> raw_successes;
    
    void computeAggregateMetrics();
    double computeObjectiveValue(const std::vector<double>& weights) const;
    std::string toCSVString() const;
};

// Test scenario for parameter evaluation
struct TestScenario {
    std::string name;
    std::string description;
    int difficulty_level = 1;  // 1-5, 5 being most difficult
    
    // Robot configuration
    Eigen::VectorXd start_configuration;
    Eigen::VectorXd goal_configuration;
    std::vector<Eigen::VectorXd> waypoints;
    
    // Environment
    std::shared_ptr<BVHTree> obstacle_tree;
    std::string environment_file;
    
    // Evaluation parameters
    int num_evaluation_runs = 10;
    double timeout_seconds = 30.0;
    bool require_reproducibility = true;
    
    // Scenario-specific weights for objective function
    std::vector<double> metric_weights;
    
    bool isValid() const;
    std::string getScenarioHash() const;
};

// Advanced STOMP parameter space
class StompParameterSpace {
public:
    StompParameterSpace();
    
    std::vector<ParameterDefinition> getParameterDefinitions() const;
    ParameterConfiguration createConfiguration(const std::vector<double>& values) const;
    std::vector<double> extractValues(const ParameterConfiguration& config) const;
    ParameterConfiguration getDefaultConfiguration() const;
    ParameterConfiguration getRandomConfiguration() const;
    
    // Parameter space analysis
    int getDimensionality() const { return _parameters.size(); }
    std::vector<std::pair<double, double>> getBounds() const;
    bool isValidConfiguration(const ParameterConfiguration& config) const;
    
private:
    std::vector<ParameterDefinition> _parameters;
    std::mt19937 _random_generator;
    
    void initializeParameters();
};

// Advanced RRT/Hauser parameter space
class HauserParameterSpace {
public:
    HauserParameterSpace();
    
    std::vector<ParameterDefinition> getParameterDefinitions() const;
    ParameterConfiguration createConfiguration(const std::vector<double>& values) const;
    std::vector<double> extractValues(const ParameterConfiguration& config) const;
    ParameterConfiguration getDefaultConfiguration() const;
    ParameterConfiguration getRandomConfiguration() const;
    
    int getDimensionality() const { return _parameters.size(); }
    std::vector<std::pair<double, double>> getBounds() const;
    bool isValidConfiguration(const ParameterConfiguration& config) const;
    
private:
    std::vector<ParameterDefinition> _parameters;
    std::mt19937 _random_generator;
    
    void initializeParameters();
};

// Objective function interface for parameter optimization
class ObjectiveFunction {
public:
    virtual ~ObjectiveFunction() = default;
    
    virtual double evaluate(const ParameterConfiguration& config,
                           const std::vector<TestScenario>& scenarios) = 0;
    
    virtual EvaluationMetrics getDetailedMetrics() const = 0;
    virtual void setWeights(const std::vector<double>& weights) = 0;
    virtual std::vector<std::string> getMetricNames() const = 0;
    
    // Multi-objective support
    virtual std::vector<double> evaluateMultiObjective(const ParameterConfiguration& config,
                                                      const std::vector<TestScenario>& scenarios) = 0;
};

// Comprehensive objective function implementation
class ComprehensiveObjectiveFunction : public ObjectiveFunction {
public:
    ComprehensiveObjectiveFunction(std::shared_ptr<RobotArm> robot);
    
    double evaluate(const ParameterConfiguration& config,
                   const std::vector<TestScenario>& scenarios) override;
    
    std::vector<double> evaluateMultiObjective(const ParameterConfiguration& config,
                                              const std::vector<TestScenario>& scenarios) override;
    
    EvaluationMetrics getDetailedMetrics() const override { return _last_metrics; }
    void setWeights(const std::vector<double>& weights) override { _weights = weights; }
    std::vector<std::string> getMetricNames() const override;
    
    // Configuration
    void setTimeout(double seconds) { _timeout_seconds = seconds; }
    void setNumRuns(int runs) { _num_evaluation_runs = runs; }
    void setParallelEvaluation(bool enabled) { _parallel_evaluation = enabled; }
    
private:
    std::shared_ptr<RobotArm> _robot;
    EvaluationMetrics _last_metrics;
    std::vector<double> _weights;
    double _timeout_seconds = 30.0;
    int _num_evaluation_runs = 10;
    bool _parallel_evaluation = true;
    
    EvaluationMetrics evaluateSTOMPConfiguration(const ParameterConfiguration& config,
                                                const TestScenario& scenario);
    EvaluationMetrics evaluateHauserConfiguration(const ParameterConfiguration& config,
                                                 const TestScenario& scenario);
    
    double computeWeightedObjective(const EvaluationMetrics& metrics) const;
    EvaluationMetrics aggregateMetrics(const std::vector<EvaluationMetrics>& metrics) const;
};

// Gaussian Process for Bayesian optimization
class GaussianProcess {
public:
    GaussianProcess(int input_dim);
    
    void addDataPoint(const std::vector<double>& x, double y);
    void fit();
    
    std::pair<double, double> predict(const std::vector<double>& x) const;  // mean, variance
    double acquisitionFunction(const std::vector<double>& x, const std::string& type = "EI") const;
    
    void setKernel(const std::string& kernel_type, const std::vector<double>& hyperparams);
    void optimizeHyperparameters();
    
    int getNumDataPoints() const { return _X.size(); }
    double getBestObservedValue() const;
    
private:
    int _input_dim;
    std::vector<std::vector<double>> _X;  // Input data
    std::vector<double> _y;               // Output data
    
    // Kernel parameters
    std::string _kernel_type = "RBF";
    std::vector<double> _hyperparams;
    
    // GP model
    Eigen::MatrixXd _K_inv;  // Inverse covariance matrix
    Eigen::VectorXd _alpha;  // Precomputed alpha vector
    bool _is_fitted = false;
    
    double kernelFunction(const std::vector<double>& x1, const std::vector<double>& x2) const;
    Eigen::MatrixXd computeCovarianceMatrix(const std::vector<std::vector<double>>& X) const;
    double expectedImprovement(const std::vector<double>& x) const;
    double upperConfidenceBound(const std::vector<double>& x, double beta = 2.0) const;
};

// Bayesian optimization for parameter tuning
class BayesianOptimizer {
public:
    BayesianOptimizer(std::unique_ptr<ObjectiveFunction> objective,
                     int max_iterations = 100);
    
    void setParameterSpace(std::shared_ptr<ParameterSpace> space);
    void addInitialConfiguration(const ParameterConfiguration& config);
    void setAcquisitionFunction(const std::string& type);  // "EI", "UCB", "PI"
    void setConvergenceCriteria(double tolerance, int patience);
    
    // Main optimization methods
    ParameterConfiguration optimize();
    ParameterConfiguration optimizeMultiObjective();  // Pareto optimization
    
    // Progress monitoring
    void setProgressCallback(std::function<void(int, const ParameterConfiguration&, double)> callback);
    std::vector<ParameterConfiguration> getEvaluationHistory() const;
    std::vector<double> getObjectiveHistory() const;
    
    // Analysis
    ParameterConfiguration getBestConfiguration() const;
    double getBestObjectiveValue() const;
    void generateConvergencePlot(const std::string& filename) const;
    void generateParameterImportanceAnalysis(const std::string& filename) const;
    
private:
    std::unique_ptr<ObjectiveFunction> _objective;
    std::unique_ptr<GaussianProcess> _gp;
    
    int _max_iterations;
    std::string _acquisition_type = "EI";
    double _convergence_tolerance = 1e-6;
    int _convergence_patience = 10;
    
    std::vector<ParameterConfiguration> _evaluation_history;
    std::vector<double> _objective_history;
    std::function<void(int, const ParameterConfiguration&, double)> _progress_callback;
    
    std::vector<double> nextSample();
    bool hasConverged() const;
    void updateGaussianProcess();
    
    // Multi-objective optimization helpers
    std::vector<ParameterConfiguration> _pareto_front;
    void updateParetoFront(const ParameterConfiguration& config, const std::vector<double>& objectives);
    bool isDominated(const std::vector<double>& obj1, const std::vector<double>& obj2) const;
};

// Main enhanced parameter tuning framework
class EnhancedParameterTuning {
public:
    EnhancedParameterTuning();
    ~EnhancedParameterTuning() = default;
    
    // Setup
    void setRobotModel(const std::string& urdf_path);
    void addTestScenario(const TestScenario& scenario);
    void loadScenariosFromFile(const std::string& scenario_file);
    void setOutputDirectory(const std::string& output_dir);
    
    // Algorithm configuration
    void enableSTOMPTuning(bool enable = true);
    void enableHauserTuning(bool enable = true);
    void setOptimizationMethod(const std::string& method);  // "bayesian", "grid", "random", "evolutionary"
    
    // Execution parameters
    void setMaxIterations(int iterations);
    void setMaxEvaluationTime(double seconds);
    void setParallelEvaluation(bool enabled, int max_threads = 0);
    void setRandomSeed(unsigned int seed);
    
    // Objective function configuration
    void setObjectiveWeights(const std::vector<double>& weights);
    void enableMultiObjectiveOptimization(bool enable = true);
    void addCustomMetric(const std::string& name, std::function<double(const EvaluationMetrics&)> metric);
    
    // Main execution
    bool runParameterTuning();
    bool runQuickTuning(int max_configs = 20);
    bool runComparisonStudy();  // Compare different optimization methods
    
    // Results access
    ParameterConfiguration getBestSTOMPConfiguration() const;
    ParameterConfiguration getBestHauserConfiguration() const;
    std::vector<ParameterConfiguration> getParetoFront() const;
    
    // Analysis and reporting
    void generateComprehensiveReport(const std::string& report_path) const;
    void generateVisualizationPlots(const std::string& plots_dir) const;
    void exportResults(const std::string& format, const std::string& filename) const;  // "csv", "json", "xml"
    
    // Statistical analysis
    void performSensitivityAnalysis(const std::string& output_file) const;
    void generateParameterCorrelationMatrix(const std::string& output_file) const;
    void validateBestConfigurations(int validation_runs = 50) const;
    
    // Interactive features
    void launchInteractiveTuningInterface() const;
    void enableRealTimeVisualization(bool enable = true);
    
private:
    // Core components
    std::shared_ptr<RobotArm> _robot;
    std::vector<TestScenario> _test_scenarios;
    std::string _output_directory;
    
    // Algorithm spaces
    std::unique_ptr<StompParameterSpace> _stomp_space;
    std::unique_ptr<HauserParameterSpace> _hauser_space;
    
    // Optimization components
    std::unique_ptr<BayesianOptimizer> _stomp_optimizer;
    std::unique_ptr<BayesianOptimizer> _hauser_optimizer;
    std::unique_ptr<ComprehensiveObjectiveFunction> _objective_function;
    
    // Configuration
    bool _enable_stomp_tuning = true;
    bool _enable_hauser_tuning = true;
    std::string _optimization_method = "bayesian";
    int _max_iterations = 100;
    double _max_evaluation_time = 3600.0;  // 1 hour
    bool _parallel_evaluation = true;
    int _max_threads = 0;
    unsigned int _random_seed = 42;
    
    // Objective configuration
    std::vector<double> _objective_weights;
    bool _multi_objective_enabled = false;
    std::map<std::string, std::function<double(const EvaluationMetrics&)>> _custom_metrics;
    
    // Results storage
    mutable std::mutex _results_mutex;
    std::vector<ParameterConfiguration> _stomp_results;
    std::vector<ParameterConfiguration> _hauser_results;
    std::vector<ParameterConfiguration> _pareto_front;
    
    // Progress tracking
    std::atomic<int> _completed_evaluations{0};
    std::atomic<int> _total_evaluations{0};
    std::function<void(double)> _progress_callback;
    
    // Private methods
    void initializeComponents();
    void validateConfiguration() const;
    void setupDefaultObjectiveWeights();
    void setupDefaultTestScenarios();
    
    bool runBayesianOptimization();
    bool runGridSearch();
    bool runRandomSearch();
    bool runEvolutionaryOptimization();
    
    void updateParetoFront(const ParameterConfiguration& config, const std::vector<double>& objectives);
    void generateProgressReport() const;
    void saveIntermediateResults() const;
    
    // Analysis helpers
    void computeParameterImportance(const std::vector<ParameterConfiguration>& configs,
                                   std::map<std::string, double>& importance) const;
    void performStatisticalTests(const std::vector<ParameterConfiguration>& configs1,
                                const std::vector<ParameterConfiguration>& configs2,
                                std::string& report) const;
};

// Utility functions
namespace TuningUtils {
    std::vector<TestScenario> generateStandardTestSuite();
    std::vector<TestScenario> generateClinicalTestSuite();
    std::vector<TestScenario> generateStressTestSuite();
    
    std::vector<double> getDefaultObjectiveWeights();
    std::vector<double> getClinicalObjectiveWeights();
    std::vector<double> getPerformanceObjectiveWeights();
    
    void validateParameterConfiguration(const ParameterConfiguration& config, std::string& error_msg);
    std::string generateConfigurationReport(const ParameterConfiguration& config, const EvaluationMetrics& metrics);
    
    // Statistical utilities
    double computeStatisticalSignificance(const std::vector<double>& data1, const std::vector<double>& data2);
    double computeEffectSize(const std::vector<double>& data1, const std::vector<double>& data2);
    std::pair<double, double> computeConfidenceInterval(const std::vector<double>& data, double confidence = 0.95);
}

} // namespace ParameterTuning

#endif // ENHANCED_PARAMETER_TUNING_H