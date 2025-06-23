#ifndef ADVANCED_STOMP_HAUSER_BENCHMARK_FRAMEWORK_H
#define ADVANCED_STOMP_HAUSER_BENCHMARK_FRAMEWORK_H

#include <vector>
#include <string>
#include <memory>
#include <chrono>
#include <future>
#include <random>
#include <unordered_map>

#include "TrajectoryLib/Motion/MotionGenerator.h"
#include "TrajectoryLib/Planning/PathPlanner.h"
#include "TrajectoryLib/Robot/RobotArm.h"
#include "GeometryLib/BVHTree.h"
#include "Hauser10/DynamicPath.h"

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>

namespace AdvancedBenchmark {

// Forward declarations
class ScenarioGenerator;
class MetricsCalculator;
class StatisticalAnalyzer;

// Enhanced algorithm configurations
struct AdvancedStompConfig {
    // Core STOMP parameters
    int numNoisyTrajectories = 50;
    int numBestSamples = 10;
    int maxIterations = 1000;
    double dt = 0.05;
    double learningRate = 0.3;
    double temperature = 8.0;
    
    // Advanced features
    bool adaptiveSampling = true;
    bool earlyTermination = true;
    bool hybridOptimization = true;
    
    // Clinical-specific parameters
    double clinicalSafetyMargin = 0.05;  // 5cm safety buffer
    double maxJerkLimit = 10.0;          // Patient comfort constraint
    bool enforceOrientationStability = true;
    
    // Performance optimization
    bool useParallelization = true;
    int maxThreads = 0;  // 0 = auto-detect
    double convergenceThreshold = 1e-6;
};

struct AdvancedHauserConfig {
    // Velocity and acceleration limits (medical-safe)
    Eigen::VectorXd velMax = (Eigen::VectorXd(7) << 1.5, 1.5, 1.5, 1.5, 2.0, 2.0, 2.0).finished();
    Eigen::VectorXd accMax = (Eigen::VectorXd(7) << 10.0, 5.0, 7.5, 10.0, 12.5, 15.0, 15.0).finished();
    
    // Advanced smoothing parameters
    int shortcutIterations = 200;
    double toleranceThreshold = 0.001;
    bool adaptiveShortcutting = true;
    
    // Clinical constraints
    double maxVelocityNearPatient = 0.5;
    bool enforceGentleMotion = true;
    
    // Joint limits
    Eigen::VectorXd jointMin = (Eigen::VectorXd(7) << -2.8973, -1.7628, -2.8973, -3.0718, -2.8973, -0.0175, -2.8973).finished();
    Eigen::VectorXd jointMax = (Eigen::VectorXd(7) << 2.8973, 1.7628, 2.8973, -0.0698, 2.8973, 3.7525, 2.8973).finished();
};

// Test scenario definition
struct TestScenario {
    std::string name;
    std::string description;
    int complexityLevel;  // 1-5, 5 being most complex
    
    // Scenario parameters
    Eigen::VectorXd startConfiguration;
    Eigen::VectorXd goalConfiguration;
    std::vector<Eigen::VectorXd> waypoints;
    std::vector<Eigen::Affine3d> taskSpacePoses;
    
    // Environment setup
    std::string environmentFile;
    std::shared_ptr<BVHTree> obstacleTree;
    
    // Expected performance bounds
    double expectedPlanningTimeMs = -1;  // -1 = no expectation
    double minimumSuccessRate = 0.8;
    double maximumJerk = 50.0;
    
    // Clinical relevance
    bool isClinicalScenario = false;
    std::string clinicalProcedure;
    double clinicalImportanceWeight = 1.0;
};

// Comprehensive benchmark results
struct BenchmarkResult {
    // Scenario identification
    std::string scenarioName;
    std::string algorithmName;
    int runId;
    
    // Performance metrics
    double planningTimeMs;
    double executionTimeMs;
    bool planningSuccess;
    bool trajectoryValid;
    int iterations;
    
    // Quality metrics
    double pathLength;
    double smoothnessScore;
    double jerkMetric;
    double energyEstimate;
    double trajectoryConsistency;
    
    // Safety metrics
    double minimumClearance;
    double averageClearance;
    double weightedClearance;
    std::vector<double> linkClearances;
    double jointLimitProximity;
    bool hasCollision;
    
    // Clinical metrics (if applicable)
    double examinationCoverage = -1;
    double probeStability = -1;
    double patientComfortScore = -1;
    double clinicalWorkflowScore = -1;
    
    // Computational metrics
    double memoryUsageMB;
    double cpuUtilization;
    int threadsUsed;
    
    // Trajectory data
    std::vector<Eigen::VectorXd> trajectoryPoints;
    std::vector<double> timeStamps;
    std::vector<Eigen::VectorXd> velocities;
    std::vector<Eigen::VectorXd> accelerations;
    
    // Statistical metadata
    std::chrono::system_clock::time_point timestamp;
    std::string systemInfo;
    std::string configurationHash;
};

// Statistical summary for comparative analysis
struct StatisticalSummary {
    std::string metricName;
    std::string algorithmName;
    
    // Descriptive statistics
    double mean;
    double median;
    double standardDeviation;
    double variance;
    double skewness;
    double kurtosis;
    
    // Robust statistics
    double q25;  // 25th percentile
    double q75;  // 75th percentile
    double iqr;  // Interquartile range
    double mad;  // Median absolute deviation
    
    // Confidence intervals
    double confidenceLevel = 0.95;
    double lowerCI;
    double upperCI;
    
    // Sample information
    int sampleSize;
    int validSamples;
    double outlierRate;
};

// Comparison results between algorithms
struct ComparisonResult {
    std::string metricName;
    std::string algorithm1;
    std::string algorithm2;
    
    // Hypothesis testing
    double pValue;
    double testStatistic;
    std::string testType;
    bool isSignificant;
    double significanceLevel = 0.05;
    
    // Effect size analysis
    double cohensD;
    double glassDelta;
    std::string effectSizeMagnitude;
    
    // Practical significance
    double minimumPracticalDifference;
    bool isPracticallySignificant;
    
    // Performance comparison
    double algorithm1Mean;
    double algorithm2Mean;
    double relativeDifference;  // (alg2 - alg1) / alg1
    std::string winner;
};

// Main benchmark framework class
class BenchmarkFramework {
public:
    BenchmarkFramework();
    ~BenchmarkFramework();
    
    // Configuration
    void setStompConfig(const AdvancedStompConfig& config);
    void setHauserConfig(const AdvancedHauserConfig& config);
    void setRobotUrdf(const std::string& urdfPath);
    void setOutputDirectory(const std::string& outputDir);
    
    // Scenario management
    void addScenario(const TestScenario& scenario);
    void loadScenariosFromFile(const std::string& scenarioFile);
    void generateSyntheticScenarios(int count, int complexityLevel);
    void loadClinicalScenarios(const std::string& clinicalDataPath);
    
    // Execution control
    void setNumberOfRuns(int runs);
    void setParallelExecution(bool enabled, int maxThreads = 0);
    void setEarlyTermination(bool enabled, double statisticalPower = 0.8);
    void setProgressCallback(std::function<void(double)> callback);
    
    // Main execution methods
    bool runBenchmark();
    bool runQuickTest(int scenarioCount = 5, int runsPerScenario = 10);
    bool runScenario(const TestScenario& scenario, int runs);
    
    // Individual algorithm execution
    BenchmarkResult runStompOnScenario(const TestScenario& scenario, int runId);
    BenchmarkResult runHauserOnScenario(const TestScenario& scenario, int runId);
    
    // Results access
    std::vector<BenchmarkResult> getAllResults() const;
    std::vector<BenchmarkResult> getResultsForScenario(const std::string& scenarioName) const;
    std::vector<BenchmarkResult> getResultsForAlgorithm(const std::string& algorithmName) const;
    
    // Statistical analysis
    std::vector<StatisticalSummary> computeStatisticalSummaries() const;
    std::vector<ComparisonResult> compareAlgorithms() const;
    void generateStatisticalReport(const std::string& reportPath) const;
    
    // Data export
    void exportResultsToCSV(const std::string& csvPath) const;
    void exportResultsToJSON(const std::string& jsonPath) const;
    void exportTrajectoriesToCSV(const std::string& csvPath) const;
    
    // Visualization support
    void generatePlots(const std::string& plotsDirectory) const;
    void launchInteractiveDashboard() const;
    
    // Validation and diagnostics
    bool validateConfiguration() const;
    void printSystemInfo() const;
    double estimateExecutionTime() const;
    
private:
    // Configuration
    AdvancedStompConfig _stompConfig;
    AdvancedHauserConfig _hauserConfig;
    std::string _robotUrdfPath;
    std::string _outputDirectory;
    
    // Test setup
    std::vector<TestScenario> _scenarios;
    int _runsPerScenario;
    bool _parallelExecution;
    int _maxThreads;
    bool _earlyTermination;
    double _statisticalPower;
    
    // Results storage
    std::vector<BenchmarkResult> _results;
    mutable std::mutex _resultsMutex;
    
    // Progress tracking
    std::function<void(double)> _progressCallback;
    std::atomic<int> _completedRuns;
    std::atomic<int> _totalRuns;
    
    // Core components
    std::unique_ptr<RobotArm> _robotArm;
    std::unique_ptr<ScenarioGenerator> _scenarioGenerator;
    std::unique_ptr<MetricsCalculator> _metricsCalculator;
    std::unique_ptr<StatisticalAnalyzer> _statisticalAnalyzer;
    
    // Random number generation
    std::mt19937 _randomGenerator;
    
    // Private helper methods
    void initializeRobot();
    void initializeComponents();
    BenchmarkResult createResultTemplate(const TestScenario& scenario, const std::string& algorithm, int runId);
    void populateSystemInfo(BenchmarkResult& result);
    void validateResult(BenchmarkResult& result);
    bool shouldTerminateEarly() const;
    void updateProgress();
    
    // Algorithm-specific execution helpers
    std::vector<MotionGenerator::TrajectoryPoint> executeStompPlanning(
        const TestScenario& scenario, BenchmarkResult& result);
    std::vector<MotionGenerator::TrajectoryPoint> executeHauserPlanning(
        const TestScenario& scenario, BenchmarkResult& result);
    
    // Metrics computation helpers
    void computeQualityMetrics(const std::vector<MotionGenerator::TrajectoryPoint>& trajectory, 
                              BenchmarkResult& result);
    void computeSafetyMetrics(const std::vector<MotionGenerator::TrajectoryPoint>& trajectory,
                             const std::shared_ptr<BVHTree>& obstacles, BenchmarkResult& result);
    void computeClinicalMetrics(const std::vector<MotionGenerator::TrajectoryPoint>& trajectory,
                               const TestScenario& scenario, BenchmarkResult& result);
    
    // Utility methods
    std::string generateConfigurationHash() const;
    std::string getCurrentSystemInfo() const;
    void ensureOutputDirectory() const;
};

// Utility functions
namespace BenchmarkUtils {
    double computeJerkMetric(const std::vector<MotionGenerator::TrajectoryPoint>& trajectory);
    double computePathLength(const std::vector<MotionGenerator::TrajectoryPoint>& trajectory);
    double computeSmoothnessScore(const std::vector<MotionGenerator::TrajectoryPoint>& trajectory);
    double computeEnergyEstimate(const std::vector<MotionGenerator::TrajectoryPoint>& trajectory, const RobotArm& arm);
    bool validateTrajectory(const std::vector<MotionGenerator::TrajectoryPoint>& trajectory, const RobotArm& arm);
    std::string formatDuration(std::chrono::milliseconds duration);
    std::string generateTimestamp();
}

// Configuration validation
namespace ConfigValidation {
    bool validateStompConfig(const AdvancedStompConfig& config, std::string& errorMessage);
    bool validateHauserConfig(const AdvancedHauserConfig& config, std::string& errorMessage);
    bool validateScenario(const TestScenario& scenario, std::string& errorMessage);
}

} // namespace AdvancedBenchmark

#endif // ADVANCED_STOMP_HAUSER_BENCHMARK_FRAMEWORK_H