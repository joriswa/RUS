#ifndef CLINICAL_SCENARIOS_H
#define CLINICAL_SCENARIOS_H

#include "../core/benchmark_framework.h"
#include <vector>
#include <string>
#include <memory>
#include <map>

namespace AdvancedBenchmark {

// Clinical examination types
enum class ExaminationType {
    KNEE_ARTHRITIS_ASSESSMENT,
    THYROID_NODULE_EXAMINATION,
    CARDIAC_SCREENING,
    ABDOMINAL_ORGAN_SCAN,
    VASCULAR_DOPPLER,
    MUSCULOSKELETAL_INJURY,
    PRENATAL_EXAMINATION,
    BREAST_SCREENING
};

// Anatomical region definitions
struct AnatomicalRegion {
    std::string name;
    Eigen::Vector3d centerPosition;
    Eigen::Vector3d dimensions;  // x, y, z extents
    double safetyMargin = 0.05;  // 5cm default safety margin
    std::vector<Eigen::Vector3d> criticalPoints;  // High-precision required points
    std::vector<Eigen::Vector3d> avoidanceZones;  // Areas to avoid
};

// Clinical pose requirements
struct ClinicalPoseRequirement {
    Eigen::Affine3d targetPose;
    double positionTolerance = 0.002;    // 2mm tolerance
    double orientationTolerance = 0.087; // 5 degrees in radians
    double dwellTime = 1.0;              // Seconds to hold position
    bool requiresContact = false;        // Whether probe must contact surface
    double contactForce = 0.0;           // Required contact force (N)
    std::string anatomicalLandmark;      // Description of target anatomy
    int priority = 1;                    // 1=critical, 2=important, 3=optional
};

// Clinical examination protocol
struct ClinicalProtocol {
    ExaminationType type;
    std::string name;
    std::string description;
    AnatomicalRegion anatomicalRegion;
    std::vector<ClinicalPoseRequirement> requiredPoses;
    std::vector<ClinicalPoseRequirement> optionalPoses;
    
    // Examination parameters
    double totalExaminationTime = 300.0;  // 5 minutes default
    double maxProbeVelocity = 0.1;        // m/s for patient comfort
    double maxProbeAcceleration = 0.5;    // m/s² for patient comfort
    bool requiresBreathing = false;       // Whether patient breathing affects poses
    double breathingPeriod = 4.0;         // Seconds per breathing cycle
    
    // Quality requirements
    double minimumCoverage = 0.85;        // 85% of poses must be achieved
    double maxPositionError = 0.005;      // 5mm maximum position error
    double maxOrientationError = 0.175;   // 10 degrees maximum orientation error
    
    // Clinical scoring weights
    double accuracyWeight = 0.4;
    double smoothnessWeight = 0.3;
    double speedWeight = 0.2;
    double comfortWeight = 0.1;
};

class ClinicalScenariosGenerator {
public:
    ClinicalScenariosGenerator();
    ~ClinicalScenariosGenerator() = default;
    
    // Main generation methods
    std::vector<TestScenario> generateAllClinicalScenarios();
    std::vector<TestScenario> generateScenariosForExamination(ExaminationType type);
    TestScenario generateScenarioFromProtocol(const ClinicalProtocol& protocol);
    
    // Specific examination generators
    TestScenario generateKneeArthritisScenario();
    TestScenario generateThyroidExaminationScenario();
    TestScenario generateCardiacScreeningScenario();
    TestScenario generateAbdominalScanScenario();
    TestScenario generateVascularDopplerScenario();
    TestScenario generateMusculoskeletalScenario();
    TestScenario generatePrenatalExaminationScenario();
    TestScenario generateBreastScreeningScenario();
    
    // Protocol management
    void addCustomProtocol(const ClinicalProtocol& protocol);
    ClinicalProtocol getProtocol(ExaminationType type) const;
    std::vector<ClinicalProtocol> getAllProtocols() const;
    
    // Anatomical region management
    void defineAnatomicalRegion(const std::string& name, const AnatomicalRegion& region);
    AnatomicalRegion getAnatomicalRegion(const std::string& name) const;
    
    // Pose generation utilities
    std::vector<Eigen::Affine3d> generateScanningPoses(const AnatomicalRegion& region, 
                                                       int numPoses, 
                                                       const std::string& scanPattern = "grid");
    std::vector<Eigen::Affine3d> generateApproachPoses(const std::vector<Eigen::Affine3d>& scanPoses,
                                                       double approachDistance = 0.1);
    
    // Clinical validation
    bool validateClinicalScenario(const TestScenario& scenario, std::string& errorMessage);
    double computeClinicalRelevanceScore(const TestScenario& scenario);
    
    // Configuration
    void setDefaultSafetyMargins(double margin);
    void setDefaultVelocityLimits(double maxVel, double maxAcc);
    void enableBreathingSimulation(bool enable, double period = 4.0);
    
private:
    // Protocol storage
    std::map<ExaminationType, ClinicalProtocol> _protocols;
    std::map<std::string, AnatomicalRegion> _anatomicalRegions;
    
    // Default parameters
    double _defaultSafetyMargin = 0.05;
    double _defaultMaxVelocity = 0.1;
    double _defaultMaxAcceleration = 0.5;
    bool _breathingSimulationEnabled = false;
    double _breathingPeriod = 4.0;
    
    // Initialization methods
    void initializeDefaultProtocols();
    void initializeAnatomicalRegions();
    
    // Knee examination specifics
    ClinicalProtocol createKneeArthritisProtocol();
    std::vector<ClinicalPoseRequirement> generateKneeScanningPoses();
    AnatomicalRegion defineKneeRegion();
    
    // Thyroid examination specifics
    ClinicalProtocol createThyroidProtocol();
    std::vector<ClinicalPoseRequirement> generateThyroidScanningPoses();
    AnatomicalRegion defineThyroidRegion();
    
    // Cardiac examination specifics
    ClinicalProtocol createCardiacProtocol();
    std::vector<ClinicalPoseRequirement> generateCardiacScanningPoses();
    AnatomicalRegion defineCardiacRegion();
    
    // Abdominal examination specifics
    ClinicalProtocol createAbdominalProtocol();
    std::vector<ClinicalPoseRequirement> generateAbdominalScanningPoses();
    AnatomicalRegion defineAbdominalRegion();
    
    // Vascular examination specifics
    ClinicalProtocol createVascularDopplerProtocol();
    std::vector<ClinicalPoseRequirement> generateVascularScanningPoses();
    AnatomicalRegion defineVascularRegion();
    
    // Musculoskeletal examination specifics
    ClinicalProtocol createMusculoskeletalProtocol();
    std::vector<ClinicalPoseRequirement> generateMusculoskeletalScanningPoses();
    AnatomicalRegion defineMusculoskeletalRegion();
    
    // Prenatal examination specifics
    ClinicalProtocol createPrenatalProtocol();
    std::vector<ClinicalPoseRequirement> generatePrenatalScanningPoses();
    AnatomicalRegion definePrenatalRegion();
    
    // Breast screening specifics
    ClinicalProtocol createBreastScreeningProtocol();
    std::vector<ClinicalPoseRequirement> generateBreastScanningPoses();
    AnatomicalRegion defineBreastRegion();
    
    // Utility methods
    Eigen::Affine3d createPoseFromPositionAndNormal(const Eigen::Vector3d& position,
                                                    const Eigen::Vector3d& normal,
                                                    const Eigen::Vector3d& up = Eigen::Vector3d::UnitZ());
    
    std::vector<Eigen::Vector3d> generateGridPattern(const AnatomicalRegion& region, int gridResolution);
    std::vector<Eigen::Vector3d> generateSpiralPattern(const AnatomicalRegion& region, int numPoints);
    std::vector<Eigen::Vector3d> generateRandomPattern(const AnatomicalRegion& region, int numPoints);
    
    Eigen::VectorXd poseToJointConfiguration(const Eigen::Affine3d& pose, 
                                            const Eigen::VectorXd& seedConfiguration);
    
    bool isPoseReachable(const Eigen::Affine3d& pose, const Eigen::VectorXd& currentJoints);
    bool isPoseSafe(const Eigen::Affine3d& pose, const AnatomicalRegion& region);
    
    // Breathing simulation
    Eigen::Affine3d applyBreathingMotion(const Eigen::Affine3d& basePose, 
                                        double time, 
                                        double amplitude = 0.01);
    
    // Clinical scoring
    double computeExaminationCoverage(const std::vector<ClinicalPoseRequirement>& achieved,
                                     const std::vector<ClinicalPoseRequirement>& required);
    double computeComfortScore(const std::vector<MotionGenerator::TrajectoryPoint>& trajectory);
    double computeAccuracyScore(const std::vector<Eigen::Affine3d>& achievedPoses,
                               const std::vector<ClinicalPoseRequirement>& requiredPoses);
};

// Utility functions for clinical scenarios
namespace ClinicalUtils {
    std::string examinationTypeToString(ExaminationType type);
    ExaminationType stringToExaminationType(const std::string& typeStr);
    
    // Standard anatomical coordinate frames
    Eigen::Affine3d getAnatomicalCoordinateFrame(const std::string& anatomicalRegion);
    
    // Medical device coordinate transformations
    Eigen::Affine3d probeToAnatomicalTransform();
    Eigen::Affine3d robotToProbeTransform();
    
    // Clinical validation helpers
    bool isWithinAnatomicalLimits(const Eigen::Affine3d& pose, const AnatomicalRegion& region);
    bool satisfiesErgonomicConstraints(const Eigen::VectorXd& jointConfiguration);
    bool meetsClinicalAccuracyRequirements(const Eigen::Affine3d& achievedPose,
                                          const ClinicalPoseRequirement& requirement);
    
    // Patient safety checks
    bool isVelocitySafeForPatient(double velocity, ExaminationType examination);
    bool isAccelerationSafeForPatient(double acceleration, ExaminationType examination);
    bool isForceSafeForPatient(double force, ExaminationType examination);
    
    // Clinical data export
    void exportProtocolToXML(const ClinicalProtocol& protocol, const std::string& filename);
    ClinicalProtocol importProtocolFromXML(const std::string& filename);
    
    void exportScenarioToMedicalFormat(const TestScenario& scenario, const std::string& filename);
}

// Constants for clinical scenarios
namespace ClinicalConstants {
    // Safety limits for different examination types
    const double KNEE_MAX_VELOCITY = 0.08;      // m/s
    const double THYROID_MAX_VELOCITY = 0.05;   // m/s (very gentle)
    const double CARDIAC_MAX_VELOCITY = 0.12;   // m/s
    const double ABDOMINAL_MAX_VELOCITY = 0.10; // m/s
    
    const double KNEE_MAX_ACCELERATION = 0.4;      // m/s²
    const double THYROID_MAX_ACCELERATION = 0.2;   // m/s² (very gentle)
    const double CARDIAC_MAX_ACCELERATION = 0.6;   // m/s²
    const double ABDOMINAL_MAX_ACCELERATION = 0.5; // m/s²
    
    // Contact force limits
    const double KNEE_MAX_CONTACT_FORCE = 10.0;      // N
    const double THYROID_MAX_CONTACT_FORCE = 3.0;    // N (very gentle)
    const double CARDIAC_MAX_CONTACT_FORCE = 8.0;    // N
    const double ABDOMINAL_MAX_CONTACT_FORCE = 7.0;  // N
    
    // Accuracy requirements
    const double CLINICAL_POSITION_TOLERANCE = 0.002;    // 2mm
    const double CLINICAL_ORIENTATION_TOLERANCE = 0.087; // 5 degrees
    
    // Standard examination times
    const double KNEE_EXAMINATION_TIME = 420.0;      // 7 minutes
    const double THYROID_EXAMINATION_TIME = 300.0;   // 5 minutes
    const double CARDIAC_EXAMINATION_TIME = 600.0;   // 10 minutes
    const double ABDOMINAL_EXAMINATION_TIME = 480.0; // 8 minutes
}

} // namespace AdvancedBenchmark

#endif // CLINICAL_SCENARIOS_H