#include "TrajectoryLib/Logging.h"

// Define the global logger instance for TrajectoryLib
trajectory_src::severity_logger<trajectory_severity_level> g_trajectory_logger;

// Optional: Initialize logging for TrajectoryLib
namespace {
    struct TrajectoryLogInit {
        TrajectoryLogInit() {
            // Add common attributes like timestamp
            trajectory_logging::add_common_attributes();
            
            // Set default severity level to warning for TrajectoryLib (disables RobotArm logging by default)
            // To re-enable RobotArm debug logging, change trajectory_warning to trajectory_debug or trajectory_info
            trajectory_logging::core::get()->set_filter(
                trajectory_expr::attr<trajectory_severity_level>("Severity") >= trajectory_warning
            );
        }
    };
    
    // Static instance to ensure initialization
    static TrajectoryLogInit trajectory_log_init;
}
