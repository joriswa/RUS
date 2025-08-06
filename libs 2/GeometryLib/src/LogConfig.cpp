#include "GeometryLib/Logging.h"

#ifndef GEOMETRYLIB_DISABLE_LOGGING
// Define the global logger instance for GeometryLib
geometry_src::severity_logger<geometry_severity_level> g_geometry_logger;

// Optional: Initialize logging for GeometryLib
namespace {
    struct GeometryLogInit {
        GeometryLogInit() {
            // Add common attributes like timestamp
            geometry_logging::add_common_attributes();
            
            // Set default severity level to info for GeometryLib
            geometry_logging::core::get()->set_filter(
                geometry_expr::attr<geometry_severity_level>("Severity") >= geometry_info
            );
        }
    };
    
    // Static instance to ensure initialization
    static GeometryLogInit geometry_log_init;
}
#endif
