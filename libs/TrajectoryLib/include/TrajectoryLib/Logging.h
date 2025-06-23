#ifndef TRAJECTORYLIB_LOGGING_H
#define TRAJECTORYLIB_LOGGING_H

// Self-contained logging for TrajectoryLib using Boost.Log
#include <boost/log/core.hpp>
#include <boost/log/trivial.hpp>
#include <boost/log/expressions.hpp>
#include <boost/log/sinks/text_file_backend.hpp>
#include <boost/log/sinks/text_ostream_backend.hpp>
#include <boost/log/utility/setup/file.hpp>
#include <boost/log/utility/setup/console.hpp>
#include <boost/log/utility/setup/common_attributes.hpp>
#include <boost/log/sources/severity_logger.hpp>
#include <boost/log/sources/record_ostream.hpp>
#include <boost/log/attributes/timer.hpp>
#include <boost/log/attributes/named_scope.hpp>
#include <boost/log/support/date_time.hpp>
#include <boost/core/null_deleter.hpp>
#include <iostream>
#include <fstream>
#include <chrono>

namespace trajectory_logging = boost::log;
namespace trajectory_src = boost::log::sources;
namespace trajectory_expr = boost::log::expressions;

// Define severity levels for TrajectoryLib
enum trajectory_severity_level
{
    trajectory_trace,
    trajectory_debug,
    trajectory_info,
    trajectory_warning,
    trajectory_error,
    trajectory_fatal
};

// Output operator for severity level
template<typename CharT, typename TraitsT>
std::basic_ostream<CharT, TraitsT>& operator<<(std::basic_ostream<CharT, TraitsT>& strm, trajectory_severity_level lvl)
{
    static const char* const str[] = {
        "TRACE",
        "DEBUG", 
        "INFO",
        "WARNING",
        "ERROR",
        "FATAL"
    };
    if (static_cast<std::size_t>(lvl) < (sizeof(str) / sizeof(*str)))
        strm << str[lvl];
    else
        strm << static_cast<int>(lvl);
    return strm;
}

// Global logger instance for TrajectoryLib
extern trajectory_src::severity_logger<trajectory_severity_level> g_trajectory_logger;

// Basic logging macros for TrajectoryLib
#define TRAJECTORY_LOG_TRACE BOOST_LOG_SEV(g_trajectory_logger, trajectory_trace)
#define TRAJECTORY_LOG_DEBUG BOOST_LOG_SEV(g_trajectory_logger, trajectory_debug)
#define TRAJECTORY_LOG_INFO BOOST_LOG_SEV(g_trajectory_logger, trajectory_info)
#define TRAJECTORY_LOG_WARNING BOOST_LOG_SEV(g_trajectory_logger, trajectory_warning)
#define TRAJECTORY_LOG_ERROR BOOST_LOG_SEV(g_trajectory_logger, trajectory_error)
#define TRAJECTORY_LOG_FATAL BOOST_LOG_SEV(g_trajectory_logger, trajectory_fatal)

// Compatibility macros
#define LOG_TRACE TRAJECTORY_LOG_TRACE
#define LOG_DEBUG TRAJECTORY_LOG_DEBUG
#define LOG_INFO TRAJECTORY_LOG_INFO
#define LOG_WARNING TRAJECTORY_LOG_WARNING
#define LOG_ERROR TRAJECTORY_LOG_ERROR
#define LOG_FATAL TRAJECTORY_LOG_FATAL

// Additional compatibility macros used in existing code
#define LOG_INIT(component) TRAJECTORY_LOG_INFO << "[" << component << "] [INIT] "
#define LOG_CONFIG(component) TRAJECTORY_LOG_INFO << "[" << component << "] [CONFIG] "
#define LOG_OPERATION(component) TRAJECTORY_LOG_INFO << "[" << component << "] [OP] "
#define LOG_RESULT(component) TRAJECTORY_LOG_INFO << "[" << component << "] [RESULT] "
#define LOG_ALGORITHM_START(operation) TRAJECTORY_LOG_INFO << "[ALGO] Starting " << operation
#define LOG_ALGORITHM_END(operation, success) TRAJECTORY_LOG_INFO << "[ALGO] " << operation << " " << (success ? "completed successfully" : "failed")
#define LOG_OPTIMIZATION(msg) TRAJECTORY_LOG_DEBUG << "[OPTIMIZATION] " << msg

// Operation logging macros
#define LOG_OPERATION_START(operation) TRAJECTORY_LOG_INFO << "[OP] Starting " << operation
#define LOG_OPERATION_END(operation, success) TRAJECTORY_LOG_INFO << "[OP] " << operation << " " << (success ? "completed successfully" : "failed")

// Robot-specific macros
#define LOG_ROBOT_CONFIG(msg) TRAJECTORY_LOG_INFO << "[ROBOT] [CONFIG] " << msg
#define LOG_KINEMATICS(msg) TRAJECTORY_LOG_DEBUG << "[KINEMATICS] " << msg
#define LOG_COLLISION(msg) TRAJECTORY_LOG_DEBUG << "[COLLISION] " << msg
#define LOG_OBJECT_LIFECYCLE(component, msg) TRAJECTORY_LOG_DEBUG << "[" << component << "] [LIFECYCLE] " << msg
#define LOG_STATUS(component) TRAJECTORY_LOG_INFO << "[" << component << "] [STATUS] "

// Performance logging placeholder (simplified for now)
#define LOG_PERF_START(timer_name) TRAJECTORY_LOG_DEBUG << "[PERF] Starting " << #timer_name
#define LOG_PERF_END(timer_name) TRAJECTORY_LOG_DEBUG << "[PERF] Ending " << #timer_name
#define LOG_ITERATION_MILESTONE(iter, total, cost) TRAJECTORY_LOG_DEBUG << "[MILESTONE] Iteration " << iter << "/" << total << ", cost: " << cost
#define LOG_TRAJECTORY_COMPLETE(points, duration) TRAJECTORY_LOG_INFO << "[RESULT] Trajectory complete: " << points << " points, " << duration << "s duration"
#define LOG_COST_UPDATE(type, cost) TRAJECTORY_LOG_DEBUG << "[COST] " << type << " cost: " << cost
#define LOG_PROGRESS(current, total, task) TRAJECTORY_LOG_INFO << "[PROGRESS] [" << current << "/" << total << "] " << task

// TrajectoryLib-specific component names
#define STOMP "STOMP"
#define ROBOT "ROBOT"
#define PATH "PATH"
#define KINEMATICS "KINEMATICS"
#define IK "IK"
#define COLLISION "COLLISION"
#define REPOSITION "REPOSITION"
#define TRAJECTORY "TRAJECTORY"
#define COST "COST"
#define OPTIMIZATION "OPTIMIZATION"

// STOMP Algorithm logging
#define LOG_STOMP_START() TRAJECTORY_LOG_INFO << "[STOMP] [ALGO] Starting optimization"
#define LOG_STOMP_END(success) TRAJECTORY_LOG_INFO << "[STOMP] [ALGO] optimization " << (success ? "completed successfully" : "failed")
#define LOG_STOMP_ITERATION(iter, total) TRAJECTORY_LOG_DEBUG << "[STOMP] [ITER] " << iter << "/" << total
#define LOG_STOMP_CONVERGENCE(iterations, final_cost) \
    TRAJECTORY_LOG_INFO << "[STOMP] [RESULT] Converged after " << iterations << " iterations, final cost: " << final_cost

// Robot Configuration logging
#define LOG_ROBOT_CONFIG_START() TRAJECTORY_LOG_INFO << "[ROBOT] [INIT] "
#define LOG_ROBOT_CONFIG_PARAM(param, value) TRAJECTORY_LOG_INFO << "[ROBOT] [CONFIG] " << param << ": " << value
#define LOG_ROBOT_STATUS(status) TRAJECTORY_LOG_INFO << "[ROBOT] [STATUS] " << status

// Path Planning logging
#define LOG_PATH_PLANNING_START() TRAJECTORY_LOG_INFO << "[PATH] [ALGO] Starting planning"
#define LOG_PATH_PLANNING_END(success) TRAJECTORY_LOG_INFO << "[PATH] [ALGO] planning " << (success ? "completed successfully" : "failed")
#define LOG_PATH_WAYPOINT(index, pose) TRAJECTORY_LOG_DEBUG << "[PATH] [OP] Waypoint " << index << ": " << pose
#define LOG_PATH_RESULT(waypoints, length) TRAJECTORY_LOG_INFO << "[PATH] [RESULT] Generated " << waypoints << " waypoints, path length: " << length

// Kinematics logging
#define LOG_KINEMATICS_FORWARD(joint_config) TRAJECTORY_LOG_DEBUG << "[KINEMATICS] [OP] Forward kinematics for: " << joint_config
#define LOG_KINEMATICS_INVERSE(target_pose) TRAJECTORY_LOG_DEBUG << "[KINEMATICS] [OP] Inverse kinematics for: " << target_pose
#define LOG_KINEMATICS_ERROR(error_msg) TRAJECTORY_LOG_ERROR << "[KINEMATICS] [COMPUTATION] " << error_msg

// Cost and optimization logging
#define LOG_COST_COMPUTATION(cost_type, value) TRAJECTORY_LOG_DEBUG << "[COST] [OP] " << cost_type << " cost: " << value
#define LOG_COST_UPDATE_DETAILED(iteration, total_cost, improvement) \
    TRAJECTORY_LOG_DEBUG << "[COST] Iteration " << iteration << ", cost: " << total_cost << ", improvement: " << improvement
#define LOG_COST_BREAKDOWN(smooth_cost, collision_cost, total) \
    TRAJECTORY_LOG_INFO << "[COST] [RESULT] Smoothness: " << smooth_cost << ", Collision: " << collision_cost << ", Total: " << total

// Collision detection logging with configurable verbosity
#ifndef ENABLE_COLLISION_DEBUG_LOGGING
#define ENABLE_COLLISION_DEBUG_LOGGING 0  // Disabled by default
#endif

#define LOG_COLLISION_CHECK_START(object_count) TRAJECTORY_LOG_DEBUG << "[COLLISION] [OP] Checking " << object_count << " objects"

#if ENABLE_COLLISION_DEBUG_LOGGING
#define LOG_COLLISION_DETECTED(object1, object2, distance) \
    TRAJECTORY_LOG_WARNING << "[COLLISION] [DETECTED] " << object1 << " - " << object2 << " (distance: " << distance << ")"
#else
#define LOG_COLLISION_DETECTED(object1, object2, distance) do {} while(0)
#endif

#define LOG_COLLISION_FREE() TRAJECTORY_LOG_INFO << "[COLLISION] [STATUS] path is collision-free"
#define LOG_COLLISION_AVOIDANCE_APPLIED(method) TRAJECTORY_LOG_DEBUG << "[COLLISION] [OP] Applied avoidance method: " << method

// Trajectory generation logging  
#define LOG_TRAJECTORY_START(target_points) TRAJECTORY_LOG_INFO << "[TRAJECTORY] [ALGO] Starting generation for " << target_points << " points"
#define LOG_TRAJECTORY_END(success, actual_points) TRAJECTORY_LOG_INFO << "[TRAJECTORY] [ALGO] generation " << (success ? "completed successfully" : "failed") << ", generated " << actual_points << " points"
#define LOG_TRAJECTORY_POINT(index, time, position) \
    TRAJECTORY_LOG_TRACE << "[TRAJECTORY] Point " << index << " at t=" << time << "s, pos=" << position
#define LOG_TRAJECTORY_TIMING(total_duration, avg_velocity) \
    TRAJECTORY_LOG_INFO << "[TRAJECTORY] [RESULT] Duration: " << total_duration << "s, avg velocity: " << avg_velocity

// IK (Inverse Kinematics) logging
#define LOG_IK_ATTEMPT_START(pose_index) TRAJECTORY_LOG_DEBUG << "[IK] [OP] Solving pose " << pose_index
#define LOG_IK_ATTEMPT_DETAIL(pose_index, attempt, method) \
    TRAJECTORY_LOG_DEBUG << "[IK] Pose " << pose_index << ", attempt " << attempt << " using " << method
#define LOG_IK_SUCCESS(pose_index, joint_distance, solve_time) \
    TRAJECTORY_LOG_INFO << "[IK] [RESULT] Pose " << pose_index << " solved in " << solve_time << "ms, joint distance: " << joint_distance
#define LOG_IK_FAILURE(pose_index, reason, attempts) \
    TRAJECTORY_LOG_ERROR << "[IK] [SOLVE_FAILED] Pose " << pose_index << " after " << attempts << " attempts: " << reason

// Repositioning logging
#define LOG_REPOSITIONING_NEEDED(pose_index, reason) \
    TRAJECTORY_LOG_WARNING << "[REPOSITION] [REQUIRED] Pose " << pose_index << ": " << reason
#define LOG_REPOSITIONING_START(pose_index, strategy) \
    TRAJECTORY_LOG_INFO << "[REPOSITION] [ALGO] Starting pose " << pose_index << " using " << strategy
#define LOG_REPOSITIONING_END(pose_index, success, new_pose) \
    TRAJECTORY_LOG_INFO << "[REPOSITION] [ALGO] pose " << pose_index << " " << (success ? "completed successfully" : "failed") << (success ? (", new pose: " + new_pose) : "")
#define LOG_REPOSITIONING_FALLBACK(pose_index, fallback_method) \
    TRAJECTORY_LOG_DEBUG << "[REPOSITION] [OP] Pose " << pose_index << " using fallback: " << fallback_method

// File I/O logging
#define LOG_FILE_READ(path) TRAJECTORY_LOG_DEBUG << "[FILE] Reading: " << path
#define LOG_FILE_WRITE(path) TRAJECTORY_LOG_DEBUG << "[FILE] Writing: " << path
#define LOG_FILE_ERROR(path, error) TRAJECTORY_LOG_ERROR << "[FILE] Error with " << path << ": " << error

#endif // TRAJECTORYLIB_LOGGING_H