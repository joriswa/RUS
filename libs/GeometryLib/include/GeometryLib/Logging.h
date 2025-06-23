#ifndef GEOMETRYLIB_LOGGING_H
#define GEOMETRYLIB_LOGGING_H

// LOGGING PERFORMANCE OPTIMIZATION:
// To disable expensive logging by default, uncomment the next line:
#define GEOMETRYLIB_DISABLE_LOGGING
// To enable logging for debugging, comment out the line above
//
// This disables all LOG_ macros related to:
// - BVH tree operations and queries
// - Collision detection and intersection tests  
// - Geometric calculations and kinematics
// - Spatial transformations and proximity queries
// - Mesh operations and simplification
// - Distance computations and gradient calculations
//
// The logging infrastructure remains intact and can be re-enabled by
// commenting out the GEOMETRYLIB_DISABLE_LOGGING define above.

#ifndef GEOMETRYLIB_DISABLE_LOGGING
// Self-contained logging for GeometryLib using Boost.Log
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
#endif

#include <iostream>
#include <fstream>
#include <chrono>

#ifndef GEOMETRYLIB_DISABLE_LOGGING
namespace geometry_logging = boost::log;
namespace geometry_src = boost::log::sources;
namespace geometry_expr = boost::log::expressions;

// Define severity levels for GeometryLib
enum geometry_severity_level
{
    geometry_trace,
    geometry_debug,
    geometry_info,
    geometry_warning,
    geometry_error,
    geometry_fatal
};

// Output operator for severity level
template<typename CharT, typename TraitsT>
std::basic_ostream<CharT, TraitsT>& operator<<(std::basic_ostream<CharT, TraitsT>& strm, geometry_severity_level lvl)
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

// Global logger instance for GeometryLib
extern geometry_src::severity_logger<geometry_severity_level> g_geometry_logger;
#endif

#ifdef GEOMETRYLIB_DISABLE_LOGGING
// Basic logging macros for GeometryLib - DISABLED (no-op)
#define GEOMETRY_LOG_TRACE if(false) std::cerr
#define GEOMETRY_LOG_DEBUG if(false) std::cerr
#define GEOMETRY_LOG_INFO if(false) std::cerr
#define GEOMETRY_LOG_WARNING if(false) std::cerr
#define GEOMETRY_LOG_ERROR if(false) std::cerr
#define GEOMETRY_LOG_FATAL if(false) std::cerr
#else
// Basic logging macros for GeometryLib - ENABLED
#define GEOMETRY_LOG_TRACE BOOST_LOG_SEV(g_geometry_logger, geometry_trace)
#define GEOMETRY_LOG_DEBUG BOOST_LOG_SEV(g_geometry_logger, geometry_debug)
#define GEOMETRY_LOG_INFO BOOST_LOG_SEV(g_geometry_logger, geometry_info)
#define GEOMETRY_LOG_WARNING BOOST_LOG_SEV(g_geometry_logger, geometry_warning)
#define GEOMETRY_LOG_ERROR BOOST_LOG_SEV(g_geometry_logger, geometry_error)
#define GEOMETRY_LOG_FATAL BOOST_LOG_SEV(g_geometry_logger, geometry_fatal)
#endif

#ifdef GEOMETRYLIB_DISABLE_LOGGING
// Missing macros used in BVHTree.cpp - DISABLED (no-op)
#define LOG_COLLISION_DETECTED(obj1, obj2, distance) do {} while(0)
#define LOG_ALGORITHM_START(component, operation) do {} while(0)
#define LOG_ALGORITHM_END(component, operation, success) do {} while(0)
#define LOG_CONFIG(component) if(false) std::cerr
#define LOG_RESULT(component) if(false) std::cerr
#else
// Missing macros used in BVHTree.cpp - ENABLED
#define LOG_COLLISION_DETECTED(obj1, obj2, distance) \
    GEOMETRY_LOG_WARNING << "[COLLISION] Detected collision between " << obj1 << " and " << obj2 << " (distance: " << distance << ")"
#define LOG_ALGORITHM_START(component, operation) \
    GEOMETRY_LOG_INFO << "[" << component << "] [ALGO] Starting " << operation
#define LOG_ALGORITHM_END(component, operation, success) \
    GEOMETRY_LOG_INFO << "[" << component << "] [ALGO] " << operation << " " << (success ? "completed successfully" : "failed")
#define LOG_CONFIG(component) GEOMETRY_LOG_INFO << "[" << component << "] [CONFIG] "
#define LOG_RESULT(component) GEOMETRY_LOG_INFO << "[" << component << "] [RESULT] "
#endif

// GeometryLib-specific component names
#define BVH "BVH"
#define OBSTACLE "OBSTACLE"
#define TREE "TREE"
#define COLLISION_GEOMETRY "COLLISION_GEOMETRY"
#define MESH "MESH"
#define SPATIAL "SPATIAL"
#define GEOMETRY_CALC "GEOMETRY_CALC"

#ifdef GEOMETRYLIB_DISABLE_LOGGING
// BVH (Bounding Volume Hierarchy) logging - DISABLED (no-op)
#define LOG_BVH_BUILD_START(node_count) do {} while(0)
#define LOG_BVH_BUILD_END(success, depth, leaf_count) do {} while(0)
#define LOG_BVH_QUERY_START(query_type) do {} while(0)
#define LOG_BVH_QUERY_RESULT(query_type, result_count, query_time) do {} while(0)
#define LOG_BVH_NODE_SPLIT(node_id, split_axis, split_value) do {} while(0)
#define LOG_BVH_OPTIMIZATION(old_nodes, new_nodes, improvement) do {} while(0)
#else
// BVH (Bounding Volume Hierarchy) logging - ENABLED
#define LOG_BVH_BUILD_START(node_count) GEOMETRY_LOG_INFO << "[BVH] [ALGO] Starting building tree with " << node_count << " nodes"
#define LOG_BVH_BUILD_END(success, depth, leaf_count) \
    GEOMETRY_LOG_INFO << "[BVH] [ALGO] building " << (success ? "completed successfully" : "failed") << ", depth: " << depth << ", leaves: " << leaf_count
#define LOG_BVH_QUERY_START(query_type) GEOMETRY_LOG_DEBUG << "[BVH] [OP] Starting " << query_type << " query"
#define LOG_BVH_QUERY_RESULT(query_type, result_count, query_time) \
    GEOMETRY_LOG_INFO << "[BVH] [RESULT] " << query_type << " query: " << result_count << " results in " << query_time << "ms"
#define LOG_BVH_NODE_SPLIT(node_id, split_axis, split_value) \
    GEOMETRY_LOG_DEBUG << "[BVH] Node " << node_id << " split on " << split_axis << " at " << split_value
#define LOG_BVH_OPTIMIZATION(old_nodes, new_nodes, improvement) \
    GEOMETRY_LOG_INFO << "[BVH] [RESULT] Optimized: " << old_nodes << " -> " << new_nodes << " nodes (" << improvement << "% reduction)"
#endif

#ifdef GEOMETRYLIB_DISABLE_LOGGING
// Obstacle logging - DISABLED (no-op)
#define LOG_OBSTACLE_LOAD_START(file_path) do {} while(0)
#define LOG_OBSTACLE_LOAD_END(success, count) do {} while(0)
#define LOG_OBSTACLE_ADD(obstacle_id, type, bounds) do {} while(0)
#define LOG_OBSTACLE_REMOVE(obstacle_id) do {} while(0)
#define LOG_OBSTACLE_COLLISION_CHECK(obstacle_id, object, result) do {} while(0)
#define LOG_OBSTACLE_UPDATE(obstacle_id, change_type) do {} while(0)
#else
// Obstacle logging - ENABLED
#define LOG_OBSTACLE_LOAD_START(file_path) GEOMETRY_LOG_DEBUG << "[OBSTACLE] [OP] Loading obstacles from " << file_path
#define LOG_OBSTACLE_LOAD_END(success, count) \
    GEOMETRY_LOG_INFO << "[OBSTACLE] [RESULT] Loaded " << count << " obstacles " << (success ? "successfully" : "with errors")
#define LOG_OBSTACLE_ADD(obstacle_id, type, bounds) \
    GEOMETRY_LOG_DEBUG << "[OBSTACLE] [OP] Added " << type << " obstacle " << obstacle_id << " with bounds: " << bounds
#define LOG_OBSTACLE_REMOVE(obstacle_id) GEOMETRY_LOG_DEBUG << "[OBSTACLE] [OP] Removed obstacle " << obstacle_id
#define LOG_OBSTACLE_COLLISION_CHECK(obstacle_id, object, result) \
    GEOMETRY_LOG_DEBUG << "[OBSTACLE] Collision check with " << obstacle_id << " and " << object << ": " << (result ? "collision" : "clear")
#define LOG_OBSTACLE_UPDATE(obstacle_id, change_type) \
    GEOMETRY_LOG_DEBUG << "[OBSTACLE] [OP] Updated obstacle " << obstacle_id << ": " << change_type
#endif

#ifdef GEOMETRYLIB_DISABLE_LOGGING
// Spatial tree logging - DISABLED (no-op)
#define LOG_TREE_INSERT(item_id, position) do {} while(0)
#define LOG_TREE_REMOVE(item_id) do {} while(0)
#define LOG_TREE_BALANCE_START() do {} while(0)
#define LOG_TREE_BALANCE_END(old_depth, new_depth) do {} while(0)
#define LOG_TREE_SEARCH(search_type, region, result_count) do {} while(0)
#else
// Spatial tree logging - ENABLED
#define LOG_TREE_INSERT(item_id, position) GEOMETRY_LOG_DEBUG << "[TREE] Inserted item " << item_id << " at " << position
#define LOG_TREE_REMOVE(item_id) GEOMETRY_LOG_DEBUG << "[TREE] Removed item " << item_id
#define LOG_TREE_BALANCE_START() GEOMETRY_LOG_DEBUG << "[TREE] [OP] Starting tree rebalancing"
#define LOG_TREE_BALANCE_END(old_depth, new_depth) \
    GEOMETRY_LOG_INFO << "[TREE] [RESULT] Rebalancing complete: depth " << old_depth << " -> " << new_depth
#define LOG_TREE_SEARCH(search_type, region, result_count) \
    GEOMETRY_LOG_DEBUG << "[TREE] [OP] " << search_type << " search in region " << region << " found " << result_count << " items"
#endif

#ifdef GEOMETRYLIB_DISABLE_LOGGING
// Collision geometry logging - DISABLED (no-op) - expensive intersection tests
#define LOG_COLLISION_GEOMETRY_INIT(geometry_type, detail_level) do {} while(0)
#define LOG_COLLISION_PRIMITIVE_CREATE(primitive_type, parameters) do {} while(0)
#define LOG_COLLISION_MESH_SIMPLIFY(original_faces, simplified_faces, error_tolerance) do {} while(0)
#define LOG_COLLISION_DISTANCE_COMPUTE(obj1, obj2, distance) do {} while(0)
#else
// Collision geometry logging - ENABLED
#define LOG_COLLISION_GEOMETRY_INIT(geometry_type, detail_level) \
    GEOMETRY_LOG_INFO << "[COLLISION_GEOMETRY] [INIT] " << geometry_type << " with detail level " << detail_level
#define LOG_COLLISION_PRIMITIVE_CREATE(primitive_type, parameters) \
    GEOMETRY_LOG_DEBUG << "[COLLISION_GEOMETRY] [OP] Created " << primitive_type << " primitive: " << parameters
#define LOG_COLLISION_MESH_SIMPLIFY(original_faces, simplified_faces, error_tolerance) \
    GEOMETRY_LOG_INFO << "[COLLISION_GEOMETRY] [RESULT] Mesh simplified: " << original_faces << " -> " << simplified_faces << " faces (tolerance: " << error_tolerance << ")"
#define LOG_COLLISION_DISTANCE_COMPUTE(obj1, obj2, distance) \
    GEOMETRY_LOG_DEBUG << "[COLLISION_GEOMETRY] Distance between " << obj1 << " and " << obj2 << ": " << distance
#endif

#ifdef GEOMETRYLIB_DISABLE_LOGGING
// Mesh operations logging - DISABLED (no-op)
#define LOG_MESH_LOAD_START(file_path, format) do {} while(0)
#define LOG_MESH_LOAD_END(success, vertices, faces) do {} while(0)
#define LOG_MESH_PROCESS_START(operation) do {} while(0)
#define LOG_MESH_PROCESS_END(operation, success) do {} while(0)
#define LOG_MESH_VALIDATE(errors, warnings) do {} while(0)
#define LOG_MESH_OPTIMIZE(optimization_type, improvement) do {} while(0)
#else
// Mesh operations logging - ENABLED
#define LOG_MESH_LOAD_START(file_path, format) GEOMETRY_LOG_DEBUG << "[MESH] [OP] Loading " << format << " mesh from " << file_path
#define LOG_MESH_LOAD_END(success, vertices, faces) \
    GEOMETRY_LOG_INFO << "[MESH] [RESULT] Mesh load " << (success ? "successful" : "failed") << ": " << vertices << " vertices, " << faces << " faces"
#define LOG_MESH_PROCESS_START(operation) GEOMETRY_LOG_INFO << "[MESH] [ALGO] Starting " << operation
#define LOG_MESH_PROCESS_END(operation, success) GEOMETRY_LOG_INFO << "[MESH] [ALGO] " << operation << " " << (success ? "completed successfully" : "failed")
#define LOG_MESH_VALIDATE(errors, warnings) \
    GEOMETRY_LOG_INFO << "[MESH] [STATUS] validation complete - " << errors << " errors, " << warnings << " warnings"
#define LOG_MESH_OPTIMIZE(optimization_type, improvement) \
    GEOMETRY_LOG_INFO << "[MESH] [RESULT] " << optimization_type << " optimization: " << improvement << "% improvement"
#endif

#ifdef GEOMETRYLIB_DISABLE_LOGGING
// Spatial operations logging - DISABLED (no-op) - expensive geometric calculations
#define LOG_SPATIAL_TRANSFORM(object_id, transform_type, parameters) do {} while(0)
#define LOG_SPATIAL_INTERSECTION(obj1, obj2, intersection_type, result) do {} while(0)
#define LOG_SPATIAL_BOUNDS_UPDATE(object_id, old_bounds, new_bounds) do {} while(0)
#define LOG_SPATIAL_PROXIMITY_QUERY(center, radius, result_count) do {} while(0)
#else
// Spatial operations logging - ENABLED
#define LOG_SPATIAL_TRANSFORM(object_id, transform_type, parameters) \
    GEOMETRY_LOG_DEBUG << "[SPATIAL] [OP] " << object_id << " " << transform_type << ": " << parameters
#define LOG_SPATIAL_INTERSECTION(obj1, obj2, intersection_type, result) \
    GEOMETRY_LOG_DEBUG << "[SPATIAL] [OP] " << obj1 << " ∩ " << obj2 << " (" << intersection_type << "): " << result
#define LOG_SPATIAL_BOUNDS_UPDATE(object_id, old_bounds, new_bounds) \
    GEOMETRY_LOG_DEBUG << "[SPATIAL] " << object_id << " bounds: " << old_bounds << " -> " << new_bounds
#define LOG_SPATIAL_PROXIMITY_QUERY(center, radius, result_count) \
    GEOMETRY_LOG_DEBUG << "[SPATIAL] [OP] Proximity query at " << center << " (r=" << radius << "): " << result_count << " objects"
#endif

#ifdef GEOMETRYLIB_DISABLE_LOGGING
// Geometric calculations logging - DISABLED (no-op) - expensive kinematics & numerical computations
#define LOG_GEOMETRY_CALC_START(calculation_type, input_data) do {} while(0)
#define LOG_GEOMETRY_CALC_RESULT(calculation_type, result, computation_time) do {} while(0)
#define LOG_GEOMETRY_PRECISION_WARNING(calculation, precision_loss) do {} while(0)
#define LOG_GEOMETRY_NUMERICAL_ERROR(calculation, error_details) do {} while(0)
#else
// Geometric calculations logging - ENABLED
#define LOG_GEOMETRY_CALC_START(calculation_type, input_data) \
    GEOMETRY_LOG_DEBUG << "[GEOMETRY_CALC] [OP] Starting " << calculation_type << " with input: " << input_data
#define LOG_GEOMETRY_CALC_RESULT(calculation_type, result, computation_time) \
    GEOMETRY_LOG_DEBUG << "[GEOMETRY_CALC] [RESULT] " << calculation_type << " result: " << result << " (computed in " << computation_time << "ms)"
#define LOG_GEOMETRY_PRECISION_WARNING(calculation, precision_loss) \
    GEOMETRY_LOG_WARNING << "[GEOMETRY_CALC] [PRECISION] " << calculation << " precision loss: " << precision_loss
#define LOG_GEOMETRY_NUMERICAL_ERROR(calculation, error_details) \
    GEOMETRY_LOG_ERROR << "[GEOMETRY_CALC] [NUMERICAL] " << calculation << " error: " << error_details
#endif

#endif // GEOMETRYLIB_LOGGING_H
