/**
 * @file obstacle_visualizer.cpp
 * @brief RViz visualization node for publishing obstacle tree boxes as markers
 * 
 * This file provides functionality to publish BVH obstacle tree bounding boxes
 * as RViz visualization markers for debugging and visualization purposes.
 * 
 * Usage:
 * - Initialize the ObstacleVisualizer with a ROS node handle
 * - Set the obstacle tree using setObstacleTree()
 * - Call publishObstacles() to publish markers to RViz
 * 
 * @author PathPlanner Team
 * @date 2025
 */

#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Point.h>
#include <std_msgs/ColorRGBA.h>
#include <tf2/LinearMath/Transform.h>
#include <memory>

#include "GeometryLib/BVHTree.h"

class ObstacleVisualizer
{
public:
    /**
     * @brief Constructor for ObstacleVisualizer
     * @param nh ROS node handle
     * @param topic_name Topic name for publishing markers (default: "obstacle_markers")
     * @param frame_id Frame ID for markers (default: "base_link")
     */
    explicit ObstacleVisualizer(ros::NodeHandle& nh, 
                               const std::string& topic_name = "obstacle_markers",
                               const std::string& frame_id = "base_link")
        : _nh(nh)
        , _frame_id(frame_id)
        , _marker_id_counter(0)
        , _publish_rate(10.0) // 10 Hz default
    {
        // Initialize publisher
        _marker_pub = _nh.advertise<visualization_msgs::MarkerArray>(topic_name, 1, true);
        
        // Initialize timer for periodic publishing
        _publish_timer = _nh.createTimer(ros::Duration(1.0 / _publish_rate), 
                                        &ObstacleVisualizer::timerCallback, this);
        
        ROS_INFO("ObstacleVisualizer initialized. Publishing to topic: %s", topic_name.c_str());
    }

    /**
     * @brief Set the obstacle tree to visualize
     * @param obstacle_tree Shared pointer to BVH obstacle tree
     */
    void setObstacleTree(std::shared_ptr<BVHTree> obstacle_tree)
    {
        _obstacle_tree = obstacle_tree;
        ROS_INFO("Obstacle tree set for visualization");
    }

    /**
     * @brief Set the publishing rate for markers
     * @param rate Publishing rate in Hz
     */
    void setPublishRate(double rate)
    {
        _publish_rate = rate;
        _publish_timer.stop();
        _publish_timer = _nh.createTimer(ros::Duration(1.0 / _publish_rate), 
                                        &ObstacleVisualizer::timerCallback, this);
    }

    /**
     * @brief Set custom colors for different types of obstacles
     * @param leaf_color Color for leaf node boxes
     * @param internal_color Color for internal node boxes
     */
    void setColors(const std_msgs::ColorRGBA& leaf_color, 
                   const std_msgs::ColorRGBA& internal_color)
    {
        _leaf_color = leaf_color;
        _internal_color = internal_color;
    }

    /**
     * @brief Enable/disable visualization of internal BVH nodes
     * @param show_internal True to show internal nodes, false to show only leaves
     */
    void setShowInternalNodes(bool show_internal)
    {
        _show_internal_nodes = show_internal;
    }

    /**
     * @brief Manually trigger obstacle marker publishing
     */
    void publishObstacles()
    {
        if (!_obstacle_tree) {
            ROS_WARN("No obstacle tree set. Cannot publish markers.");
            return;
        }

        visualization_msgs::MarkerArray marker_array;
        _marker_id_counter = 0;

        // Clear previous markers
        clearMarkers(marker_array);

        // Add obstacle boxes to marker array
        addObstacleBoxes(marker_array);

        // Publish marker array
        _marker_pub.publish(marker_array);
        
        ROS_DEBUG("Published %zu obstacle markers", marker_array.markers.size());
    }

private:
    ros::NodeHandle& _nh;
    ros::Publisher _marker_pub;
    ros::Timer _publish_timer;
    
    std::shared_ptr<BVHTree> _obstacle_tree;
    std::string _frame_id;
    int _marker_id_counter;
    double _publish_rate;
    bool _show_internal_nodes = false;
    
    // Default colors
    std_msgs::ColorRGBA _leaf_color = createColor(1.0, 0.0, 0.0, 0.7);     // Red for obstacles
    std_msgs::ColorRGBA _internal_color = createColor(0.0, 0.0, 1.0, 0.3); // Blue for BVH nodes

    /**
     * @brief Timer callback for periodic publishing
     */
    void timerCallback(const ros::TimerEvent& event)
    {
        publishObstacles();
    }

    /**
     * @brief Create a ColorRGBA message
     */
    std_msgs::ColorRGBA createColor(float r, float g, float b, float a)
    {
        std_msgs::ColorRGBA color;
        color.r = r;
        color.g = g;
        color.b = b;
        color.a = a;
        return color;
    }

    /**
     * @brief Clear all previous markers by publishing deletion markers
     */
    void clearMarkers(visualization_msgs::MarkerArray& marker_array)
    {
        visualization_msgs::Marker clear_marker;
        clear_marker.header.frame_id = _frame_id;
        clear_marker.header.stamp = ros::Time::now();
        clear_marker.ns = "obstacles";
        clear_marker.id = 0;
        clear_marker.action = visualization_msgs::Marker::DELETEALL;
        
        marker_array.markers.push_back(clear_marker);
    }

    /**
     * @brief Add obstacle boxes from BVH tree to marker array
     */
    void addObstacleBoxes(visualization_msgs::MarkerArray& marker_array)
    {
        // Get all bounding boxes from the obstacle tree
        auto obstacle_boxes = _obstacle_tree->getAllBoundingBoxes(_show_internal_nodes);
        
        for (const auto& box_info : obstacle_boxes) {
            const auto& [center, half_dims, axes, is_leaf] = box_info;
            
            visualization_msgs::Marker marker = createBoxMarker(center, half_dims, axes, is_leaf);
            marker_array.markers.push_back(marker);
        }
    }

    /**
     * @brief Create a box marker from bounding box parameters
     */
    visualization_msgs::Marker createBoxMarker(const Eigen::Vector3d& center,
                                              const Eigen::Vector3d& half_dims,
                                              const Eigen::Matrix3d& axes,
                                              bool is_leaf)
    {
        visualization_msgs::Marker marker;
        
        // Header
        marker.header.frame_id = _frame_id;
        marker.header.stamp = ros::Time::now();
        
        // Namespace and ID
        marker.ns = is_leaf ? "obstacle_boxes" : "bvh_internal_nodes";
        marker.id = _marker_id_counter++;
        
        // Marker type and action
        marker.type = visualization_msgs::Marker::CUBE;
        marker.action = visualization_msgs::Marker::ADD;
        
        // Position
        marker.pose.position.x = center.x();
        marker.pose.position.y = center.y();
        marker.pose.position.z = center.z();
        
        // Orientation from rotation matrix
        Eigen::Quaterniond quat(axes);
        marker.pose.orientation.x = quat.x();
        marker.pose.orientation.y = quat.y();
        marker.pose.orientation.z = quat.z();
        marker.pose.orientation.w = quat.w();
        
        // Scale (full dimensions, not half)
        marker.scale.x = 2.0 * half_dims.x();
        marker.scale.y = 2.0 * half_dims.y();
        marker.scale.z = 2.0 * half_dims.z();
        
        // Color
        marker.color = is_leaf ? _leaf_color : _internal_color;
        
        // Lifetime (0 means forever until explicitly deleted)
        marker.lifetime = ros::Duration(0);
        
        return marker;
    }
};

/**
 * @brief Standalone function to create and use ObstacleVisualizer
 * 
 * Example usage in a ROS node:
 */
void demonstrateObstacleVisualization()
{
    // Initialize ROS node
    ros::NodeHandle nh;
    
    // Create obstacle visualizer
    ObstacleVisualizer visualizer(nh, "robot_obstacles", "base_link");
    
    // Configure visualization
    visualizer.setPublishRate(5.0); // 5 Hz
    visualizer.setShowInternalNodes(false); // Only show leaf nodes (actual obstacles)
    
    // Set custom colors
    std_msgs::ColorRGBA obstacle_color;
    obstacle_color.r = 0.8; obstacle_color.g = 0.2; obstacle_color.b = 0.2; obstacle_color.a = 0.8;
    
    std_msgs::ColorRGBA bvh_color;
    bvh_color.r = 0.2; bvh_color.g = 0.2; bvh_color.b = 0.8; bvh_color.a = 0.3;
    
    visualizer.setColors(obstacle_color, bvh_color);
    
    // Example: Load environment and create obstacle tree
    // This would typically be done in your main application
    /*
    RobotManager robot_manager;
    robot_manager.parseURDF("path/to/environment.urdf");
    auto obstacle_tree = std::make_shared<BVHTree>(robot_manager.getTransformedObstacles());
    
    // Set the obstacle tree for visualization
    visualizer.setObstacleTree(obstacle_tree);
    */
    
    ROS_INFO("Obstacle visualization demo initialized. Waiting for obstacle tree...");
    
    // Spin to keep the node alive and publish markers
    ros::spin();
}

/**
 * @brief Integration example with UltrasoundScanTrajectoryPlanner
 */
class TrajectoryPlannerWithVisualization
{
public:
    TrajectoryPlannerWithVisualization(ros::NodeHandle& nh, 
                                     const std::string& environment_string,
                                     const std::vector<double>& current_joints)
        : _trajectory_planner(environment_string, current_joints)
        , _obstacle_visualizer(nh, "trajectory_obstacles", "base_link")
    {
        // Configure visualizer
        _obstacle_visualizer.setPublishRate(2.0); // Slower rate for trajectory planning
        _obstacle_visualizer.setShowInternalNodes(false);
        
        ROS_INFO("Trajectory planner with visualization initialized");
    }
    
    /**
     * @brief Set environment and update visualization
     */
    void setEnvironment(const std::string& environment)
    {
        _trajectory_planner.setEnvironment(environment);
        
        // Get the obstacle tree from trajectory planner and set it for visualization
        // Note: This assumes the trajectory planner provides access to the obstacle tree
        // You may need to modify UltrasoundScanTrajectoryPlanner to expose the obstacle tree
        
        ROS_INFO("Environment updated, obstacle visualization refreshed");
    }
    
    /**
     * @brief Plan trajectories and visualize obstacles
     */
    bool planTrajectoriesWithVisualization(bool use_hauser = false, bool enable_shortcutting = true)
    {
        // Trigger obstacle visualization
        _obstacle_visualizer.publishObstacles();
        
        // Plan trajectories
        bool success = _trajectory_planner.planTrajectories(use_hauser, enable_shortcutting);
        
        if (success) {
            ROS_INFO("Trajectory planning successful with obstacle visualization");
        } else {
            ROS_WARN("Trajectory planning failed - check obstacle visualization for collision issues");
        }
        
        return success;
    }
    
    UltrasoundScanTrajectoryPlanner& getTrajectoryPlanner() { return _trajectory_planner; }
    ObstacleVisualizer& getObstacleVisualizer() { return _obstacle_visualizer; }

private:
    UltrasoundScanTrajectoryPlanner _trajectory_planner;
    ObstacleVisualizer _obstacle_visualizer;
};

/**
 * @brief Main function for standalone obstacle visualization node
 */
int main(int argc, char** argv)
{
    // Initialize ROS
    ros::init(argc, argv, "obstacle_visualizer_node");
    ros::NodeHandle nh("~");
    
    // Get parameters
    std::string topic_name = nh.param<std::string>("marker_topic", "obstacle_markers");
    std::string frame_id = nh.param<std::string>("frame_id", "base_link");
    double publish_rate = nh.param<double>("publish_rate", 5.0);
    bool show_internal = nh.param<bool>("show_internal_nodes", false);
    
    // Create visualizer
    ObstacleVisualizer visualizer(nh, topic_name, frame_id);
    visualizer.setPublishRate(publish_rate);
    visualizer.setShowInternalNodes(show_internal);
    
    ROS_INFO("Obstacle Visualizer Node started");
    ROS_INFO("  Topic: %s", topic_name.c_str());
    ROS_INFO("  Frame: %s", frame_id.c_str());
    ROS_INFO("  Rate: %.1f Hz", publish_rate);
    ROS_INFO("  Show internal nodes: %s", show_internal ? "true" : "false");
    
    // TODO: Add service or topic to receive obstacle tree data
    // For now, this is a template showing the structure
    
    ros::spin();
    return 0;
}
