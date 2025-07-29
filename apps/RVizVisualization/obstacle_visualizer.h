/**
 * @file obstacle_visualizer.h
 * @brief Header file for RViz obstacle visualization
 * 
 * @author PathPlanner Team
 * @date 2025
 */

#ifndef OBSTACLE_VISUALIZER_H
#define OBSTACLE_VISUALIZER_H

#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
#include <std_msgs/ColorRGBA.h>
#include <memory>

// Forward declarations
class BVHTree;

/**
 * @brief Class for visualizing obstacle tree bounding boxes in RViz
 */
class ObstacleVisualizer
{
public:
    /**
     * @brief Constructor for ObstacleVisualizer
     * @param nh ROS node handle
     * @param topic_name Topic name for publishing markers
     * @param frame_id Frame ID for markers
     */
    explicit ObstacleVisualizer(ros::NodeHandle& nh, 
                               const std::string& topic_name = "obstacle_markers",
                               const std::string& frame_id = "base_link");

    /**
     * @brief Set the obstacle tree to visualize
     * @param obstacle_tree Shared pointer to BVH obstacle tree
     */
    void setObstacleTree(std::shared_ptr<BVHTree> obstacle_tree);

    /**
     * @brief Set the publishing rate for markers
     * @param rate Publishing rate in Hz
     */
    void setPublishRate(double rate);

    /**
     * @brief Set custom colors for different types of obstacles
     * @param leaf_color Color for leaf node boxes
     * @param internal_color Color for internal node boxes
     */
    void setColors(const std_msgs::ColorRGBA& leaf_color, 
                   const std_msgs::ColorRGBA& internal_color);

    /**
     * @brief Enable/disable visualization of internal BVH nodes
     * @param show_internal True to show internal nodes, false to show only leaves
     */
    void setShowInternalNodes(bool show_internal);

    /**
     * @brief Manually trigger obstacle marker publishing
     */
    void publishObstacles();

private:
    ros::NodeHandle& _nh;
    ros::Publisher _marker_pub;
    ros::Timer _publish_timer;
    
    std::shared_ptr<BVHTree> _obstacle_tree;
    std::string _frame_id;
    int _marker_id_counter;
    double _publish_rate;
    bool _show_internal_nodes;
    
    std_msgs::ColorRGBA _leaf_color;
    std_msgs::ColorRGBA _internal_color;

    void timerCallback(const ros::TimerEvent& event);
    std_msgs::ColorRGBA createColor(float r, float g, float b, float a);
    void clearMarkers(visualization_msgs::MarkerArray& marker_array);
    void addObstacleBoxes(visualization_msgs::MarkerArray& marker_array);
    visualization_msgs::Marker createBoxMarker(const Eigen::Vector3d& center,
                                              const Eigen::Vector3d& half_dims,
                                              const Eigen::Matrix3d& axes,
                                              bool is_leaf);
};

/**
 * @brief Integration class combining trajectory planning with obstacle visualization
 */
class TrajectoryPlannerWithVisualization
{
public:
    TrajectoryPlannerWithVisualization(ros::NodeHandle& nh, 
                                     const std::string& environment_string,
                                     const std::vector<double>& current_joints);
    
    void setEnvironment(const std::string& environment);
    bool planTrajectoriesWithVisualization(bool use_hauser = false, bool enable_shortcutting = true);
    
    // Accessors
    class UltrasoundScanTrajectoryPlanner& getTrajectoryPlanner();
    ObstacleVisualizer& getObstacleVisualizer();

private:
    class UltrasoundScanTrajectoryPlanner _trajectory_planner;
    ObstacleVisualizer _obstacle_visualizer;
};

#endif // OBSTACLE_VISUALIZER_H
