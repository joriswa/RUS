#pragma once

#include <ros/ros.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseArray.h>
#include <sensor_msgs/JointState.h>
#include <custom_msgs/String_StringBool.h>
#include <shape_msgs/Mesh.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
#include <std_msgs/ColorRGBA.h>

#include <vector>
#include <string>
#include <memory>
#include <mutex>
#include <custom_msgs/ScanPoseSequencePlan.h>
#include <iostream>

#include "../libs/USLib/include/USLib/USTrajectoryPlanner.h"
#include "GeometryLib/BVHTree.h"
#include "GeometryLib/Obstacle.h"

/**
 * @brief ObstacleVisualizer class for RViz visualization
 */
class ObstacleVisualizer
{
public:
    explicit ObstacleVisualizer(ros::NodeHandle& nh, 
                               const std::string& topic_name = "obstacle_markers",
                               const std::string& frame_id = "base_link");
    
    void setObstacleTree(std::shared_ptr<BVHTree> obstacle_tree);
    void setPublishRate(double rate);
    void setColors(const std_msgs::ColorRGBA& leaf_color, 
                   const std_msgs::ColorRGBA& internal_color);
    void setShowInternalNodes(bool show_internal);
    void publishObstacles();
    
    /**
     * @brief Static utility method to create RGBA colors
     */
    static std_msgs::ColorRGBA createColor(float r, float g, float b, float a);

private:
    ros::NodeHandle& _nh;
    ros::Publisher _marker_pub;
    ros::Timer _publish_timer;
    
    std::shared_ptr<BVHTree> _obstacle_tree;
    std::string _frame_id;
    int _marker_id_counter;
    double _publish_rate;
    bool _show_internal_nodes = false;
    
    std_msgs::ColorRGBA _leaf_color;
    std_msgs::ColorRGBA _internal_color;

    void timerCallback(const ros::TimerEvent& event);
    void clearMarkers(visualization_msgs::MarkerArray& marker_array);
    void addObstacleBoxes(visualization_msgs::MarkerArray& marker_array);
    void addObstacleBoxesRecursive(const BVHNode* node, visualization_msgs::MarkerArray& marker_array);
    visualization_msgs::Marker createBoxMarker(const Eigen::Vector3d& center,
                                              const Eigen::Vector3d& half_dims,
                                              const Eigen::Matrix3d& axes,
                                              bool is_leaf);
};

/**
 * @brief Enhanced PlanningInterface with obstacle visualization
 */
class PlanningInterface
{
public:
    PlanningInterface(ros::NodeHandle &nh);
    ~PlanningInterface();

    bool initialize();

private:
    ros::NodeHandle _nh;
    ros::ServiceServer _scan_sequence_service;
    ros::ServiceServer _environment_update_service;
    ros::ServiceServer _visualization_service;

    std::vector<std::string> _joint_names;
    std::string _robot_description_path;
    std::string _env_description_path;

    UltrasoundScanTrajectoryPlanner *_usPlanner;
    std::unique_ptr<ObstacleVisualizer> _obstacle_visualizer;

    // Existing methods
    void printScanPoseSequenceRequest(const custom_msgs::ScanPoseSequencePlan::Request &req);
    std::pair<std::vector<Eigen::Affine3d>, Eigen::VectorXd> processScanPoseSequenceRequest(const custom_msgs::ScanPoseSequencePlan::Request &req);
    std::vector<unsigned char> boolsToBytes(const std::vector<bool> &bool_flags);

    bool updateEnvironmentCallback(
        custom_msgs::String_StringBool::Request &req,
        custom_msgs::String_StringBool::Response &res);

    bool scanPoseSequenceCallback(
        custom_msgs::ScanPoseSequencePlan::Request &req,
        custom_msgs::ScanPoseSequencePlan::Response &res);

    // New visualization methods
    bool visualizeObstaclesCallback(
        custom_msgs::String_StringBool::Request &req,
        custom_msgs::String_StringBool::Response &res);
    
    void setupObstacleVisualization();
    void updateObstacleVisualization();
};
