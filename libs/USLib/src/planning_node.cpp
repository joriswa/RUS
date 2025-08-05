#include "USLib/planning_node.h"
#include "TrajectoryLib/Logger.h"
#include <sstream>
#include <iomanip>

// ObstacleVisualizer Implementation
ObstacleVisualizer::ObstacleVisualizer(ros::NodeHandle& nh, 
                                     const std::string& topic_name,
                                     const std::string& frame_id)
    : _nh(nh), _frame_id(frame_id), _marker_id_counter(0), _publish_rate(1.0)
{
    _marker_pub = _nh.advertise<visualization_msgs::MarkerArray>(topic_name, 1, true);
    
    // Set default colors
    _leaf_color = createColor(1.0f, 0.0f, 0.0f, 0.7f);      // Red for leaf nodes
    _internal_color = createColor(0.0f, 0.0f, 1.0f, 0.3f);  // Blue for internal nodes
    
    LOG_INFO << "ObstacleVisualizer initialized with topic: " << topic_name;
}

void ObstacleVisualizer::setObstacleTree(std::shared_ptr<BVHTree> obstacle_tree)
{
    _obstacle_tree = obstacle_tree;
    LOG_INFO << "Obstacle tree set for visualization";
}

void ObstacleVisualizer::setPublishRate(double rate)
{
    _publish_rate = rate;
    if (_publish_timer.isValid()) {
        _publish_timer.stop();
        _publish_timer = _nh.createTimer(ros::Duration(1.0 / _publish_rate), 
                                        &ObstacleVisualizer::timerCallback, this);
    }
}

void ObstacleVisualizer::setColors(const std_msgs::ColorRGBA& leaf_color, 
                                  const std_msgs::ColorRGBA& internal_color)
{
    _leaf_color = leaf_color;
    _internal_color = internal_color;
}

void ObstacleVisualizer::setShowInternalNodes(bool show_internal)
{
    _show_internal_nodes = show_internal;
}

std_msgs::ColorRGBA ObstacleVisualizer::createColor(float r, float g, float b, float a)
{
    std_msgs::ColorRGBA color;
    color.r = r;
    color.g = g;
    color.b = b;
    color.a = a;
    return color;
}

void ObstacleVisualizer::publishObstacles()
{
    if (!_obstacle_tree) {
        LOG_WARNING << "No obstacle tree available for visualization";
        return;
    }
    
    visualization_msgs::MarkerArray marker_array;
    clearMarkers(marker_array);
    addObstacleBoxes(marker_array);
    
    _marker_pub.publish(marker_array);
    LOG_DEBUG << "Published " << marker_array.markers.size() << " obstacle markers";
}

void ObstacleVisualizer::timerCallback(const ros::TimerEvent& event)
{
    publishObstacles();
}

void ObstacleVisualizer::clearMarkers(visualization_msgs::MarkerArray& marker_array)
{
    visualization_msgs::Marker clear_marker;
    clear_marker.header.frame_id = _frame_id;
    clear_marker.header.stamp = ros::Time::now();
    clear_marker.action = visualization_msgs::Marker::DELETEALL;
    marker_array.markers.push_back(clear_marker);
    _marker_id_counter = 0;
}

void ObstacleVisualizer::addObstacleBoxes(visualization_msgs::MarkerArray& marker_array)
{
    if (_obstacle_tree && _obstacle_tree->getRoot()) {
        addObstacleBoxesRecursive(_obstacle_tree->getRoot(), marker_array);
    }
}

void ObstacleVisualizer::addObstacleBoxesRecursive(const BVHNode* node, 
                                                  visualization_msgs::MarkerArray& marker_array)
{
    if (!node) return;
    
    bool is_leaf = (node->left == nullptr && node->right == nullptr);
    
    // Only visualize leaf nodes by default, unless internal nodes are explicitly enabled
    if (!is_leaf && !_show_internal_nodes) {
        addObstacleBoxesRecursive(node->left, marker_array);
        addObstacleBoxesRecursive(node->right, marker_array);
        return;
    }
    
    // Create marker for this node
    Eigen::Vector3d center = node->box.center();
    Eigen::Vector3d half_dims = node->box.sizes() * 0.5;
    Eigen::Matrix3d axes = Eigen::Matrix3d::Identity(); // Assuming axis-aligned boxes
    
    visualization_msgs::Marker marker = createBoxMarker(center, half_dims, axes, is_leaf);
    marker_array.markers.push_back(marker);
    
    // Recursively process children for internal nodes
    if (!is_leaf) {
        addObstacleBoxesRecursive(node->left, marker_array);
        addObstacleBoxesRecursive(node->right, marker_array);
    }
}

visualization_msgs::Marker ObstacleVisualizer::createBoxMarker(const Eigen::Vector3d& center,
                                                              const Eigen::Vector3d& half_dims,
                                                              const Eigen::Matrix3d& axes,
                                                              bool is_leaf)
{
    visualization_msgs::Marker marker;
    marker.header.frame_id = _frame_id;
    marker.header.stamp = ros::Time::now();
    marker.id = _marker_id_counter++;
    marker.type = visualization_msgs::Marker::CUBE;
    marker.action = visualization_msgs::Marker::ADD;
    
    marker.pose.position.x = center.x();
    marker.pose.position.y = center.y();
    marker.pose.position.z = center.z();
    marker.pose.orientation.w = 1.0; // Identity quaternion for axis-aligned boxes
    
    marker.scale.x = half_dims.x() * 2.0;
    marker.scale.y = half_dims.y() * 2.0;
    marker.scale.z = half_dims.z() * 2.0;
    
    marker.color = is_leaf ? _leaf_color : _internal_color;
    
    return marker;
}

// PlanningInterface Implementation
PlanningInterface::PlanningInterface(ros::NodeHandle &nh) : _nh(nh), _usPlanner(nullptr)
{
    LOG_INFO << "Initializing PlanningInterface";
}

PlanningInterface::~PlanningInterface()
{
    if (_usPlanner) {
        delete _usPlanner;
        _usPlanner = nullptr;
    }
    LOG_INFO << "PlanningInterface destroyed";
}

bool PlanningInterface::initialize()
{
    try {
        // Get parameters from ROS parameter server
        _nh.param<std::string>("robot_description_path", _robot_description_path, "");
        _nh.param<std::string>("env_description_path", _env_description_path, "");
        
        if (_robot_description_path.empty()) {
            LOG_ERROR << "robot_description_path parameter not set";
            return false;
        }
        
        // Initialize joint names (assuming 7-DOF robot)
        _joint_names = {"panda_joint1", "panda_joint2", "panda_joint3", "panda_joint4", 
                       "panda_joint5", "panda_joint6", "panda_joint7"};
        
        // Initialize USTrajectoryPlanner
        std::string environment_string = _env_description_path.empty() ? 
                                       "<?xml version=\"1.0\"?><environment></environment>" : 
                                       _env_description_path;
        
        _usPlanner = new UltrasoundScanTrajectoryPlanner(environment_string);
        
        // Setup ROS services
        _scan_sequence_service = _nh.advertiseService("scan_pose_sequence_plan", 
            &PlanningInterface::scanPoseSequenceCallback, this);
        
        _environment_update_service = _nh.advertiseService("update_environment", 
            &PlanningInterface::updateEnvironmentCallback, this);
        
        _visualization_service = _nh.advertiseService("visualize_obstacles", 
            &PlanningInterface::visualizeObstaclesCallback, this);
        
        // Setup obstacle visualization
        setupObstacleVisualization();
        
        LOG_INFO << "PlanningInterface initialized successfully";
        return true;
        
    } catch (const std::exception& e) {
        LOG_ERROR << "Failed to initialize PlanningInterface: " << e.what();
        return false;
    }
}

bool PlanningInterface::scanPoseSequenceCallback(
    custom_msgs::ScanPoseSequencePlan::Request &req,
    custom_msgs::ScanPoseSequencePlan::Response &res)
{
    LOG_INFO << "Received scan pose sequence planning request";
    printScanPoseSequenceRequest(req);
    
    // Initialize response with neutral/safe defaults
    res.success = false;
    res.message = "Planning in progress";
    res.trajectory_points.clear();
    res.contact_force_flags.clear();
    
    try {
        // Process the request to get poses and current joints
        auto [poses, currentJoints] = processScanPoseSequenceRequest(req);
        
        if (poses.empty()) {
            res.success = false;
            res.message = "No valid poses provided in request";
            LOG_WARNING << res.message;
            return true; // Return true for service call success, but with failure result
        }
        
        // Set poses and current joints in planner
        _usPlanner->setPoses(poses);
        _usPlanner->setCurrentJoints(currentJoints);
        
        // Robust planning with no duration filtering (0.0 = disabled)
        bool planningSuccess = false;
        std::string planningDetails;
        
        try {
            // First try robust planning without duration filtering
            planningSuccess = _usPlanner->planTrajectoriesRobust(
                req.use_hauser_for_repositioning, 
                req.enable_shortcutting, 
                0.0  // No duration filtering - remove categorical 4-second discarding
            );
            planningDetails = "Robust planning with fallback handling";
            
        } catch (const std::exception& e) {
            LOG_WARNING << "Robust planning failed: " << e.what() << " - trying regular planning";
            
            try {
                // Fallback to regular planning
                planningSuccess = _usPlanner->planTrajectories(
                    req.use_hauser_for_repositioning, 
                    req.enable_shortcutting
                );
                planningDetails = "Fallback to regular planning";
                
            } catch (const std::exception& e2) {
                LOG_ERROR << "All planning methods failed: " << e2.what();
                planningSuccess = false;
                planningDetails = "All planning methods failed: " + std::string(e2.what());
            }
        }
        
        if (planningSuccess) {
            // Get planned trajectories
            auto trajectories = _usPlanner->getTrajectories();
            
            if (!trajectories.empty()) {
                // Convert trajectories to ROS message format
                std::vector<bool> contactFlags;
                trajectory_msgs::JointTrajectory fullTrajectory;
                fullTrajectory.joint_names = _joint_names;
                
                double currentTime = 0.0;
                for (const auto& [trajectoryPoints, isContactForce] : trajectories) {
                    for (const auto& point : trajectoryPoints) {
                        trajectory_msgs::JointTrajectoryPoint rosPoint;
                        rosPoint.positions = point.position;
                        rosPoint.velocities = point.velocity;
                        rosPoint.accelerations = point.acceleration;
                        rosPoint.time_from_start = ros::Duration(currentTime + point.time);
                        
                        fullTrajectory.points.push_back(rosPoint);
                        contactFlags.push_back(isContactForce);
                    }
                    
                    // Update time for next segment
                    if (!trajectoryPoints.empty()) {
                        currentTime += trajectoryPoints.back().time;
                    }
                }
                
                res.trajectory_points = fullTrajectory;
                res.contact_force_flags = boolsToBytes(contactFlags);
                res.success = true;
                
                std::ostringstream msg;
                msg << "Planning successful using " << planningDetails 
                    << ". Generated " << trajectories.size() << " segments with " 
                    << fullTrajectory.points.size() << " total points. "
                    << std::count(contactFlags.begin(), contactFlags.end(), true) 
                    << " contact force points.";
                res.message = msg.str();
                
                LOG_INFO << res.message;
                
            } else {
                res.success = false;
                res.message = "Planning reported success but returned no trajectories";
                LOG_WARNING << res.message;
            }
            
        } else {
            res.success = false;
            std::ostringstream msg;
            msg << "Planning failed after trying multiple approaches. " << planningDetails 
                << ". This may indicate: (1) no valid solution exists, "
                << "(2) poses are unreachable, or (3) environment constraints are too restrictive.";
            res.message = msg.str();
            LOG_WARNING << res.message;
        }
        
    } catch (const std::exception& e) {
        res.success = false;
        std::ostringstream msg;
        msg << "Exception during planning: " << e.what() 
            << ". Planning system encountered an unexpected error.";
        res.message = msg.str();
        LOG_ERROR << res.message;
    }
    
    // Always return true for ROS service call success
    // The actual planning result is indicated by res.success
    return true;
}

bool PlanningInterface::updateEnvironmentCallback(
    custom_msgs::String_StringBool::Request &req,
    custom_msgs::String_StringBool::Response &res)
{
    LOG_INFO << "Received environment update request";
    
    try {
        if (_usPlanner) {
            _usPlanner->setEnvironment(req.input_string);
            updateObstacleVisualization();
            
            res.success = true;
            res.message = "Environment updated successfully";
            LOG_INFO << "Environment updated successfully";
        } else {
            res.success = false;
            res.message = "USTrajectoryPlanner not initialized";
            LOG_ERROR << "USTrajectoryPlanner not initialized for environment update";
        }
    } catch (const std::exception& e) {
        res.success = false;
        res.message = "Failed to update environment: " + std::string(e.what());
        LOG_ERROR << "Failed to update environment: " << e.what();
    }
    
    return true;
}

bool PlanningInterface::visualizeObstaclesCallback(
    custom_msgs::String_StringBool::Request &req,
    custom_msgs::String_StringBool::Response &res)
{
    LOG_INFO << "Received obstacle visualization request";
    
    try {
        if (_obstacle_visualizer) {
            _obstacle_visualizer->publishObstacles();
            res.success = true;
            res.message = "Obstacle visualization published";
            LOG_INFO << "Obstacle visualization published";
        } else {
            res.success = false;
            res.message = "Obstacle visualizer not initialized";
            LOG_WARNING << "Obstacle visualizer not initialized";
        }
    } catch (const std::exception& e) {
        res.success = false;
        res.message = "Failed to publish obstacle visualization: " + std::string(e.what());
        LOG_ERROR << "Failed to publish obstacle visualization: " << e.what();
    }
    
    return true;
}

void PlanningInterface::setupObstacleVisualization()
{
    try {
        _obstacle_visualizer = std::make_unique<ObstacleVisualizer>(_nh, "obstacle_markers", "base_link");
        _obstacle_visualizer->setPublishRate(1.0); // 1 Hz
        
        if (_usPlanner) {
            auto obstacleTree = _usPlanner->getObstacleTree();
            if (obstacleTree) {
                _obstacle_visualizer->setObstacleTree(obstacleTree);
            }
        }
        
        LOG_INFO << "Obstacle visualization setup completed";
    } catch (const std::exception& e) {
        LOG_ERROR << "Failed to setup obstacle visualization: " << e.what();
    }
}

void PlanningInterface::updateObstacleVisualization()
{
    try {
        if (_obstacle_visualizer && _usPlanner) {
            auto obstacleTree = _usPlanner->getObstacleTree();
            if (obstacleTree) {
                _obstacle_visualizer->setObstacleTree(obstacleTree);
                _obstacle_visualizer->publishObstacles();
                LOG_DEBUG << "Obstacle visualization updated";
            }
        }
    } catch (const std::exception& e) {
        LOG_ERROR << "Failed to update obstacle visualization: " << e.what();
    }
}

void PlanningInterface::printScanPoseSequenceRequest(const custom_msgs::ScanPoseSequencePlan::Request &req)
{
    LOG_INFO << "Scan Pose Sequence Request:";
    LOG_INFO << "  - Poses count: " << req.scan_poses.poses.size();
    LOG_INFO << "  - Current joints: [" << req.current_joints.position.size() << " joints]";
    LOG_INFO << "  - Use Hauser for repositioning: " << (req.use_hauser_for_repositioning ? "yes" : "no");
    LOG_INFO << "  - Enable shortcutting: " << (req.enable_shortcutting ? "yes" : "no");
    
    // Log joint values with reasonable precision
    if (!req.current_joints.position.empty()) {
        std::ostringstream joints_str;
        joints_str << std::fixed << std::setprecision(3);
        for (size_t i = 0; i < req.current_joints.position.size(); ++i) {
            if (i > 0) joints_str << ", ";
            joints_str << req.current_joints.position[i];
        }
        LOG_DEBUG << "  - Joint values: [" << joints_str.str() << "]";
    }
}

std::pair<std::vector<Eigen::Affine3d>, Eigen::VectorXd> 
PlanningInterface::processScanPoseSequenceRequest(const custom_msgs::ScanPoseSequencePlan::Request &req)
{
    std::vector<Eigen::Affine3d> poses;
    Eigen::VectorXd currentJoints;
    
    // Convert ROS poses to Eigen format
    poses.reserve(req.scan_poses.poses.size());
    for (const auto& pose : req.scan_poses.poses) {
        Eigen::Affine3d eigenPose = Eigen::Affine3d::Identity();
        
        // Set translation
        eigenPose.translation() << pose.position.x, pose.position.y, pose.position.z;
        
        // Set rotation from quaternion
        Eigen::Quaterniond quat(pose.orientation.w, pose.orientation.x, 
                               pose.orientation.y, pose.orientation.z);
        eigenPose.linear() = quat.toRotationMatrix();
        
        poses.push_back(eigenPose);
    }
    
    // Convert current joints
    if (!req.current_joints.position.empty()) {
        currentJoints.resize(req.current_joints.position.size());
        for (size_t i = 0; i < req.current_joints.position.size(); ++i) {
            currentJoints[i] = req.current_joints.position[i];
        }
    }
    
    return std::make_pair(poses, currentJoints);
}

std::vector<unsigned char> PlanningInterface::boolsToBytes(const std::vector<bool> &bool_flags)
{
    std::vector<unsigned char> bytes;
    bytes.reserve((bool_flags.size() + 7) / 8); // Ceiling division
    
    for (size_t i = 0; i < bool_flags.size(); i += 8) {
        unsigned char byte = 0;
        for (int j = 0; j < 8 && (i + j) < bool_flags.size(); ++j) {
            if (bool_flags[i + j]) {
                byte |= (1 << j);
            }
        }
        bytes.push_back(byte);
    }
    
    return bytes;
}