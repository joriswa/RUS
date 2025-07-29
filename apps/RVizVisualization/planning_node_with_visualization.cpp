#include "planning_node.h"
#include "GeometryLib/BVHTree.h"

// ============================================================================
// ObstacleVisualizer Implementation
// ============================================================================

ObstacleVisualizer::ObstacleVisualizer(ros::NodeHandle& nh)
    : _marker_pub(nh.advertise<visualization_msgs::MarkerArray>("obstacle_markers", 10))
    , _leaf_color(ObstacleVisualizer::createColor(1.0, 0.0, 0.0, 0.7))     // Red for obstacles
    , _internal_color(ObstacleVisualizer::createColor(0.0, 0.0, 1.0, 0.3)) // Blue for BVH nodes
{
    _marker_pub = _nh.advertise<visualization_msgs::MarkerArray>(topic_name, 1, true);
    _publish_timer = _nh.createTimer(ros::Duration(1.0 / _publish_rate), 
                                    &ObstacleVisualizer::timerCallback, this);
    
    ROS_INFO("ObstacleVisualizer initialized. Publishing to topic: %s", topic_name.c_str());
}

void ObstacleVisualizer::setObstacleTree(std::shared_ptr<BVHTree> obstacle_tree)
{
    _obstacle_tree = obstacle_tree;
    ROS_INFO("Obstacle tree set for visualization");
}

void ObstacleVisualizer::setPublishRate(double rate)
{
    _publish_rate = rate;
    _publish_timer.stop();
    _publish_timer = _nh.createTimer(ros::Duration(1.0 / _publish_rate), 
                                    &ObstacleVisualizer::timerCallback, this);
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

void ObstacleVisualizer::publishObstacles()
{
    if (!_obstacle_tree) {
        ROS_WARN_THROTTLE(5.0, "No obstacle tree set. Cannot publish markers.");
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

void ObstacleVisualizer::timerCallback(const ros::TimerEvent& event)
{
    publishObstacles();
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

void ObstacleVisualizer::clearMarkers(visualization_msgs::MarkerArray& marker_array)
{
    visualization_msgs::Marker clear_marker;
    clear_marker.header.frame_id = _frame_id;
    clear_marker.header.stamp = ros::Time::now();
    clear_marker.ns = "obstacles";
    clear_marker.id = 0;
    clear_marker.action = visualization_msgs::Marker::DELETEALL;
    
    marker_array.markers.push_back(clear_marker);
}

void ObstacleVisualizer::addObstacleBoxes(visualization_msgs::MarkerArray& marker_array)
{
    if (!_obstacle_tree) {
        return;
    }
    
    // Traverse the BVH tree and create markers for obstacle boxes
    addObstacleBoxesRecursive(_obstacle_tree->getRoot(), marker_array);
}

void ObstacleVisualizer::addObstacleBoxesRecursive(const BVHNode* node, visualization_msgs::MarkerArray& marker_array)
{
    if (!node) {
        return;
    }
    
    // Check if this is a leaf node (contains actual obstacles)
    bool is_leaf = !node->obstacles.empty();
    
    if (is_leaf) {
        // Create markers for actual obstacles in leaf nodes
        for (const auto& obstacle : node->obstacles) {
            if (obstacle) {
                // Try to cast to BoxObstacle to get transform information
                if (auto box_obstacle = std::dynamic_pointer_cast<BoxObstacle>(obstacle)) {
                    // Get the obstacle's transform (includes position AND orientation)
                    Eigen::Affine3d transform = box_obstacle->getTransform();
                    Eigen::Vector3d scale = box_obstacle->getScale();
                    
                    // Extract center from transform
                    Eigen::Vector3d center = transform.translation();
                    
                    // Extract orientation matrix from transform
                    Eigen::Matrix3d axes = transform.rotation();
                    
                    // Half dimensions from scale
                    Eigen::Vector3d half_dims = scale * 0.5;
                    
                    visualization_msgs::Marker marker = createBoxMarker(center, half_dims, axes, true);
                    marker_array.markers.push_back(marker);
                } else {
                    // Fallback for other obstacle types - use bounding box
                    Eigen::AlignedBox3d bbox = obstacle->getBoundingBox();
                    Eigen::Vector3d center = bbox.center();
                    Eigen::Vector3d half_dims = bbox.sizes() * 0.5;
                    Eigen::Matrix3d axes = Eigen::Matrix3d::Identity();
                    
                    visualization_msgs::Marker marker = createBoxMarker(center, half_dims, axes, true);
                    marker_array.markers.push_back(marker);
                }
            }
        }
    } else if (_show_internal_nodes) {
        // Show internal BVH node bounding boxes for debugging
        Eigen::Vector3d center = node->boundingBox.center();
        Eigen::Vector3d half_dims = node->boundingBox.sizes() * 0.5;
        Eigen::Matrix3d axes = Eigen::Matrix3d::Identity();
        
        visualization_msgs::Marker marker = createBoxMarker(center, half_dims, axes, false);
        marker_array.markers.push_back(marker);
    }
    
    // Recursively process child nodes
    addObstacleBoxesRecursive(node->left.get(), marker_array);
    addObstacleBoxesRecursive(node->right.get(), marker_array);
}

visualization_msgs::Marker ObstacleVisualizer::createBoxMarker(const Eigen::Vector3d& center,
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

// ============================================================================
// PlanningInterface Implementation
// ============================================================================

PlanningInterface::PlanningInterface(ros::NodeHandle &nh) : _nh(nh)
{
    ROS_INFO("PlanningInterface: Constructor initialized with obstacle visualization!");
}

PlanningInterface::~PlanningInterface()
{
    delete _usPlanner;
    ROS_INFO("PlanningInterface: Destructor called");
}

bool PlanningInterface::initialize()
{
    // Load parameters from ROS parameter server
    if (!_nh.getParam("robot_description_path", _robot_description_path))
    {
        ROS_WARN("Parameter 'robot_description_path' not found, using default");
        _robot_description_path = "/home/lovis/WorkspaceRUS/src/path_planning_space/res/panda.urdf";
    }

    if (!_nh.getParam("environment_path", _env_description_path))
    {
        ROS_WARN("Parameter 'environment_path' not found, using default");
        _env_description_path = "/home/lovis/WorkspaceRUS/src/path_planning_space/res/obstacles.xml";
    }

    // Initialize trajectory planner
    std::vector<double> default_joints = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    _usPlanner = new UltrasoundScanTrajectoryPlanner(_robot_description_path, default_joints);
    _usPlanner->setEnvironment(_env_description_path);

    // Setup obstacle visualization
    setupObstacleVisualization();

    // Register services
    _scan_sequence_service = _nh.advertiseService(
        "/scan_pose_sequence",
        &PlanningInterface::scanPoseSequenceCallback,
        this);

    _environment_update_service = _nh.advertiseService(
        "/environment_update",
        &PlanningInterface::updateEnvironmentCallback,
        this);

    _visualization_service = _nh.advertiseService(
        "/visualize_obstacles",
        &PlanningInterface::visualizeObstaclesCallback,
        this);

    ROS_INFO("PlanningInterface: Services registered and initialization complete");
    ROS_INFO("Available services:");
    ROS_INFO("  - /scan_pose_sequence: Plan trajectory sequence");
    ROS_INFO("  - /environment_update: Update environment");
    ROS_INFO("  - /visualize_obstacles: Trigger obstacle visualization");
    
    return true;
}

void PlanningInterface::setupObstacleVisualization()
{
    // Initialize obstacle visualizer
    _obstacle_visualizer = std::make_unique<ObstacleVisualizer>(
        _nh, "planning_obstacles", "base_link");
    
    // Configure visualization
    _obstacle_visualizer->setPublishRate(2.0); // 2 Hz for planning context
    _obstacle_visualizer->setShowInternalNodes(false); // Only show actual obstacles
    
    // Set custom colors
    std_msgs::ColorRGBA obstacle_color = ObstacleVisualizer::createColor(0.8, 0.2, 0.2, 0.8); // Red
    std_msgs::ColorRGBA bvh_color = ObstacleVisualizer::createColor(0.2, 0.2, 0.8, 0.3); // Blue
    _obstacle_visualizer->setColors(obstacle_color, bvh_color);
    
    // Update visualization with current environment
    updateObstacleVisualization();
    
    ROS_INFO("Obstacle visualization setup complete");
}

void PlanningInterface::updateObstacleVisualization()
{
    if (_usPlanner && _obstacle_visualizer) {
        // Get obstacle tree from trajectory planner
        auto obstacle_tree = _usPlanner->getObstacleTree();
        if (obstacle_tree) {
            _obstacle_visualizer->setObstacleTree(obstacle_tree);
            ROS_INFO("Updated obstacle visualization with environment data");
        } else {
            ROS_WARN("No obstacle tree available from trajectory planner");
        }
    } else {
        ROS_WARN("Cannot update obstacle visualization - components not properly initialized");
    }
}

std::pair<std::vector<Eigen::Affine3d>, Eigen::VectorXd>
PlanningInterface::processScanPoseSequenceRequest(const custom_msgs::ScanPoseSequencePlan::Request &req)
{
    std::vector<Eigen::Affine3d> eigen_poses;
    eigen_poses.reserve(req.scan_poses.size());
    for (const auto &pose : req.scan_poses)
    {
        Eigen::Quaterniond q(
            pose.orientation.w,
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z);
        Eigen::Affine3d affine = Eigen::Affine3d::Identity();
        affine.linear() = q.normalized().toRotationMatrix();

        Eigen::Matrix3d rot180_x;
        rot180_x = Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitX()).toRotationMatrix();
        // affine.linear() = rot180_x * affine.linear();

        affine.translation() = Eigen::Vector3d(
            pose.position.x,
            pose.position.y,
            pose.position.z);

        eigen_poses.push_back(affine);
    }

    Eigen::VectorXd joints;
    if (!req.start_joints.position.empty())
    {
        joints = Eigen::Map<const Eigen::VectorXd>(
            req.start_joints.position.data(),
            req.start_joints.position.size());
    }

    return std::make_pair(eigen_poses, joints);
}

std::vector<unsigned char> PlanningInterface::boolsToBytes(const std::vector<bool> &bool_flags)
{
    std::vector<unsigned char> byte_flags;
    byte_flags.reserve(bool_flags.size());
    for (bool flag : bool_flags)
    {
        byte_flags.push_back(static_cast<unsigned char>(flag));
    }
    return byte_flags;
}

void PlanningInterface::printScanPoseSequenceRequest(const custom_msgs::ScanPoseSequencePlan::Request &req)
{
    ROS_INFO("Received scan_pose_sequence request with %lu poses", req.scan_poses.size());

    for (size_t i = 0; i < req.scan_poses.size(); ++i)
    {
        const auto &pose = req.scan_poses[i];
        ROS_INFO("Pose %lu:", i + 1);
        ROS_INFO("  Position: x=%.3f, y=%.3f, z=%.3f",
                 pose.position.x, pose.position.y, pose.position.z);
        ROS_INFO("  Orientation: x=%.3f, y=%.3f, z=%.3f, w=%.3f",
                 pose.orientation.x, pose.orientation.y,
                 pose.orientation.z, pose.orientation.w);
    }

    ROS_INFO("Start joints:");
    std::string joint_names_str;
    for (size_t i = 0; i < req.start_joints.name.size(); ++i)
    {
        joint_names_str += req.start_joints.name[i];
        if (i != req.start_joints.name.size() - 1)
            joint_names_str += ", ";
    }
    ROS_INFO("  Joint names: %s", joint_names_str.c_str());

    std::string joint_positions_str;
    for (size_t i = 0; i < req.start_joints.position.size(); ++i)
    {
        joint_positions_str += std::to_string(req.start_joints.position[i]);
        if (i != req.start_joints.position.size() - 1)
            joint_positions_str += ", ";
    }
    ROS_INFO("  Positions: %s", joint_positions_str.c_str());
}

std::pair<std::vector<trajectory_msgs::JointTrajectory>, std::vector<unsigned char>>
convertPlannerTrajectoriesToROS(
    const std::vector<std::pair<std::vector<MotionGenerator::TrajectoryPoint>, bool>> &planner_traj,
    const std::vector<std::string> &joint_names)
{
    std::vector<trajectory_msgs::JointTrajectory> ros_trajectories;
    std::vector<unsigned char> trajectory_flags;

    for (const auto &[segment, flag] : planner_traj)
    {
        trajectory_msgs::JointTrajectory ros_traj;
        ros_traj.joint_names = joint_names;

        for (const auto &point : segment)
        {
            trajectory_msgs::JointTrajectoryPoint ros_point;

            // Convert positions
            ros_point.positions = point.position;

            // Convert velocities if available
            if (!point.velocity.empty())
            {
                ros_point.velocities = point.velocity;
            }

            // Convert accelerations if available
            if (!point.acceleration.empty())
            {
                ros_point.accelerations = point.acceleration;
            }

            ros_point.time_from_start = ros::Duration(point.time);

            ros_traj.points.push_back(ros_point);
        }

        ros_trajectories.push_back(ros_traj);
        trajectory_flags.push_back(flag ? 1 : 0); // Convert bool to unsigned char
    }

    return {ros_trajectories, trajectory_flags};
}

bool PlanningInterface::updateEnvironmentCallback(
    custom_msgs::String_StringBool::Request &req,
    custom_msgs::String_StringBool::Response &res)
{
    ROS_INFO("Received update_environment request with path: %s", req.request_string.c_str());

    // Update environment path if provided
    if (!req.request_string.empty()) {
        _env_description_path = req.request_string;
    }

    _usPlanner->setEnvironment(_env_description_path.c_str());
    
    // Update obstacle visualization
    updateObstacleVisualization();

    res.response_bool = true;
    res.response_msg = "Environment updated and visualization refreshed";

    return true;
}

bool PlanningInterface::visualizeObstaclesCallback(
    custom_msgs::String_StringBool::Request &req,
    custom_msgs::String_StringBool::Response &res)
{
    ROS_INFO("Manual obstacle visualization requested");
    
    try {
        updateObstacleVisualization();
        _obstacle_visualizer->publishObstacles();
        
        res.response_bool = true;
        res.response_msg = "Obstacle visualization published successfully";
    } catch (const std::exception& e) {
        ROS_ERROR("Failed to publish obstacle visualization: %s", e.what());
        res.response_bool = false;
        res.response_msg = "Failed to publish obstacle visualization: " + std::string(e.what());
    }

    return true;
}

bool PlanningInterface::scanPoseSequenceCallback(
    custom_msgs::ScanPoseSequencePlan::Request &req,
    custom_msgs::ScanPoseSequencePlan::Response &res)
{
    printScanPoseSequenceRequest(req);
    auto [poses, joints] = processScanPoseSequenceRequest(req);

    _usPlanner->setCurrentJoints(joints);
    _usPlanner->setPoses(poses);

    // Publish obstacle visualization before planning
    ROS_INFO("Publishing obstacle visualization before trajectory planning");
    _obstacle_visualizer->publishObstacles();

    bool hauser = false;
    ros::param::get("hauser", hauser);
    
    try {
        // First try normal planning
        bool planningSuccess = _usPlanner->planTrajectories(hauser);
        
        // If normal planning fails, try partial solutions
        if (!planningSuccess) {
            ROS_WARN("Standard trajectory planning failed, attempting partial solution planning...");
            planningSuccess = _usPlanner->planTrajectoriesPartial(hauser, true, true); // Allow partial solutions
        }
        
        if (!planningSuccess)
        {
            ROS_ERROR("Both standard and partial trajectory planning failed");
            res.success = false;
            res.message = "Planning failed completely - check logs for specific details. No trajectories could be generated for any poses.";
            return true;
        }
    } catch (const std::exception& e) {
        ROS_ERROR("Trajectory planning exception: %s", e.what());
        res.success = false;
        res.message = "Planning failed with exception: " + std::string(e.what()) + ". This typically indicates invalid input parameters.";
        return true;
    }

    const auto &planner_trajectories = _usPlanner->getTrajectories();
    if (planner_trajectories.empty())
    {
        ROS_ERROR("Planner returned empty trajectories");
        res.success = false;
        res.message = "No valid trajectories generated";
        return true;
    }

    for (auto trajectory : planner_trajectories)
    {
        auto [points, flag] = trajectory;
        std::cout << "Trajectory with " << points.size() << " points and flag: " << (flag ? "true" : "false") << std::endl;
    }

    auto [ros_trajectories, trajectory_flags] = convertPlannerTrajectoriesToROS(
        planner_trajectories,
        req.start_joints.name);

    res.joint_trajectories = ros_trajectories;
    res.trajectory_flags = trajectory_flags;
    res.success = true;
    
    // Determine if this is a complete or partial solution
    size_t requested_poses = req.scan_poses.size();
    size_t generated_segments = ros_trajectories.size();
    
    if (requested_poses == 1) {
        res.message = "Successfully generated trajectory for single pose with obstacle visualization";
    } else {
        // For multi-pose requests, estimate completeness
        // Each pose typically generates: repositioning + contact force trajectory
        size_t expected_segments = requested_poses * 2; // Rough estimate
        
        if (generated_segments >= expected_segments * 0.8) { // 80% threshold for "complete"
            res.message = "Successfully generated " + std::to_string(generated_segments) + 
                         " trajectory segments (complete solution) with obstacle visualization";
        } else {
            res.message = "Generated " + std::to_string(generated_segments) + 
                         " trajectory segments (partial solution - some poses may be unreachable) with obstacle visualization";
        }
    }

    return true;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "planning_node_with_visualization");
    ros::NodeHandle nh;
    ros::AsyncSpinner spinner(0);
    spinner.start();

    ROS_INFO("Starting Planning Node with Obstacle Visualization");

    PlanningInterface server(nh);
    if (!server.initialize()) {
        ROS_FATAL("Failed to initialize planning interface");
        return -1;
    }

    ROS_INFO("Planning node with obstacle visualization ready!");
    ROS_INFO("Use 'rosservice call /visualize_obstacles' to manually trigger obstacle visualization");

    ros::waitForShutdown();
    return 0;
}
