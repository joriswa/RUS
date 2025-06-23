# Checkpoint Planning Algorithm for Ultrasound Scanning Trajectories

## Problem Statement

The checkpoint planning algorithm addresses the challenge of converting a sequence of desired end-effector poses (position and orientation) into feasible robot joint configurations for ultrasound scanning applications. The algorithm must balance motion continuity preferences with workspace constraints, collision avoidance, and ultrasound-specific orientation requirements.

## Algorithm Overview

### Input and Output

**Input:**
- Sequence of target poses $\mathbf{T}_i \in SE(3)$ representing desired ultrasound probe positions and orientations
- Current joint configuration $\mathbf{q}_0 \in \mathbb{R}^7$ (Franka robot)
- Obstacle environment representation (BVH tree)

**Output:**
- Joint configurations $\mathbf{q}_i \in \mathbb{R}^7$ for each target pose
- Validity flags indicating reachable poses
- Motion segments identifying continuous trajectory portions

### Core Algorithm

The algorithm processes target poses sequentially through three main phases:

#### Phase 1: Initial Pose Resolution
The first target pose $\mathbf{T}_1$ is processed using a global optimization approach (`selectGoalPose`) that:
- Samples the 7th joint angle $q_7$ uniformly over its range $[-2.897, 2.897]$ radians
- Solves analytical inverse kinematics for each sample using the Franka robot model
- Evaluates solutions based on collision clearance and manipulability metrics
- Selects the configuration minimizing a composite cost function

#### Phase 2: Sequential Continuity-Constrained Planning
For subsequent poses $\mathbf{T}_i$ (i = 2, ..., n), the algorithm attempts to maintain motion continuity by:

1. **Directional Change Detection**: Computes angular difference between consecutive probe orientations:
   $$\Delta\theta = \arccos(|\mathbf{z}_{i-1} \cdot \mathbf{z}_i|)$$
   where $\mathbf{z}_i$ is the probe direction (Z-axis) for pose $i$

2. **Inverse Kinematics Search**: If $\Delta\theta \leq 0.52$ rad (30°), performs progressive IK search:
   - Samples $q_7$ with expanding radius: $r = 0.05 + 0.15 \cdot \frac{k}{K}$ where $k$ is attempt number and $K = 120$ max attempts
   - For each sample, solves analytical IK and validates:
     - Joint distance constraint: $\|\mathbf{q}_i - \mathbf{q}_{i-1}\|_2 \leq 2.7$ rad
     - Individual joint limits: $|q_{i,j} - q_{i-1,j}| \leq 1.3$ rad for all joints $j$
     - Collision-free configuration

3. **Solution Selection**: Among valid solutions, selects configuration minimizing joint-space distance for motion continuity

#### Phase 3: Repositioning Strategy
When continuity constraints cannot be satisfied or directional changes exceed threshold:
- Applies global optimization (`selectGoalPose`) to find feasible configuration
- Marks as repositioning point requiring discrete jump in joint space
- Updates reference configuration for subsequent continuity planning

### Motion Segmentation

The algorithm partitions the checkpoint sequence into continuous motion segments using linear boundary detection:

**Boundary Conditions:**
- Invalid checkpoints (IK failures): Hard boundaries requiring path gaps
- Repositioning points: Soft boundaries requiring discrete transitions

**Segment Creation:**
- Linear scan identifies boundary indices
- Creates segments $S_k = [i_{start}, i_{end}]$ between boundaries
- Each segment represents poses achievable through continuous joint-space motion

### Ultrasound-Specific Adaptations

#### Orientation Monitoring
Unlike general manipulation tasks, ultrasound scanning requires strict probe orientation control for acoustic coupling. The algorithm enforces repositioning when probe direction changes exceed 30°, ensuring consistent anatomical visualization.

#### Collision-Aware Configuration Selection
Medical environments contain complex obstacles (patient anatomy, equipment). The algorithm integrates collision checking at every IK solution, using bounding box representations of robot links validated against environment obstacles.

## Computational Complexity

- **Time Complexity**: O(n × k) where n is pose count and k ≤ 120 IK attempts per pose
- **Space Complexity**: O(n) for checkpoint storage and segment representation
- **Early Termination**: IK search terminates when finding solutions with joint distance < 0.5 rad and max joint change < 0.8 rad

## Algorithmic Contributions

1. **Hybrid Motion Strategy**: Combines continuous motion preference with strategic repositioning for workspace constraints
2. **Directional Repositioning**: Novel orientation-based repositioning criterion specific to ultrasound probe requirements  
3. **Progressive IK Search**: Efficient exploration-exploitation balance in configuration space
4. **Linear Segmentation**: Computationally efficient decomposition for subsequent trajectory optimization

This checkpoint planning approach enables robust trajectory generation for ultrasound scanning by addressing the unique constraints of medical robotics while maintaining computational efficiency suitable for clinical workflows.
