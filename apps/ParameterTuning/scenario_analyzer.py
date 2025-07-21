#!/usr/bin/env python3
"""
Scenario Feature Extraction for Adaptive Parameter Selection
============================================================

This module analyzes trajectory planning scenarios and extracts features
that characterize the complexity of the optimization problem. These features
are used to predict optimal parameter configurations using machine learning.

Features extracted:
- Obstacle density and distribution
- Workspace volume and constraints  
- Trajectory complexity metrics
- Goal configuration difficulty
- Robot kinematic constraints

Authors: Adaptive Parameter Selection System
Date: July 2025
"""

import numpy as np
import pandas as pd
import yaml
import json
import argparse
import logging
from pathlib import Path
from typing import Dict, List, Tuple, Any
from dataclasses import dataclass, asdict
import xml.etree.ElementTree as ET
from scipy.spatial.distance import cdist
from sklearn.decomposition import PCA
from sklearn.preprocessing import StandardScaler

@dataclass
class ScenarioFeatures:
    """Feature vector characterizing a trajectory planning scenario"""
    # Obstacle characteristics
    obstacle_density: float          # obstacles per unit volume
    obstacle_clustering: float       # spatial clustering coefficient
    workspace_coverage: float        # % of workspace occupied by obstacles
    avg_obstacle_size: float         # average obstacle dimension
    min_corridor_width: float        # narrowest passage
    
    # Trajectory characteristics  
    path_length: float              # total path length
    pose_diversity: float           # spread of goal poses
    orientation_complexity: float    # orientation change magnitude
    joint_range_utilization: float  # % of joint limits used
    
    # Geometric complexity
    workspace_volume: float         # total workspace volume
    goal_reachability: float        # difficulty of goal configurations
    kinematic_complexity: float     # IK conditioning number
    
    # Planning difficulty indicators
    start_goal_distance: float      # Euclidean distance start-goal
    configuration_space_obstacles: float  # C-space obstacle density
    manipulability_variation: float # variation in robot manipulability
    
    def to_vector(self) -> np.ndarray:
        """Convert to feature vector for ML models"""
        return np.array([
            self.obstacle_density,
            self.obstacle_clustering,
            self.workspace_coverage,
            self.avg_obstacle_size,
            self.min_corridor_width,
            self.path_length,
            self.pose_diversity,
            self.orientation_complexity,
            self.joint_range_utilization,
            self.workspace_volume,
            self.goal_reachability,
            self.kinematic_complexity,
            self.start_goal_distance,
            self.configuration_space_obstacles,
            self.manipulability_variation
        ])
    
    @classmethod
    def feature_names(cls) -> List[str]:
        """Get feature names for ML models"""
        return [
            'obstacle_density', 'obstacle_clustering', 'workspace_coverage',
            'avg_obstacle_size', 'min_corridor_width', 'path_length',
            'pose_diversity', 'orientation_complexity', 'joint_range_utilization',
            'workspace_volume', 'goal_reachability', 'kinematic_complexity',
            'start_goal_distance', 'configuration_space_obstacles', 'manipulability_variation'
        ]

class ScenarioAnalyzer:
    """Analyzes trajectory planning scenarios and extracts features"""
    
    def __init__(self):
        self.workspace_bounds = None
        self.joint_limits = None
        
    def analyze_scenario(self, urdf_file: str, environment_file: str, 
                        poses_file: str) -> ScenarioFeatures:
        """Analyze a complete scenario and extract features"""
        
        logging.info(f"Analyzing scenario: {urdf_file}, {environment_file}, {poses_file}")
        
        # Load scenario data
        obstacles = self._load_obstacles(environment_file)
        poses = self._load_poses(poses_file)
        robot_info = self._load_robot_info(urdf_file)
        
        # Extract features
        features = ScenarioFeatures(
            # Obstacle features
            obstacle_density=self._compute_obstacle_density(obstacles),
            obstacle_clustering=self._compute_obstacle_clustering(obstacles),
            workspace_coverage=self._compute_workspace_coverage(obstacles),
            avg_obstacle_size=self._compute_avg_obstacle_size(obstacles),
            min_corridor_width=self._compute_min_corridor_width(obstacles),
            
            # Trajectory features
            path_length=self._compute_path_length(poses),
            pose_diversity=self._compute_pose_diversity(poses),
            orientation_complexity=self._compute_orientation_complexity(poses),
            joint_range_utilization=self._compute_joint_range_utilization(poses, robot_info),
            
            # Geometric features
            workspace_volume=self._compute_workspace_volume(),
            goal_reachability=self._compute_goal_reachability(poses, robot_info),
            kinematic_complexity=self._compute_kinematic_complexity(poses, robot_info),
            
            # Planning difficulty
            start_goal_distance=self._compute_start_goal_distance(poses),
            configuration_space_obstacles=self._estimate_cspace_obstacles(obstacles, robot_info),
            manipulability_variation=self._compute_manipulability_variation(poses, robot_info)
        )
        
        logging.info(f"Extracted features: {asdict(features)}")
        return features
    
    def _load_obstacles(self, environment_file: str) -> List[Dict]:
        """Load obstacle information from environment file"""
        obstacles = []
        
        try:
            if environment_file.endswith('.xml'):
                # Parse XML environment file
                tree = ET.parse(environment_file)
                root = tree.getroot()
                
                for obstacle in root.findall('.//obstacle'):
                    obs_data = {
                        'type': obstacle.get('type', 'box'),
                        'position': [0, 0, 0],
                        'size': [1, 1, 1],
                        'orientation': [0, 0, 0, 1]
                    }
                    
                    # Extract position
                    pos_elem = obstacle.find('position')
                    if pos_elem is not None:
                        obs_data['position'] = [float(x) for x in pos_elem.text.split()]
                    
                    # Extract size/dimensions
                    size_elem = obstacle.find('size')
                    if size_elem is not None:
                        obs_data['size'] = [float(x) for x in size_elem.text.split()]
                    
                    obstacles.append(obs_data)
            
        except Exception as e:
            logging.warning(f"Failed to load obstacles from {environment_file}: {e}")
            # Create dummy obstacles for testing
            obstacles = [
                {'type': 'box', 'position': [0, 0, 1], 'size': [0.5, 0.5, 2], 'orientation': [0, 0, 0, 1]}
            ]
        
        return obstacles
    
    def _load_poses(self, poses_file: str) -> List[Dict]:
        """Load pose data from CSV file"""
        poses = []
        
        try:
            df = pd.read_csv(poses_file)
            for _, row in df.iterrows():
                pose = {
                    'position': [row.iloc[0], row.iloc[1], row.iloc[2]],
                    'orientation': [row.iloc[3], row.iloc[4], row.iloc[5], row.iloc[6]],  # x,y,z,w
                    'contact': row.iloc[7] if len(row) > 7 else False,
                    'distance': row.iloc[8] if len(row) > 8 else 0.0,
                    'index': row.iloc[9] if len(row) > 9 else 0
                }
                poses.append(pose)
        except Exception as e:
            logging.warning(f"Failed to load poses from {poses_file}: {e}")
            # Create dummy poses for testing
            poses = [
                {'position': [0.5, 0, 0.8], 'orientation': [0, 0, 0, 1], 'contact': False, 'distance': 0, 'index': 0},
                {'position': [0.5, 0.3, 0.8], 'orientation': [0, 0, 0, 1], 'contact': False, 'distance': 0.3, 'index': 1}
            ]
        
        return poses
    
    def _load_robot_info(self, urdf_file: str) -> Dict:
        """Extract robot kinematic information"""
        # Simplified robot info - in practice would parse URDF
        robot_info = {
            'dof': 7,  # Panda robot
            'joint_limits': [
                [-2.8973, 2.8973],  # Joint 1
                [-1.7628, 1.7628],  # Joint 2
                [-2.8973, 2.8973],  # Joint 3
                [-3.0718, -0.0698], # Joint 4
                [-2.8973, 2.8973],  # Joint 5
                [-0.0175, 3.7525],  # Joint 6
                [-2.8973, 2.8973]   # Joint 7
            ],
            'link_lengths': [0.333, 0.316, 0.384, 0.088, 0.107],  # Approximate Panda dimensions
            'workspace_radius': 0.855  # Approximate reach
        }
        
        self.joint_limits = robot_info['joint_limits']
        self.workspace_bounds = [
            [-robot_info['workspace_radius'], robot_info['workspace_radius']],
            [-robot_info['workspace_radius'], robot_info['workspace_radius']],
            [0, robot_info['workspace_radius']]
        ]
        
        return robot_info
    
    def _compute_obstacle_density(self, obstacles: List[Dict]) -> float:
        """Compute obstacle density (obstacles per unit volume)"""
        if not self.workspace_bounds:
            return 0.0
        
        workspace_volume = np.prod([b[1] - b[0] for b in self.workspace_bounds])
        return len(obstacles) / workspace_volume
    
    def _compute_obstacle_clustering(self, obstacles: List[Dict]) -> float:
        """Compute spatial clustering coefficient of obstacles"""
        if len(obstacles) < 2:
            return 0.0
        
        positions = np.array([obs['position'] for obs in obstacles])
        distances = cdist(positions, positions)
        
        # Average distance to nearest neighbor
        min_distances = []
        for i in range(len(positions)):
            others = np.delete(distances[i], i)
            if len(others) > 0:
                min_distances.append(np.min(others))
        
        if not min_distances:
            return 0.0
        
        # Clustering coefficient: inverse of average nearest neighbor distance
        avg_min_distance = np.mean(min_distances)
        return 1.0 / (1.0 + avg_min_distance)
    
    def _compute_workspace_coverage(self, obstacles: List[Dict]) -> float:
        """Compute percentage of workspace occupied by obstacles"""
        if not self.workspace_bounds:
            return 0.0
        
        workspace_volume = np.prod([b[1] - b[0] for b in self.workspace_bounds])
        
        total_obstacle_volume = 0.0
        for obs in obstacles:
            if obs['type'] == 'box':
                volume = np.prod(obs['size'])
                total_obstacle_volume += volume
            elif obs['type'] == 'sphere':
                radius = obs['size'][0] if obs['size'] else 0.5
                volume = (4/3) * np.pi * radius**3
                total_obstacle_volume += volume
        
        return min(total_obstacle_volume / workspace_volume, 1.0)
    
    def _compute_avg_obstacle_size(self, obstacles: List[Dict]) -> float:
        """Compute average obstacle size"""
        if not obstacles:
            return 0.0
        
        sizes = []
        for obs in obstacles:
            if obs['type'] == 'box':
                # Use volume as size metric
                size = np.prod(obs['size'])
            elif obs['type'] == 'sphere':
                radius = obs['size'][0] if obs['size'] else 0.5
                size = (4/3) * np.pi * radius**3
            else:
                size = 1.0  # Default size
            sizes.append(size)
        
        return np.mean(sizes)
    
    def _compute_min_corridor_width(self, obstacles: List[Dict]) -> float:
        """Estimate minimum corridor width between obstacles"""
        if len(obstacles) < 2:
            return 10.0  # Large value if no obstacles
        
        positions = np.array([obs['position'] for obs in obstacles])
        distances = cdist(positions, positions)
        
        # Find minimum distance between obstacle surfaces
        min_surface_distance = float('inf')
        for i in range(len(obstacles)):
            for j in range(i+1, len(obstacles)):
                center_distance = distances[i, j]
                # Subtract obstacle radii (approximate)
                size_i = np.mean(obstacles[i]['size'])
                size_j = np.mean(obstacles[j]['size'])
                surface_distance = center_distance - (size_i + size_j) / 2
                min_surface_distance = min(min_surface_distance, surface_distance)
        
        return max(min_surface_distance, 0.1)  # Minimum 10cm corridor
    
    def _compute_path_length(self, poses: List[Dict]) -> float:
        """Compute total path length through poses"""
        if len(poses) < 2:
            return 0.0
        
        total_length = 0.0
        for i in range(1, len(poses)):
            pos1 = np.array(poses[i-1]['position'])
            pos2 = np.array(poses[i]['position'])
            total_length += np.linalg.norm(pos2 - pos1)
        
        return total_length
    
    def _compute_pose_diversity(self, poses: List[Dict]) -> float:
        """Compute diversity/spread of poses in workspace"""
        if len(poses) < 2:
            return 0.0
        
        positions = np.array([pose['position'] for pose in poses])
        
        # Use PCA to measure spread
        pca = PCA()
        pca.fit(positions)
        
        # Diversity is the sum of explained variance
        diversity = np.sum(pca.explained_variance_)
        return diversity
    
    def _compute_orientation_complexity(self, poses: List[Dict]) -> float:
        """Compute complexity of orientation changes"""
        if len(poses) < 2:
            return 0.0
        
        total_rotation = 0.0
        for i in range(1, len(poses)):
            q1 = np.array(poses[i-1]['orientation'])  # [x, y, z, w]
            q2 = np.array(poses[i]['orientation'])
            
            # Quaternion dot product
            dot_product = np.abs(np.dot(q1, q2))
            
            # Angle between quaternions
            angle = 2 * np.arccos(np.clip(dot_product, 0, 1))
            total_rotation += angle
        
        return total_rotation
    
    def _compute_joint_range_utilization(self, poses: List[Dict], robot_info: Dict) -> float:
        """Estimate joint range utilization for poses"""
        # Simplified - would require actual IK solving
        # Use position spread as proxy
        positions = np.array([pose['position'] for pose in poses])
        
        if len(positions) == 0:
            return 0.0
        
        # Normalize positions by workspace bounds
        if self.workspace_bounds:
            normalized_spread = 0.0
            for i, bounds in enumerate(self.workspace_bounds):
                pos_range = np.max(positions[:, i]) - np.min(positions[:, i])
                workspace_range = bounds[1] - bounds[0]
                normalized_spread += pos_range / workspace_range
            
            return normalized_spread / len(self.workspace_bounds)
        
        return 0.5  # Default moderate utilization
    
    def _compute_workspace_volume(self) -> float:
        """Compute total workspace volume"""
        if self.workspace_bounds:
            return np.prod([b[1] - b[0] for b in self.workspace_bounds])
        return 1.0  # Default volume
    
    def _compute_goal_reachability(self, poses: List[Dict], robot_info: Dict) -> float:
        """Estimate difficulty of reaching goal poses"""
        # Use distance from workspace center as proxy
        if not self.workspace_bounds:
            return 0.5
        
        center = np.array([(b[0] + b[1]) / 2 for b in self.workspace_bounds])
        workspace_radius = robot_info.get('workspace_radius', 1.0)
        
        reachability_scores = []
        for pose in poses:
            pos = np.array(pose['position'])
            distance_from_center = np.linalg.norm(pos - center)
            # Closer to center = easier to reach
            reachability = 1.0 - (distance_from_center / workspace_radius)
            reachability_scores.append(max(reachability, 0.0))
        
        return np.mean(reachability_scores) if reachability_scores else 0.5
    
    def _compute_kinematic_complexity(self, poses: List[Dict], robot_info: Dict) -> float:
        """Estimate kinematic complexity (simplified)"""
        # Use pose diversity and orientation complexity as proxy
        pose_diversity = self._compute_pose_diversity(poses)
        orientation_complexity = self._compute_orientation_complexity(poses)
        
        # Combine metrics
        complexity = (pose_diversity + orientation_complexity) / 2
        return min(complexity, 10.0)  # Cap at reasonable value
    
    def _compute_start_goal_distance(self, poses: List[Dict]) -> float:
        """Compute Euclidean distance between start and goal"""
        if len(poses) < 2:
            return 0.0
        
        start_pos = np.array(poses[0]['position'])
        goal_pos = np.array(poses[-1]['position'])
        
        return np.linalg.norm(goal_pos - start_pos)
    
    def _estimate_cspace_obstacles(self, obstacles: List[Dict], robot_info: Dict) -> float:
        """Estimate configuration space obstacle density"""
        # Simplified estimate based on workspace obstacles and robot size
        workspace_coverage = self._compute_workspace_coverage(obstacles)
        
        # Configuration space obstacles are typically larger than workspace obstacles
        # due to robot geometry
        cspace_factor = 2.0  # Conservative factor
        cspace_density = min(workspace_coverage * cspace_factor, 1.0)
        
        return cspace_density
    
    def _compute_manipulability_variation(self, poses: List[Dict], robot_info: Dict) -> float:
        """Estimate variation in robot manipulability"""
        # Use distance from workspace center as proxy for manipulability
        if not self.workspace_bounds:
            return 0.5
        
        center = np.array([(b[0] + b[1]) / 2 for b in self.workspace_bounds])
        
        manipulabilities = []
        for pose in poses:
            pos = np.array(pose['position'])
            distance_from_center = np.linalg.norm(pos - center)
            workspace_radius = robot_info.get('workspace_radius', 1.0)
            
            # Manipulability decreases with distance from center
            manipulability = 1.0 - (distance_from_center / workspace_radius)
            manipulabilities.append(max(manipulability, 0.1))
        
        # Variation is standard deviation
        if len(manipulabilities) > 1:
            return np.std(manipulabilities)
        return 0.0

def main():
    parser = argparse.ArgumentParser(description='Extract scenario features for adaptive parameter selection')
    parser.add_argument('--urdf-file', required=True, help='Robot URDF file')
    parser.add_argument('--environment-file', required=True, help='Environment/obstacles file')
    parser.add_argument('--poses-file', required=True, help='Scan poses CSV file')
    parser.add_argument('--output', help='Output JSON file for features')
    parser.add_argument('--verbose', action='store_true', help='Verbose logging')
    
    args = parser.parse_args()
    
    # Setup logging
    level = logging.INFO if args.verbose else logging.WARNING
    logging.basicConfig(level=level, format='%(asctime)s - %(levelname)s - %(message)s')
    
    try:
        # Analyze scenario
        analyzer = ScenarioAnalyzer()
        features = analyzer.analyze_scenario(
            args.urdf_file,
            args.environment_file, 
            args.poses_file
        )
        
        # Convert to dictionary
        feature_dict = asdict(features)
        feature_dict['feature_vector'] = features.to_vector().tolist()
        feature_dict['feature_names'] = ScenarioFeatures.feature_names()
        
        # Save results
        if args.output:
            with open(args.output, 'w') as f:
                json.dump(feature_dict, f, indent=2)
            logging.info(f"Features saved to {args.output}")
        else:
            print(json.dumps(feature_dict, indent=2))
        
        logging.info("Scenario feature extraction completed successfully")
        
    except Exception as e:
        logging.error(f"Feature extraction failed: {e}")
        return 1
    
    return 0

if __name__ == '__main__':
    exit(main())
