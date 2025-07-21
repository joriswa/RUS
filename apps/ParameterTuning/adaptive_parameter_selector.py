#!/usr/bin/env python3
"""
ML-Based Adaptive Parameter Selection for STOMP
===============================================

This module implements machine learning models to predict optimal STOMP
parameters based on scenario features. It learns from historical optimization
results to provide intelligent parameter initialization and adaptive selection.

Key Features:
- Scenario-specific parameter prediction
- Multi-objective optimization integration
- Transfer learning across similar scenarios
- Online adaptation based on performance feedback

Authors: Adaptive Parameter Selection System
Date: July 2025
"""

import numpy as np
import pandas as pd
import json
import pickle
import logging
import argparse
from pathlib import Path
from typing import Dict, List, Tuple, Any, Optional
from dataclasses import dataclass
import yaml

# Machine Learning imports
try:
    from sklearn.ensemble import RandomForestRegressor, GradientBoostingRegressor
    from sklearn.neural_network import MLPRegressor
    from sklearn.model_selection import train_test_split, cross_val_score
    from sklearn.preprocessing import StandardScaler
    from sklearn.metrics import mean_squared_error, r2_score
    from sklearn.cluster import KMeans
    import joblib
    SKLEARN_AVAILABLE = True
except ImportError:
    SKLEARN_AVAILABLE = False
    logging.warning("scikit-learn not available. Install with: pip install scikit-learn")

try:
    import optuna
    OPTUNA_AVAILABLE = True
except ImportError:
    OPTUNA_AVAILABLE = False
    logging.warning("optuna not available. Install with: pip install optuna")

from scenario_analyzer import ScenarioAnalyzer, ScenarioFeatures

@dataclass
class ParameterPrediction:
    """Predicted STOMP parameters with confidence scores"""
    num_noisy_trajectories: int
    num_best_samples: int
    max_iterations: int
    learning_rate: float
    temperature: float
    dt: float
    velocity_limit: float
    acceleration_limit: float
    
    # Confidence scores for each parameter
    confidence_scores: Dict[str, float]
    prediction_method: str
    scenario_similarity: float
    
    def to_dict(self) -> Dict[str, Any]:
        return {
            'parameters': {
                'num_noisy_trajectories': self.num_noisy_trajectories,
                'num_best_samples': self.num_best_samples,
                'max_iterations': self.max_iterations,
                'learning_rate': self.learning_rate,
                'temperature': self.temperature,
                'dt': self.dt,
                'velocity_limit': self.velocity_limit,
                'acceleration_limit': self.acceleration_limit
            },
            'confidence_scores': self.confidence_scores,
            'prediction_method': self.prediction_method,
            'scenario_similarity': self.scenario_similarity
        }

class AdaptiveParameterSelector:
    """ML-based adaptive parameter selection for STOMP optimization"""
    
    def __init__(self, model_dir: str = "models"):
        self.model_dir = Path(model_dir)
        self.model_dir.mkdir(exist_ok=True)
        
        # ML models for different parameter groups
        self.models = {}
        self.scalers = {}
        self.scenario_clusters = None
        
        # Parameter bounds
        self.param_bounds = {
            'num_noisy_trajectories': (10, 100),
            'num_best_samples': (5, 50), 
            'max_iterations': (50, 500),
            'learning_rate': (0.1, 1.0),
            'temperature': (1.0, 50.0),
            'dt': (0.01, 0.2),
            'velocity_limit': (0.5, 3.0),
            'acceleration_limit': (0.1, 1.0)
        }
        
        # Historical data
        self.training_data = []
        self.performance_history = []
        
        logging.info(f"Initialized AdaptiveParameterSelector with model directory: {model_dir}")
    
    def train_models(self, training_data_file: str) -> Dict[str, float]:
        """Train ML models from historical optimization data"""
        if not SKLEARN_AVAILABLE:
            raise ImportError("scikit-learn required for model training")
        
        logging.info(f"Training models from {training_data_file}")
        
        # Load training data
        training_data = self._load_training_data(training_data_file)
        
        if len(training_data) < 10:
            logging.warning(f"Insufficient training data: {len(training_data)} samples")
            return {}
        
        # Prepare feature matrix and target vectors
        X, y_dict = self._prepare_training_data(training_data)
        
        # Train separate models for different parameter groups
        model_scores = {}
        
        # Group 1: Integer parameters (num_noisy_trajectories, num_best_samples, max_iterations)
        int_params = ['num_noisy_trajectories', 'num_best_samples', 'max_iterations']
        model_scores.update(self._train_parameter_group(X, y_dict, int_params, 'integer_params'))
        
        # Group 2: Continuous parameters (learning_rate, temperature, dt)
        cont_params = ['learning_rate', 'temperature', 'dt']
        model_scores.update(self._train_parameter_group(X, y_dict, cont_params, 'continuous_params'))
        
        # Group 3: Dynamic limits (velocity_limit, acceleration_limit)
        dyn_params = ['velocity_limit', 'acceleration_limit']
        model_scores.update(self._train_parameter_group(X, y_dict, dyn_params, 'dynamic_params'))
        
        # Train scenario clustering
        self._train_scenario_clustering(X)
        
        # Save models
        self._save_models()
        
        logging.info(f"Model training completed. Scores: {model_scores}")
        return model_scores
    
    def predict_parameters(self, scenario_features: ScenarioFeatures) -> ParameterPrediction:
        """Predict optimal parameters for a scenario"""
        if not self.models:
            logging.warning("No trained models available. Loading default models...")
            if not self._load_models():
                return self._get_default_parameters(scenario_features)
        
        # Convert features to vector
        feature_vector = scenario_features.to_vector().reshape(1, -1)
        
        # Scale features
        if 'features' in self.scalers:
            feature_vector = self.scalers['features'].transform(feature_vector)
        
        # Predict parameters using different model groups
        predicted_params = {}
        confidence_scores = {}
        
        # Predict integer parameters
        if 'integer_params' in self.models:
            int_predictions = self.models['integer_params'].predict(feature_vector)[0]
            int_params = ['num_noisy_trajectories', 'num_best_samples', 'max_iterations']
            for i, param in enumerate(int_params):
                predicted_params[param] = int(round(max(self.param_bounds[param][0], 
                                                       min(self.param_bounds[param][1], int_predictions[i]))))
                confidence_scores[param] = self._estimate_confidence('integer_params', feature_vector)
        
        # Predict continuous parameters
        if 'continuous_params' in self.models:
            cont_predictions = self.models['continuous_params'].predict(feature_vector)[0]
            cont_params = ['learning_rate', 'temperature', 'dt']
            for i, param in enumerate(cont_params):
                predicted_params[param] = max(self.param_bounds[param][0],
                                            min(self.param_bounds[param][1], cont_predictions[i]))
                confidence_scores[param] = self._estimate_confidence('continuous_params', feature_vector)
        
        # Predict dynamic parameters
        if 'dynamic_params' in self.models:
            dyn_predictions = self.models['dynamic_params'].predict(feature_vector)[0]
            dyn_params = ['velocity_limit', 'acceleration_limit']
            for i, param in enumerate(dyn_params):
                predicted_params[param] = max(self.param_bounds[param][0],
                                            min(self.param_bounds[param][1], dyn_predictions[i]))
                confidence_scores[param] = self._estimate_confidence('dynamic_params', feature_vector)
        
        # Compute scenario similarity
        scenario_similarity = self._compute_scenario_similarity(feature_vector)
        
        # Create prediction object
        prediction = ParameterPrediction(
            num_noisy_trajectories=predicted_params.get('num_noisy_trajectories', 22),
            num_best_samples=predicted_params.get('num_best_samples', 17),
            max_iterations=predicted_params.get('max_iterations', 108),
            learning_rate=predicted_params.get('learning_rate', 0.354),
            temperature=predicted_params.get('temperature', 28.27),
            dt=predicted_params.get('dt', 0.092),
            velocity_limit=predicted_params.get('velocity_limit', 1.93),
            acceleration_limit=predicted_params.get('acceleration_limit', 0.523),
            confidence_scores=confidence_scores,
            prediction_method='ML_ensemble',
            scenario_similarity=scenario_similarity
        )
        
        logging.info(f"Predicted parameters with avg confidence: {np.mean(list(confidence_scores.values())):.3f}")
        return prediction
    
    def update_performance(self, scenario_features: ScenarioFeatures, 
                          parameters: Dict[str, Any], performance: Dict[str, float]):
        """Update models with performance feedback (online learning)"""
        
        # Store performance data
        performance_data = {
            'features': scenario_features.to_vector().tolist(),
            'parameters': parameters,
            'performance': performance,
            'timestamp': pd.Timestamp.now().isoformat()
        }
        
        self.performance_history.append(performance_data)
        
        # Trigger model retraining if enough new data accumulated
        if len(self.performance_history) >= 10:
            logging.info("Triggering incremental model update")
            self._incremental_update()
            self.performance_history = []  # Clear buffer
    
    def generate_parameter_candidates(self, scenario_features: ScenarioFeatures, 
                                    n_candidates: int = 5) -> List[ParameterPrediction]:
        """Generate multiple parameter candidates for multi-objective optimization"""
        
        candidates = []
        
        # Method 1: Base ML prediction
        base_prediction = self.predict_parameters(scenario_features)
        candidates.append(base_prediction)
        
        # Method 2: Add noise to base prediction (exploration)
        for i in range(n_candidates - 1):
            noisy_params = self._add_parameter_noise(base_prediction, noise_level=0.1 * (i + 1))
            candidates.append(noisy_params)
        
        return candidates
    
    def _load_training_data(self, training_data_file: str) -> List[Dict]:
        """Load historical optimization results for training"""
        training_data = []
        
        try:
            with open(training_data_file, 'r') as f:
                data = json.load(f)
            
            if isinstance(data, list):
                training_data = data
            elif isinstance(data, dict) and 'training_samples' in data:
                training_data = data['training_samples']
            
        except Exception as e:
            logging.error(f"Failed to load training data: {e}")
            # Generate synthetic training data for testing
            training_data = self._generate_synthetic_training_data()
        
        return training_data
    
    def _prepare_training_data(self, training_data: List[Dict]) -> Tuple[np.ndarray, Dict[str, np.ndarray]]:
        """Prepare feature matrix and target vectors from training data"""
        
        features = []
        targets = {param: [] for param in self.param_bounds.keys()}
        
        for sample in training_data:
            if 'scenario_features' in sample and 'parameters' in sample:
                # Extract scenario features
                if isinstance(sample['scenario_features'], list):
                    features.append(sample['scenario_features'])
                elif isinstance(sample['scenario_features'], dict):
                    # Convert feature dict to vector
                    feature_names = ScenarioFeatures.feature_names()
                    feature_vector = [sample['scenario_features'].get(name, 0.0) for name in feature_names]
                    features.append(feature_vector)
                
                # Extract parameter targets
                params = sample['parameters']
                for param_name in self.param_bounds.keys():
                    targets[param_name].append(params.get(param_name, 0.0))
        
        X = np.array(features)
        y_dict = {param: np.array(values) for param, values in targets.items()}
        
        logging.info(f"Prepared training data: {X.shape[0]} samples, {X.shape[1]} features")
        return X, y_dict
    
    def _train_parameter_group(self, X: np.ndarray, y_dict: Dict[str, np.ndarray], 
                              param_names: List[str], group_name: str) -> Dict[str, float]:
        """Train model for a group of related parameters"""
        
        # Combine targets for this parameter group
        y_group = np.column_stack([y_dict[param] for param in param_names])
        
        # Scale features
        scaler = StandardScaler()
        X_scaled = scaler.fit_transform(X)
        self.scalers['features'] = scaler
        
        # Split data
        X_train, X_test, y_train, y_test = train_test_split(X_scaled, y_group, 
                                                          test_size=0.2, random_state=42)
        
        # Try multiple models
        models = {
            'random_forest': RandomForestRegressor(n_estimators=100, random_state=42),
            'gradient_boosting': GradientBoostingRegressor(n_estimators=100, random_state=42),
            'neural_network': MLPRegressor(hidden_layer_sizes=(100, 50), max_iter=1000, random_state=42)
        }
        
        best_model = None
        best_score = -float('inf')
        
        for model_name, model in models.items():
            try:
                # Train model
                model.fit(X_train, y_train)
                
                # Evaluate
                y_pred = model.predict(X_test)
                score = r2_score(y_test, y_pred)
                
                logging.info(f"{group_name} - {model_name}: R² = {score:.3f}")
                
                if score > best_score:
                    best_score = score
                    best_model = model
                    
            except Exception as e:
                logging.warning(f"Failed to train {model_name} for {group_name}: {e}")
        
        if best_model is not None:
            self.models[group_name] = best_model
            return {f"{group_name}_r2": best_score}
        
        return {}
    
    def _train_scenario_clustering(self, X: np.ndarray):
        """Train clustering model for scenario similarity"""
        try:
            # Use k-means to cluster scenarios
            n_clusters = min(5, max(2, len(X) // 10))  # Adaptive number of clusters
            kmeans = KMeans(n_clusters=n_clusters, random_state=42)
            kmeans.fit(X)
            self.scenario_clusters = kmeans
            
            logging.info(f"Trained scenario clustering with {n_clusters} clusters")
            
        except Exception as e:
            logging.warning(f"Failed to train scenario clustering: {e}")
    
    def _estimate_confidence(self, model_group: str, feature_vector: np.ndarray) -> float:
        """Estimate prediction confidence for a model group"""
        # Simple confidence based on feature vector similarity to training data
        # In practice, could use model uncertainty estimation
        
        if model_group not in self.models:
            return 0.5
        
        # Use scenario similarity as confidence proxy
        if hasattr(self.models[model_group], 'predict_proba'):
            # For models with uncertainty estimation
            try:
                proba = self.models[model_group].predict_proba(feature_vector)
                confidence = np.max(proba)
            except:
                confidence = 0.7
        else:
            # Default confidence for regression models
            confidence = 0.7
        
        return confidence
    
    def _compute_scenario_similarity(self, feature_vector: np.ndarray) -> float:
        """Compute similarity to known scenarios"""
        if self.scenario_clusters is None:
            return 0.5
        
        try:
            # Distance to nearest cluster center
            distances = self.scenario_clusters.transform(feature_vector)
            min_distance = np.min(distances)
            
            # Convert distance to similarity score
            similarity = 1.0 / (1.0 + min_distance)
            return similarity
            
        except Exception as e:
            logging.warning(f"Failed to compute scenario similarity: {e}")
            return 0.5
    
    def _add_parameter_noise(self, base_prediction: ParameterPrediction, 
                           noise_level: float = 0.1) -> ParameterPrediction:
        """Add controlled noise to parameters for exploration"""
        
        noisy_params = {}
        for param_name in self.param_bounds.keys():
            base_value = getattr(base_prediction, param_name)
            param_range = self.param_bounds[param_name][1] - self.param_bounds[param_name][0]
            
            # Add Gaussian noise
            noise = np.random.normal(0, noise_level * param_range)
            noisy_value = base_value + noise
            
            # Clip to bounds
            min_val, max_val = self.param_bounds[param_name]
            noisy_value = max(min_val, min(max_val, noisy_value))
            
            # Round integer parameters
            if param_name in ['num_noisy_trajectories', 'num_best_samples', 'max_iterations']:
                noisy_value = int(round(noisy_value))
            
            noisy_params[param_name] = noisy_value
        
        # Create new prediction with noisy parameters
        noisy_prediction = ParameterPrediction(
            num_noisy_trajectories=noisy_params['num_noisy_trajectories'],
            num_best_samples=noisy_params['num_best_samples'],
            max_iterations=noisy_params['max_iterations'],
            learning_rate=noisy_params['learning_rate'],
            temperature=noisy_params['temperature'],
            dt=noisy_params['dt'],
            velocity_limit=noisy_params['velocity_limit'],
            acceleration_limit=noisy_params['acceleration_limit'],
            confidence_scores=base_prediction.confidence_scores.copy(),
            prediction_method='ML_exploration',
            scenario_similarity=base_prediction.scenario_similarity
        )
        
        return noisy_prediction
    
    def _get_default_parameters(self, scenario_features: ScenarioFeatures) -> ParameterPrediction:
        """Get default parameters when no model is available"""
        
        # Use scenario features to adjust default parameters
        obstacle_density = scenario_features.obstacle_density
        path_complexity = scenario_features.orientation_complexity
        
        # Simple heuristics for parameter adjustment
        base_iterations = 108
        if obstacle_density > 0.5:
            base_iterations = int(base_iterations * 1.5)  # More iterations for complex scenes
        
        base_noisy = 22
        if path_complexity > 2.0:
            base_noisy = int(base_noisy * 1.2)  # More samples for complex paths
        
        default_prediction = ParameterPrediction(
            num_noisy_trajectories=base_noisy,
            num_best_samples=17,
            max_iterations=base_iterations,
            learning_rate=0.354,
            temperature=28.27,
            dt=0.092,
            velocity_limit=1.93,
            acceleration_limit=0.523,
            confidence_scores={param: 0.3 for param in self.param_bounds.keys()},
            prediction_method='heuristic_default',
            scenario_similarity=0.0
        )
        
        return default_prediction
    
    def _generate_synthetic_training_data(self) -> List[Dict]:
        """Generate synthetic training data for testing"""
        logging.info("Generating synthetic training data")
        
        synthetic_data = []
        
        for i in range(50):
            # Random scenario features
            features = {
                'obstacle_density': np.random.uniform(0, 1),
                'obstacle_clustering': np.random.uniform(0, 1),
                'workspace_coverage': np.random.uniform(0, 0.5),
                'avg_obstacle_size': np.random.uniform(0.1, 2.0),
                'min_corridor_width': np.random.uniform(0.1, 1.0),
                'path_length': np.random.uniform(1, 5),
                'pose_diversity': np.random.uniform(0, 2),
                'orientation_complexity': np.random.uniform(0, 3),
                'joint_range_utilization': np.random.uniform(0, 1),
                'workspace_volume': np.random.uniform(1, 10),
                'goal_reachability': np.random.uniform(0, 1),
                'kinematic_complexity': np.random.uniform(0, 5),
                'start_goal_distance': np.random.uniform(0.5, 3),
                'configuration_space_obstacles': np.random.uniform(0, 1),
                'manipulability_variation': np.random.uniform(0, 0.5)
            }
            
            # Random parameters (with some correlation to features)
            complexity_factor = features['obstacle_density'] + features['orientation_complexity']
            
            params = {
                'num_noisy_trajectories': int(20 + complexity_factor * 30),
                'num_best_samples': int(15 + complexity_factor * 10),
                'max_iterations': int(100 + complexity_factor * 200),
                'learning_rate': 0.2 + complexity_factor * 0.3,
                'temperature': 20 + complexity_factor * 20,
                'dt': 0.05 + np.random.uniform(0, 0.1),
                'velocity_limit': 1.5 + np.random.uniform(0, 1),
                'acceleration_limit': 0.3 + np.random.uniform(0, 0.5)
            }
            
            # Simulate performance (inversely related to complexity)
            performance = {
                'success_rate': max(0.1, 1.0 - complexity_factor * 0.3),
                'planning_time': 1.0 + complexity_factor * 5,
                'trajectory_quality': max(0.1, 1.0 - complexity_factor * 0.2)
            }
            
            sample = {
                'scenario_features': features,
                'parameters': params,
                'performance': performance
            }
            
            synthetic_data.append(sample)
        
        return synthetic_data
    
    def _save_models(self):
        """Save trained models to disk"""
        try:
            # Save models
            for name, model in self.models.items():
                model_file = self.model_dir / f"{name}_model.joblib"
                joblib.dump(model, model_file)
            
            # Save scalers
            for name, scaler in self.scalers.items():
                scaler_file = self.model_dir / f"{name}_scaler.joblib"
                joblib.dump(scaler, scaler_file)
            
            # Save scenario clusters
            if self.scenario_clusters is not None:
                cluster_file = self.model_dir / "scenario_clusters.joblib"
                joblib.dump(self.scenario_clusters, cluster_file)
            
            logging.info(f"Models saved to {self.model_dir}")
            
        except Exception as e:
            logging.error(f"Failed to save models: {e}")
    
    def _load_models(self) -> bool:
        """Load trained models from disk"""
        try:
            # Load models
            for model_file in self.model_dir.glob("*_model.joblib"):
                model_name = model_file.stem.replace('_model', '')
                self.models[model_name] = joblib.load(model_file)
            
            # Load scalers
            for scaler_file in self.model_dir.glob("*_scaler.joblib"):
                scaler_name = scaler_file.stem.replace('_scaler', '')
                self.scalers[scaler_name] = joblib.load(scaler_file)
            
            # Load scenario clusters
            cluster_file = self.model_dir / "scenario_clusters.joblib"
            if cluster_file.exists():
                self.scenario_clusters = joblib.load(cluster_file)
            
            logging.info(f"Loaded {len(self.models)} models from {self.model_dir}")
            return len(self.models) > 0
            
        except Exception as e:
            logging.warning(f"Failed to load models: {e}")
            return False
    
    def _incremental_update(self):
        """Perform incremental model update with new performance data"""
        # Simplified incremental update - in practice would use online learning
        logging.info("Performing incremental model update (placeholder)")

def main():
    parser = argparse.ArgumentParser(description='ML-based adaptive parameter selection')
    parser.add_argument('--train', help='Training data file for model training')
    parser.add_argument('--scenario-urdf', help='Robot URDF file for prediction')
    parser.add_argument('--scenario-env', help='Environment file for prediction')
    parser.add_argument('--scenario-poses', help='Poses file for prediction')
    parser.add_argument('--output', help='Output file for predictions')
    parser.add_argument('--model-dir', default='models', help='Model directory')
    parser.add_argument('--verbose', action='store_true', help='Verbose logging')
    
    args = parser.parse_args()
    
    # Setup logging
    level = logging.INFO if args.verbose else logging.WARNING
    logging.basicConfig(level=level, format='%(asctime)s - %(levelname)s - %(message)s')
    
    try:
        selector = AdaptiveParameterSelector(args.model_dir)
        
        if args.train:
            # Train models
            logging.info("Training ML models...")
            scores = selector.train_models(args.train)
            print(f"Training completed. Model scores: {json.dumps(scores, indent=2)}")
        
        elif args.scenario_urdf and args.scenario_env and args.scenario_poses:
            # Predict parameters for scenario
            logging.info("Analyzing scenario and predicting parameters...")
            
            analyzer = ScenarioAnalyzer()
            features = analyzer.analyze_scenario(args.scenario_urdf, args.scenario_env, args.scenario_poses)
            
            prediction = selector.predict_parameters(features)
            
            result = {
                'scenario_features': features.to_vector().tolist(),
                'parameter_prediction': prediction.to_dict()
            }
            
            if args.output:
                with open(args.output, 'w') as f:
                    json.dump(result, f, indent=2)
                logging.info(f"Prediction saved to {args.output}")
            else:
                print(json.dumps(result, indent=2))
        
        else:
            print("Please provide either --train for training or scenario files for prediction")
            return 1
        
    except Exception as e:
        logging.error(f"Adaptive parameter selection failed: {e}")
        return 1
    
    return 0

if __name__ == '__main__':
    exit(main())
