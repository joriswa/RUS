#!/usr/bin/env python3
"""
Landmark-based registration for knee ultrasound using robot force feedback.
Targets specific anatomical points on the leg to establish model-to-reality correspondence.
"""

import numpy as np
import pandas as pd
from typing import List, Dict, Tuple, Optional
from pathlib import Path
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import json
from datetime import datetime
from scipy.spatial.transform import Rotation
from scipy.optimize import minimize
import seaborn as sns

class KneeLandmarkRegistration:
    """
    Landmark-based registration system for knee ultrasound procedures.
    Uses robot force feedback to target anatomical landmarks and register STL model.
    """
    
    def __init__(self, stl_model_path: Optional[str] = None):
        self.stl_model_path = stl_model_path
        
        # Force feedback parameters
        self.contact_force_threshold = 2.0  # Newtons
        self.max_safe_force = 10.0  # Newtons
        self.force_measurement_noise = 0.1  # Standard deviation in Newtons
        
        # Registration parameters
        self.max_registration_error = 0.005  # 5mm acceptable error
        self.min_landmarks_for_registration = 3  # Minimum 3 points for 3D registration
        
        # Define anatomical landmarks for knee ultrasound
        self.knee_landmarks = self._define_knee_landmarks()
        
        # Storage for detected landmarks
        self.detected_landmarks = []
        self.model_landmarks = []
        self.registration_transform = None
        self.registration_uncertainty = None
    
    def _define_knee_landmarks(self) -> Dict[str, Dict]:
        """Define key anatomical landmarks for knee registration."""
        return {
            'patella_apex': {
                'description': 'Apex of the patella (kneecap)',
                'approximate_position': np.array([0.0, 0.05, 0.0]),  # Relative to knee center
                'accessibility': 'high',  # Easy to reach with probe
                'reliability': 'high',    # Consistent landmark
                'force_approach_direction': np.array([0, 1, 0])  # Approach from above
            },
            'medial_femoral_condyle': {
                'description': 'Medial condyle of femur',
                'approximate_position': np.array([-0.03, 0.02, 0.0]),
                'accessibility': 'medium',
                'reliability': 'high',
                'force_approach_direction': np.array([-1, 0, 0])
            },
            'lateral_femoral_condyle': {
                'description': 'Lateral condyle of femur',
                'approximate_position': np.array([0.03, 0.02, 0.0]),
                'accessibility': 'medium',
                'reliability': 'high',
                'force_approach_direction': np.array([1, 0, 0])
            },
            'tibial_tuberosity': {
                'description': 'Tibial tuberosity',
                'approximate_position': np.array([0.0, -0.04, 0.02]),
                'accessibility': 'high',
                'reliability': 'medium',
                'force_approach_direction': np.array([0, -1, 0])
            },
            'medial_tibial_plateau': {
                'description': 'Medial edge of tibial plateau',
                'approximate_position': np.array([-0.025, -0.02, 0.0]),
                'accessibility': 'medium',
                'reliability': 'medium',
                'force_approach_direction': np.array([-1, -1, 0]) / np.sqrt(2)
            },
            'lateral_tibial_plateau': {
                'description': 'Lateral edge of tibial plateau',
                'approximate_position': np.array([0.025, -0.02, 0.0]),
                'accessibility': 'medium',
                'reliability': 'medium',
                'force_approach_direction': np.array([1, -1, 0]) / np.sqrt(2)
            },
            'patellar_tendon_insertion': {
                'description': 'Patellar tendon insertion point',
                'approximate_position': np.array([0.0, -0.03, 0.015]),
                'accessibility': 'high',
                'reliability': 'medium',
                'force_approach_direction': np.array([0, -1, 1]) / np.sqrt(2)
            }
        }
    
    def simulate_robot_force_detection(self, target_position: np.ndarray, 
                                     approach_direction: np.ndarray,
                                     real_landmark_position: np.ndarray,
                                     landmark_info: Dict = None) -> Dict:
        """
        Simulate robot moving to detect a landmark using force feedback.
        In real implementation, this would interface with robot controller.
        """
        
        # Simulate robot approach trajectory
        approach_distance = 0.05  # 5cm approach distance
        start_position = target_position - approach_direction * approach_distance
        
        # Simulate contact detection
        contact_distance = np.linalg.norm(target_position - real_landmark_position)
        
        # Add measurement noise
        measured_force = 0.0
        contact_detected = False
        final_position = target_position.copy()
        
        # Simulate more realistic contact detection - robot searches in vicinity
        # Higher chance of contact for accessible landmarks
        search_radius = 0.025  # 2.5cm search radius
        if contact_distance < search_radius:
            # Probability of contact detection based on accessibility and distance
            if landmark_info:
                accessibility_factor = {'high': 0.95, 'medium': 0.8, 'low': 0.6}[
                    landmark_info.get('accessibility', 'medium')]
            else:
                accessibility_factor = 0.8
            contact_probability = accessibility_factor * (1 - contact_distance / search_radius)
            if np.random.random() < contact_probability:
                # Simulate force buildup
                measured_force = self.contact_force_threshold + np.random.normal(0, self.force_measurement_noise)
                contact_detected = True
                # Actual contact position with some error
                final_position = real_landmark_position + np.random.normal(0, 0.002, 3)  # 2mm accuracy
            else:
                measured_force = np.random.normal(0, self.force_measurement_noise)
        else:
            # No contact - probe reaches target without resistance
            measured_force = np.random.normal(0, self.force_measurement_noise)
        
        return {
            'contact_detected': contact_detected,
            'measured_force': measured_force,
            'contact_position': final_position,
            'approach_trajectory': np.array([start_position, final_position]),
            'measurement_uncertainty': 0.002 if contact_detected else 0.01
        }
    
    def target_landmark_sequence(self, landmark_names: List[str] = None) -> List[Dict]:
        """
        Execute a sequence of landmark targeting movements.
        """
        
        if landmark_names is None:
            # Use high-reliability landmarks first
            landmark_names = ['patella_apex', 'medial_femoral_condyle', 'lateral_femoral_condyle', 
                            'tibial_tuberosity', 'medial_tibial_plateau', 'lateral_tibial_plateau']
        
        detection_results = []
        
        print(f"🎯 Starting landmark detection sequence...")
        print(f"   Targeting {len(landmark_names)} anatomical landmarks")
        
        for i, landmark_name in enumerate(landmark_names):
            if landmark_name not in self.knee_landmarks:
                print(f"   ⚠️  Unknown landmark: {landmark_name}")
                continue
            
            landmark_info = self.knee_landmarks[landmark_name]
            
            print(f"\n📍 [{i+1}/{len(landmark_names)}] Targeting: {landmark_info['description']}")
            
            # Simulate real landmark position with some variation from model
            model_position = landmark_info['approximate_position']
            # Reduce variation to increase contact success rate for demo
            real_position = model_position + np.random.normal(0, 0.005, 3)  # 5mm variation instead of 1cm
            
            # Simulate robot detection
            detection_result = self.simulate_robot_force_detection(
                target_position=model_position,
                approach_direction=landmark_info['force_approach_direction'],
                real_landmark_position=real_position,
                landmark_info=landmark_info
            )
            
            detection_result.update({
                'landmark_name': landmark_name,
                'landmark_info': landmark_info,
                'model_position': model_position,
                'real_position': real_position
            })
            
            detection_results.append(detection_result)
            
            if detection_result['contact_detected']:
                print(f"   ✅ Contact detected! Force: {detection_result['measured_force']:.2f}N")
                print(f"   📐 Position error: {np.linalg.norm(detection_result['contact_position'] - real_position)*1000:.1f}mm")
            else:
                print(f"   ❌ No contact detected")
        
        self.detected_landmarks = detection_results
        return detection_results
    
    def estimate_registration_transform(self, detection_results: List[Dict]) -> Dict:
        """
        Estimate the registration transform from detected landmarks.
        Uses least-squares fitting to find the best rigid transformation.
        """
        
        # Filter successful detections
        successful_detections = [r for r in detection_results if r['contact_detected']]
        
        if len(successful_detections) < self.min_landmarks_for_registration:
            raise ValueError(f"Need at least {self.min_landmarks_for_registration} landmarks, got {len(successful_detections)}")
        
        print(f"\n🧮 Computing registration transform from {len(successful_detections)} landmarks...")
        
        # Prepare point sets for registration
        model_points = np.array([r['model_position'] for r in successful_detections])
        detected_points = np.array([r['contact_position'] for r in successful_detections])
        
        # Compute centroids
        model_centroid = np.mean(model_points, axis=0)
        detected_centroid = np.mean(detected_points, axis=0)
        
        # Center the points
        model_centered = model_points - model_centroid
        detected_centered = detected_points - detected_centroid
        
        # Compute optimal rotation using SVD (Kabsch algorithm)
        H = model_centered.T @ detected_centered
        U, S, Vt = np.linalg.svd(H)
        R = Vt.T @ U.T
        
        # Ensure proper rotation (determinant = 1)
        if np.linalg.det(R) < 0:
            Vt[-1, :] *= -1
            R = Vt.T @ U.T
        
        # Compute translation
        t = detected_centroid - R @ model_centroid
        
        # Create homogeneous transformation matrix
        T = np.eye(4)
        T[:3, :3] = R
        T[:3, 3] = t
        
        # Estimate registration uncertainty
        transformed_model_points = (R @ model_points.T).T + t
        residuals = detected_points - transformed_model_points
        registration_error = np.mean(np.linalg.norm(residuals, axis=1))
        
        uncertainty_metrics = self._analyze_registration_uncertainty(
            successful_detections, R, t, residuals
        )
        
        registration_result = {
            'transformation_matrix': T,
            'rotation': R,
            'translation': t,
            'registration_error': registration_error,
            'uncertainty_metrics': uncertainty_metrics,
            'num_landmarks': len(successful_detections),
            'landmark_names': [r['landmark_name'] for r in successful_detections]
        }
        
        self.registration_transform = registration_result
        
        print(f"   ✅ Registration complete!")
        print(f"   📊 Mean error: {registration_error*1000:.2f}mm")
        print(f"   🎯 Landmarks used: {registration_result['landmark_names']}")
        
        return registration_result
    
    def _analyze_registration_uncertainty(self, detections: List[Dict], 
                                        R: np.ndarray, t: np.ndarray, 
                                        residuals: np.ndarray) -> Dict:
        """Analyze uncertainty in the registration."""
        
        # Measurement uncertainty propagation
        measurement_uncertainties = [d['measurement_uncertainty'] for d in detections]
        mean_measurement_uncertainty = np.mean(measurement_uncertainties)
        
        # Geometric uncertainty based on landmark distribution
        landmark_positions = np.array([d['contact_position'] for d in detections])
        landmark_spread = np.std(landmark_positions, axis=0)
        geometric_uncertainty = np.mean(landmark_spread)
        
        # Residual analysis
        residual_magnitudes = np.linalg.norm(residuals, axis=1)
        max_residual = np.max(residual_magnitudes)
        std_residual = np.std(residual_magnitudes)
        
        # Overall uncertainty estimate
        total_uncertainty = np.sqrt(
            mean_measurement_uncertainty**2 + 
            geometric_uncertainty**2 + 
            std_residual**2
        )
        
        return {
            'measurement_uncertainty': mean_measurement_uncertainty,
            'geometric_uncertainty': geometric_uncertainty,
            'residual_std': std_residual,
            'max_residual': max_residual,
            'total_uncertainty': total_uncertainty,
            'confidence_score': max(0, 1 - total_uncertainty / self.max_registration_error)
        }
    
    def validate_registration_quality(self) -> Dict:
        """Validate the quality of the registration for clinical use."""
        
        if self.registration_transform is None:
            raise ValueError("No registration available. Run registration first.")
        
        uncertainty = self.registration_transform['uncertainty_metrics']
        error = self.registration_transform['registration_error']
        
        # Clinical validation criteria
        validation_results = {
            'acceptable_for_clinical_use': error < self.max_registration_error,
            'high_confidence': uncertainty['confidence_score'] > 0.8,
            'adequate_landmarks': self.registration_transform['num_landmarks'] >= 4,
            'uncertainty_within_limits': uncertainty['total_uncertainty'] < 0.01,  # 1cm
        }
        
        validation_results['overall_quality'] = all(validation_results.values())
        
        # Recommendations
        recommendations = []
        if error >= self.max_registration_error:
            recommendations.append("Registration error too high - retarget landmarks")
        if uncertainty['confidence_score'] <= 0.8:
            recommendations.append("Low confidence - add more landmarks")
        if self.registration_transform['num_landmarks'] < 4:
            recommendations.append("Insufficient landmarks - target additional points")
        
        validation_results['recommendations'] = recommendations
        
        return validation_results
    
    def integrate_with_stomp_planning(self, registration_result: Dict) -> Dict:
        """
        Integrate registration results with STOMP trajectory planning.
        Updates planning constraints based on registration uncertainty.
        """
        
        uncertainty = registration_result['uncertainty_metrics']['total_uncertainty']
        
        # Adjust STOMP parameters based on registration quality
        stomp_adjustments = {
            'collision_cost_weight': 25.0 * (1 + uncertainty / 0.01),  # Increase safety with uncertainty
            'smoothness_cost_weight': 5.0 * (1 + uncertainty / 0.02),   # More smoothing with uncertainty
            'safety_margin_expansion': uncertainty * 2,  # Expand safety margins
            'convergence_threshold': 1e-4 * (1 + uncertainty / 0.005),  # Looser convergence
            'uncertainty_penalty_weight': 10.0 * uncertainty  # Direct uncertainty penalty
        }
        
        # Define uncertainty-aware objective function
        def uncertainty_aware_objective(trajectory_value: float) -> float:
            """Add uncertainty penalty to trajectory objective."""
            uncertainty_penalty = stomp_adjustments['uncertainty_penalty_weight'] * uncertainty
            return trajectory_value + uncertainty_penalty
        
        integration_result = {
            'registration_transform': registration_result['transformation_matrix'],
            'stomp_parameter_adjustments': stomp_adjustments,
            'uncertainty_objective_function': uncertainty_aware_objective,
            'recommended_safety_clearance': 0.02 + uncertainty * 2,  # Base 2cm + uncertainty
            'quality_metrics': {
                'registration_error': registration_result['registration_error'],
                'uncertainty_estimate': uncertainty,
                'confidence_score': registration_result['uncertainty_metrics']['confidence_score']
            }
        }
        
        return integration_result
    
    def visualize_registration_results(self, detection_results: List[Dict], 
                                     registration_result: Dict) -> None:
        """Create comprehensive visualization of registration results."""
        
        fig = plt.figure(figsize=(20, 12))
        
        # 3D visualization of landmarks and registration
        ax1 = fig.add_subplot(2, 3, 1, projection='3d')
        
        successful_detections = [r for r in detection_results if r['contact_detected']]
        failed_detections = [r for r in detection_results if not r['contact_detected']]
        
        # Plot successful landmarks
        if successful_detections:
            model_pos = np.array([d['model_position'] for d in successful_detections])
            detected_pos = np.array([d['contact_position'] for d in successful_detections])
            
            ax1.scatter(model_pos[:, 0], model_pos[:, 1], model_pos[:, 2], 
                       c='blue', s=100, alpha=0.7, label='Model Landmarks')
            ax1.scatter(detected_pos[:, 0], detected_pos[:, 1], detected_pos[:, 2], 
                       c='red', s=100, alpha=0.7, label='Detected Landmarks')
            
            # Draw connections
            for mp, dp in zip(model_pos, detected_pos):
                ax1.plot([mp[0], dp[0]], [mp[1], dp[1]], [mp[2], dp[2]], 
                        'gray', alpha=0.5, linestyle='--')
        
        ax1.set_xlabel('X (m)')
        ax1.set_ylabel('Y (m)')
        ax1.set_zlabel('Z (m)')
        ax1.set_title('Landmark Registration (3D)')
        ax1.legend()
        
        # Registration error analysis
        ax2 = fig.add_subplot(2, 3, 2)
        if successful_detections:
            errors = [np.linalg.norm(d['contact_position'] - d['real_position']) * 1000 
                     for d in successful_detections]
            landmark_names = [d['landmark_name'].replace('_', '\n') for d in successful_detections]
            
            bars = ax2.bar(range(len(errors)), errors, color='skyblue', alpha=0.7)
            ax2.set_xticks(range(len(errors)))
            ax2.set_xticklabels(landmark_names, rotation=45, ha='right')
            ax2.set_ylabel('Detection Error (mm)')
            ax2.set_title('Landmark Detection Accuracy')
            ax2.axhline(y=5, color='red', linestyle='--', label='5mm threshold')
            ax2.legend()
            
            # Color bars based on error level
            for i, (bar, error) in enumerate(zip(bars, errors)):
                if error > 5:
                    bar.set_color('red')
                elif error > 2:
                    bar.set_color('orange')
        
        # Force measurement visualization
        ax3 = fig.add_subplot(2, 3, 3)
        forces = [d['measured_force'] for d in detection_results]
        contact_status = ['Contact' if d['contact_detected'] else 'No Contact' 
                         for d in detection_results]
        
        colors = ['green' if c == 'Contact' else 'red' for c in contact_status]
        ax3.scatter(range(len(forces)), forces, c=colors, s=100, alpha=0.7)
        ax3.axhline(y=self.contact_force_threshold, color='blue', linestyle='--', 
                   label=f'Threshold ({self.contact_force_threshold}N)')
        ax3.set_xlabel('Landmark Index')
        ax3.set_ylabel('Measured Force (N)')
        ax3.set_title('Force Feedback Analysis')
        ax3.legend()
        
        # Uncertainty analysis
        ax4 = fig.add_subplot(2, 3, 4)
        if 'uncertainty_metrics' in registration_result:
            uncertainty = registration_result['uncertainty_metrics']
            categories = ['Measurement', 'Geometric', 'Residual', 'Total']
            values = [
                uncertainty['measurement_uncertainty'] * 1000,
                uncertainty['geometric_uncertainty'] * 1000,
                uncertainty['residual_std'] * 1000,
                uncertainty['total_uncertainty'] * 1000
            ]
            
            bars = ax4.bar(categories, values, color=['blue', 'green', 'orange', 'red'], alpha=0.7)
            ax4.set_ylabel('Uncertainty (mm)')
            ax4.set_title('Registration Uncertainty Breakdown')
            ax4.tick_params(axis='x', rotation=45)
        
        # Registration quality metrics
        ax5 = fig.add_subplot(2, 3, 5)
        validation = self.validate_registration_quality()
        
        quality_metrics = {
            'Clinical\nAcceptable': validation['acceptable_for_clinical_use'],
            'High\nConfidence': validation['high_confidence'],
            'Adequate\nLandmarks': validation['adequate_landmarks'],
            'Uncertainty\nOK': validation['uncertainty_within_limits']
        }
        
        colors = ['green' if v else 'red' for v in quality_metrics.values()]
        y_pos = range(len(quality_metrics))
        
        ax5.barh(y_pos, [1]*len(quality_metrics), color=colors, alpha=0.7)
        ax5.set_yticks(y_pos)
        ax5.set_yticklabels(list(quality_metrics.keys()))
        ax5.set_xlabel('Pass/Fail')
        ax5.set_title('Registration Quality Check')
        ax5.set_xlim(0, 1)
        
        # Overall summary
        ax6 = fig.add_subplot(2, 3, 6)
        ax6.axis('off')
        
        summary_text = f"""
Registration Summary:
• Landmarks detected: {len(successful_detections)}/{len(detection_results)}
• Registration error: {registration_result['registration_error']*1000:.2f}mm
• Total uncertainty: {registration_result['uncertainty_metrics']['total_uncertainty']*1000:.2f}mm
• Confidence score: {registration_result['uncertainty_metrics']['confidence_score']:.3f}
• Clinical ready: {'✅' if validation['overall_quality'] else '❌'}

Recommendations:
""" + '\n'.join([f"• {rec}" for rec in validation['recommendations'][:3]])
        
        ax6.text(0.05, 0.95, summary_text, transform=ax6.transAxes, 
                fontsize=10, verticalalignment='top', fontfamily='monospace')
        
        plt.tight_layout()
        
        # Save visualization
        output_dir = Path("trajectory_optimization/data/visualizations")
        output_dir.mkdir(parents=True, exist_ok=True)
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        plt.savefig(output_dir / f"landmark_registration_{timestamp}.png", 
                   dpi=300, bbox_inches='tight')
        plt.show()
    
    def export_registration_report(self, detection_results: List[Dict], 
                                 registration_result: Dict, 
                                 integration_result: Dict) -> None:
        """Export comprehensive registration report."""
        
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        output_dir = Path("trajectory_optimization/data/reports")
        output_dir.mkdir(parents=True, exist_ok=True)
        
        report_file = output_dir / f"landmark_registration_report_{timestamp}.md"
        
        with open(report_file, 'w') as f:
            f.write("# Landmark-Based Registration Report\n\n")
            f.write(f"Generated: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}\n\n")
            
            f.write("## Executive Summary\n\n")
            f.write("Force-based landmark registration for knee ultrasound trajectory planning.\n")
            f.write("Robot targets anatomical landmarks to establish model-to-reality correspondence.\n\n")
            
            # Detection results
            f.write("## Landmark Detection Results\n\n")
            successful = [r for r in detection_results if r['contact_detected']]
            f.write(f"- **Total landmarks targeted**: {len(detection_results)}\n")
            f.write(f"- **Successfully detected**: {len(successful)}\n")
            f.write(f"- **Detection rate**: {len(successful)/len(detection_results)*100:.1f}%\n\n")
            
            f.write("### Individual Landmark Results\n\n")
            for result in detection_results:
                status = "✅" if result['contact_detected'] else "❌"
                f.write(f"**{result['landmark_name']}** {status}\n")
                f.write(f"- Force: {result['measured_force']:.2f}N\n")
                if result['contact_detected']:
                    error = np.linalg.norm(result['contact_position'] - result['real_position']) * 1000
                    f.write(f"- Detection error: {error:.2f}mm\n")
                f.write(f"- Reliability: {result['landmark_info']['reliability']}\n\n")
            
            # Registration quality
            f.write("## Registration Quality\n\n")
            f.write(f"- **Registration error**: {registration_result['registration_error']*1000:.2f}mm\n")
            f.write(f"- **Total uncertainty**: {registration_result['uncertainty_metrics']['total_uncertainty']*1000:.2f}mm\n")
            f.write(f"- **Confidence score**: {registration_result['uncertainty_metrics']['confidence_score']:.3f}\n")
            
            validation = self.validate_registration_quality()
            f.write(f"- **Clinical ready**: {'Yes' if validation['overall_quality'] else 'No'}\n\n")
            
            # STOMP integration
            f.write("## STOMP Integration Parameters\n\n")
            adjustments = integration_result['stomp_parameter_adjustments']
            for param, value in adjustments.items():
                f.write(f"- **{param}**: {value:.4f}\n")
            f.write(f"\n- **Recommended safety clearance**: {integration_result['recommended_safety_clearance']*1000:.1f}mm\n\n")
            
            # Recommendations
            if validation['recommendations']:
                f.write("## Recommendations\n\n")
                for rec in validation['recommendations']:
                    f.write(f"- {rec}\n")
                f.write("\n")
            
            f.write("## Clinical Implications\n\n")
            f.write("- Registration quality directly impacts trajectory planning safety\n")
            f.write("- Force-based detection provides more reliable landmarks than visual methods\n")
            f.write("- Uncertainty estimates enable adaptive planning strategies\n")
            f.write("- Real-time registration updates possible during procedure\n")
        
        print(f"📄 Registration report exported to: {report_file}")
        
        # Export data as JSON
        json_file = output_dir / f"landmark_registration_data_{timestamp}.json"
        export_data = {
            'detection_results': detection_results,
            'registration_result': {k: v.tolist() if isinstance(v, np.ndarray) else v 
                                  for k, v in registration_result.items()},
            'integration_result': integration_result,
            'validation_result': validation
        }
        
        # Convert numpy arrays to lists for JSON serialization
        def convert_numpy(obj):
            if isinstance(obj, np.ndarray):
                return obj.tolist()
            elif isinstance(obj, np.integer):
                return int(obj)
            elif isinstance(obj, np.floating):
                return float(obj)
            elif isinstance(obj, dict):
                return {k: convert_numpy(v) for k, v in obj.items()}
            elif isinstance(obj, list):
                return [convert_numpy(v) for v in obj]
            return obj
        
        export_data = convert_numpy(export_data)
        
        with open(json_file, 'w') as f:
            json.dump(export_data, f, indent=2)
        
        print(f"📊 Registration data exported to: {json_file}")

def demo_landmark_registration():
    """Demonstrate the landmark-based registration system."""
    
    print("🏥 Knee Landmark Registration Demo")
    print("=" * 50)
    
    # Initialize registration system
    registrator = KneeLandmarkRegistration()
    
    # Execute landmark detection sequence
    detection_results = registrator.target_landmark_sequence()
    
    # Estimate registration transform
    registration_result = registrator.estimate_registration_transform(detection_results)
    
    # Validate registration quality
    validation = registrator.validate_registration_quality()
    
    # Integrate with STOMP planning
    integration_result = registrator.integrate_with_stomp_planning(registration_result)
    
    # Create visualizations
    registrator.visualize_registration_results(detection_results, registration_result)
    
    # Export comprehensive report
    registrator.export_registration_report(detection_results, registration_result, integration_result)
    
    print(f"\n✅ Registration Demo Complete!")
    print(f"Registration Error: {registration_result['registration_error']*1000:.2f}mm")
    print(f"Clinical Ready: {'Yes' if validation['overall_quality'] else 'No'}")
    
    return registrator, detection_results, registration_result

if __name__ == "__main__":
    demo_landmark_registration()
