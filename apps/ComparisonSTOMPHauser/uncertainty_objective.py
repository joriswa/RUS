#!/usr/bin/env python3
"""
Simple uncertainty-aware objective function modification.
Adds robustness penalty to existing STOMP optimization.
"""

import numpy as np

class UncertaintyAwareObjective:
    def __init__(self, base_objective_function, uncertainty_radius=0.02):
        """
        Wrap existing objective function with uncertainty awareness.
        
        Args:
            base_objective_function: Your existing STOMP objective function
            uncertainty_radius: Registration uncertainty in meters (2cm = 0.02m)
        """
        self.base_objective = base_objective_function
        self.uncertainty_radius = uncertainty_radius
        
    def __call__(self, trajectory, *args, **kwargs):
        """
        Evaluate trajectory under uncertainty.
        Simple approach: sample a few offset positions and average the cost.
        """
        base_cost = self.base_objective(trajectory, *args, **kwargs)
        
        # Simple uncertainty sampling - just check a few offset positions
        uncertainty_costs = []
        n_samples = 5  # Keep it simple
        
        for _ in range(n_samples):
            # Generate random offset within uncertainty radius
            offset = self.generate_random_offset()
            
            # Evaluate objective with offset (simplified - you'd apply to your model)
            # For now, just add some noise based on offset magnitude
            offset_magnitude = np.linalg.norm(offset)
            uncertainty_penalty = offset_magnitude / self.uncertainty_radius * 0.1
            
            perturbed_cost = base_cost + uncertainty_penalty
            uncertainty_costs.append(perturbed_cost)
        
        # Return average cost (or worst-case, or weighted average)
        robust_cost = base_cost + np.mean(uncertainty_costs) * 0.2  # Weight uncertainty
        
        return robust_cost
    
    def generate_random_offset(self):
        """Generate random 3D offset within uncertainty radius."""
        # Simple uniform sampling in sphere
        direction = np.random.randn(3)
        direction = direction / np.linalg.norm(direction)
        magnitude = np.random.uniform(0, self.uncertainty_radius)
        return direction * magnitude

# Example integration with existing STOMP
def integrate_with_existing_stomp():
    """Example of how to integrate with your existing code."""
    
    # Your existing objective function (placeholder)
    def my_existing_objective(trajectory):
        # Your current STOMP objective calculation
        planning_time_cost = 1.0  # placeholder
        success_rate_cost = 0.5   # placeholder
        safety_cost = 0.3         # placeholder
        return planning_time_cost + success_rate_cost + safety_cost
    
    # Wrap with uncertainty awareness
    uncertain_objective = UncertaintyAwareObjective(
        my_existing_objective, 
        uncertainty_radius=0.02  # 2cm
    )
    
    # Use in your existing STOMP optimization
    test_trajectory = np.random.randn(10, 3)  # Placeholder trajectory
    
    original_cost = my_existing_objective(test_trajectory)
    robust_cost = uncertain_objective(test_trajectory)
    
    print(f"Original cost: {original_cost:.4f}")
    print(f"Uncertainty-aware cost: {robust_cost:.4f}")
    print(f"Robustness penalty: {robust_cost - original_cost:.4f}")

if __name__ == "__main__":
    integrate_with_existing_stomp()
