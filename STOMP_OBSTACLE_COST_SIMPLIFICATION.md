# STOMP Obstacle Cost Calculator Simplification

## Overview
Simplified the obstacle cost calculator to follow the original STOMP paper approach with proper sphere velocity scaling.

## Key Changes Made

### 1. **Simplified Cost Calculation Structure**
- **Before**: Complex per-timestep cost accumulation with collision penalties
- **After**: Clean interval-based cost calculation (N-1 intervals instead of N timesteps)

### 2. **Added Missing Sphere Velocity Scaling**
```cpp
// Calculate link velocity between timesteps
Eigen::Vector3d linkVelocity = (nextCenter - currentCenter) / dt;

// Position sphere with velocity scaling (missing in previous implementation)
Eigen::Vector3d sphereCenter = currentCenter + velocityScaling * linkVelocity * dt;
```

### 3. **Simplified Cost Function**
- **Before**: Complex penetration calculations with normalization
- **After**: Simple quadratic cost as in original STOMP paper
```cpp
// Simple quadratic cost 
double penetration = clearanceRadius - surfaceDistance;
cost += penetration * penetration;
```

### 4. **Removed Unnecessary Complexity**
- Removed collision penalty logic (commented out code)
- Removed complex cost capping and averaging
- Removed normalization and violation ratios
- Simplified to core STOMP paper formulation

### 5. **Updated Parameters**
- **clearanceRadius**: Increased from 3cm to 5cm for better safety margins
- **velocityScaling**: Added 0.1 factor for sphere positioning prediction

## Benefits

1. **Follows Original Paper**: Now matches the STOMP paper's sphere-based obstacle avoidance
2. **Proper Motion Prediction**: Velocity scaling accounts for robot movement during trajectory execution
3. **Simpler and Faster**: Reduced computational complexity while maintaining effectiveness
4. **Better Tuning**: Simplified parameters are easier to understand and tune

## Technical Details

### Sphere Positioning Formula
```
sphereCenter = currentCenter + velocityScaling * linkVelocity * dt
```

Where:
- `currentCenter`: Current link bounding box center
- `linkVelocity`: Rate of change of link position
- `velocityScaling`: Scaling factor (0.1) to predict future position
- `dt`: Time step between trajectory points

### Cost Calculation
```
if (surfaceDistance < clearanceRadius) {
    penetration = clearanceRadius - surfaceDistance
    cost += penetration²
}
```

This provides the quadratic cost increase as obstacles are approached, encouraging smooth avoidance.

## Result
The obstacle cost calculator now properly implements the original STOMP paper's approach with sphere velocity scaling, providing more accurate and simpler obstacle avoidance behavior.
