# STOMP Collision Checking Implementation

## Overview
Added actual collision detection with flat penalty to the obstacle cost calculator, providing both distance-based smooth costs and discrete collision penalties.

## Implementation Details

### **Added Collision Detection Logic**
```cpp
// Check for actual collision at current timestep
bool hasCollision = false;
auto currentBoxes = currentArm.getCollisionBoxes();
int collisionCheckIndex = 0;
for (const auto &bbox : currentBoxes) {
    ++collisionCheckIndex;
    if (collisionCheckIndex < 2) // Skip base links as in armHasCollision
        continue;
    
    auto [bboxCenter, bboxHalfDims, bboxAxes] = bbox;
    if (_obstacleTree->isBoxIntersecting(bboxCenter, bboxHalfDims, bboxAxes)) {
        hasCollision = true;
        break;
    }
}

// Add flat collision penalty if any collision detected
if (hasCollision) {
    cost += collisionPenalty;
}
```

### **Key Features**

1. **Actual BVH Collision Detection**: Uses the same collision checking logic as `MotionGenerator::armHasCollision`
2. **Flat Penalty System**: Adds a fixed cost (`collisionPenalty = 100.0`) when any collision is detected
3. **Per-Timestep Checking**: Checks each timestep in the trajectory for collisions
4. **Base Link Exclusion**: Skips the first two base links to match existing collision checking behavior
5. **Early Termination**: Breaks out of loop as soon as any collision is found for efficiency

### **Cost Function Structure**

The obstacle cost now has two components:

1. **Distance-Based Cost**: Smooth cost that increases as robot approaches obstacles (using SDF)
   ```cpp
   double penetration = clearanceRadius - surfaceDistance;
   cost += std::max(0.0, penetration) * linkVelocity.norm();
   ```

2. **Collision Penalty**: Flat fee added when actual collision occurs
   ```cpp
   if (hasCollision) {
       cost += collisionPenalty; // 100.0
   }
   ```

### **Parameters**

- **`clearanceRadius`**: 0.01m (1cm) - threshold for distance-based cost
- **`collisionPenalty`**: 100.0 - flat fee for actual collisions
- **Link exclusion**: Skips first 2 base links (same as `armHasCollision`)

## Benefits

1. **Strong Collision Avoidance**: High flat penalty strongly discourages collision trajectories
2. **Smooth Guidance**: Distance-based cost provides smooth gradients away from obstacles
3. **Computational Efficiency**: Early termination when collision detected
4. **Consistency**: Uses same collision logic as existing validation functions
5. **Balanced Cost**: Combines smooth and discrete penalties for effective optimization

## Integration

- **Distance cost**: Guides robot away from obstacles smoothly
- **Collision penalty**: Provides strong signal against infeasible trajectories
- **Velocity scaling**: Accounts for link movement speed in distance cost
- **BVH tree**: Leverages existing fast collision detection infrastructure

## Result

The obstacle cost calculator now provides both smooth distance-based guidance and strong discrete collision penalties, leading to more effective obstacle avoidance during STOMP optimization while maintaining computational efficiency.
