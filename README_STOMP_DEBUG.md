# STOMP Environment Debug Tool

This tool allows you to debug STOMP trajectory optimization with realistic obstacle environments using the same infrastructure as the main USTrajectoryPlanner.

## Features

- **Real Environment Loading**: Uses URDF/XML files to load obstacle environments
- **Multiple Test Scenarios**: Pre-defined start/goal configurations for different difficulty levels
- **Comprehensive Debug Output**: Detailed STOMP iteration logging and analysis
- **Two Testing Modes**: Direct STOMP testing and full USTrajectoryPlanner integration
- **Configurable Parameters**: Multiple scenario configurations (fast, thorough, exploration, minimal)

## Building

1. Make sure you have the main project built first
2. Run the build script:
   ```bash
   ./build_debug.sh
   ```

## Usage

```bash
./debug_stomp_with_environment [environment_file.xml] [scenario]
```

### Parameters

- **environment_file.xml**: Path to URDF/XML environment file
- **scenario**: Testing configuration (fast, thorough, exploration, minimal)

### Scenarios

- **fast**: Quick testing (8 samples, 30 iterations, 15s timeout)
- **thorough**: Balanced testing (16 samples, 100 iterations, 60s timeout) 
- **exploration**: Difficult problems (24 samples, 150 iterations, 120s timeout)
- **minimal**: Minimal testing (4 samples, 20 iterations, 10s timeout)

### Examples

```bash
# Quick test with simple environment
./debug_stomp_with_environment debug_environment_simple.xml fast

# Thorough test with complex environment  
./debug_stomp_with_environment debug_environment_complex.xml thorough

# Exploration test for difficult problems
./debug_stomp_with_environment debug_environment_complex.xml exploration
```

## Sample Environments

Two sample environment files are provided:

### debug_environment_simple.xml
- Simple table and wall obstacles
- Good for basic STOMP functionality testing
- Recommended for initial debugging

### debug_environment_complex.xml  
- Multiple obstacles including tables, equipment racks, pillars
- Narrow passages and overhead constraints
- Challenging scenarios for thorough testing

## What It Tests

### 1. Environment Analysis
- Loads and validates URDF environment
- Analyzes obstacle tree structure

### 2. Start/Goal Validation
- Checks if start and goal positions are collision-free
- Warns about problematic configurations

### 3. Direct STOMP Testing
- Runs STOMP directly on motion generator
- Provides detailed debug output for each iteration
- Shows cost breakdowns, collision analysis, constraint violations

### 4. USTrajectoryPlanner Integration
- Tests full pipeline using USTrajectoryPlanner
- Validates integration with existing system
- Compares results between approaches

## Debug Output

The tool provides comprehensive debug information:

- **Configuration Analysis**: STOMP parameters and settings
- **Iteration Details**: Cost statistics, collision-free samples, trajectory changes
- **Cost Breakdown**: Obstacle costs vs constraint costs with violation details
- **Collision Analysis**: Specific timesteps and joint angles where collisions occur
- **Convergence Tracking**: Early stopping, cost convergence, time limits

## Output Files

- **debug_stomp_environment_trajectory.csv**: Generated trajectory data
- **Console Output**: Detailed debug information and analysis

## Troubleshooting

### Common Issues

1. **Environment file not found**: Ensure URDF/XML file exists and path is correct
2. **Build failures**: Make sure main project is built first
3. **Start/goal collisions**: Check debug output for collision warnings
4. **STOMP timeouts**: Try 'fast' scenario or increase timeout in code

### Debug Tips

1. Start with simple environments and 'fast' scenario
2. Check start/goal collision status first
3. Look for patterns in cost breakdown (obstacle vs constraint dominant)
4. Monitor collision-free sample generation rates
5. Adjust STOMP parameters based on debug output

## Customization

You can customize the tool by:

1. **Modifying scenarios**: Edit `createDebugScenarios()` function
2. **Adding new configurations**: Extend `createScenarioConfig()` function  
3. **Creating custom environments**: Write new URDF/XML files
4. **Adjusting analysis**: Modify analysis functions for specific metrics

## Integration with Main System

This debug tool uses the exact same infrastructure as the main USTrajectoryPlanner:

- Same environment parsing (`RobotManager.parseURDF()`)
- Same obstacle handling (`BVHTree` from `getTransformedObstacles()`)
- Same STOMP implementation with your recent debug enhancements
- Same motion generation pipeline

This ensures that debugging results directly translate to understanding main system behavior.
