# Parameter Sensitivity Diagnostic Report

## Executive Summary

This report analyzes the parameter sensitivity of trajectory planning algorithms
to determine why parameter variations appear to have minimal impact on optimization objectives.

## Test Results Summary

### Stomp Extreme

- **Objective Range**: 0.00004562
- **Coefficient of Variation**: 0.00003345
- **Mean Objective**: 0.500095
- **Standard Deviation**: 0.00001673

### Hauser Extreme

- **Objective Range**: 0.00000134
- **Coefficient of Variation**: 0.00000134
- **Mean Objective**: 0.500082
- **Standard Deviation**: 0.00000067

### Stomp Exploration Constant Sweep

- **Objective Range**: 0.00000000
- **Coefficient of Variation**: 0.00000000
- **Mean Objective**: 0.000000
- **Standard Deviation**: 0.00000000

### Stomp Num Noisy Trajectories Sweep

- **Objective Range**: 0.00000000
- **Coefficient of Variation**: 0.00000000
- **Mean Objective**: 0.000000
- **Standard Deviation**: 0.00000000

### Stomp Learning Rate Sweep

- **Objective Range**: 0.00000000
- **Coefficient of Variation**: 0.00000000
- **Mean Objective**: 0.000000
- **Standard Deviation**: 0.00000000

### Hauser Max Deviation Sweep

- **Objective Range**: 0.00000000
- **Coefficient of Variation**: 0.00000000
- **Mean Objective**: 0.000000
- **Standard Deviation**: 0.00000000

### Hauser Time Step Sweep

- **Objective Range**: 0.00000000
- **Coefficient of Variation**: 0.00000000
- **Mean Objective**: 0.000000
- **Standard Deviation**: 0.00000000

### Hauser Tolerance Sweep

- **Objective Range**: 0.00000000
- **Coefficient of Variation**: 0.00000000
- **Mean Objective**: 0.000000
- **Standard Deviation**: 0.00000000

## Diagnostic Conclusions

### Potential Issues Identified:

1. **Low Parameter Sensitivity**: If objective ranges are < 0.001, parameters have minimal impact
2. **Metric Saturation**: If success rates are consistently 0 or planning always times out
3. **Objective Function Issues**: If individual metrics don't vary meaningfully
4. **C++ Evaluator Problems**: If raw outputs show unexpected patterns

### Recommendations:

1. **If success rates are consistently 0**: 
   - Scenarios may be too difficult or impossible
   - Robot kinematics or obstacle constraints may be preventing solutions
   - Consider simpler test scenarios

2. **If planning times are consistently at timeout**:
   - Reduce scenario complexity
   - Increase timeout values
   - Check for fundamental planning issues

3. **If objective variations are minimal but metrics vary**:
   - Objective function weights may need adjustment
   - Consider different metric combinations
   - Evaluate metric scaling

4. **If no variation in any metrics**:
   - C++ evaluator may not be properly using parameters
   - Algorithm implementations may ignore certain parameters
   - Configuration parsing may have issues

### Next Steps:

1. Review this diagnostic report and identify the primary issue
2. Implement targeted fixes based on findings
3. Re-run parameter optimization with corrected system
4. Validate that parameters now have meaningful impact

---
*Generated automatically by Parameter Sensitivity Diagnostic Tool*
