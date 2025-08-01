#!/bin/bash

# Success Rate Optimization Test Script
# Tests different cost weight configurations to improve success rate

echo "=== SUCCESS RATE OPTIMIZATION TEST ==="
echo "Testing configurable cost weights for improved success rate"
echo ""

CONFIG_FILE="success_rate_config.yaml"
TEST_RESULTS="success_rate_test_results.json"
BASELINE_CONFIG="optimized_heavy_penalty_config.yaml"

echo "Configuration 1: High Collision Penalty (Current Optimized)"
echo "-----------------------------------------------------------"
time ./build/apps/ParameterTuning/parameter_evaluator $BASELINE_CONFIG > baseline_success_test.log 2>&1

echo ""
echo "Configuration 2: Success Rate Priority (Relaxed Constraints)"
echo "------------------------------------------------------------"

# Create success rate optimized config
cat > $CONFIG_FILE << EOF
# STOMP Success Rate Optimization Config
numNoisyTrajectories: 15
numBestSamples: 10
maxIterations: 200
N: 40
learningRate: 0.7
temperature: 12.0

# Cost weights optimized for success rate
obstacleCostWeight: 8.0
smoothnessCostWeight: 0.3
velocityCostWeight: 0.1
accelerationCostWeight: 0.1
goalCostWeight: 5.0
collisionPenaltyWeight: 80.0
prioritizeSuccessRate: true
constraintRelaxationFactor: 1.8

# Early stopping for efficiency
enableEarlyStopping: true
earlyStoppingPatience: 1
maxComputeTimeMs: 4000.0

# Optimized joint noise
jointStdDevs: [0.149691, 0.0132654, 0.0450644, 0.103875, 0.0647333, 0.131444, 0.113796]
EOF

time ./build/apps/ParameterTuning/parameter_evaluator $CONFIG_FILE > success_rate_test.log 2>&1

echo ""
echo "Configuration 3: Maximum Success Rate (Very Relaxed)"
echo "---------------------------------------------------"

# Create maximum success rate config
cat > max_success_config.yaml << EOF
# STOMP Max Success Rate Config (Quality sacrificed for success)
numNoisyTrajectories: 20
numBestSamples: 12
maxIterations: 150
N: 35
learningRate: 0.6
temperature: 15.0

# Extreme success rate optimization
obstacleCostWeight: 12.0
smoothnessCostWeight: 0.1
velocityCostWeight: 0.05
accelerationCostWeight: 0.05
goalCostWeight: 8.0
collisionPenaltyWeight: 120.0
prioritizeSuccessRate: true
constraintRelaxationFactor: 2.5

# Aggressive early stopping
enableEarlyStopping: true
earlyStoppingPatience: 1
maxComputeTimeMs: 3000.0

# Optimized joint noise
jointStdDevs: [0.149691, 0.0132654, 0.0450644, 0.103875, 0.0647333, 0.131444, 0.113796]
EOF

time ./build/apps/ParameterTuning/parameter_evaluator max_success_config.yaml > max_success_test.log 2>&1

echo ""
echo "=== RESULTS SUMMARY ==="
echo ""

echo "Baseline (Optimized Quality):"
if [ -f baseline_success_test.log ]; then
    grep -E "(Success rate|Planning time|Failed|Successful)" baseline_success_test.log | tail -5
fi

echo ""
echo "Success Rate Priority:"
if [ -f success_rate_test.log ]; then
    grep -E "(Success rate|Planning time|Failed|Successful)" success_rate_test.log | tail -5
fi

echo ""
echo "Maximum Success Rate:"
if [ -f max_success_test.log ]; then
    grep -E "(Success rate|Planning time|Failed|Successful)" max_success_test.log | tail -5
fi

echo ""
echo "Log files created:"
ls -la *success*.log *success*.yaml 2>/dev/null || echo "No log files found"
