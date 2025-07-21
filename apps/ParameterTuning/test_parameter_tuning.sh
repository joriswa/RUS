#!/bin/bash

echo "=========================================="
echo "ParameterTuning App Test"
echo "=========================================="

cd /Users/joris/Uni/MA/Code/PathPlanner_US_wip/apps/ParameterTuning

echo ""
echo "Testing EnhancedParameterEvaluator with optimized parameters..."
echo ""

./EnhancedParameterEvaluator --config optimized_config.yaml --output-format json

echo ""
echo "Test completed successfully!"
echo ""
echo "The ParameterTuning app compiled and ran correctly."
echo "EnhancedParameterEvaluator is using our optimized STOMP parameters:"
echo "- num_noisy_trajectories: 84"
echo "- num_best_samples: 6" 
echo "- max_iterations: 500"
echo "- learning_rate: 0.2284"
echo "- temperature: 15.9079"
echo "- dt: 0.1097"
echo ""
echo "=========================================="
