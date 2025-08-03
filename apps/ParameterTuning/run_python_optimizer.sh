#!/bin/bash
# Script to run the Python comprehensive STOMP optimizer with correct paths

# Change to the correct directory
cd /Users/joris/Uni/MA/Code/PathPlanner_US_wip/apps/ParameterTuning

# Default parameters
TRIALS=50
TRAJECTORY_PAIRS=20
TIMEOUT=360

# Parse command line arguments
while [[ $# -gt 0 ]]; do
    case $1 in
        --trials)
            TRIALS="$2"
            shift 2
            ;;
        --trajectory-pairs)
            TRAJECTORY_PAIRS="$2"
            shift 2
            ;;
        --timeout)
            TIMEOUT="$2"
            shift 2
            ;;
        --help|-h)
            echo "Usage: $0 [OPTIONS]"
            echo "Options:"
            echo "  --trials N               Number of optimization trials (default: 50)"
            echo "  --trajectory-pairs N     Trajectory pairs per evaluation (default: 20)"
            echo "  --timeout N              Timeout per trial in seconds (default: 360)"
            echo "  --help, -h               Show this help message"
            echo ""
            echo "Examples:"
            echo "  $0                                    # Use default parameters"
            echo "  $0 --trials 100 --trajectory-pairs 10"
            echo "  $0 --trials 25 --timeout 300"
            exit 0
            ;;
        *)
            echo "Unknown option: $1"
            echo "Use --help for usage information"
            exit 1
            ;;
    esac
done

# Check if Python script exists
PYTHON_SCRIPT="comprehensive_stomp_optimizer.py"
if [ ! -f "$PYTHON_SCRIPT" ]; then
    echo "Error: Python script '$PYTHON_SCRIPT' not found"
    exit 1
fi

# Check if test_evaluator_simple.py exists (dependency)
if [ ! -f "test_evaluator_simple.py" ]; then
    echo "Error: Dependency 'test_evaluator_simple.py' not found"
    exit 1
fi

# Check if parameter_evaluator executable exists
EVALUATOR_PATH="../../build/apps/ParameterTuning/parameter_evaluator"
if [ ! -f "$EVALUATOR_PATH" ]; then
    echo "Error: parameter_evaluator executable not found at '$EVALUATOR_PATH'"
    echo "Make sure to build the project first with CMake"
    exit 1
fi

echo "🎯 Starting Comprehensive Multi-Objective STOMP Optimization..."
echo "📁 Working directory: $(pwd)"
echo "🐍 Python script: $PYTHON_SCRIPT"
echo "🔧 C++ Evaluator: $EVALUATOR_PATH"
echo "📊 Parameters:"
echo "   - Trials: $TRIALS"
echo "   - Trajectory pairs: $TRAJECTORY_PAIRS"
echo "   - Timeout: ${TIMEOUT}s"
echo ""
echo "📋 Methodology: NSGAII (methodologically sound, no problematic weighted sums)"
echo "🔧 Fixed learning_rate = 1.0, optimizing all other parameters including cost weights"
echo ""

# Run the optimizer
python3 "$PYTHON_SCRIPT" --trials "$TRIALS" --trajectory-pairs "$TRAJECTORY_PAIRS" --timeout "$TIMEOUT"

RESULT=$?

if [ $RESULT -eq 0 ]; then
    echo ""
    echo "✅ Comprehensive optimization completed successfully!"
    echo "📊 Check 'comprehensive_optimization_results/' for detailed analysis"
    echo "📈 Use generated plots to justify parameter selection based on your priorities"
else
    echo ""
    echo "❌ Optimization failed with exit code: $RESULT"
    exit $RESULT
fi
