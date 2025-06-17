#!/bin/bash

# =============================================================================
# STOMP vs HAUSER TRAJECTORY PLANNING COMPARISON RUNNER
# =============================================================================
# This script runs a comprehensive comparison between STOMP and Hauser trajectory
# planning algorithms using real kinematic metrics and statistical analysis.
#
# Features:
# - Automated compilation and execution
# - Real trajectory analysis with proper metrics
# - Statistical significance testing
# - Publication-quality visualizations
# - Comprehensive error handling
# =============================================================================

set -e  # Exit on any error

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Configuration
DEFAULT_POSES=10
DEFAULT_RUNS_PER_POSE=5
RESULTS_DIR="results"
PLOTS_DIR="plots"
BUILD_DIR="../../build"

# Function to print colored output
print_status() {
    echo -e "${GREEN}[INFO]${NC} $1"
}

print_warning() {
    echo -e "${YELLOW}[WARNING]${NC} $1"
}

print_error() {
    echo -e "${RED}[ERROR]${NC} $1"
}

print_header() {
    echo -e "${BLUE}$1${NC}"
}

# Function to check dependencies
check_dependencies() {
    print_status "Checking dependencies..."
    
    # Check for required tools
    local missing_deps=()
    
    if ! command -v cmake &> /dev/null; then
        missing_deps+=("cmake")
    fi
    
    if ! command -v make &> /dev/null; then
        missing_deps+=("make")
    fi
    
    if ! command -v python3 &> /dev/null; then
        missing_deps+=("python3")
    fi
    
    # Check Python packages
    if ! python3 -c "import pandas, numpy, matplotlib, seaborn, scipy" &> /dev/null; then
        print_warning "Some Python packages may be missing. Install with:"
        print_warning "pip3 install pandas numpy matplotlib seaborn scipy"
    fi
    
    if [ ${#missing_deps[@]} -ne 0 ]; then
        print_error "Missing dependencies: ${missing_deps[*]}"
        exit 1
    fi
    
    print_status "All dependencies satisfied ✓"
}

# Function to build the comparison executable
build_comparison() {
    print_status "Building comparison executable..."
    
    # Check if build directory exists
    if [ ! -d "$BUILD_DIR" ]; then
        print_status "Creating build directory..."
        mkdir -p "$BUILD_DIR"
        cd "$BUILD_DIR"
        cmake ..
        cd - > /dev/null
    fi
    
    # Build the project
    cd "$BUILD_DIR"
    
    # Get number of CPU cores (cross-platform)
    if command -v nproc >/dev/null 2>&1; then
        NCORES=$(nproc)
    elif command -v sysctl >/dev/null 2>&1; then
        NCORES=$(sysctl -n hw.ncpu)
    else
        NCORES=4  # fallback
    fi
    
    if make -j${NCORES} > build.log 2>&1; then
        print_status "Build successful ✓"
    else
        print_error "Build failed. Check build.log for details."
        tail -20 build.log
        exit 1
    fi
    cd - > /dev/null
    
    # Check if the executable exists
    EXEC_PATH="$BUILD_DIR/apps/ComparisonSTOMPHauser/STOMPHauserComparison"
    if [ ! -f "$EXEC_PATH" ]; then
        print_error "Executable not found: $EXEC_PATH"
        print_warning "Trying alternative build approach..."
        
        # Try building directly in the comparison directory
        if [ -f "stomp_hauser_comparison.cpp" ]; then
            print_status "Attempting direct compilation..."
            g++ -std=c++17 -O3 -I../../libs/TrajectoryLib/include -I../../libs/USLib/include \
                -I../../libs/GeometryLib/include -I../../libs/Hauser10/include \
                -I/usr/include/eigen3 -I/usr/include/qt5 \
                stomp_hauser_comparison.cpp \
                -L$BUILD_DIR/libs -lTrajectoryLib -lUSLib -lGeometryLib -lHauser10 \
                -lQt5Core -o STOMPHauserComparison
            EXEC_PATH="./STOMPHauserComparison"
        else
            print_error "Source file not found. Please ensure stomp_hauser_comparison.cpp exists."
            exit 1
        fi
    fi
}

# Function to create output directories
setup_directories() {
    print_status "Setting up output directories..."
    
    mkdir -p "$RESULTS_DIR"
    mkdir -p "$PLOTS_DIR"
    
    # Clean previous results if they exist
    if [ -f "$RESULTS_DIR/comparison_results.csv" ]; then
        print_warning "Previous results found. Backing up..."
        mv "$RESULTS_DIR/comparison_results.csv" "$RESULTS_DIR/comparison_results_backup_$(date +%Y%m%d_%H%M%S).csv"
    fi
}

# Function to run the trajectory comparison
run_comparison() {
    local num_poses=${1:-$DEFAULT_POSES}
    local runs_per_pose=${2:-$DEFAULT_RUNS_PER_POSE}
    local total_runs=$((num_poses * runs_per_pose * 2))  # 2 algorithms
    
    print_header "=============================================================="
    print_header "RUNNING STOMP vs HAUSER COMPARISON"
    print_header "=============================================================="
    print_status "Configuration:"
    print_status "  Poses: $num_poses"
    print_status "  Runs per pose: $runs_per_pose"
    print_status "  Total runs: $total_runs"
    print_status "  Results will be saved to: $RESULTS_DIR/"
    print_header "=============================================================="
    
    # Check if robot model and environment files exist
    if [ ! -f "../../res/scenario_1/panda_US.urdf" ]; then
        print_error "Robot model not found: ../../res/scenario_1/panda_US.urdf"
        exit 1
    fi
    
    if [ ! -f "../../res/scenario_1/obstacles.xml" ]; then
        print_error "Environment file not found: ../../res/scenario_1/obstacles.xml"
        exit 1
    fi
    
    # Run the comparison
    print_status "Starting trajectory comparison..."
    print_warning "This may take several minutes depending on configuration..."
    
    local start_time=$(date +%s)
    
    # Cross-platform timeout implementation
    if command -v timeout >/dev/null 2>&1; then
        # Linux timeout command
        if timeout 1800 "$EXEC_PATH" "$num_poses" "$runs_per_pose" > "$RESULTS_DIR/comparison_results.csv" 2> "$RESULTS_DIR/comparison_stderr.log"; then
            comparison_success=true
        else
            comparison_success=false
        fi
    elif command -v gtimeout >/dev/null 2>&1; then
        # macOS with coreutils installed (brew install coreutils)
        if gtimeout 1800 "$EXEC_PATH" "$num_poses" "$runs_per_pose" > "$RESULTS_DIR/comparison_results.csv" 2> "$RESULTS_DIR/comparison_stderr.log"; then
            comparison_success=true
        else
            comparison_success=false
        fi
    else
        # Fallback: run without timeout (macOS default)
        print_warning "No timeout command available, running without time limit..."
        if "$EXEC_PATH" "$num_poses" "$runs_per_pose" > "$RESULTS_DIR/comparison_results.csv" 2> "$RESULTS_DIR/comparison_stderr.log"; then
            comparison_success=true
        else
            comparison_success=false
        fi
    fi
    
    if [ "$comparison_success" = true ]; then
        local end_time=$(date +%s)
        local duration=$((end_time - start_time))
        print_status "Comparison completed successfully in ${duration}s ✓"
        
        # Check if results file has data
        local result_lines=$(wc -l < "$RESULTS_DIR/comparison_results.csv")
        if [ "$result_lines" -lt 2 ]; then
            print_error "Results file appears empty or invalid"
            cat "$RESULTS_DIR/comparison_stderr.log"
            exit 1
        fi
        
        print_status "Generated $((result_lines - 1)) result records"
        
    else
        print_error "Comparison failed or timed out (30 minutes)"
        if [ -f "$RESULTS_DIR/comparison_stderr.log" ]; then
            print_error "Error log:"
            tail -20 "$RESULTS_DIR/comparison_stderr.log"
        fi
        exit 1
    fi
}

# Function to run the analysis
run_analysis() {
    print_status "Running trajectory analysis..."
    
    # Check if Python script exists
    if [ ! -f "trajectory_analysis.py" ]; then
        print_error "Analysis script not found: trajectory_analysis.py"
        exit 1
    fi
    
    # Make sure the script is executable
    chmod +x trajectory_analysis.py
    
    # Create plots directory
    mkdir -p "$PLOTS_DIR"
    
    # Run analysis
    cd "$PLOTS_DIR"
    if python3 ../trajectory_analysis.py "../$RESULTS_DIR/comparison_results.csv" > analysis_output.txt 2>&1; then
        print_status "Analysis completed successfully ✓"
        cd - > /dev/null
        
        # Show key results
        if [ -f "$PLOTS_DIR/analysis_output.txt" ]; then
            print_status "Analysis summary:"
            grep -E "(Success Rate|RMS Jerk|Planning Time|✓|→)" "$PLOTS_DIR/analysis_output.txt" || true
        fi
        
    else
        print_error "Analysis failed"
        cd - > /dev/null
        if [ -f "$PLOTS_DIR/analysis_output.txt" ]; then
            tail -20 "$PLOTS_DIR/analysis_output.txt"
        fi
        exit 1
    fi
}

# Function to generate summary report
generate_report() {
    local report_file="$RESULTS_DIR/comparison_summary.md"
    
    print_status "Generating summary report..."
    
    cat > "$report_file" << EOF
# STOMP vs Hauser Trajectory Planning Comparison Report

**Generated:** $(date)
**Configuration:** $1 poses, $2 runs per pose

## Files Generated

### Data Files
- \`comparison_results.csv\` - Raw trajectory metrics data
- \`comparison_stderr.log\` - Execution log

### Visualizations
- \`comprehensive_performance_dashboard.png/pdf\` - Main comparison dashboard
- \`detailed_kinematic_analysis.png/pdf\` - Detailed kinematic analysis

### Analysis Output
- \`analysis_output.txt\` - Complete statistical analysis results

## Key Metrics Analyzed

### Real Trajectory Metrics
- **Kinematic Analysis**: Velocity, acceleration, jerk (calculated via numerical differentiation)
- **Smoothness**: RMS jerk, smoothness score, motion consistency
- **Safety**: Minimum clearance, collision detection, safety scores
- **Performance**: Planning time, success rate, path length

### Statistical Tests
- T-tests for parametric comparisons
- Mann-Whitney U tests for non-parametric comparisons
- Effect size calculations (Cohen's d)
- Significance testing with multiple comparisons

## Algorithm Characteristics

### STOMP (Stochastic Trajectory Optimization)
- Uses noise sampling and iterative optimization
- Typically faster planning times
- May produce more variable motion profiles
- Good for finding feasible solutions quickly

### Hauser (Parabolic Ramp Planning)
- Uses parabolic velocity profiles for smooth motion
- Inherently smooth due to continuous acceleration
- May take longer due to path planning + smoothing
- Generally produces lower jerk trajectories

## Usage Notes

This comparison uses **real trajectory analysis** with:
1. Actual trajectory file parsing
2. Numerical differentiation for kinematic calculations
3. Geometric collision checking for clearance metrics
4. Statistical significance testing for meaningful comparisons

For questions or issues, refer to the implementation in:
- \`stomp_hauser_comparison.cpp\`
- \`trajectory_analysis.py\`
EOF

    print_status "Report saved to: $report_file"
}

# Function to clean up temporary files
cleanup() {
    print_status "Cleaning up temporary files..."
    
    # Remove any temporary trajectory files
    rm -f /tmp/stomp_traj_*.csv
    rm -f /tmp/hauser_traj_*.csv
    
    print_status "Cleanup completed ✓"
}

# Function to show usage
show_usage() {
    echo "Usage: $0 [num_poses] [runs_per_pose]"
    echo ""
    echo "Arguments:"
    echo "  num_poses       Number of different poses to test (default: $DEFAULT_POSES)"
    echo "  runs_per_pose   Number of runs per pose (default: $DEFAULT_RUNS_PER_POSE)"
    echo ""
    echo "Example:"
    echo "  $0 15 8    # Test 15 poses with 8 runs each (240 total runs)"
    echo ""
    echo "Output:"
    echo "  Results saved to: $RESULTS_DIR/"
    echo "  Plots saved to: $PLOTS_DIR/"
    echo ""
}

# Main execution
main() {
    # Parse arguments
    local num_poses=${1:-$DEFAULT_POSES}
    local runs_per_pose=${2:-$DEFAULT_RUNS_PER_POSE}
    
    # Validate arguments
    if ! [[ "$num_poses" =~ ^[0-9]+$ ]] || [ "$num_poses" -lt 1 ]; then
        print_error "Invalid number of poses: $num_poses"
        show_usage
        exit 1
    fi
    
    if ! [[ "$runs_per_pose" =~ ^[0-9]+$ ]] || [ "$runs_per_pose" -lt 1 ]; then
        print_error "Invalid runs per pose: $runs_per_pose"
        show_usage
        exit 1
    fi
    
    # Warn for large configurations
    local total_runs=$((num_poses * runs_per_pose * 2))
    if [ "$total_runs" -gt 200 ]; then
        print_warning "Large configuration detected ($total_runs total runs)"
        print_warning "This may take a very long time. Continue? (y/N)"
        read -r response
        if [[ ! "$response" =~ ^[Yy]$ ]]; then
            print_status "Aborted by user"
            exit 0
        fi
    fi
    
    print_header "=============================================================="
    print_header "STOMP vs HAUSER TRAJECTORY COMPARISON SUITE"
    print_header "Real Metrics • Statistical Analysis • Publication Plots"
    print_header "=============================================================="
    
    # Execute pipeline
    check_dependencies
    build_comparison
    setup_directories
    run_comparison "$num_poses" "$runs_per_pose"
    run_analysis
    generate_report "$num_poses" "$runs_per_pose"
    cleanup
    
    print_header "=============================================================="
    print_header "COMPARISON COMPLETED SUCCESSFULLY!"
    print_header "=============================================================="
    print_status "Results available in:"
    print_status "  📊 Data: $RESULTS_DIR/comparison_results.csv"
    print_status "  📈 Plots: $PLOTS_DIR/"
    print_status "  📝 Report: $RESULTS_DIR/comparison_summary.md"
    print_header "=============================================================="
}

# Handle script arguments
if [[ "${BASH_SOURCE[0]}" == "${0}" ]]; then
    if [[ "$1" == "-h" ]] || [[ "$1" == "--help" ]]; then
        show_usage
        exit 0
    fi
    
    main "$@"
fi