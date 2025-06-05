#!/bin/bash
# 
# Automated workflow script for running scan process tests and generating plots
# This script automates the complete testing and analysis pipeline.
#
# Usage:
#   ./run_scan_analysis.sh [options]
#
# Options:
#   --build-only     Only build the test application
#   --test-only      Only run tests (skip building)
#   --plot-only      Only generate plots from existing CSV
#   --single         Run single test instead of modular tests
#   --quiet          Reduce output verbosity
#   --help           Show this help message

set -e  # Exit on any error

# Default settings
BUILD_DIR="../../../build"
APP_DIR="apps/ScanProcessTest"
VENV_DIR="plot_env"
BUILD_ONLY=false
TEST_ONLY=false
PLOT_ONLY=false
RUN_SINGLE=false
QUIET_MODE=false

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Function to print colored output
print_status() {
    echo -e "${GREEN}[INFO]${NC} $1"
}

print_warning() {
    echo -e "${YELLOW}[WARN]${NC} $1"
}

print_error() {
    echo -e "${RED}[ERROR]${NC} $1"
}

print_header() {
    echo -e "${BLUE}=== $1 ===${NC}"
}

# Function to show help
show_help() {
    cat << EOF
Scan Process Test Analysis Workflow

This script automates the complete pipeline for running scan process tests
and generating comprehensive analysis plots optimized for printing.

USAGE:
    $0 [OPTIONS]

OPTIONS:
    --build-only        Only build the test application and exit
    --test-only         Only run tests, skip building and plotting
    --plot-only         Only generate plots from existing CSV files
    --single            Run single test instead of modular test suite
    --quiet             Reduce output verbosity for tests
    --help              Show this help message and exit

WORKFLOW:
    1. Build the ScanProcessTest application (unless --test-only or --plot-only)
    2. Run the scan process tests (unless --build-only or --plot-only)
    3. Set up Python virtual environment if needed
    4. Generate comprehensive analysis plots (unless --build-only or --test-only)

OUTPUT:
    - CSV results in build/apps/ScanProcessTest/
    - Analysis plots in build/apps/ScanProcessTest/plots/

EXAMPLES:
    $0                      # Run complete workflow
    $0 --single             # Run single test and generate plots
    $0 --build-only         # Only build the application
    $0 --plot-only          # Only generate plots from existing data
    $0 --quiet              # Run with reduced output

REQUIREMENTS:
    - CMake and build tools for C++ compilation
    - Python 3 with pip for plotting
    - Virtual environment support (python3 -m venv)

For more information, see README.md
EOF
}

# Parse command line arguments
while [[ $# -gt 0 ]]; do
    case $1 in
        --build-only)
            BUILD_ONLY=true
            shift
            ;;
        --test-only)
            TEST_ONLY=true
            shift
            ;;
        --plot-only)
            PLOT_ONLY=true
            shift
            ;;
        --single)
            RUN_SINGLE=true
            shift
            ;;
        --quiet)
            QUIET_MODE=true
            shift
            ;;
        --help|-h)
            show_help
            exit 0
            ;;
        *)
            print_error "Unknown option: $1"
            print_status "Use --help for usage information"
            exit 1
            ;;
    esac
done

# Function to check if command exists
command_exists() {
    command -v "$1" >/dev/null 2>&1
}

# Function to build the application
build_application() {
    print_header "Building Scan Process Test Application"
    
    # Check if build directory exists
    if [ ! -d "$BUILD_DIR" ]; then
        print_error "Build directory not found: $BUILD_DIR"
        print_status "Please run cmake configuration first"
        exit 1
    fi
    
    # Navigate to build directory and build
    cd "$BUILD_DIR"
    
    print_status "Building ScanProcessTest target..."
    if make ScanProcessTest; then
        print_status "✓ Build completed successfully"
    else
        print_error "Build failed"
        exit 1
    fi
    
    # Return to original directory
    cd - > /dev/null
}

# Function to run tests
run_tests() {
    print_header "Running Scan Process Tests"
    
    # Check if executable exists
    EXECUTABLE="$BUILD_DIR/$APP_DIR/ScanProcessTest"
    if [ ! -f "$EXECUTABLE" ]; then
        print_error "Test executable not found: $EXECUTABLE"
        print_status "Try running with --build-only first or without --test-only"
        exit 1
    fi
    
    # Prepare test arguments
    TEST_ARGS=""
    if [ "$RUN_SINGLE" = true ]; then
        TEST_ARGS="--single"
        print_status "Running single test..."
    else
        print_status "Running modular test suite..."
    fi
    
    if [ "$QUIET_MODE" = true ]; then
        TEST_ARGS="$TEST_ARGS --quiet"
    fi
    
    # Run the tests
    cd "$BUILD_DIR/$APP_DIR"
    if ./ScanProcessTest $TEST_ARGS; then
        print_status "✓ Tests completed successfully"
        
        # List generated CSV files
        CSV_FILES=$(ls scan_test_results_*.csv 2>/dev/null || true)
        if [ -n "$CSV_FILES" ]; then
            print_status "Generated CSV files:"
            for file in $CSV_FILES; do
                echo "    - $file"
            done
        fi
    else
        print_error "Tests failed"
        exit 1
    fi
    
    # Return to original directory
    cd - > /dev/null
}

# Function to setup Python environment
setup_python_environment() {
    print_header "Setting up Python Environment"
    
    # Check if Python 3 is available
    if ! command_exists python3; then
        print_error "Python 3 is not available"
        print_status "Please install Python 3 to generate plots"
        exit 1
    fi
    
    print_status "Python 3 version: $(python3 --version)"
    
    # Create virtual environment if it doesn't exist
    if [ ! -d "$VENV_DIR" ]; then
        print_status "Creating Python virtual environment..."
        python3 -m venv "$VENV_DIR"
    else
        print_status "Using existing virtual environment"
    fi
    
    # Activate virtual environment and install packages
    source "$VENV_DIR/bin/activate"
    
    print_status "Installing/updating required packages..."
    pip install --upgrade pip --quiet
    pip install pandas numpy matplotlib seaborn --quiet
    
    print_status "✓ Python environment ready"
}

# Function to generate plots
generate_plots() {
    print_header "Generating Analysis Plots"
    
    # Find the most recent CSV file in build directory
    CSV_FILE=""
    BUILD_CSV_DIR="$BUILD_DIR/$APP_DIR"
    
    if [ -d "$BUILD_CSV_DIR" ]; then
        # Find most recent CSV file
        CSV_FILE=$(find "$BUILD_CSV_DIR" -name "scan_test_results_*.csv" -type f -exec ls -t {} + | head -n1 2>/dev/null || true)
    fi
    
    if [ -z "$CSV_FILE" ]; then
        print_error "No CSV results file found"
        print_status "Please run tests first or specify a CSV file manually"
        exit 1
    fi
    
    print_status "Using CSV file: $(basename "$CSV_FILE")"
    
    # Activate Python environment and run plotting script
    source "$VENV_DIR/bin/activate"
    
    if python plot_scan_results.py "$CSV_FILE"; then
        print_status "✓ Plots generated successfully"
        
        # List generated plot files
        PLOT_DIR="$(dirname "$CSV_FILE")/plots"
        if [ -d "$PLOT_DIR" ]; then
            print_status "Generated plot files in $PLOT_DIR:"
            ls -1 "$PLOT_DIR"/*.png 2>/dev/null | xargs -I {} basename {} | sed 's/^/    - /'
        fi
    else
        print_error "Plot generation failed"
        exit 1
    fi
    
    deactivate
}

# Main execution flow
main() {
    print_header "Scan Process Test Analysis Workflow"
    print_status "Started at $(date)"
    
    # Validate we're in the right directory
    if [ ! -f "plot_scan_results.py" ]; then
        print_error "Must be run from the ScanProcessTest directory"
        print_status "Current directory: $(pwd)"
        exit 1
    fi
    
    # Execute workflow based on options
    if [ "$PLOT_ONLY" = true ]; then
        setup_python_environment
        generate_plots
    elif [ "$BUILD_ONLY" = true ]; then
        build_application
    elif [ "$TEST_ONLY" = true ]; then
        run_tests
        setup_python_environment
        generate_plots
    else
        # Full workflow
        build_application
        run_tests
        setup_python_environment
        generate_plots
    fi
    
    print_header "Workflow Completed Successfully"
    print_status "Finished at $(date)"
    
    # Show final summary
    if [ "$BUILD_ONLY" = false ]; then
        echo ""
        print_status "Summary:"
        if [ "$PLOT_ONLY" = false ]; then
            echo "  ✓ Tests executed"
        fi
        if [ "$TEST_ONLY" = false ] && [ "$BUILD_ONLY" = false ]; then
            echo "  ✓ Analysis plots generated"
            echo "  📊 Check the plots/ directory for publication-ready visualizations"
        fi
    fi
}

# Run main function
main "$@"
