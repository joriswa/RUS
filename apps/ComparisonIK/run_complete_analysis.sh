#!/bin/bash

# =============================================================================
# ComparisonIK Complete Analysis Pipeline
# =============================================================================
# This script provides a one-command solution to:
# 1. Build the C++ ThreeMethodComparison executable
# 2. Run the comparison to generate data
# 3. Execute all Python analysis scripts
# 4. Generate comprehensive visualizations
# =============================================================================

set -e  # Exit on any error

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
PURPLE='\033[0;35m'
CYAN='\033[0;36m'
NC='\033[0m' # No Color

# Function to print colored headers
print_header() {
    echo -e "\n${BLUE}========================================${NC}"
    echo -e "${BLUE}$1${NC}"
    echo -e "${BLUE}========================================${NC}\n"
}

print_step() {
    echo -e "\n${CYAN}🔄 $1${NC}"
}

print_success() {
    echo -e "${GREEN}✅ $1${NC}"
}

print_error() {
    echo -e "${RED}❌ $1${NC}"
}

print_warning() {
    echo -e "${YELLOW}⚠️  $1${NC}"
}

# Get script directory
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"
BUILD_DIR="$PROJECT_ROOT/build"

print_header "COMPARISONIK COMPLETE ANALYSIS PIPELINE"
echo -e "📁 Project root: ${PROJECT_ROOT}"
echo -e "📁 Script directory: ${SCRIPT_DIR}"
echo -e "📁 Build directory: ${BUILD_DIR}"
echo -e "⏰ Started at: $(date)"

# =============================================================================
# STEP 1: BUILD C++ EXECUTABLE
# =============================================================================
print_header "STEP 1: BUILDING C++ EXECUTABLE"

cd "$PROJECT_ROOT"

# Check if build directory exists, create if not
if [ ! -d "$BUILD_DIR" ]; then
    print_step "Creating build directory"
    mkdir -p "$BUILD_DIR"
fi

cd "$BUILD_DIR"

# Configure with CMake
print_step "Configuring CMake"
if ! cmake ..; then
    print_error "CMake configuration failed!"
    exit 1
fi

# Build the entire project (which includes ThreeMethodComparison)
print_step "Building project with ThreeMethodComparison executable"
if ! make -j$(nproc 2>/dev/null || sysctl -n hw.ncpu 2>/dev/null || echo 4); then
    print_error "Build failed!"
    exit 1
fi

# Check if executable was created
EXECUTABLE_PATH="$BUILD_DIR/apps/ComparisonIK/ThreeMethodComparison"
if [ ! -f "$EXECUTABLE_PATH" ]; then
    print_error "Executable not found at: $EXECUTABLE_PATH"
    exit 1
fi

print_success "ThreeMethodComparison executable built successfully"

# =============================================================================
# STEP 2: RUN C++ COMPARISON
# =============================================================================
print_header "STEP 2: RUNNING IK METHODS COMPARISON"

cd "$SCRIPT_DIR"

print_step "Executing ThreeMethodComparison"
echo -e "📊 This will test Newton-Raphson, SA-Optimized, and Grid Search methods"
echo -e "📊 Results will be saved to: three_method_comparison_results.csv"

# Run the executable and capture output
if ! "$EXECUTABLE_PATH"; then
    print_error "ThreeMethodComparison execution failed!"
    exit 1
fi

# Check if results file was generated
if [ ! -f "$SCRIPT_DIR/three_method_comparison_results.csv" ]; then
    print_error "Results CSV file was not generated!"
    exit 1
fi

print_success "Comparison completed - data generated successfully"

# Show quick data summary
TOTAL_ROWS=$(wc -l < "$SCRIPT_DIR/three_method_comparison_results.csv")
TOTAL_TESTS=$((TOTAL_ROWS - 1))  # Subtract header
echo -e "📈 Generated $TOTAL_TESTS test results"

# =============================================================================
# STEP 3: RUN PYTHON ANALYSES
# =============================================================================
print_header "STEP 3: RUNNING PYTHON ANALYSIS SUITE"

cd "$SCRIPT_DIR"

# Check if Python is available
if ! command -v python3 &> /dev/null; then
    print_error "Python3 is not available!"
    exit 1
fi

# Check for required Python packages
print_step "Checking Python dependencies"
REQUIRED_PACKAGES=("pandas" "numpy" "matplotlib" "seaborn")
MISSING_PACKAGES=()

for package in "${REQUIRED_PACKAGES[@]}"; do
    if ! python3 -c "import $package" 2>/dev/null; then
        MISSING_PACKAGES+=("$package")
    fi
done

if [ ${#MISSING_PACKAGES[@]} -ne 0 ]; then
    print_warning "Missing Python packages: ${MISSING_PACKAGES[*]}"
    print_step "Installing missing packages"
    
    for package in "${MISSING_PACKAGES[@]}"; do
        echo -e "Installing $package..."
        if ! pip3 install "$package"; then
            print_error "Failed to install $package"
            exit 1
        fi
    done
fi

print_success "Python dependencies verified"

# Create plots directory
mkdir -p plots

# Define analysis scripts in execution order (in comparison_methods directory)
ANALYSES=(
    "comparison_methods/master_plot_generator.py:Complete Analysis with Master Plot Generator"
    "plot_variance_analysis.py:Original Variance Analysis with Overall and Per-Joint Boxplots"
)

# Run each analysis
SUCCESSFUL_ANALYSES=0
TOTAL_ANALYSES=${#ANALYSES[@]}

for analysis in "${ANALYSES[@]}"; do
    IFS=':' read -r script_name description <<< "$analysis"
    
    if [ -f "$script_name" ]; then
        print_step "Running $description"
        
        if python3 "$script_name"; then
            print_success "$description completed"
            ((SUCCESSFUL_ANALYSES++))
        else
            print_error "$description failed"
        fi
    else
        print_warning "Script $script_name not found - skipping"
    fi
    
    echo  # Add spacing between analyses
done

# =============================================================================
# STEP 4: ANALYSIS SUMMARY
# =============================================================================
print_header "STEP 4: ANALYSIS SUMMARY"

echo -e "🏁 Pipeline completed at: $(date)"
echo -e "📊 Successful analyses: $SUCCESSFUL_ANALYSES/$TOTAL_ANALYSES"

if [ $SUCCESSFUL_ANALYSES -eq $TOTAL_ANALYSES ]; then
    print_success "All analyses completed successfully!"
    
    echo -e "\n📁 Generated files:"
    echo -e "   📄 two_method_comparison_results.csv (raw data)"
    
    if [ -d "plots" ]; then
        echo -e "   📊 Plots in plots/ directory:"
        for plot in plots/*.png plots/*.pdf; do
            if [ -f "$plot" ]; then
                echo -e "      • $(basename "$plot")"
            fi
        done
    fi
    
    echo -e "\n🎯 Next steps:"
    echo -e "   • Review plots in the plots/ directory"
    echo -e "   • Analyze CSV data for detailed metrics"
    echo -e "   • Consider running additional focused analyses"
    
else
    print_warning "Some analyses failed - check error messages above"
    echo -e "\n🔧 Troubleshooting:"
    echo -e "   • Ensure all Python dependencies are installed"
    echo -e "   • Check that the CSV data file was generated correctly"
    echo -e "   • Review individual script error messages"
fi

echo -e "\n${PURPLE}=== PIPELINE COMPLETE ===${NC}\n"

# Set exit code based on success
if [ $SUCCESSFUL_ANALYSES -eq $TOTAL_ANALYSES ]; then
    exit 0
else
    exit 1
fi
