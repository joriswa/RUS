#!/bin/bash

# Build script for RUS libraries
# Usage: ./build.sh [basic|full|clean]

set -e

BUILD_DIR="build"
INSTALL_PREFIX="install"

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

print_status() {
    echo -e "${BLUE}[INFO]${NC} $1"
}

print_success() {
    echo -e "${GREEN}[SUCCESS]${NC} $1"
}

print_warning() {
    echo -e "${YELLOW}[WARNING]${NC} $1"
}

print_error() {
    echo -e "${RED}[ERROR]${NC} $1"
}

check_dependencies() {
    print_status "Checking dependencies..."
    
    # Check for required tools
    command -v cmake >/dev/null 2>&1 || { print_error "cmake is required but not installed."; exit 1; }
    command -v make >/dev/null 2>&1 || { print_error "make is required but not installed."; exit 1; }
    
    # Check for required libraries (basic)
    pkg-config --exists eigen3 || { print_warning "eigen3 not found. Install libeigen3-dev"; }
    
    # Check for advanced dependencies
    pkg-config --exists Qt5Core >/dev/null 2>&1 && HAVE_QT5=1 || HAVE_QT5=0
    pkg-config --exists orocos-kdl >/dev/null 2>&1 && HAVE_KDL=1 || HAVE_KDL=0
    
    if [[ $HAVE_QT5 -eq 1 && $HAVE_KDL -eq 1 ]]; then
        print_success "All dependencies available for full build"
        return 0
    else
        print_warning "Some dependencies missing. Only basic libraries will build."
        return 1
    fi
}

clean_build() {
    print_status "Cleaning build directory..."
    rm -rf "$BUILD_DIR"
    print_success "Build directory cleaned"
}

build_basic() {
    print_status "Building basic libraries (GeometryLib, Hauser10)..."
    
    mkdir -p "$BUILD_DIR"
    cd "$BUILD_DIR"
    
    cmake .. \
        -DCMAKE_BUILD_TYPE=Release \
        -DCMAKE_INSTALL_PREFIX="$INSTALL_PREFIX" \
        -DBUILD_ADVANCED_LIBS=OFF \
        -DBUILD_APPS=OFF
    
    make -j$(nproc)
    make install
    
    cd ..
    
    print_success "Basic libraries built successfully!"
    print_status "Built libraries:"
    ls -la "$BUILD_DIR/$INSTALL_PREFIX/lib/"*.a 2>/dev/null || true
}

build_full() {
    print_status "Building all libraries and applications..."
    
    mkdir -p "$BUILD_DIR"
    cd "$BUILD_DIR"
    
    cmake .. \
        -DCMAKE_BUILD_TYPE=Release \
        -DCMAKE_INSTALL_PREFIX="$INSTALL_PREFIX" \
        -DBUILD_ADVANCED_LIBS=ON \
        -DBUILD_APPS=ON
    
    if make -j$(nproc); then
        make install
        print_success "Full build completed successfully!"
    else
        print_error "Full build failed. Trying basic build..."
        cd ..
        rm -rf "$BUILD_DIR"
        build_basic
        return 1
    fi
    
    cd ..
    
    print_status "Built libraries:"
    ls -la "$BUILD_DIR/$INSTALL_PREFIX/lib/"*.a 2>/dev/null || true
    
    print_status "Built applications:"
    ls -la "$BUILD_DIR/$INSTALL_PREFIX/bin/"* 2>/dev/null || true
}

show_usage() {
    echo "Usage: $0 [basic|full|clean|help]"
    echo ""
    echo "Commands:"
    echo "  basic  - Build only core libraries (GeometryLib, Hauser10)"
    echo "  full   - Build all libraries and applications (default)"
    echo "  clean  - Clean build directory"
    echo "  help   - Show this help message"
    echo ""
    echo "The build artifacts will be installed in: $BUILD_DIR/$INSTALL_PREFIX/"
}

# Main script logic
case "${1:-full}" in
    "basic")
        check_dependencies
        build_basic
        ;;
    "full")
        if check_dependencies; then
            build_full
        else
            print_warning "Falling back to basic build due to missing dependencies"
            build_basic
        fi
        ;;
    "clean")
        clean_build
        ;;
    "help"|"-h"|"--help")
        show_usage
        ;;
    *)
        print_error "Unknown command: $1"
        show_usage
        exit 1
        ;;
esac