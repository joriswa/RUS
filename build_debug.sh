#!/bin/bash

# Build script for STOMP environment debug tool

echo "=== Building STOMP Environment Debug Tool ==="

# Ensure we're in the right directory
if [ ! -f "debug_stomp_with_environment.cpp" ]; then
    echo "Error: debug_stomp_with_environment.cpp not found. Run this script from the project root."
    exit 1
fi

# Create debug build directory
mkdir -p debug_build
cd debug_build

# Configure with CMake
echo "Configuring with CMake..."
cmake -DCMAKE_BUILD_TYPE=Debug -f ../CMakeLists_debug.txt ..

# Build the project
echo "Building..."
make -j$(nproc 2>/dev/null || sysctl -n hw.ncpu 2>/dev/null || echo 4)

if [ $? -eq 0 ]; then
    echo "✓ Build successful!"
    echo ""
    echo "To run the debug tool:"
    echo "  cd debug_build"
    echo "  ./debug_stomp_with_environment [environment.xml] [scenario]"
    echo ""
    echo "Examples:"
    echo "  ./debug_stomp_with_environment ../res/environments/simple.xml fast"
    echo "  ./debug_stomp_with_environment ../res/environments/complex.xml thorough"
    echo ""
    echo "Available scenarios: fast, thorough, exploration, minimal"
else
    echo "❌ Build failed!"
    exit 1
fi
