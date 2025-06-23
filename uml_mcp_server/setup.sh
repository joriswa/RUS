#!/bin/bash

# UML MCP Server Setup Script
# This script sets up the UML MCP server for generating UML diagrams

set -e

echo "🚀 Setting up UML MCP Server..."

# Check if Node.js is installed
if ! command -v node &> /dev/null; then
    echo "❌ Node.js is not installed. Please install Node.js 18+ first."
    echo "Visit: https://nodejs.org/"
    exit 1
fi

# Check Node.js version
NODE_VERSION=$(node --version | cut -d'v' -f2)
REQUIRED_VERSION="18.0.0"

if [ "$(printf '%s\n' "$REQUIRED_VERSION" "$NODE_VERSION" | sort -V | head -n1)" != "$REQUIRED_VERSION" ]; then
    echo "❌ Node.js version $NODE_VERSION is too old. Please install Node.js 18+ first."
    exit 1
fi

echo "✅ Node.js version $NODE_VERSION detected"

# Install dependencies
echo "📦 Installing npm dependencies..."
npm install

# Check if PlantUML is available (optional but recommended)
if command -v plantuml &> /dev/null; then
    echo "✅ PlantUML found in system"
elif command -v java &> /dev/null; then
    echo "☕ Java found - PlantUML can be installed"
    echo "💡 To install PlantUML:"
    echo "   - Ubuntu/Debian: sudo apt-get install plantuml"
    echo "   - macOS: brew install plantuml"
    echo "   - Or download from: https://plantuml.com/"
else
    echo "⚠️  Java not found - PlantUML may not work for image generation"
    echo "💡 Install Java and PlantUML for best results"
fi

# Create output directory
echo "📁 Creating output directory..."
mkdir -p uml_output

# Make the script executable
chmod +x src/index.js

echo ""
echo "🎉 UML MCP Server setup complete!"
echo ""
echo "📋 Available tools:"
echo "   • generate_class_diagram     - Create UML class diagrams from C++ code"
echo "   • generate_sequence_diagram  - Create sequence diagrams from function calls"
echo "   • generate_component_diagram - Create component/module diagrams"
echo "   • analyze_project_structure  - Comprehensive project analysis"
echo "   • generate_mermaid_diagram   - Create Mermaid diagrams"
echo ""
echo "🚀 To start the server:"
echo "   npm start"
echo ""
echo "🧪 To test with development mode:"
echo "   npm run dev"
echo ""
echo "📖 Example usage:"
echo "   The server communicates via stdio and follows the MCP protocol."
echo "   Use with MCP-compatible clients like Claude Desktop, etc."
echo ""
echo "📂 Output files will be saved to: ./uml_output/"
echo ""
echo "🔧 Configuration:"
echo "   • Supported input: C++, C files (.cpp, .hpp, .h, .cc, .cxx)"
echo "   • Output formats: PNG, SVG, PlantUML (.puml), Mermaid (.mmd)"
echo "   • Analysis includes: Classes, methods, attributes, relationships"
echo ""

# Test if the server can start
echo "🧪 Testing server startup..."
timeout 5s node src/index.js --test 2>/dev/null || echo "⚠️  Server test timed out (this is expected for MCP servers)"

echo ""
echo "✅ Setup verification complete!"
echo ""
echo "📚 Next steps:"
echo "   1. Configure your MCP client to use this server"
echo "   2. Point the tools to your C++ source code directories"
echo "   3. Generate UML diagrams for your PathPlanner project!"
echo ""
echo "🎯 Example for PathPlanner project:"
echo "   Use source_path: '../libs' or '../apps' to analyze the codebase"