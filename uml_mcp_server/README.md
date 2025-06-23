# UML MCP Server

A Model Context Protocol (MCP) server for generating UML diagrams from C++ source code. This server provides tools to analyze codebases and generate various types of UML diagrams including class diagrams, sequence diagrams, and component diagrams.

## Features

- **Class Diagrams**: Generate UML class diagrams from C++ header and source files
- **Sequence Diagrams**: Create sequence diagrams showing function call flows
- **Component Diagrams**: Visualize module and component relationships
- **Project Analysis**: Comprehensive codebase analysis with metrics
- **Multiple Formats**: Support for PNG, SVG, PlantUML, and Mermaid formats
- **Flexible Analysis**: Configurable privacy levels and dependency inclusion

## Prerequisites

- Node.js 18.0.0 or higher
- npm (comes with Node.js)
- Java (optional, for PlantUML image generation)
- PlantUML (optional, for enhanced diagram rendering)

## Installation

1. Navigate to the UML MCP server directory:
```bash
cd PathPlanner_US_wip/uml_mcp_server
```

2. Run the setup script:
```bash
chmod +x setup.sh
./setup.sh
```

Or install manually:
```bash
npm install
mkdir -p uml_output
```

## Usage

### Starting the Server

```bash
npm start
```

For development with auto-reload:
```bash
npm run dev
```

### Available Tools

#### 1. `generate_class_diagram`
Generate UML class diagrams from C++ source files.

**Parameters:**
- `source_path` (required): Path to source directory or files
- `output_path` (optional): Output file path (default: `uml_output/class_diagram.png`)
- `format` (optional): Output format - `png`, `svg`, `puml`, `txt` (default: `png`)
- `include_private` (optional): Include private members (default: `false`)
- `include_dependencies` (optional): Include class dependencies (default: `true`)

**Example:**
```json
{
  "name": "generate_class_diagram",
  "arguments": {
    "source_path": "../libs",
    "output_path": "uml_output/pathplanner_classes.png",
    "format": "png",
    "include_private": true,
    "include_dependencies": true
  }
}
```

#### 2. `generate_sequence_diagram`
Generate UML sequence diagrams from code analysis.

**Parameters:**
- `source_path` (required): Path to source files
- `entry_function` (required): Entry point function for sequence analysis
- `output_path` (optional): Output file path (default: `uml_output/sequence_diagram.png`)
- `format` (optional): Output format (default: `png`)
- `max_depth` (optional): Maximum call depth to analyze (default: `5`)

**Example:**
```json
{
  "name": "generate_sequence_diagram",
  "arguments": {
    "source_path": "../apps",
    "entry_function": "main",
    "output_path": "uml_output/execution_flow.png",
    "max_depth": 3
  }
}
```

#### 3. `generate_component_diagram`
Generate UML component diagrams showing module structure.

**Parameters:**
- `source_path` (required): Path to source directory
- `output_path` (optional): Output file path (default: `uml_output/component_diagram.png`)
- `format` (optional): Output format (default: `png`)
- `group_by` (optional): How to group components - `directory`, `namespace`, `none` (default: `directory`)

**Example:**
```json
{
  "name": "generate_component_diagram",
  "arguments": {
    "source_path": "../",
    "output_path": "uml_output/pathplanner_components.png",
    "group_by": "directory"
  }
}
```

#### 4. `analyze_project_structure`
Analyze and visualize overall project architecture.

**Parameters:**
- `project_path` (required): Path to project root
- `output_path` (optional): Output file path (default: `uml_output/project_analysis.md`)
- `include_diagrams` (optional): Generate accompanying UML diagrams (default: `true`)
- `diagram_types` (optional): Types of diagrams to generate (default: `["class", "component"]`)

**Example:**
```json
{
  "name": "analyze_project_structure",
  "arguments": {
    "project_path": "../",
    "output_path": "uml_output/pathplanner_analysis.md",
    "include_diagrams": true,
    "diagram_types": ["class", "component", "package"]
  }
}
```

#### 5. `generate_mermaid_diagram`
Generate Mermaid diagrams from source code.

**Parameters:**
- `source_path` (required): Path to source files
- `diagram_type` (optional): Type of Mermaid diagram - `classDiagram`, `flowchart`, `gitgraph`, `mindmap` (default: `classDiagram`)
- `output_path` (optional): Output file path (default: `uml_output/mermaid_diagram.mmd`)

**Example:**
```json
{
  "name": "generate_mermaid_diagram",
  "arguments": {
    "source_path": "../libs",
    "diagram_type": "classDiagram",
    "output_path": "uml_output/pathplanner_mermaid.mmd"
  }
}
```

## Supported File Types

The server analyzes the following C++ file extensions:
- `.cpp`, `.cc`, `.cxx`, `.c++` (source files)
- `.hpp`, `.h`, `.hxx`, `.h++` (header files)

## Output Formats

- **PNG**: Raster image format (requires PlantUML with Java)
- **SVG**: Vector image format (requires PlantUML with Java)
- **PUML**: PlantUML source code (text format)
- **MMD**: Mermaid source code (text format)
- **TXT**: Plain text representation

## Configuration

### MCP Client Configuration

To use this server with an MCP client (like Claude Desktop), add it to your configuration:

```json
{
  "mcpServers": {
    "uml-server": {
      "command": "node",
      "args": ["path/to/PathPlanner_US_wip/uml_mcp_server/src/index.js"]
    }
  }
}
```

### Environment Setup

For optimal performance, ensure you have:

1. **Java installed** (for PlantUML image generation):
```bash
# Check if Java is installed
java -version
```

2. **PlantUML installed** (optional but recommended):
```bash
# Ubuntu/Debian
sudo apt-get install plantuml

# macOS
brew install plantuml

# Or download from https://plantuml.com/
```

## Examples for PathPlanner Project

### Analyze the entire PathPlanner codebase:
```json
{
  "name": "analyze_project_structure",
  "arguments": {
    "project_path": "../",
    "include_diagrams": true,
    "diagram_types": ["class", "component"]
  }
}
```

### Generate class diagram for libraries:
```json
{
  "name": "generate_class_diagram",
  "arguments": {
    "source_path": "../libs",
    "include_private": false,
    "include_dependencies": true
  }
}
```

### Generate component diagram showing module structure:
```json
{
  "name": "generate_component_diagram",
  "arguments": {
    "source_path": "../",
    "group_by": "directory"
  }
}
```

## Output Directory

All generated files are saved to the `uml_output/` directory by default. This directory is created automatically during setup.

## Troubleshooting

### Common Issues

1. **"PlantUML not found" error:**
   - Install Java and PlantUML
   - Use `format: "puml"` to generate PlantUML source code instead

2. **"Permission denied" error:**
   - Ensure the script has execute permissions: `chmod +x setup.sh`
   - Check write permissions for the output directory

3. **"Node.js version too old" error:**
   - Update Node.js to version 18.0.0 or higher
   - Use a Node.js version manager like nvm

4. **Empty or malformed diagrams:**
   - Check that the source path contains valid C++ files
   - Verify file extensions are supported
   - Review the analysis output for parsing errors

### Debug Mode

Run the server in development mode for detailed logging:
```bash
npm run dev
```

## Contributing

This UML MCP server is designed specifically for the PathPlanner project but can be adapted for other C++ codebases. Key areas for enhancement:

- Enhanced C++ parsing for complex templates and modern C++ features
- Support for additional diagram types (activity, state, etc.)
- Integration with more UML rendering engines
- Custom styling and theming options

## License

MIT License - see the project root for license details.

## Related Tools

- [PlantUML](https://plantuml.com/) - UML diagram generation
- [Mermaid](https://mermaid.js.org/) - Markdown-based diagramming
- [Model Context Protocol](https://modelcontextprotocol.io/) - Protocol specification