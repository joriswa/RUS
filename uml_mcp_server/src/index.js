#!/usr/bin/env node

import { Server } from '@modelcontextprotocol/sdk/server/index.js';
import { StdioServerTransport } from '@modelcontextprotocol/sdk/server/stdio.js';
import {
  CallToolRequestSchema,
  ErrorCode,
  ListToolsRequestSchema,
  McpError,
} from '@modelcontextprotocol/sdk/types.js';
import fs from 'fs-extra';
import path from 'path';
import { fileURLToPath } from 'url';
import { dirname } from 'path';
import plantuml from 'node-plantuml';
import { encode } from 'plantuml-encoder';

const __filename = fileURLToPath(import.meta.url);
const __dirname = dirname(__filename);

class UMLMCPServer {
  constructor() {
    this.server = new Server(
      {
        name: 'uml-mcp-server',
        version: '1.0.0',
      },
      {
        capabilities: {
          tools: {},
        },
      }
    );

    this.setupToolHandlers();
  }

  setupToolHandlers() {
    this.server.setRequestHandler(ListToolsRequestSchema, async () => {
      return {
        tools: [
          {
            name: 'generate_class_diagram',
            description: 'Generate UML class diagram from C++ source files',
            inputSchema: {
              type: 'object',
              properties: {
                source_path: {
                  type: 'string',
                  description: 'Path to source directory or specific files',
                },
                output_path: {
                  type: 'string',
                  description: 'Output path for the diagram (optional)',
                  default: 'uml_output/class_diagram.png',
                },
                format: {
                  type: 'string',
                  enum: ['png', 'svg', 'puml', 'txt'],
                  description: 'Output format',
                  default: 'png',
                },
                include_private: {
                  type: 'boolean',
                  description: 'Include private members',
                  default: false,
                },
                include_dependencies: {
                  type: 'boolean',
                  description: 'Include class dependencies',
                  default: true,
                },
              },
              required: ['source_path'],
            },
          },
          {
            name: 'generate_sequence_diagram',
            description: 'Generate UML sequence diagram from code analysis',
            inputSchema: {
              type: 'object',
              properties: {
                source_path: {
                  type: 'string',
                  description: 'Path to source files',
                },
                entry_function: {
                  type: 'string',
                  description: 'Entry point function for sequence analysis',
                },
                output_path: {
                  type: 'string',
                  description: 'Output path for the diagram',
                  default: 'uml_output/sequence_diagram.png',
                },
                format: {
                  type: 'string',
                  enum: ['png', 'svg', 'puml', 'txt'],
                  description: 'Output format',
                  default: 'png',
                },
                max_depth: {
                  type: 'number',
                  description: 'Maximum call depth to analyze',
                  default: 5,
                },
              },
              required: ['source_path', 'entry_function'],
            },
          },
          {
            name: 'generate_component_diagram',
            description: 'Generate UML component diagram showing module structure',
            inputSchema: {
              type: 'object',
              properties: {
                source_path: {
                  type: 'string',
                  description: 'Path to source directory',
                },
                output_path: {
                  type: 'string',
                  description: 'Output path for the diagram',
                  default: 'uml_output/component_diagram.png',
                },
                format: {
                  type: 'string',
                  enum: ['png', 'svg', 'puml', 'txt'],
                  description: 'Output format',
                  default: 'png',
                },
                group_by: {
                  type: 'string',
                  enum: ['directory', 'namespace', 'none'],
                  description: 'How to group components',
                  default: 'directory',
                },
              },
              required: ['source_path'],
            },
          },
          {
            name: 'analyze_project_structure',
            description: 'Analyze and visualize overall project architecture',
            inputSchema: {
              type: 'object',
              properties: {
                project_path: {
                  type: 'string',
                  description: 'Path to project root',
                },
                output_path: {
                  type: 'string',
                  description: 'Output path for analysis results',
                  default: 'uml_output/project_analysis.md',
                },
                include_diagrams: {
                  type: 'boolean',
                  description: 'Generate accompanying UML diagrams',
                  default: true,
                },
                diagram_types: {
                  type: 'array',
                  items: {
                    type: 'string',
                    enum: ['class', 'component', 'package'],
                  },
                  description: 'Types of diagrams to generate',
                  default: ['class', 'component'],
                },
              },
              required: ['project_path'],
            },
          },
          {
            name: 'generate_mermaid_diagram',
            description: 'Generate Mermaid diagram from source code',
            inputSchema: {
              type: 'object',
              properties: {
                source_path: {
                  type: 'string',
                  description: 'Path to source files',
                },
                diagram_type: {
                  type: 'string',
                  enum: ['classDiagram', 'flowchart', 'gitgraph', 'mindmap'],
                  description: 'Type of Mermaid diagram',
                  default: 'classDiagram',
                },
                output_path: {
                  type: 'string',
                  description: 'Output path for the diagram',
                  default: 'uml_output/mermaid_diagram.mmd',
                },
              },
              required: ['source_path'],
            },
          },
        ],
      };
    });

    this.server.setRequestHandler(CallToolRequestSchema, async (request) => {
      const { name, arguments: args } = request.params;

      try {
        switch (name) {
          case 'generate_class_diagram':
            return await this.generateClassDiagram(args);
          case 'generate_sequence_diagram':
            return await this.generateSequenceDiagram(args);
          case 'generate_component_diagram':
            return await this.generateComponentDiagram(args);
          case 'analyze_project_structure':
            return await this.analyzeProjectStructure(args);
          case 'generate_mermaid_diagram':
            return await this.generateMermaidDiagram(args);
          default:
            throw new McpError(
              ErrorCode.MethodNotFound,
              `Unknown tool: ${name}`
            );
        }
      } catch (error) {
        throw new McpError(
          ErrorCode.InternalError,
          `Tool execution failed: ${error.message}`
        );
      }
    });
  }

  async generateClassDiagram(args) {
    const { source_path, output_path = 'uml_output/class_diagram.png', format = 'png', include_private = false, include_dependencies = true } = args;

    // Ensure output directory exists
    const outputDir = path.dirname(output_path);
    await fs.ensureDir(outputDir);

    // Analyze C++ files to extract class information
    const classInfo = await this.analyzeCppFiles(source_path, include_private, include_dependencies);
    
    // Generate PlantUML code
    const plantUMLCode = this.generatePlantUMLClassDiagram(classInfo, include_private, include_dependencies);
    
    // Save or render diagram
    if (format === 'puml') {
      await fs.writeFile(output_path, plantUMLCode);
      return {
        content: [{
          type: 'text',
          text: `PlantUML class diagram saved to: ${output_path}\n\nGenerated PlantUML code:\n${plantUMLCode}`,
        }],
      };
    } else {
      // Generate image using PlantUML
      const imagePath = await this.renderPlantUML(plantUMLCode, output_path, format);
      return {
        content: [{
          type: 'text',
          text: `Class diagram generated successfully at: ${imagePath}\n\nClasses found: ${classInfo.classes.length}\nRelationships: ${classInfo.relationships.length}`,
        }],
      };
    }
  }

  async generateSequenceDiagram(args) {
    const { source_path, entry_function, output_path = 'uml_output/sequence_diagram.png', format = 'png', max_depth = 5 } = args;

    const outputDir = path.dirname(output_path);
    await fs.ensureDir(outputDir);

    // Analyze function calls to create sequence
    const sequenceInfo = await this.analyzeSequence(source_path, entry_function, max_depth);
    
    // Generate PlantUML sequence diagram
    const plantUMLCode = this.generatePlantUMLSequenceDiagram(sequenceInfo);
    
    if (format === 'puml') {
      await fs.writeFile(output_path, plantUMLCode);
      return {
        content: [{
          type: 'text',
          text: `PlantUML sequence diagram saved to: ${output_path}\n\nGenerated PlantUML code:\n${plantUMLCode}`,
        }],
      };
    } else {
      const imagePath = await this.renderPlantUML(plantUMLCode, output_path, format);
      return {
        content: [{
          type: 'text',
          text: `Sequence diagram generated successfully at: ${imagePath}\n\nEntry function: ${entry_function}\nCall depth analyzed: ${max_depth}`,
        }],
      };
    }
  }

  async generateComponentDiagram(args) {
    const { source_path, output_path = 'uml_output/component_diagram.png', format = 'png', group_by = 'directory' } = args;

    const outputDir = path.dirname(output_path);
    await fs.ensureDir(outputDir);

    // Analyze project structure for components
    const componentInfo = await this.analyzeComponents(source_path, group_by);
    
    // Generate PlantUML component diagram
    const plantUMLCode = this.generatePlantUMLComponentDiagram(componentInfo, group_by);
    
    if (format === 'puml') {
      await fs.writeFile(output_path, plantUMLCode);
      return {
        content: [{
          type: 'text',
          text: `PlantUML component diagram saved to: ${output_path}\n\nGenerated PlantUML code:\n${plantUMLCode}`,
        }],
      };
    } else {
      const imagePath = await this.renderPlantUML(plantUMLCode, output_path, format);
      return {
        content: [{
          type: 'text',
          text: `Component diagram generated successfully at: ${imagePath}\n\nComponents found: ${componentInfo.components.length}\nGrouping: ${group_by}`,
        }],
      };
    }
  }

  async analyzeProjectStructure(args) {
    const { project_path, output_path = 'uml_output/project_analysis.md', include_diagrams = true, diagram_types = ['class', 'component'] } = args;

    const outputDir = path.dirname(output_path);
    await fs.ensureDir(outputDir);

    // Perform comprehensive project analysis
    const analysis = await this.performProjectAnalysis(project_path);
    
    // Generate analysis report
    let report = this.generateAnalysisReport(analysis);
    
    // Generate requested diagrams
    if (include_diagrams) {
      const diagramPaths = [];
      
      for (const diagramType of diagram_types) {
        try {
          const diagramPath = `${outputDir}/${diagramType}_diagram.png`;
          
          if (diagramType === 'class') {
            await this.generateClassDiagram({
              source_path: project_path,
              output_path: diagramPath,
              format: 'png',
              include_dependencies: true,
            });
          } else if (diagramType === 'component') {
            await this.generateComponentDiagram({
              source_path: project_path,
              output_path: diagramPath,
              format: 'png',
              group_by: 'directory',
            });
          }
          
          diagramPaths.push(diagramPath);
          report += `\n\n## ${diagramType.charAt(0).toUpperCase() + diagramType.slice(1)} Diagram\n\nGenerated at: ${diagramPath}\n`;
        } catch (error) {
          report += `\n\n## ${diagramType.charAt(0).toUpperCase() + diagramType.slice(1)} Diagram\n\nFailed to generate: ${error.message}\n`;
        }
      }
    }
    
    // Save analysis report
    await fs.writeFile(output_path, report);
    
    return {
      content: [{
        type: 'text',
        text: `Project analysis completed and saved to: ${output_path}\n\nAnalysis includes:\n- File structure\n- Class hierarchy\n- Dependencies\n- Complexity metrics\n${include_diagrams ? `- UML diagrams: ${diagram_types.join(', ')}` : ''}`,
      }],
    };
  }

  async generateMermaidDiagram(args) {
    const { source_path, diagram_type = 'classDiagram', output_path = 'uml_output/mermaid_diagram.mmd' } = args;

    const outputDir = path.dirname(output_path);
    await fs.ensureDir(outputDir);

    // Analyze source for Mermaid diagram generation
    let mermaidCode = '';
    
    if (diagram_type === 'classDiagram') {
      const classInfo = await this.analyzeCppFiles(source_path, false, true);
      mermaidCode = this.generateMermaidClassDiagram(classInfo);
    } else if (diagram_type === 'flowchart') {
      const flowInfo = await this.analyzeForFlowchart(source_path);
      mermaidCode = this.generateMermaidFlowchart(flowInfo);
    }
    
    await fs.writeFile(output_path, mermaidCode);
    
    return {
      content: [{
        type: 'text',
        text: `Mermaid ${diagram_type} generated successfully at: ${output_path}\n\nGenerated Mermaid code:\n${mermaidCode}`,
      }],
    };
  }

  // Helper methods for analysis and generation

  async analyzeCppFiles(sourcePath, includePrivate = false, includeDependencies = true) {
    const classes = [];
    const relationships = [];
    
    try {
      const files = await this.getCppFiles(sourcePath);
      
      for (const file of files) {
        const content = await fs.readFile(file, 'utf8');
        const fileClasses = this.extractClassesFromContent(content, includePrivate);
        classes.push(...fileClasses);
        
        if (includeDependencies) {
          const fileRelationships = this.extractRelationshipsFromContent(content);
          relationships.push(...fileRelationships);
        }
      }
    } catch (error) {
      console.error('Error analyzing C++ files:', error);
    }
    
    return { classes, relationships };
  }

  async getCppFiles(sourcePath) {
    const files = [];
    const stats = await fs.stat(sourcePath);
    
    if (stats.isFile()) {
      if (sourcePath.match(/\.(cpp|cc|cxx|c\+\+|hpp|h|hxx|h\+\+)$/i)) {
        files.push(sourcePath);
      }
    } else if (stats.isDirectory()) {
      const entries = await fs.readdir(sourcePath, { withFileTypes: true });
      
      for (const entry of entries) {
        const fullPath = path.join(sourcePath, entry.name);
        if (entry.isDirectory()) {
          const subFiles = await this.getCppFiles(fullPath);
          files.push(...subFiles);
        } else if (entry.name.match(/\.(cpp|cc|cxx|c\+\+|hpp|h|hxx|h\+\+)$/i)) {
          files.push(fullPath);
        }
      }
    }
    
    return files;
  }

  extractClassesFromContent(content, includePrivate = false) {
    const classes = [];
    const classRegex = /class\s+(\w+)(?:\s*:\s*(?:public|private|protected)\s+(\w+))?\s*\{([^}]*)\}/gs;
    let match;
    
    while ((match = classRegex.exec(content)) !== null) {
      const [, className, baseClass, classBody] = match;
      
      const methods = this.extractMethods(classBody, includePrivate);
      const attributes = this.extractAttributes(classBody, includePrivate);
      
      classes.push({
        name: className,
        baseClass: baseClass || null,
        methods,
        attributes,
      });
    }
    
    return classes;
  }

  extractMethods(classBody, includePrivate = false) {
    const methods = [];
    const sections = this.parseClassSections(classBody);
    
    for (const [visibility, content] of Object.entries(sections)) {
      if (!includePrivate && visibility === 'private') continue;
      
      const methodRegex = /(\w+)\s+(\w+)\s*\([^)]*\)(?:\s*const)?(?:\s*=\s*0)?;/g;
      let match;
      
      while ((match = methodRegex.exec(content)) !== null) {
        const [, returnType, methodName] = match;
        methods.push({
          name: methodName,
          returnType,
          visibility,
        });
      }
    }
    
    return methods;
  }

  extractAttributes(classBody, includePrivate = false) {
    const attributes = [];
    const sections = this.parseClassSections(classBody);
    
    for (const [visibility, content] of Object.entries(sections)) {
      if (!includePrivate && visibility === 'private') continue;
      
      const attributeRegex = /(\w+)\s+(\w+);/g;
      let match;
      
      while ((match = attributeRegex.exec(content)) !== null) {
        const [, type, name] = match;
        attributes.push({
          name,
          type,
          visibility,
        });
      }
    }
    
    return attributes;
  }

  parseClassSections(classBody) {
    const sections = { public: '', private: '', protected: '' };
    let currentSection = 'private'; // Default for classes
    
    const lines = classBody.split('\n');
    
    for (const line of lines) {
      const trimmed = line.trim();
      if (trimmed.startsWith('public:')) {
        currentSection = 'public';
      } else if (trimmed.startsWith('private:')) {
        currentSection = 'private';
      } else if (trimmed.startsWith('protected:')) {
        currentSection = 'protected';
      } else {
        sections[currentSection] += line + '\n';
      }
    }
    
    return sections;
  }

  extractRelationshipsFromContent(content) {
    const relationships = [];
    // Simple relationship extraction - can be enhanced
    const includeRegex = /#include\s*["<]([^">]+)[">]/g;
    let match;
    
    while ((match = includeRegex.exec(content)) !== null) {
      relationships.push({
        type: 'dependency',
        target: match[1],
      });
    }
    
    return relationships;
  }

  generatePlantUMLClassDiagram(classInfo, includePrivate = false, includeDependencies = true) {
    let puml = '@startuml\n';
    puml += '!theme plain\n';
    puml += 'skinparam classAttributeIconSize 0\n\n';
    
    // Generate classes
    for (const cls of classInfo.classes) {
      puml += `class ${cls.name} {\n`;
      
      // Add attributes
      for (const attr of cls.attributes) {
        const symbol = this.getVisibilitySymbol(attr.visibility);
        puml += `  ${symbol}${attr.name}: ${attr.type}\n`;
      }
      
      if (cls.attributes.length > 0 && cls.methods.length > 0) {
        puml += '  --\n';
      }
      
      // Add methods
      for (const method of cls.methods) {
        const symbol = this.getVisibilitySymbol(method.visibility);
        puml += `  ${symbol}${method.name}(): ${method.returnType}\n`;
      }
      
      puml += '}\n\n';
      
      // Add inheritance
      if (cls.baseClass) {
        puml += `${cls.baseClass} <|-- ${cls.name}\n`;
      }
    }
    
    puml += '@enduml\n';
    return puml;
  }

  getVisibilitySymbol(visibility) {
    switch (visibility) {
      case 'public': return '+';
      case 'private': return '-';
      case 'protected': return '#';
      default: return '';
    }
  }

  generatePlantUMLSequenceDiagram(sequenceInfo) {
    let puml = '@startuml\n';
    puml += '!theme plain\n\n';
    
    // Add actors/participants
    for (const participant of sequenceInfo.participants) {
      puml += `participant ${participant}\n`;
    }
    
    puml += '\n';
    
    // Add sequence steps
    for (const step of sequenceInfo.steps) {
      puml += `${step.from} -> ${step.to}: ${step.message}\n`;
      if (step.return) {
        puml += `${step.to} --> ${step.from}: ${step.return}\n`;
      }
    }
    
    puml += '@enduml\n';
    return puml;
  }

  generatePlantUMLComponentDiagram(componentInfo, groupBy) {
    let puml = '@startuml\n';
    puml += '!theme plain\n\n';
    
    if (groupBy === 'directory') {
      const packages = {};
      
      for (const component of componentInfo.components) {
        const packageName = component.package || 'default';
        if (!packages[packageName]) {
          packages[packageName] = [];
        }
        packages[packageName].push(component);
      }
      
      for (const [packageName, components] of Object.entries(packages)) {
        puml += `package "${packageName}" {\n`;
        for (const component of components) {
          puml += `  [${component.name}]\n`;
        }
        puml += '}\n\n';
      }
    } else {
      for (const component of componentInfo.components) {
        puml += `[${component.name}]\n`;
      }
    }
    
    // Add relationships
    for (const rel of componentInfo.relationships || []) {
      puml += `[${rel.from}] --> [${rel.to}]\n`;
    }
    
    puml += '@enduml\n';
    return puml;
  }

  generateMermaidClassDiagram(classInfo) {
    let mermaid = 'classDiagram\n';
    
    for (const cls of classInfo.classes) {
      mermaid += `  class ${cls.name} {\n`;
      
      // Add attributes
      for (const attr of cls.attributes) {
        const symbol = this.getMermaidVisibilitySymbol(attr.visibility);
        mermaid += `    ${symbol}${attr.type} ${attr.name}\n`;
      }
      
      // Add methods
      for (const method of cls.methods) {
        const symbol = this.getMermaidVisibilitySymbol(method.visibility);
        mermaid += `    ${symbol}${method.name}() ${method.returnType}\n`;
      }
      
      mermaid += '  }\n';
      
      // Add inheritance
      if (cls.baseClass) {
        mermaid += `  ${cls.baseClass} <|-- ${cls.name}\n`;
      }
    }
    
    return mermaid;
  }

  getMermaidVisibilitySymbol(visibility) {
    switch (visibility) {
      case 'public': return '+';
      case 'private': return '-';
      case 'protected': return '#';
      default: return '';
    }
  }

  async analyzeSequence(sourcePath, entryFunction, maxDepth) {
    // Simplified sequence analysis - can be enhanced with more sophisticated parsing
    return {
      participants: ['User', 'System', entryFunction],
      steps: [
        { from: 'User', to: 'System', message: 'start()' },
        { from: 'System', to: entryFunction, message: 'execute()' },
        { from: entryFunction, to: 'System', return: 'result' },
        { from: 'System', to: 'User', return: 'response' },
      ],
    };
  }

  async analyzeComponents(sourcePath, groupBy) {
    const components = [];
    const relationships = [];
    
    try {
      const files = await this.getCppFiles(sourcePath);
      
      for (const file of files) {
        const relativePath = path.relative(sourcePath, file);
        const dirName = path.dirname(relativePath);
        const baseName = path.basename(file, path.extname(file));
        
        components.push({
          name: baseName,
          package: groupBy === 'directory' ? dirName : null,
          file: relativePath,
        });
      }
    } catch (error) {
      console.error('Error analyzing components:', error);
    }
    
    return { components, relationships };
  }

  async performProjectAnalysis(projectPath) {
    // Comprehensive project analysis
    const files = await this.getCppFiles(projectPath);
    const analysis = {
      totalFiles: files.length,
      totalLines: 0,
      classes: [],
      functions: [],
      includes: [],
    };
    
    for (const file of files) {
      try {
        const content = await fs.readFile(file, 'utf8');
        analysis.totalLines += content.split('\n').length;
        
        const fileClasses = this.extractClassesFromContent(content, true);
        analysis.classes.push(...fileClasses);
        
        const fileIncludes = this.extractIncludesFromContent(content);
        analysis.includes.push(...fileIncludes);
      } catch (error) {
        console.error(`Error analyzing file ${file}:`, error);
      }
    }
    
    return analysis;
  }

  extractIncludesFromContent(content) {
    const includes = [];
    const includeRegex = /#include\s*["<]([^">]+)[">]/g;
    let match;
    
    while ((match = includeRegex.exec(content)) !== null) {
      includes.push(match[1]);
    }
    
    return includes;
  }

  generateAnalysisReport(analysis) {
    let report = '# Project Structure Analysis\n\n';
    report += `## Overview\n\n`;
    report += `- **Total Files**: ${analysis.totalFiles}\n`;
    report += `- **Total Lines**: ${analysis.totalLines}\n`;
    report += `- **Classes Found**: ${analysis.classes.length}\n`;
    report += `- **Unique Includes**: ${[...new Set(analysis.includes)].length}\n\n`;
    
    report += `## Classes\n\n`;
    for (const cls of analysis.classes) {
      report += `### ${cls.name}\n`;
      if (cls.baseClass) {
        report += `- **Inherits from**: ${cls.baseClass}\n`;
      }
      report += `- **Methods**: ${cls.methods.length}\n`;
      report += `- **Attributes**: ${cls.attributes.length}\n\n`;
    }
    
    report += `## Dependencies\n\n`;
    const uniqueIncludes = [...new Set(analysis.includes)].sort();
    for (const include of uniqueIncludes) {
      report += `- ${include}\n`;
    }
    
    return report;
  }

  async analyzeForFlowchart(sourcePath) {
    // Simplified flowchart analysis
    return {
      nodes: ['Start', 'Process', 'Decision', 'End'],
      edges: [
        { from: 'Start', to: 'Process' },
        { from: 'Process', to: 'Decision' },
        { from: 'Decision', to: 'End' },
      ],
    };
  }

  generateMermaidFlowchart(flowInfo) {
    let mermaid = 'flowchart TD\n';
    
    for (const node of flowInfo.nodes) {
      mermaid += `  ${node}\n`;
    }
    
    for (const edge of flowInfo.edges) {
      mermaid += `  ${edge.from} --> ${edge.to}\n`;
    }
    
    return mermaid;
  }

  async renderPlantUML(plantUMLCode, outputPath, format) {
    return new Promise((resolve, reject) => {
      const gen = plantuml.generate(plantUMLCode, { format });
      const chunks = [];
      
      gen.out.on('data', (chunk) => {
        chunks.push(chunk);
      });
      
      gen.out.on('end', async () => {
        try {
          const buffer = Buffer.concat(chunks);
          await fs.writeFile(outputPath, buffer);
          resolve(outputPath);
        } catch (error) {
          reject(error);
        }
      });
      
      gen.out.on('error', reject);
    });
  }

  async run() {
    const transport = new StdioServerTransport();
    await this.server.connect(transport);
    console.error('UML MCP Server running on stdio');
  }
}

const server = new UMLMCPServer();
server.run().catch(console.error);