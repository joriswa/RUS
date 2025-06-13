# Trajectory Planning Evaluator

A comprehensive evaluation system for trajectory planning algorithms with parameter optimization and automated visualization.

## Overview

This application evaluates trajectory planning algorithms across various parameter configurations:
- **STOMP** with parameter variations
- **Hauser algorithms**: RRT, RRT*, Informed RRT*, Bi-RRT
- Combined configurations for comprehensive analysis

## Features

- **C++ Evaluation Engine**: Runs trajectory planning evaluations with configurable parameters
- **Python Visualization**: Generates boxplot PDFs for each metric
- **Automated Orchestration**: Makefile and scripts coordinate C++ execution and Python plotting
- **Modular Design**: Clean separation between evaluation, data processing, and visualization

## Structure

```
TrajectoryPlanningEvaluator/
├── src/                    # C++ source files
├── include/                # C++ header files
├── python/                 # Python visualization scripts
├── config/                 # Configuration files
├── data/                   # Input data (poses, scenarios)
├── results/                # Output data and plots
├── CMakeLists.txt          # Build configuration
├── Makefile               # Orchestration script
└── README.md              # This file
```

## Usage

1. **Build the evaluator:**
   ```bash
   make build
   ```

2. **Run full evaluation:**
   ```bash
   make evaluate
   ```

3. **Generate plots only:**
   ```bash
   make plots
   ```

4. **Clean results:**
   ```bash
   make clean
   ```

## Configuration

Edit `config/evaluation_params.yaml` to modify:
- Algorithm parameters
- Test scenarios
- Output settings

## Metrics Analyzed

- Planning time
- Execution time
- Success rate
- Iterations required
- Computation efficiency
- Path quality metrics
