#!/bin/bash

# Setup Script for Smart Multi-Objective STOMP Parameter Optimization
# ====================================================================

echo "Setting up Smart Multi-Objective STOMP Parameter Optimization System..."

# Create Python virtual environment if it doesn't exist
if [ ! -d "venv_smart_optimization" ]; then
    echo "Creating Python virtual environment..."
    python3 -m venv venv_smart_optimization
fi

# Activate virtual environment
source venv_smart_optimization/bin/activate

# Upgrade pip
echo "Upgrading pip..."
pip install --upgrade pip

# Install required packages
echo "Installing Python dependencies..."

# Core scientific computing
pip install numpy pandas scipy matplotlib seaborn

# Machine learning
pip install scikit-learn optuna

# Multi-objective optimization
pip install pymoo

# Data processing and visualization
pip install pyyaml

# Additional utilities
pip install tqdm joblib

# Check installations
echo "Verifying installations..."
python3 -c "
import numpy as np
import pandas as pd
import sklearn
import pymoo
import optuna
import yaml
print('✓ All core dependencies installed successfully')
"

# Create necessary directories
echo "Creating directory structure..."
mkdir -p results/smart_optimization
mkdir -p models/adaptive_models
mkdir -p logs
mkdir -p config

# Create requirements.txt for future reference
echo "Creating requirements.txt..."
pip freeze > requirements.txt

echo "Setup completed successfully!"
echo ""
echo "To activate the environment in the future, run:"
echo "  source venv_smart_optimization/bin/activate"
echo ""
echo "To run the smart optimizer:"
echo "  python3 smart_optimizer.py --config config/smart_optimizer_config.yaml --scenarios config/scenarios_config.yaml"
