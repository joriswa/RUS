#!/usr/bin/env python3
"""
Debug script to check the Optuna study structure
"""

import optuna
import pandas as pd

def debug_study_structure():
    # Load the study
    study = optuna.load_study(study_name="test_integration", storage="sqlite:///optuna_studies.db")
    df = study.trials_dataframe()
    
    print("Study Information:")
    print(f"Number of trials: {len(df)}")
    print(f"Best value: {study.best_value}")
    print(f"Best params: {study.best_params}")
    
    print("\\nDataFrame columns:")
    for col in sorted(df.columns):
        print(f"  {col}")
    
    print("\\nSample trial data:")
    print(df.head(2).to_string())
    
    print("\\nParameter columns:")
    param_cols = [col for col in df.columns if col.startswith(('params_', 'stomp_', 'rrt_', 'hauser_'))]
    for col in param_cols:
        print(f"  {col}")
    
    if len(param_cols) == 0:
        print("\\nNo parameter columns found! Checking params structure...")
        for i, trial in enumerate(study.trials[:3]):
            print(f"Trial {i} params: {trial.params}")

if __name__ == "__main__":
    debug_study_structure()
