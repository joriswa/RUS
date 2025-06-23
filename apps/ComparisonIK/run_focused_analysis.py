#!/usr/bin/env python3
"""
Master script to run all focused ComparisonIK analyses.
Generates individual plots for each analysis category:
1. Execution Time Analysis
2. Success Rate Analysis  
3. Clearance Analysis
4. Configuration Variance Analysis
"""

import subprocess
import sys
import os
from datetime import datetime

def run_analysis_script(script_name, description):
    """Run an analysis script and handle errors."""
    print(f"\n🔄 Running {description}...")
    print("-" * 50)
    
    try:
        result = subprocess.run([sys.executable, script_name], 
                              capture_output=True, text=True, cwd=os.getcwd())
        
        if result.returncode == 0:
            print(result.stdout)
            print(f"✅ {description} completed successfully!")
        else:
            print(f"❌ {description} failed!")
            print("STDOUT:", result.stdout)
            print("STDERR:", result.stderr)
            return False
            
    except Exception as e:
        print(f"❌ Error running {description}: {e}")
        return False
    
    return True

def main():
    """Run all focused analysis scripts."""
    
    print("🚀 FOCUSED COMPARISONIK ANALYSIS SUITE")
    print("="*60)
    print(f"Generated on: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}")
    print("="*60)
    
    # Define analysis scripts
    analyses = [
        ("plot_execution_time_analysis.py", "Execution Time Analysis"),
        ("plot_success_rate_analysis.py", "Success Rate Analysis"),
        ("plot_clearance_analysis.py", "Clearance Analysis"),
        ("plot_configuration_variance_analysis.py", "Configuration Variance Analysis"),
        ("plot_joint_limit_analysis.py", "Joint Limit Avoidance Analysis")
    ]
    
    # Check if data file exists
    if not os.path.exists('two_method_comparison_results.csv'):
        print("❌ Data file 'two_method_comparison_results.csv' not found!")
        print("   Please run the TwoMethodComparison executable first.")
        return False
    
    # Create plots directory
    os.makedirs('plots', exist_ok=True)
    
    # Run all analyses
    successful_analyses = 0
    total_analyses = len(analyses)
    
    for script_name, description in analyses:
        if os.path.exists(script_name):
            if run_analysis_script(script_name, description):
                successful_analyses += 1
        else:
            print(f"❌ Script {script_name} not found!")
    
    # Summary
    print(f"\n📊 ANALYSIS SUITE SUMMARY")
    print("="*60)
    print(f"✅ Successful analyses: {successful_analyses}/{total_analyses}")
    
    if successful_analyses == total_analyses:
        print(f"\n🎉 All analyses completed successfully!")
        print(f"📁 Check the 'plots/' directory for generated visualizations:")
        
        expected_plots = [
            "execution_time_analysis.png",
            "success_rate_analysis.png", 
            "clearance_analysis.png",
            "configuration_variance_analysis.png",
            "joint_limit_analysis.png"
        ]
        
        for plot_file in expected_plots:
            plot_path = os.path.join('plots', plot_file)
            if os.path.exists(plot_path):
                print(f"   ✅ {plot_file}")
            else:
                print(f"   ❌ {plot_file} (missing)")
    else:
        print(f"\n⚠️  Some analyses failed. Check error messages above.")
    
    print("="*60)
    
    return successful_analyses == total_analyses

if __name__ == "__main__":
    success = main()
    sys.exit(0 if success else 1)
