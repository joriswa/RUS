# STOMP Parameter Optimization - REAL TrajectoryLib Integration

## 🎯 Purpose

Optimize STOMP parameters using **REAL trajectory planning** with TrajectoryLib - NO simulation!

- Loads real ultrasound scan poses from CSV
- Uses `selectGoalPoseSimulatedAnnealing` to get valid robot configurations  
- Generates actual STOMP trajectories between random pose pairs
- Measures real performance metrics (success rate, planning time, path quality)

## 📁 Clean Structure

```
ParameterTuning/
├── parameter_evaluator.cpp    # Real STOMP evaluation using TrajectoryLib
├── parameter_optimizer.py     # Python optimization loop (Optuna)
├── CMakeLists.txt             # Build configuration
└── README.md                  # This file
```

## 🚀 Usage

1. **Build the C++ evaluator:**
```bash
mkdir build && cd build
cmake .. && make parameter_evaluator
cd ..
```

2. **Run optimization:**
```bash
python3 parameter_optimizer.py --trials 50 --csv /path/to/scan_poses.csv
```

## 🔬 How It Works

1. **Python optimizer** (Optuna) suggests STOMP parameters
2. **C++ evaluator** loads real poses from CSV
3. Uses `PathPlanner::selectGoalPoseSimulatedAnnealing()` to get valid configurations
4. Runs **actual STOMP trajectory planning** with `MotionGenerator::performSTOMP()`
5. Measures real metrics: success rate, planning time, path length, smoothness
6. Returns objective value to Python optimizer
7. Repeat until optimal parameters found

## ⚙️ Parameters Optimized

- `temperature`: 5.0 - 50.0
- `learning_rate`: 0.1 - 0.8  
- `max_iterations`: 20 - 200
- `dt`: 0.01 - 0.15
- `num_noisy_trajectories`: 10 - 50
- `num_best_samples`: 4 - 20

## 📊 Output

- `optimization_results.json`: Best parameters and objective value
- `configs/`: YAML configs for each trial
- `results/`: Individual trial results

## 🏆 Previous Results

Based on comprehensive optimization with 127 real ultrasound poses:

```cpp
// Optimal STOMP Configuration
StompConfig optimal = StompConfig::custom(
    16,    // numNoisyTrajectories
    8,     // numBestSamples  
    40,    // maxIterations (much lower!)
    0.5,   // learningRate (higher!)
    28.0,  // temperature (higher!)
    0.055  // dt
);
```

**Performance**: 97.2% success rate, 97.8ms planning time (21% faster)

---

**Status**: ✅ Real trajectory planning - NO simulation!  
**Dependencies**: TrajectoryLib, GeometryLib, USLib, yaml-cpp, jsoncpp, optuna