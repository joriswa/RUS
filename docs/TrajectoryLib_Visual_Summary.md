# TrajectoryLib Visual Architecture Summary

## Quick Reference Architecture

```
TrajectoryLib Architecture Overview
=====================================

┌─────────────────────────────────────────────────────────────────────────────┐
│                           EXTERNAL APPLICATIONS                              │
├─────────────────┬─────────────────┬─────────────────┬────────────────────────┤
│     USLib       │  Parameter      │   PathPlanner   │    Evaluation         │
│ (Medical US)    │    Tuning       │      GUI        │      Tools            │
└─────────────────┴─────────────────┴─────────────────┴────────────────────────┘
         │                 │                 │                  │
         ▼                 ▼                 ▼                  ▼
┌─────────────────────────────────────────────────────────────────────────────┐
│                           TRAJECTORYLIB CORE                                │
├─────────────────────────────────────────────────────────────────────────────┤
│                                                                             │
│  ┌─────────────┐  ┌─────────────┐  ┌─────────────┐  ┌─────────────┐        │
│  │    CORE     │  │   MOTION    │  │  PLANNING   │  │    ROBOT    │        │
│  │             │  │             │  │             │  │             │        │
│  │ • Util      │  │ • MotionGen │  │ • PathPlan  │  │ • Robot     │        │
│  │ • Spline    │  │   (STOMP)   │  │ • RRT       │  │ • RobotArm  │        │
│  │ • SimAnn    │  │ • CostCalc  │  │ • RRTStar   │  │ • RobotMgr  │        │
│  │             │  │   (6 types) │  │             │  │ • FrankaIK  │        │
│  └─────────────┘  └─────────────┘  └─────────────┘  └─────────────┘        │
│                                                                             │
│  ┌─────────────┐  ┌─────────────┐              ┌─────────────────────────┐  │
│  │    UTILS    │  │ VISUALIZE   │              │       LOGGING           │  │
│  │             │  │             │              │                         │  │
│  │ • TrajEval  │  │ • TrackBall │              │ • Logger.h              │  │
│  │             │  │   Camera    │              │ • Logging.h             │  │
│  └─────────────┘  └─────────────┘              └─────────────────────────┘  │
│                                                                             │
└─────────────────────────────────────────────────────────────────────────────┘
         │                 │                 │                  │
         ▼                 ▼                 ▼                  ▼
┌─────────────────────────────────────────────────────────────────────────────┐
│                        EXTERNAL DEPENDENCIES                                │
├─────────────┬─────────────┬─────────────┬─────────────┬────────────────────┤
│     Qt      │   Eigen3    │    Boost    │ orocos_kdl  │   GeometryLib      │
│ (GUI/3D)    │  (LinAlg)   │ (Math/Sys)  │ (Kinematic) │   Hauser10         │
└─────────────┴─────────────┴─────────────┴─────────────┴────────────────────┘

Data Flow Pipeline:
==================

Input Config → PathPlanner (RRT/RRT*) → Initial Path
                      ↓
Initial Path → MotionGenerator (STOMP) → Optimized Trajectory
                      ↓
Optimized Trajectory → TrajectoryEvaluator → Validated Result

Key Performance Features:
========================

🚀 Multi-threaded STOMP optimization
🎯 Real-time collision detection with BVH trees  
🔧 Modular cost function architecture (6 types)
🤖 Multi-robot support (Franka, Universal Robots, etc.)
⚡ SIMD-optimized linear algebra with Eigen
🔒 Medical-grade safety and validation
```

## Component Interaction Matrix

```
              Core  Motion  Planning  Robot  Utils  Visualization
Core            -     ✓       -       -      -         -
Motion          ✓     -       ✓       ✓      -         -
Planning        -     -       -       ✓      -         -
Robot           -     ✓       ✓       -      -         -
Utils           -     ✓       -       -      -         -
Visualization   -     -       -       -      -         -

Legend: ✓ = Direct dependency
```

## Algorithm Pipeline Visualization

```
STOMP Optimization Process:
==========================

1. Initial Path (from RRT)
   │
   ▼
2. Generate Noisy Trajectories (parallel)
   │
   ├─ Trajectory 1 ─┐
   ├─ Trajectory 2 ─┤
   ├─ Trajectory 3 ─┼─► Cost Calculation (6 cost functions)
   ├─ Trajectory N ─┤    • Collision Cost
   └─ ...          ─┘    • Smoothness Cost
                         • Joint Limit Cost
                         • Velocity Cost
                         • Acceleration Cost
                         • Custom Cost
   │
   ▼
3. Weight & Combine Trajectories
   │
   ▼
4. Update Current Trajectory
   │
   ▼
5. Convergence Check
   │
   ├─ No  ──► Return to Step 2
   │
   ▼ Yes
6. Optimized Trajectory Output
```

## Memory Layout Strategy

```
Memory Management Architecture:
==============================

Stack Memory:
┌─────────────────────┐
│ Local Variables     │  ← Fast access, automatic cleanup
│ Temporary Objects   │
│ Function Parameters │
└─────────────────────┘

Heap Memory (Smart Pointers):
┌─────────────────────────────┐
│ shared_ptr<RobotArm>       │  ← Shared ownership
│ unique_ptr<CostCalculator> │  ← Exclusive ownership  
│ vector<Trajectory>         │  ← Container management
└─────────────────────────────┘

Eigen Memory:
┌─────────────────────────────┐
│ Aligned Matrix Storage     │  ← SIMD optimization
│ Vectorized Operations      │  ← Performance critical
│ Cache-friendly Layout      │
└─────────────────────────────┘
```

## Thread Safety Map

```
Thread Safety Analysis:
======================

✅ THREAD-SAFE:
   • MotionGenerator (with thread pool)
   • PathPlanner (stateless algorithms)
   • TrajectoryEvaluator (read-only)
   • Utility functions (Core module)

⚠️  NEEDS SYNCHRONIZATION:
   • Robot state modifications
   • Cost calculator configurations  
   • Shared logging buffers

🔒 SYNCHRONIZATION METHODS:
   • std::mutex for robot state
   • std::atomic for counters
   • thread_local for RNG
```

This visual summary provides a quick reference for understanding the TrajectoryLib architecture at a glance, with clear diagrams showing the modular structure, data flow, and key technical characteristics.