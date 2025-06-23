# Landmark-Based Registration Report

Generated: 2025-06-18 22:23:34

## Executive Summary

Force-based landmark registration for knee ultrasound trajectory planning.
Robot targets anatomical landmarks to establish model-to-reality correspondence.

## Landmark Detection Results

- **Total landmarks targeted**: 6
- **Successfully detected**: 4
- **Detection rate**: 66.7%

### Individual Landmark Results

**patella_apex** ✅
- Force: 2.08N
- Detection error: 2.25mm
- Reliability: high

**medial_femoral_condyle** ✅
- Force: 2.16N
- Detection error: 4.35mm
- Reliability: high

**lateral_femoral_condyle** ❌
- Force: -0.18N
- Reliability: high

**tibial_tuberosity** ✅
- Force: 1.94N
- Detection error: 4.71mm
- Reliability: medium

**medial_tibial_plateau** ✅
- Force: 2.00N
- Detection error: 3.33mm
- Reliability: medium

**lateral_tibial_plateau** ❌
- Force: -0.05N
- Reliability: medium

## Registration Quality

- **Registration error**: 5.23mm
- **Total uncertainty**: 19.86mm
- **Confidence score**: 0.000
- **Clinical ready**: No

## STOMP Integration Parameters

- **collision_cost_weight**: 74.6442
- **smoothness_cost_weight**: 9.9644
- **safety_margin_expansion**: 0.0397
- **convergence_threshold**: 0.0005
- **uncertainty_penalty_weight**: 0.1986

- **Recommended safety clearance**: 59.7mm

## Recommendations

- Registration error too high - retarget landmarks
- Low confidence - add more landmarks

## Clinical Implications

- Registration quality directly impacts trajectory planning safety
- Force-based detection provides more reliable landmarks than visual methods
- Uncertainty estimates enable adaptive planning strategies
- Real-time registration updates possible during procedure
