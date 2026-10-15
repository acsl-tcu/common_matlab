AF7R31 / R13A — Source-bound batch front-end shadow
====================================================

This package is SHADOW/REPLAY ONLY. It does not patch REPLANNING_SYSTEM.m.

Rationale
---------
R12 proved that moving only the point/segment support formulas to tiny MEX
calls does not remove the 10 s bottleneck. The latest production source still
evaluates the exact nominal flat jet by repeated scalar function-handle
dispatch and then crosses MATLAB/MEX for each support query.

R13A compiles the CURRENT exact nominal flat derivative expressions into one
source-bound batch MEX and batches the R10A episode / R10B corridor geometry.

The breadth traversal changes only evaluation order. Every interval keeps the
same midpoint, half-width, speed inflation, support predicate, and fail-closed
UNSAFE rule at width <= live dt. No sampled acceptance is introduced.

Run after a completed run_flight_headless containing a real trigger
------------------------------------------------------------------
Add this package folder to the MATLAB path, then:

S31 = AF7R31_SOURCE_FORENSIC_AUDIT();

G31 = AF7R31_BUILD_GEOMETRY_BATCH_MEX();

F31 = AF7R31_BUILD_SOURCE_BOUND_FLAT_BATCH_MEX(replanner_instance);

E31 = AF7R31A_FLAT_BATCH_EQUIVALENCE_AUDIT(replanner_instance);
% Require E31.closed == 1

B31 = AF7R31B_TRUE_TRIGGER_EPISODE_CORRIDOR_SHADOW_AUDIT(replanner_instance);
% Require B31.closed == 1

Send the complete S31 / E31 / B31 output back before promotion.

Notes
-----
- MATLAB Coder is required only for the source-bound flat batch MEX.
- No current obstacle count, position, trigger time, speed, dt, mass, cable
  length, sensor range, horizon or HLC gain is embedded in the algorithm.
- The flat MEX is source-bound. If the nominal reference law changes, rebuild
  it and rerun equivalence.
- R13A does not yet replace quadprog, R9 Stage-B, rejoin, or the final
  continuous whole-body certificate. Those are the next native-core stage
  after this real-trigger dual audit closes.
