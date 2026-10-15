AF7R32 / R13B — exact small-QP + whole-body continuous shadow
================================================================

This stage does NOT patch REPLANNING_SYSTEM.m.

It contains two independent shadow accelerators:

1. Exact small-QP MEX
   Problem:
       min 0.5 z' H z
       s.t. A z >= b

   For SPD H, an optimum has an independent active set of at most n
   constraints. The MEX enumerates every active subset of cardinality
   1..min(n,m), solves the equality KKT equations, checks lambda >= 0 and the
   full inequality set, and returns the minimum objective candidate.

   There is no node cap and no heuristic pruning. This is intentionally a
   SHADOW theorem/equivalence implementation first. If H is not SPD it fails
   closed instead of regularizing H.

2. Breadth-batched whole-body certificate
   It preserves the existing theorem:
       static midpoint support margin m
       certified speeds V
       strict interval gate h < m/V

   It evaluates multiple independent leaves together using the already-built
   AF7R31 geometry MEX. The corrected suspended-load geometry remains:
       q* = -(pL_ddot* + g e3)/||pL_ddot* + g e3||
       pQ* = pL* - L q*

   There is no sampled acceptance and no fixed-depth authority.

Dependencies
------------
Keep the AF7R31 V3 folder on the MATLAB path. In particular:
- af7r31_flat_jet_batch_v3.m
- af7r31_episode_batch_v3.m
- af7r31_corridor_batch_v3.m
- af7r31_geometry_batch_mex.mexw64

Run now
-------
S32 = AF7R32_SOURCE_AUDIT();

Q32 = AF7R32_BUILD_EXACT_QP_MEX();

A32 = AF7R32A_TRUE_TRIGGER_QP_SHADOW_AUDIT(replanner_instance);
% Require A32.closed == 1.

B32 = AF7R32B_TRUE_TRIGGER_WHOLEBODY_SHADOW_AUDIT(replanner_instance);
% Require B32.closed == 1.

No new run_flight_headless is needed for these shadow audits. They replay the
stored real first Flight trigger from the existing run.

Send the full A32 and B32 outputs back before any production promotion.

What B32 also measures
----------------------
It separately times, on the same certified candidate:
- derivative-bound tightening,
- R9 Stage-B local-vector speed certificate,
- continuous rejoin certificate,
- legacy whole-body certificate,
- batch whole-body certificate.

That tells us which remaining candidate stage must enter the next native core.

Production promotion policy
----------------------------
Do not install these paths into the online generator until:
- QP feasible/infeasible decisions match,
- z/cost/slack equivalence closes,
- whole-body mathematical pass and all important continuous-certificate
  diagnostics match,
- real-trigger timing demonstrates a material reduction.

The final 4 ms authority still requires a production run with instrumentation
reduced to outer QPC and later target-Linux/onboard measurement.
