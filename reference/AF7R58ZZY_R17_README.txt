AF7R58ZZY-R17
==============
Purpose:
  Fix the R16-R2 Stage2 miss (9.7973 -> 9.6844 ms only).

Root cause of R16-R2 miss:
  1) exact runtime-packet reuse normally does not close on the first repaired reference
     because the new repaired speed certificate differs from the active nominal one;
  2) R16-R2 replaced vector nominal evaluation with 322 scalar symbolic calls
     (161 times x position+acceleration), cancelling the derivative-order reduction.

R17 change:
  - Keep the production nominal table build.
  - Retain the exact acceleration vector that production already computes at every table point.
    No new derivative evaluation is added to pre-mission table generation.
  - Stage2/postcommit use O(1) nominal table slices and evaluate only correction orders r=0,2.
  - Observer MEX and strict-FP de Casteljau MEX remain unchanged.
  - REPLANNING_SYSTEM.m, controller, plant, trajectory math, certificate acceptance and
    commit policy are unchanged.

Install:
  AF7R58ZZY_R17_INSTALL

Run:
  run_flight_headless

After run:
  S = replanner_instance.get_staged_pipeline_status();
  R = replanner_instance.get_r17_performance_status();
  disp(S); disp(R);

Rollback:
  AF7R58ZZY_R17_ROLLBACK
