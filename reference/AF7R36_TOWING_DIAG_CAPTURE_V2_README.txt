AF7R36 TOWING DIAGNOSTIC CAPTURE V2
===================================

Why V2 exists
-------------
V1 correctly changed only diagnostic fields inside the native planner, so the
trajectory was expected to remain unchanged. However, V1 incorrectly assumed a
workspace variable named R would still exist after the full simulation.

The current staged pipeline stores candidate.native_raw only transiently and
clears staged_replan_candidate after the pipeline completes. V2 therefore
persists the already-computed diagnostic RAW struct in the existing
REPLANNING_SYSTEM.generator_last_status property. It is diagnostic storage only.
It is not read by synthesis, certificates, candidate ordering, rejoin selection,
or commit.

Run once before the next simulation
-----------------------------------
OUT = AF7R36_TOWING_DIAG_CAPTURE_V2_INSTALL;
clear classes
clear mex
rehash

Then run exactly the same 50 s simulation.

After "Simulation complete! Drawing plots..."
---------------------------------------------
S = AF7R36_TOWING_ROOTCAUSE_POSTRUN;

If auto-resolution does not find agent:
S = AF7R36_TOWING_ROOTCAUSE_POSTRUN(agent);

Return the entire printed block, especially:
  DIAGNOSTIC CLASSIFICATION
  finite-sensing UAV-speed limit
  minimum Stage-A UAV-speed UB
  associated correction |d_dot| UB
  associated correction |d_ddot| UB
  associated correction |d_jerk| UB
  Stage-B finite speed-witness rejects
  Stage-B representation/inconclusive
  minimum primary correction envelope

Expected trajectory behavior
----------------------------
The trajectory MUST remain numerically unchanged in this diagnostic step.
If it changes, stop: that would violate the intended diagnostic-only contract.
