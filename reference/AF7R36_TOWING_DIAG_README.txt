AF7R36 towing root-cause diagnostic (V1)
========================================

Goal
----
Keep the current planner/certificates unchanged and identify exactly why the
MINIMUM_DEVIATION_AFFINE family fails the speed/flatness gate before the
MINIMUM_SNAP_FALLBACK is selected.

Files
-----
1) AF7R36_TOWING_DIAG_INSTALL.m
   Source-bound diagnostic-only patch + MEX rebuild.
2) AF7R36_TOWING_ROOTCAUSE_PRINT.m
   Prints the new diagnostics from the exact same test result.

Run
---
addpath('<folder containing these two files>');
OUT = AF7R36_TOWING_DIAG_INSTALL;

Then rerun the exact same scenario/audit that previously produced:
  candidate_family = MINIMUM_SNAP_FALLBACK
  primary_candidate_count = 2
  primary_speed_pass_count = 0

Then call:
  S = AF7R36_TOWING_ROOTCAUSE_PRINT(R);

If your test returns the proposal rather than RAW directly, the print function
also accepts a struct containing R.native_raw.

Safety
------
This V1 patch does NOT alter:
- candidate generation
- candidate ordering
- adaptive rejoin selection
- speed/rejoin/geometry acceptance conditions
- whole-body continuous certificate
- C7/C6 structure
- fallback behavior
- sampled acceptance policy

It only records rejection diagnostics already computed by the existing hard
speed certificate. A Stage-B finite witness is used only to prove rejection,
never to accept a trajectory.

Next decision after the result
------------------------------
A) Finite speed witness exists:
   the primary trajectory is genuinely too aggressive. Next patch should add a
   certified-candidate homotopy between minimum-deviation and the legacy
   minimum-snap endpoint, while preserving the legacy fallback exactly.

B) No finite witness, representation/inconclusive occurs:
   fix the certificate representation/tightening first; do not change trajectory
   synthesis yet.

C) Primary speed passes at a slightly later fixed t1:
   prefer a towing-aware adaptive-rejoin quality rule instead of changing the
   spatial synthesis family first.
