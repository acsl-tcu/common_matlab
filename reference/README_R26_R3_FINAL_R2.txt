AF7R58ZZY-R26-R3 FINAL R2 PACKAGE
==================================

This supersedes every prior R26/R26-R2/R26-R3 recovery/install script.

Latest failure root cause
-------------------------
The previous FINAL_RECOVERY executed `clear classes` inside its own function.
MATLAB warned that STATE_CLASS could not be cleared, but the clear command also
removed local function variables, including `root`. The next `addpath(root,...)`
therefore failed. This was a recovery-script bug, not a replanner/trajectory bug.

R2 changes
----------
- NEVER calls `clear classes`.
- NEVER synthesizes/patches the runner at runtime.
- Installs the already-reviewed exact final runner by binary copy.
- Keeps the exact final runner SHA:
  cfa9f9d61b2762fdb899515b2a6d6ddf582d550c1eaf4046574e360b9d28fbe5
- Purges diagnostic/back-up path shadows and cd's to canonical common_matlab.
- Verifies active binding for runner, R26/R25/R21/R19, production, report,
  selftest, compile helper and fast merge.
- Uses a corrected canonical launcher that executes the runner by absolute path
  in BASE workspace without quote-string construction.
- Production REPLANNING_SYSTEM and controller sources are not modified.

Run ONLY
--------
1) AF7R58ZZY_R26_R3_FINAL_R2_RECOVERY
2) AF7R58ZZY_R26_R3_RUN_CANONICAL
3) AF7R58ZZY_R26_R3_REPORT

Do not run any older R26 installer/recovery/repair again.
