AF7R58ZZY-R26-R3 FULL AUTHORITY RECOVERY
========================================

Why this package exists
-----------------------
The latest log proved that MATLAB was resolving:
  ...\reference\tp...\sources\run_flight_headless.m
instead of the canonical:
  ...\common_matlab\run_flight_headless.m

That diagnostic source snapshot has the old runner SHA:
  22a4b5fa6d5c2ccfc9a3798780782535e101424173e5eec81c8ddfb9c6f49949

So the problem is path/source authority, not merely one failed assignment.

This recovery:
- finds the real common_matlab repo by immutable SHAs, not by which(runner)
- removes tp*/source-snapshot and backup shadow directories from the active path
- changes pwd to canonical common_matlab
- restores exact reviewed R26-R3 class/report
- rebuilds the runner from exact R25 runner SHA
- applies the R26 four-stage binding and clean-profile NaN fallback
- binary-writes the runner to prevent CRLF corruption
- verifies every active class/helper path and SHA
- parses R26 and checks the runner before allowing Flight

Expected canonical final runner SHA
-----------------------------------
36a5adb8ff7846d175d30d5c59017617634aef24ffd06ad285f4eb355fdf4d98

Expected R26-R3 class SHA
-------------------------
2aa8f4e0c3ba2d272b8b5f2d7cfd921654c5f312d5f6ded5300988235ea64cd6

Run
---
AF7R58ZZY_R26_R3_FULL_AUTHORITY_RECOVERY

Only after SELFTEST CLOSED = 1:
  which run_flight_headless -all
  run_flight_headless
  AF7R58ZZY_R26_R3_REPORT

Return
------
AF7R58ZZY_R26_R3_REPORT.txt
AF7R58ZZY_R26_R3_OUT.mat

Do not run the old EOL/profile repair scripts again.
