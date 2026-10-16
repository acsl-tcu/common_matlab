AF7R44 — two-track current-system package
=========================================

This package addresses only the current simulation.

TRACK 1 — sensor-only admission + retained obstacle memory
----------------------------------------------------------
- The replanner-known obstacle set starts EMPTY.
- The private simulation truth map is used only by the simulated sensor gate.
- An obstacle becomes planner-known only after current sensor detection.
- A detected static obstacle is retained across deadline retries and through
  the certified repaired-reference interval.
- It is released only after the certified rejoin has passed, it is not
  currently sensed, and no request identity remains latched.
- No arbitrary fixed retention duration is introduced.
- Synthetic preflight/forced-hit calls cannot create persistent knowledge.
- Initialization cha='f' calls before the Flight time origin cannot create
  persistent knowledge.

TRACK 2 — broad current replanner timing / "real-machine" layers
----------------------------------------------------------------
AF7R44B is post-run/read-only. It adds no timers to the online path.

It reports the 9.75--10.30 s window with:
- whole online-before-plant time;
- whole REPLANNING_SYSTEM time;
- observer hard-support runtime;
- dependency/binding stage;
- base/certificate/geometry stage;
- sensor/trigger/replan stage;
- output/state stage;
- generator hard-critical time;
- current sensor obstacle count;
- retained-known obstacle count.

It also resolves the current switches:
- observer hard-support is independently ON when configured ON;
- ENABLE_EXECUTION_AWARE_CERTIFICATE may be true;
- while AVOIDANCE_COMMIT_POLICY='REFERENCE_ONLY', the R38 policy defers the
  full execution-aware certificate from the hot path;
- execution_aware_evaluation_count is the direct runtime evidence of whether
  the full execution-aware certificate actually ran.

Run
---
    addpath('AF7R44 package folder','-begin');
    I44 = AF7R44_INSTALL_SENSOR_MEMORY();

    dbclear all
    clear classes
    clear functions
    rehash

    A44 = AF7R44_PRE_RUN_AUDIT();
    A44.closed

Require 1.

Fresh run:
    run_flight_headless

Then:
    B44 = AF7R44B_TWO_TRACK_POSTRUN_AUDIT(logger,replanner_instance);

Also:
    T25 = AF7R25A_HIGHRES_TIMING_AUDIT(logger);
    R28 = AF7R28_BLUE_SPIKE_ATTRIBUTION(runtime_profile,logger);
    T23 = AF7R23A_R9_STAGE_TIMING_AUDIT(logger);

Send B44 in full with T25/R28/T23.

Hashes
------
AF7R43 input:
52f2e46a73c5e35aec5e436a4484dd7663564e867374a017869e604ae04842c1

AF7R44 output:
67c026dd021e26f7fcd7eefb9c1ffa9aa9023b77dd64525ae674fdaae83011d9
