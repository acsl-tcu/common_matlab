AF7R37 / R14D production integration
=====================================

Prerequisite already achieved
-----------------------------
AF7R36A closed:
- canonical MATLAB / C++ decision equal
- episode boundaries equal
- segment times equal
- sign vectors equal
- control-point max difference ~7e-14
- speed certificate difference ~3e-14
- geometry difference ~2e-14
- repeated-replan evaluator exact
- native wrapper median ~0.96 ms
- native C++ core median ~0.30 ms

What this installer changes
---------------------------
1. For the DEFAULT internal BEZIER_D15_C7 backend only:
       old: @(CTX)obj.builtin_bezier_d15_c7_proposal(CTX)
       new: @(CTX)af7r36_single_core_proposal(obj,CTX)

   Explicit user-registered callbacks are NOT overwritten.

2. Before freezing the default backend, it runs:
       af7r36_runtime_prime(obj)
   which compiles and source-checks the current nominal flat AST.

3. The existing pre-motion:
       prime_locked_backend_kernel_variants(...)
   remains intact and therefore warms the ACTUAL R14D callback before Flight.

4. MATLAB's bezier_lateral_separator_normal() is changed to call the same
   explicit canonical degenerate-eigenspace rule. The old MATLAB builtin
   planner remains in the class for audit/fallback work.

Not changed
-----------
- hard 4 ms deadline
- R9 Stage A/B theorem
- strict rejoin theorem
- continuous whole-body theorem
- sampled acceptance remains forbidden
- repeated-replan correction-chain contract
- commit policy / automatic-commit policy
- baseline reference/controller behavior when avoidance is not engaged

Run order
---------
1) Install:

    I37 = AF7R37_INSTALL_R14D_PRODUCTION();

2) Reload class/runtime:

    dbclear all
    clear classes
    clear functions
    rehash

3) Source audit:

    S37 = AF7R37A_R14D_PRODUCTION_SOURCE_AUDIT();
    S37.closed

Require 1.

4) Fresh run:

    run_flight_headless

5) Timing:

    T25 = AF7R25A_HIGHRES_TIMING_AUDIT(logger);
    R28 = AF7R28_BLUE_SPIKE_ATTRIBUTION(runtime_profile,logger);

    if exist('AF7R23A_R9_STAGE_TIMING_AUDIT','file')==2
        T23 = AF7R23A_R9_STAGE_TIMING_AUDIT(logger);
    end

The decisive authority is now the fresh production QPC callback/cycle timing,
not the shadow replay timing.

Expected interpretation
-----------------------
If request-edge QPC callback is <4 ms and status changes from -20 to the
reference-only commit / certified noncommit status dictated by the current
commit policy, the generator deadline problem is closed.

Do NOT interpret Windows/MATLAB measurements as an OS-level WCET theorem.
The project contract is still a measured <=4 ms online planner deadline on the
current platform, later rechecked on the onboard ROS2/Jetson target.

If the callback is <4 ms but commit cycle still exceeds 4 ms, the next and
ONLY target is post-callback reference-certificate/atomic-commit plumbing.
The heavy trajectory-generation core must not be redesigned again.
