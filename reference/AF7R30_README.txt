AF7R30 / R12 native support + tail-prime
=============================================

Purpose
-------
1. Move repeated exact-support calculations into a tiny C++ MEX.
2. Move the R10 projected upper-envelope reduction into the same MEX.
3. Re-prime reference/certificate/geometry in the last Takeoff cycle.
4. Keep time-allocation optimization theory-only until the normalized-time
   theorem is valid for the current physical-velocity QP.

Run order
---------
1) B30 = AF7R30_BUILD_R12_NATIVE_MEX_V2();

2) E30 = AF7R30A_NATIVE_EQUIVALENCE_AUDIT();
   Require E30.closed == 1.

3) I30 = AF7R30_INSTALL_R12_NATIVE_AND_TAILPRIME();

4) dbclear all
   clear classes
   clear functions
   rehash

5) S30 = AF7R30B_R12_SOURCE_AUDIT();
   T30 = AF7R30C_TIME_ALLOCATION_FOUNDATION();
   Require S30.closed == 1 and T30.closed == 1.

6) run_flight_headless

7) Existing audits:
   T25 = AF7R25A_HIGHRES_TIMING_AUDIT(logger);
   T24 = AF7R24A_R9_SYNTHESIS_TIMING_AUDIT(logger);
   R28 = AF7R28_BLUE_SPIKE_ATTRIBUTION(runtime_profile,logger);

If AF7R23A remains available:
   T23 = AF7R23A_R9_STAGE_TIMING_AUDIT(logger).

No sampled acceptance, no obstacle/count/speed/timing hardcoding, no deadline
relaxation, no sign-candidate reduction, and no change to final continuous
whole-body authority.

If callback remains >4 ms after R12, the next step is one native kernel for
candidate local-vector speed + adaptive whole-body certificate. The simple
T^-r time-scaling law is NOT activated in production because the current QP
uses physical component velocity variables; changing T changes normalized
shape, so a coupled theorem is required first.
