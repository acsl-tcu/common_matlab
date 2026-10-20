AF7R58ZZY-R28 NATIVE-ONCE DEVELOPMENT PACKAGE
===============================================

Purpose
-------
R27 moved 6.5116 ms of compile work into Stage1 and therefore did not reduce
end-to-end cost. R28 removes that second runtime compiler. The AF7R36 planning
MEX now emits the R23-compatible compiled derivative representation directly
from the same PlanResult candidate after plan() has returned.

Theory / noninterference
------------------------
1. af7r36_core.hpp is byte-identical to the exported authority
   SHA ad5ce1970583c0313637dcac58b8cf949092fe53e201efc61e55754fb2ea79be. Therefore episode construction, candidate synthesis, speed
   certificate and whole-body geometry certificate mathematics are unchanged.
2. The wrapper still calls PlanResult Q=plan(...) before compiled representation
   construction. The new representation cannot make an unsafe candidate pass.
3. For degree n=15 and segment duration T, derivative control points use
       Q^(r+1)_i = ((15-r)/T) * (Q^(r)_(i+1)-Q^(r)_i), r=0..5.
   R28 copies the exact R25 implementation: subtraction is forced into a
   volatile double before the separate multiplication. Installer selftest
   compares D0..D6 bitwise against af7r77_compile_dcp_mex.
4. Existing groups are retained iff group_t1 > t_commit, exactly the complement
   of AF7R58I prune t1<=t_commit. Their 3x16x7 derivative blocks are memcpy'd
   bitwise without recomputation.
5. The new group's D0 comes directly from the same PlanResult::cand.cp also
   exported as RAW.control_points. D1..D6 are calculated once from that same
   native memory.
6. Installer also exercises a synthetic base with (a) a group ending before
   commit, (b) a group ending exactly at commit, and (c) a surviving group, and
   requires the entire R23-schema packet to be bitwise identical to the R27
   reference implementation. No external MAT/capture/test data are loaded.
7. Runtime Stage3 keeps the exact R21 canonical/resource/continuous certificate
   path. It only replaces R27's deep control-point reconstruction with a
   metadata/version/support binding. Exact full-array equivalence follows
   inductively from the committed base invariant + identical prune + same native
   candidate append.
8. Compiled representation is NOT acceptance authority, but it is runtime
   execution representation; therefore source revision, reference_version,
   resource counts, appended group support and segment supports are fail-closed.

Unchanged safety authorities
----------------------------
- H = 4 s
- C7 degree-15 candidate
- continuous speed / acceleration / jerk / specific-force / qdot bounds
- continuous whole-body payload/UAV/cable obstacle certificate
- AF7R58I resource contract
- freshness/stale rejection and atomic rollback
- controller, estimator, plant

Scheduling
----------
R28 intentionally leaves the four observed times / three-interval activation
unchanged. This is an A/B measurement choice only. If every controller-excluded
research cycle is <=17 ms, the next revision should collapse the artificial
75 ms activation delay rather than keep the four-stage schedule.

MEX self-contained final form
-----------------------------
This R28 development package still uses an installer and the existing R25/R27
reference helpers for equivalence testing. They are not runtime acceptance
dependencies. After timing/theory closes, the final replanner package will carry
its own reviewed C++ sources and build the required MEX binaries from those
sources only. It will not read old MAT files, reports, captures or test results
to operate, and will never compile during Flight.

Run
---
AF7R58ZZY_R28_INSTALL
AF7R58ZZY_R28_RUN_CANONICAL
AF7R58ZZY_R28_REPORT

Return AF7R58ZZY_R28_REPORT.txt and AF7R58ZZY_R28_OUT.mat.
