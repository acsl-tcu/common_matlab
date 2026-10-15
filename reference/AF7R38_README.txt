AF7R38 — preflight cold-path burn + policy-only hot path
========================================================

Why this patch exists
---------------------
The R14D planner DID speed up. In the fresh production run:
- QPC callback median is about 2.0 ms, max 3.17 ms.
- the C++ heavy internal planner is about 0.35 ms.
- old MATLAB episode/synthesis/candidate work is gone from the hot path.

The remaining large blue bars come from two different regions:
1) Flight entry (t=5.000, 5.100, 5.125): no generator request exists there.
   The observer add-on is intentionally excluded from the old synthetic
   setup exact-do prime, so its actual first-event / consecutive-event code
   is still cold at Flight entry. Downstream cert/geometry/output code also
   sees observer-supported state for the first time there.

2) Replan request edge:
   REFERENCE_ONLY is the active commit policy, but the old hot path still
   evaluates the non-authoritative EXECUTION_AWARE diagnostic layer before
   the reference-only commit. That work can consume the 4 ms deadline even
   though it is not a REFERENCE_ONLY safety prerequisite.

What R38 changes
----------------
A. Preflight observer/full-do dry run
- During the existing exact Flight do() prime, R38 temporarily enables the
  observer for exactly two structural cases:
  * first measurement event (hasPrev=false)
  * consecutive contraction event (hasPrev=true)
- It runs the full protected do() path so cert/geometry/output first-use is
  also burned.
- observer_hard_support_internal is restored immediately after the dry run.
  Synthetic support NEVER becomes Flight authority.
- Actual Flight still initializes the observer from the actual same-event
  sensor/EKF/plant state.

B. QPC stamp prime
- af7r25_stamp_mex() is called before Flight so the first real request does
  not pay its first-use/JIT/loading cost.

C. Successful-candidate post-callback dry run
- The existing backend prime keeps the first genuinely successful native
  candidate, if one exists.
- Under REFERENCE_ONLY + auto commit, it executes the actual
  reference-only certificate + atomic commit plumbing with no deadline timer.
- Every mutable commit field is snapshotted and restored after the dry run.
- The setup candidate is never retained as authority.

D. Policy-only hot path
- Under REFERENCE_ONLY, execution-aware certification is not commit authority.
  It is deferred from the <=4 ms hot path.
- The reference-only certificate remains mandatory and unchanged.
- Under EXECUTION_AWARE, the previous execution-aware evaluation remains.

Not changed
-----------
- 4 ms hard deadline.
- R14D native generator and canonical separator.
- R9 Stage A/B.
- strict rejoin.
- continuous whole-body certificate.
- sampled acceptance remains forbidden.
- execution-aware requirements when EXECUTION_AWARE is selected.
- baseline reference/controller behavior outside avoidance.

Run
---
1)
    I38 = AF7R38_INSTALL_PREFLIGHT_AND_POLICY_HOTPATH();

2)
    dbclear all
    clear classes
    clear functions
    rehash

3)
    S38 = AF7R38A_SOURCE_AUDIT();
    S38.closed

Require 1.

4) Fresh:
    run_flight_headless

5)
    T25 = AF7R25A_HIGHRES_TIMING_AUDIT(logger);
    R28 = AF7R28_BLUE_SPIKE_ATTRIBUTION(runtime_profile,logger);
    T23 = AF7R23A_R9_STAGE_TIMING_AUDIT(logger);

Also print:
    replanner_instance.observer_preflight_prime_done
    replanner_instance.observer_preflight_prime_ms
    replanner_instance.qpc_stamp_prime_done
    replanner_instance.postcallback_prime_candidate_found
    replanner_instance.postcallback_prime_done
    replanner_instance.postcallback_prime_ms

What to expect
--------------
- The t=5.0-ish blue Flight-entry spikes should shrink substantially if they
  were the observer/cert first-use path identified by source structure.
- The first real callback tic/QPC discrepancy should shrink because the QPC
  stamp path is pre-used.
- Under REFERENCE_ONLY, the very large post-callback execution-aware
  diagnostic cost should disappear from the 4 ms path.
- If -30 remains after this, the remaining target is the actual atomic
  reference-only commit rebuild itself; the trajectory generator must not be
  redesigned again.
