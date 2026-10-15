AF7R34 / R14 — SINGLE-CALL NATIVE REPLANNING CORE (shadow first)
=================================================================

This is the architecture change requested after R13C.

Unlike R12/R13 micro-accelerators, the R14 C++ core contains the complete
heavy reference-layer chain in ONE native call:

    active flat reference
    -> adaptive unsafe episode cover
    -> corridor construction
    -> exact +/- sign enumeration
    -> exact small convex QP
    -> degree-15 C7 control points
    -> R9 Stage-A / Stage-B speed certificate
    -> strict rejoin certificate
    -> continuous whole-body certificate

The MATLAB side still performs basic request/context validation and packs the
numeric packet. That outer work was ~0.7 ms in the previous measurements and
is intentionally kept separate from the heavy native core.

Important safety properties
---------------------------
- Same degree-15 / C7 piecewise towing structure.
- Same affine unsafe component theorem.
- Same continuous interval episode predicate.
- Same exact sign disjunction; no sign bank.
- Same R9 Stage A then Stage B logic.
- Same strict rejoin test.
- Same midpoint-static-support-minus-speed-times-halfwidth whole-body theorem.
- No sampled safety acceptance.
- No fixed adaptive depth cap.
- No heuristic QP node cap.
- R13C repeated-replan model is consumed explicitly.
- The hard 4 ms deadline is NOT changed.

Nominal reference inside the C++ core
-------------------------------------
No MATLAB Coder and no Symbolic ccode are required.

During setup, the CURRENT 21 scalar nominal flat-jet expressions are obtained
from nominal_flat_derivative_fn{1:7}, normalized through str2sym, and compiled
into a persistent C++ expression AST.

The AST parser supports ordinary arithmetic, powers, parentheses and common
scalar functions such as sin/cos/tan, exp/log/sqrt/abs, hyperbolic functions,
atan2, min, max and pow.

AF7R34_PRIME_SINGLE_CORE_REFERENCE performs a 257-time direct C++/MATLAB
equivalence test before the planner audit. Unsupported expression syntax fails
closed before any production modification.

R13C repaired-on-repaired references
------------------------------------
The numeric repair-chain control points are passed to the core. Therefore the
base flat reference evaluated inside C++ is:

    current nominal flat jet + sum(previous committed repair jets)

The audit additionally builds a real one-repair model from the stored true
trigger proposal and compares the C++ evaluator against the AF7R33 MATLAB
batch model. This is specifically to prevent the old NOMINAL-only problem from
returning later.

Run order
---------
No fresh run_flight_headless is needed yet. Use the stored real first trigger
from the R13C run you just measured.

1) Add this package folder to the path.

2) Audit prerequisites:

    S34 = AF7R34_SOURCE_CONTRACT_AUDIT();

Require:
    S34.closed == 1

3) Build the one-call native core:

    B34 = AF7R34_BUILD_SINGLE_CORE_MEX();

4) Prime/verify the source-bound nominal AST:

    P34 = AF7R34_PRIME_SINGLE_CORE_REFERENCE(replanner_instance);

Require:
    P34.closed == 1

5) Run the complete true-trigger dual audit:

    A34 = AF7R34A_TRUE_TRIGGER_SINGLE_CORE_AUDIT(replanner_instance);

Require:
    A34.closed == 1

The audit reports both:
- full mathematical/result equivalence against the current production planner;
- native wrapper and pure C++ core timing.

DO NOT install a production R14 fast path yet.
This package intentionally stops at full-core shadow equivalence. Once A34 is
closed, production promotion is a small patch, whereas debugging a large
native core after putting it on the Flight path would be the wrong order.

The key result to send back is the COMPLETE A34 output, especially:

    native wrapper min/med/max [ms]
    native C++ core min/med/max [ms]
    last core episode/hess/corr/QP/build [ms]
    last core candidate / total [ms]
    single-core equivalence closed

If the C++ core is already below 4 ms with equivalence closed, the next step is
production promotion + fresh QPC flight run.

If equivalence closes but the core remains above 4 ms, the stage timings now
come from INSIDE one C++ call, so the remaining target is finally unambiguous;
we will no longer spend time moving isolated MATLAB functions to separate MEX
files.
