AF7R31 R13A V3 — no MATLAB Coder, no ccode
================================================

The machine has:
- ordinary C++ MEX compiler: YES
- current Symbolic Math Toolbox setup path: YES (the replanner itself uses
  str2sym and matlabFunction)
- MATLAB Coder/codegen: unavailable
- Symbolic ccode: unavailable

V3 therefore does not build a flat MEX.

Instead, it keeps the exact current nominal_flat_derivative_fn authority and
reduces dispatch cost in MATLAB:
1. If a derivative handle accepts vector time input, V3 calls that original
   handle once for the whole batch.
2. Otherwise, V3 reconstructs the same symbolic 3-vector with str2sym and
   creates three scalar matlabFunction handles. Each scalar expression is
   vectorized, so the batch needs only three calls for that derivative order.

This changes no trajectory formula and no safety theorem.

Run now
-------
The geometry batch MEX is already built; do not rebuild it.

Run:

E31 = AF7R31A_FLAT_BATCH_EQUIVALENCE_AUDIT_V3(replanner_instance);

Require:
    E31.closed == 1

Then:

B31 = AF7R31B_TRUE_TRIGGER_EPISODE_CORRIDOR_SHADOW_AUDIT_V3(replanner_instance);

Require:
    B31.closed == 1

Both are shadow/read-only. REPLANNING_SYSTEM.m is not modified.

What to send back
-----------------
Send the complete E31 and B31 outputs. E31 will also report:
- how many of the 7 derivative handles are directly vector-input compatible,
- batch timing for 2048 times,
- legacy 7*N scalar-dispatch timing for the same 2048 times.

This tells us whether source-bound batching is actually worth promoting before
we touch production.
