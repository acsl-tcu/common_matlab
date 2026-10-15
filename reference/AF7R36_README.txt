AF7R36 / R14D — Canonical degenerate-eigenspace separator
==========================================================

F35 proved the R14 mismatch is NOT a C++ eigensolver bug.

For the true trigger:
    progress = [0 0 4.75345...]'
    Qp top eigenvalues = [0.106666..., 0.106666...]
so the principal lateral eigenspace is exactly 2-D.

MATLAB eig() selected y.
The C++ Jacobi implementation selected x.
Both are exact principal eigendirections.

A unique coordinate-free direction cannot be extracted from an exactly
isotropic 2-D eigenspace without adding an extra tie-breaking structure.
R14D therefore makes that structure EXPLICIT and deterministic.

Canonical rule
--------------
For nonzero progress v:
1. build a deterministic orthonormal basis (b1,b2) of v-perp;
2. form the exact 2x2 symmetric projected metric;
3. solve the principal direction analytically;
4. if the two projected eigenvalues are binary64-indistinguishable, choose b1;
5. canonicalize vector sign by first nonzero world component.

For zero progress:
- use the known ellipsoid metric eigenspaces from R and a;
- construct the top eigenspace projector;
- select the first canonical world axis with nonzero projection.

The numerical ambiguity guard is not a physical clearance threshold and is
never used to accept safety. The R9, strict rejoin and continuous whole-body
certificates remain unchanged.

Equal-objective sign branch ordering
------------------------------------
When fixed-sign QP objective values are indistinguishable at binary64
resolution, -1 is ordered before +1 lexicographically. This affects only which
already-feasible branch is selected. It does not prune any sign and does not
change the disjunctive feasible union.

Run
---
Keep the previous R13/R14 helper files on the MATLAB path.

1)
    B36 = AF7R36_BUILD_CANONICAL_SINGLE_CORE_MEX();

2)
    P36 = AF7R36_PRIME_CANONICAL_SINGLE_CORE_REFERENCE(replanner_instance);
Require P36.closed == 1.

3)
    A36 = AF7R36A_CANONICAL_SINGLE_CORE_DUAL_AUDIT(replanner_instance);
Require A36.closed == 1.

The A36 comparison is no longer against the old noncanonical MATLAB eig()
trajectory. It compares:
    canonical MATLAB synthesis specification
vs
    canonical one-call C++ core

using the same current R9, strict rejoin, and whole-body theorem.

No production source is modified by this package.

If A36 closes and native wrapper/core timing remains below 4 ms, the next
step is production promotion of BOTH:
- the explicit canonical separator rule in MATLAB authority/fallback;
- the one-call native R14D core on the Flight hot path;
followed by a fresh run_flight_headless and QPC audit.
