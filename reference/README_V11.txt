AF7R57N V11 — TailSafe finite-resource closure
================================================

Why V11 is required
-------------------
V9 proved that production op=4 executes certify_tail_safe(...) before
plan_fixed_episode. V10 guarded episode/sign/QP/candidate/Stage-A/Stage-B/
geometry resources, but did not cap TailSafe's own adaptive leaves/splits.
Therefore production cap selection must not begin until TailSafe is included
in the finite resource vector.

V11 adds only resource accounting / fail-closed transport:
  216 = TailSafe leaf budget exhausted
  217 = TailSafe split budget exhausted

It does NOT change the TailSafe safety inequality, whole-body geometry,
Stage-B mathematics, C6/C7 synthesis, rejoin theorem, or production source.

Immutable entrypoint
--------------------
Run only:
    N57V11 = AF7R57N_V11_TAILSAFE_RESOURCE_CLOSURE_AUDIT();

Generated assets are version-unique:
    af7r57n_v11_trace_core.hpp
    af7r57n_v11_guarded_core_mex.cpp
    af7r57n_v11_guarded_core_mex

Entrypoint SHA-256:
    7981eafa82b14406c48b9607f0811236c1f24ba6d6791577163a3e012bc1acf3

Expected closure
----------------
- current theorem exact identity = 1
- TailSafe leaves/splits are reported in discovery
- 216/217 fault injection closes with ordinary RETURN when exercised
- all exercised resource reasons match
- exception-free transport = 1
- current success + all exercised resource reject paths remain <4 ms
- production resource caps selected = 0
- production-general <=4 ms proved = 0

Only after this closes should the source-bound QPC stress envelope be built.

Runtime-closure note
--------------------
V11 does not silently count a zero-use TailSafe split path as exercised. If current TailSafe splits are zero, V11 leaves the TailSafe runtime/timing gate open and explicitly requests a source-bound split-excitation audit.

Final entrypoint SHA-256:
    37d888f04ba694b40a32865af016e433ec83ae2c7df253585f03262a6cbde3d6
