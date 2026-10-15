AF7R31 R13A — NO-MATLAB-CODER repair
=====================================

Why the previous builder failed
-------------------------------
The previous R13A builder used MATLAB Coder's codegen command.  This machine
does not have MATLAB Coder available.  The geometry MEX build already proves
that the ordinary C++ MEX compiler (Microsoft Visual C++ 2022) is available.

V2 therefore removes the MATLAB Coder dependency completely.

How V2 remains source-bound and general
---------------------------------------
It reads the CURRENT:
    replanner.nominal_flat_derivative_fn{1:7}

For every handle it reproduces the current REPLANNING_SYSTEM symbolic parsing:
    @(t)<body>
    .^ -> ^
    .* -> *
    ./ -> /
    str2sym(<body>)

It then applies Symbolic Math Toolbox ccode() to each of the 21 exact scalar
flat-jet expressions and embeds those expressions in one ordinary C++ MEX.

No current line velocity, direction, obstacle position/count, trigger time,
dt, mass, cable length, sensor range, horizon, or HLC gain is hard-coded.

The SHA-256 signature of the exact seven current derivative handles is stored
in AF7R31_FLAT_BATCH_META.mat.  The existing AF7R31A audit compares that
signature against the live replanner and then numerically compares MATLAB and
MEX values.

Run now
-------
Do NOT rebuild the geometry MEX; it already passed.

Run:

F31 = AF7R31_BUILD_SOURCE_BOUND_FLAT_BATCH_MEX_V2(replanner_instance);

E31 = AF7R31A_FLAT_BATCH_EQUIVALENCE_AUDIT(replanner_instance);

Require:
    E31.closed == 1

Only then run:

B31 = AF7R31B_TRUE_TRIGGER_EPISODE_CORRIDOR_SHADOW_AUDIT(replanner_instance);

Require:
    B31.closed == 1

No REPLANNING_SYSTEM.m production modification occurs in any of these steps.
