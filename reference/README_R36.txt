AF7R36 CURVED SPATIAL C7 PRIMARY

run_flight_headless.m is NOT modified.

R35 RESULT
R35 closed the yaw theorem at candidate level:
  18 / 18 heading candidates passed,
  gamma > 0.
No candidate was selected only because translation still failed.

R36 CHANGE
The family-7 unsafe-span spatial correction is no longer one affine line
P+V(s-sA).

It is now two degree-15 Hermite pieces:
  (P0,V0) -> (PM,VM) -> (P1,V1)

At every spatial node, derivatives 2..7 with respect to nominal progress are
exactly zero. Therefore the spatial correction is C7 while retaining a real
internal position and tangent.

The midpoint is the mathematical midpoint of the current unsafe progress span.
It is not an obstacle-radius constant.

All spatial node states live in the plane perpendicular to nominal progress.
The QP metric is the already-used suspended-load natural-time C3 metric based
on tau=sqrt(L/g); no arbitrary smoothing weight is added.

R34 no-catch-up delay preservation is retained.
R35 path-tangent yaw C6 is retained.
Exact stored-C7 / monotone progress / finite-sensing speed / rejoin /
continuous whole-body remain hard gates.

The synthesis upper-hull rows are proposal generation only. They never replace
continuous safety acceptance.

FILES TO REPLACE
  REPLANNING_SYSTEM.m
  af7r36_core.hpp
  af7r36_single_core_mex.cpp

Do NOT replace run_flight_headless.m.

MEX REBUILD -- REQUIRED

REF = fileparts(which('af7r34_make_core_packet'));

clear mex

mex('-R2018a','-O', ...
    'COMPFLAGS="$COMPFLAGS /std:c++17"', ...
    ['-I' REF], ...
    '-output','af7r36_single_core_mex', ...
    '-outdir',REF, ...
    fullfile(REF,'af7r36_single_core_mex.cpp'));

clear mex
rehash

clearvars
clear classes
clear mex
rehash

Run the normal simulation and then:
    S36 = AF7R36_SPATIAL_AUDIT;
