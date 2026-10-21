AF7R37 TRIGGER -> FREE-REJOIN EXACT-C7 SPATIAL PRIMARY

run_flight_headless.m is NOT modified.

THIS IS THE NEXT THEORY STEP, NOT THE FINAL SYSTEM.

WHAT CHANGES
R36 bent the path mainly inside the unsafe span and then used separate
entry/exit retiming transitions. R37 instead synthesizes one exact-C7 spatial
correction over the entire trigger-to-rejoin interval.

Knots:
  trigger/support start
  aligned start/end of every currently known unsafe component
  free rejoin

At every interior knot:
  correction position and first derivative are free.
  derivatives 2..7 are exactly zero.

At both outer endpoints:
  correction derivatives 0..7 are exactly zero.

The stored degree-15 Bezier pieces share one dyadic node lattice, so C7 joins
are checked bit-for-bit.

OBSTACLE GEOMETRY
The same arbitrary-pose oriented ellipsoid support function is retained.
The corridor is synthesis only; the continuous UAV/load/cable whole-body
certificate remains the acceptance authority.

PATH QUALITY
The QP uses the suspended-load natural-time C3 metric already present in the
system. No new arbitrary smoothing weight is introduced.

For candidates with the same rejoin deadline, the preference is lexicographic:
  lower jerk bound
  lower acceleration bound
  smaller deviation envelope

Safety is still tested first by the unchanged hard certificate chain.

FREE REJOIN
The full source-valid reference boundary remains the fail-safe baseline.
Only after that baseline is independently certified, MATLAB performs a
source-bounded bisection for a shorter rejoin deadline. Every shorter trial is
rebuilt and must pass the SAME exact-C7, finite-sensing, rejoin and continuous
whole-body certificates.

There is no fixed physical "return within N seconds" upper bound.
The search range ends only at the live finite reference-validity boundary.

TIME LAW
R37 intentionally uses the identity time law s(t)=t while the spatial path is
being closed. Therefore it introduces no catch-up and no artificial retiming
constants. The nontrivial C6 time law is the next layer after this spatial
primary passes.

YAW
The R35 exact path-tangent yaw-C6 theorem remains active:
  strict horizontal half-plane gamma > 0
  analytic yaw derivatives through order 6.

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

Then run the normal simulation and:
    S37 = AF7R37_FREE_REJOIN_AUDIT;

Computation-time optimization is deliberately deferred. The existing 25-ms
stage gate remains active and is NOT weakened; predeadline diagnostics still
show whether the mathematical candidate itself succeeded before a timing miss.
