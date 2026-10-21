AF7R38 CONSTRUCTIVE EXACT-REAL C7

run_flight_headless.m is NOT modified.
The R37 trigger-to-free-rejoin spatial trajectory theory is NOT changed.

ROOT CAUSE OF R37 "exact stored-C7 candidates = 0"

The R37 constructor already uses one dyadic node lattice at each internal knot.
For adjacent exact durations K_L*h and K_R*h, the stored endpoint stencils have
steps K_L*d*2^e and K_R*d*2^e.

As exact real polynomials:

  15*(K_L*d*2^e)/(K_L*h) = 15*d*2^e/h
  15*(K_R*d*2^e)/(K_R*h) = 15*d*2^e/h.

So the first derivatives are exactly equal even when K_L != K_R.
The arithmetic-progressive 8-point stencils make derivatives 2..7 exactly
zero. The same lattice point gives exact position continuity.

The old checker differentiated both pieces in binary64 and compared the rounded
derivatives with ==. Different floating divisions can round differently, so it
can reject an exact-real join. R38 keeps that checker only as a diagnostic.

R38 HARD C7 AUTHORITY

For trigger-to-free-rejoin family 7, the native hard gate now requires the
constructive dyadic-lattice proof:
- h is an exact power of two;
- every duration is exactly K*h;
- one node lattice is shared on both sides;
- all lattice integers are below 2^52;
- outer 8-point stencils are identically zero.

No tolerance-based acceptance is introduced.

All downstream gates are unchanged:
  path-tangent yaw-C6
  finite-sensing speed
  rejoin
  continuous UAV/load/cable whole-body safety.

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

Then run:
    S38 = AF7R38_C7_AUDIT;

The next bottleneck should now be a genuine trajectory/safety property rather
than the floating endpoint equality checker.
