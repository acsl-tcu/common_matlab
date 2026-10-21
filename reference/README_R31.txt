AF7R31 ADAPTIVE-TIME FLOW DIRECT REPLACEMENT

THIS IS NOT A 5 m PATCH.
No radius threshold is used. Obstacle size and attitude enter only through the
existing oriented-ellipsoid geometry/corridor support.

WHAT R30 PROVED
R30 closed generation:
  side assignments = 2
  QP solutions = 2
  stored candidates = 2
  exact-grid rejects = 0
  family 7 seen = 2
  family 7 exact = 2

Both family-7 candidates then failed the finite-sensing speed certificate.
Therefore "trajectory unchanged" was expected: no unsafe/uncertified candidate
was committed.

WHY R30 SPEED-FAILED
The R30 diagnostic showed:
  trigger lead about 4.03 s
  but entry duration only 1 s
  tail about 12.9 s
  but exit duration only 1 s

That storage-time choice forced an unnecessarily rapid lateral entry and an
unnecessarily rapid return.

R31
- fixed H=4 s trigger unchanged;
- speed limits unchanged;
- arbitrary obstacle attitude R in SO(3) unchanged;
- uses almost the full available pre-obstacle lead for entry;
- uses a large part of the post-obstacle tail for exit;
- flow support automatically grows with obstacle extent;
- every stored join is checked for literal binary64 equality of derivatives
  0..7 before exact_c7 is allowed to be true;
- hard acceptance remains:
    exact C7 -> finite-sensing speed -> rejoin -> continuous whole-body.

For the R30 timing numbers, the timing constructor itself resolves to:
  support start = 13.25 s
  entry duration = 4 s
  flow start = 17.25 s
  flow segment = 1 s
  flow end = 26.25 s
  exit duration = 8 s
  exit end = 34.25 s
This is derived from lead/tail geometry; it is not hard-coded for the 5 m case.

IMPORTANT LIMIT
R31 is still the existing additive direct-time representation. It removes the
artificial 1 s entry/exit bottleneck generically. If a physically feasible much
larger obstacle still requires slowing nominal progress itself, the next
architecture is true path-progress retiming r(s)+s(t), not a higher speed cap.

3-D WINDOW
R31 searches BOTH ordinary Axes and UIAxes. This addresses why R30 could not
see the already-drawn 3-D plot. The audit opens one separate large third
window and leaves the runtime/trajectory windows unchanged.

REPLACE ALL THREE PRODUCTION FILES TOGETHER:
  REPLANNING_SYSTEM.m
  af7r36_core.hpp
  af7r36_single_core_mex.cpp

MEX REBUILD -- DO NOT SKIP

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

Then run the same regression case and:
  S31 = AF7R31_FLOW_AUDIT;

Important outputs:
  family 7 seen/exact/speedPASS/speedFAIL
  entry duration actually used
  exit duration actually used
  best correction-speed UB
  best Stage-A UAV-speed UB
  dedicated 3-D created / source axes class
