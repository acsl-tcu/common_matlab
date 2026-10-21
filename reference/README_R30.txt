AF7R30 SCALE-AGNOSTIC FLOW DIRECT REPLACEMENT

TARGET
This revision is NOT a "5 m obstacle patch".

The planner contract is:
- no obstacle-radius-specific threshold,
- arbitrary obstacle attitude R in SO(3),
- required offset comes from actual oriented obstacle geometry,
- maneuver support/time may grow with obstacle extent,
- speed limits are NOT increased to force a solution,
- exact-C7 / finite-sensing / rejoin / continuous whole-body remain mandatory.

"Independent of size" means size-agnostic synthesis for every PHYSICALLY
FEASIBLE case. A fully blocked workspace, an infinite wall, or a case with
insufficient stopping/lateral room must still fail closed for physical reasons.

WHY R29 HAD exact-grid rejects = 1
R29 required the arbitrary trigger time t0 itself to sit on the common
equal-duration exact-C7 lattice. Ordinary binary64 times such as 9.85 s can
lose exact equal-duration subtraction when the sequence crosses a binade
(16 s, 32 s, ...). That is a representation artifact, not an avoidance limit.

R30 GRID
R30 leaves the correction exactly zero for a short interval after trigger and
starts the candidate on a nearby globally aligned dyadic grid:

    support_start = ceil(t0/h) * h

Then entry / flow / exit all use the same exact power-of-two h.
The wait is strictly less than h and the entry must still finish before the
first unsafe time. Internal exact-C7 therefore no longer depends on arbitrary
trigger-time bits.

TRIGGER
Fixed H=4 s trigger is unchanged. No R26 early future preview.

MEX REBUILD -- DO NOT SKIP

Replace all three production files:
  REPLANNING_SYSTEM.m
  af7r36_core.hpp
  af7r36_single_core_mex.cpp

Then run:

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

TEST
Use the same current case first as a regression test, then run:

  S30 = AF7R30_FLOW_AUDIT;

The decisive transition is:
  exact-grid rejects = 0
  side assignments > 0
  QP solutions > 0
  stored candidates built > 0
  family 7 seen > 0

If family 7 then exact=1 but speedFAIL>0, the numerical representation blocker
is closed. The next architectural step is genuine progress retiming/path+time
law so obstacle extent is handled by taking more time, NOT by raising speed.

3-D
The audit also tries to open one separate large 3-D window. It detects a 3-D
source primarily from the axes camera View rather than title/ZData alone.
Plotting failure can never abort the audit.
