AF7R29 FLOW GRID + DIAGNOSTIC + DEDICATED 3-D DIRECT REPLACEMENT

WHAT WAS WRONG IN R28
1. The post-run audit looked for R28 native diagnostics in get_last_replan_status.
   A failed Stage-1 proposal never commits, so those diagnostics were not there.
   R29 persists them directly from native_raw BEFORE Stage-1 can return failure.
2. R28's dedicated 3-D helper searched by a specific title and asserted.
   That presentation bug aborted the entire audit. R29 selects the 3-D source
   by actual finite ZData and never lets plotting failure abort the audit.
3. R28 exact flow-grid search used:
       maxh = min(lead, 0.49*(tail + lead))
   which does NOT guarantee room for the exit transition.
   It then searched only very close to that h, so a smaller valid exact grid
   could exist but family 7 would remain seen=0.
   R29 uses the constructive support bound:
       h < tail/2
   via:
       maxh = min(lead, 0.49*tail)
   and searches a wider deterministic neighborhood.

SAFETY / THEORY UNCHANGED
- Fixed H=4 s trigger unchanged.
- No early-sensor preview.
- Obstacle attitude remains arbitrary R in SO(3).
- Oriented ellipsoid exact support remains in the obstacle frame.
- Acceptance remains:
    exact C7 -> finite-sensing speed -> rejoin -> continuous whole-body.
- No speed limit is relaxed.

REPLACE ALL THREE PRODUCTION FILES
  REPLANNING_SYSTEM.m
  af7r36_core.hpp
  af7r36_single_core_mex.cpp

NATIVE REVISION
  AF7R29_FLOW_GRID_DIAG_3D_DIRECT_V1

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

TEST
Use the same radius-5 m case, same fixed-H trigger and same obstacle attitude.
After simulation:

  S29 = AF7R29_FLOW_AUDIT;

The audit itself opens the third large 3-D window when 3-D plot data exist.

The decisive R29 block is:
  side assignments
  QP solutions
  stored candidates built
  stored-row rejects
  exact-grid rejects
  finite-support rejects
  common exact step h

If family 7 reaches seen>0 and exact=1 but speedFAIL>0, generation is finally
closed and the next change is genuine path + time-law retiming. We will NOT
respond by raising the allowed speed.
