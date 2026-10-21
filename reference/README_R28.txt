AF7R28 GENERAL FLOWING EXACT-C7 DIRECT REPLACEMENT

WHAT R27 RESULT MEANT
R27 family 7 had:
  seen = 0
Therefore it never reached the exact-C7/speed/rejoin/whole-body certifier.
That was a candidate-generation/representation restriction, not a safety
certificate rejection.

R28 FIX
- Fixed 4-second trigger remains unchanged.
- No early-sensor future preview.
- Family 7 no longer requires:
    * exactly one merged unsafe component,
    * component.start - t0 ~= 4 exactly,
    * obstacle episode boundaries to coincide with Bezier joins.
- A common exact binary64 time grid is placed AROUND the unsafe union.
- All corridor rows are written in the stored [P0,V] flow variables.
- Stored joins remain exact C7 with nonzero internal correction velocity.
- Candidate acceptance is still:
    exact-C7 -> finite-sensing speed -> rejoin -> continuous whole-body.

ARBITRARY OBSTACLE ATTITUDE
Preserved. Separator synthesis uses each obstacle's supplied rotation R and
axes. Exact support is evaluated in the oriented ellipsoid frame. No x/y/z
orientation special case was introduced.

3-D WINDOW
After the simulation, AF7R28_FLOW_AUDIT automatically creates one ADDITIONAL
large dedicated 3-D window. Runtime and trajectory windows remain separate.
The numerical x/y/z limits remain truthful and each span is at least 3 m, but
the screen plot box is [1 1 1] so a 50 m vertical span does not visually crush
the lateral motion.

REPLACE ALL THREE PRODUCTION FILES TOGETHER
  REPLANNING_SYSTEM.m
  af7r36_core.hpp
  af7r36_single_core_mex.cpp

NATIVE REVISION
  AF7R28_FLOW_GENERAL_EXACT_C7_DIRECT_V1

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
1. Use the same radius-5 m obstacle and the same fixed-H trigger.
2. Run the normal simulation.
3. Run:
     S28 = AF7R28_FLOW_AUDIT;

The audit opens the third, large 3-D window automatically.

Interpretation:
- family7 seen=0 again:
    read the R28 pre-cert counters; do not blame safety/speed.
- family7 seen>0, exact=1, speedFAIL>0:
    generation is fixed; the remaining issue is actual time allocation/speed,
    and the next step is the full path + retiming law, NOT a larger speed cap.
- family7 speedPASS/rejoin/geometry and commit:
    fixed-H plus flowing exact-C7 is sufficient for that case.
