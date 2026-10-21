AF7R25D DIRECT COMPLETE REPLACEMENT

Replace these THREE production source files in the reference folder:
  1. REPLANNING_SYSTEM.m
  2. af7r36_core.hpp
  3. af7r36_single_core_mex.cpp

Do not run the old R25/R25B/R25C installers after replacement.

Then in MATLAB rebuild the native MEX from the replaced sources:

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

Then:
clearvars
clear classes
clear mex
rehash

Run the normal simulation. Afterwards:
S25 = AF7R25D_DIRECT_POSTRUN;

R25D properties:
- family 6 = CURVED_C7_ZERO_JET_BELL, attempted before old production families.
- exactly two degree-15 segments per disconnected unsafe component: 0 -> A -> 0.
- no internal plateau.
- exact-real C7 at outer boundaries and peak from repeated 8 control points.
- no equal-half-duration requirement.
- family 6 does NOT inherit the old branch-3 heading gate; yaw remains independent C6.
- existing exact-C7 hard gate remains before speed certification.
- existing Stage-A/B finite-sensing speed, rejoin, and continuous whole-body acceptance gates are unchanged.
- old families remain only for migration comparison if family 6 cannot close all hard gates.
