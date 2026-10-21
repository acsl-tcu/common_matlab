AF7R25E DIRECT COMPLETE THREE-FILE REPLACEMENT

IMPORTANT:
Replace ALL THREE production source files together. Do not mix R25D and R25E.

Replace in your reference folder:
  1. REPLANNING_SYSTEM.m
  2. af7r36_core.hpp
  3. af7r36_single_core_mex.cpp

The MATLAB source and MEX wrapper are pair-bound by:
  AF7R25E_CURVED_C7_ZERO_JET_BELL_DIRECT_V1

If any old/stale MEX is loaded, REPLANNING_SYSTEM fails closed during warmup.

After copying all three files, rebuild:

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

Run the normal simulation.
Afterwards:
S25 = AF7R25E_DIRECT_POSTRUN;

R25E keeps the same R25D planner mathematics:
- family 6 = CURVED_C7_ZERO_JET_BELL
- no internal plateau
- exact-C7 structural candidate
- no equal-half-duration requirement
- existing exact-C7 hard gate remains before speed
- Stage-A/B finite-sensing speed unchanged
- rejoin unchanged
- continuous whole-body acceptance unchanged
- old families remain only for migration comparison

This revision fixes the R25D integration error:
REPLANNING_SYSTEM previously still required
AF7_REPLANNER_MIN_DEVIATION_V4_FIXED_T1
while the new MEX returned
AF7R25D_CURVED_C7_ZERO_JET_BELL_DIRECT_V1.
R25E makes the three-file set use one revision contract.
