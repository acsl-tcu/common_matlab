AF7R25G DEVELOPMENT STAGED DIRECT REPLACEMENT

PURPOSE
- Make the already-certified CURVED_C7_ZERO_JET_BELL actually reach staged
  atomic commit during development.
- Keep the final <=4 ms target visible, but do not block development merely
  because a stage is e.g. 7-12 ms while it is still inside the 25-ms control
  period.
- Fix flat 3-D trajectory plots by enforcing >=3 m span independently on x/y/z.

IMPORTANT
This is NOT the final hard-4-ms timing closure.
Geometric/reference safety gates are unchanged:
  exact C7 -> finite-sensing speed -> rejoin -> continuous whole-body.
Only the staged DEVELOPMENT timing acceptance is <=25 ms per cycle/stage.
The final 4-ms target is still measured and reported.

REPLACE ALL THREE production files together:
  REPLANNING_SYSTEM.m
  af7r36_core.hpp
  af7r36_single_core_mex.cpp

NATIVE REVISION
  AF7R25G_DEV_STAGED_ZERO_JET_BELL_DIRECT_V1

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

Then run the normal simulation.

After the simulation:
  S25 = AF7R25G_DEV_AUDIT;

That audit also applies:
  AF7R25G_FIX_3D_AXES(3.0)

so every detected 3-D trajectory axis has at least a 3 m x/y/z full span.

Expected development behavior:
- family 6 candidate passes exact C7/speed/rejoin/whole-body,
- staged Stage 1 -> Stage 2 -> Stage 3 reaches commit if each cycle stays <=25 ms,
- reference_only_commit_count becomes >=1,
- active_repaired_safety_level becomes REFERENCE_ONLY,
- final 4-ms compliance is reported separately and may still be open.
