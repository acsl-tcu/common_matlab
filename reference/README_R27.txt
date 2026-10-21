AF7R27 FLOWING EXACT-C7 DIRECT REPLACEMENT

DESIGN INTENT
- Keep the existing fixed 4-second collision trigger.
- Do NOT use the discarded R26 early-sensor-preview design.
- Do NOT solve a large obstacle by allowing extreme speed.
- Replace the zero-internal-velocity shape, when possible, with a flowing
  correction that carries nonzero correction velocity through the obstacle.

NEW FAMILY
  family 7 = CURVED_C7_FLOW_AFFINE

For one connected unsafe component:
1. The existing continuous corridor constraints solve directly for correction
   position P and correction velocity V at component entry.
2. Entry, constant-V flow, and exit are stored as degree-15 Bezier segments.
3. All stored segments use the same exact binary64 duration
      h = component.start - trigger_time
   (approximately the existing fixed H=4 s lead).
4. A common dyadic arithmetic-progression control stencil makes joins exact-C7
   as represented, while allowing NONZERO first derivative internally.
5. The quantized stored polygon is still independently subjected to the
   unchanged:
      exact-C7 -> finite-sensing speed -> rejoin -> continuous whole-body
   hard acceptance stack.

This is deliberately implemented in the existing direct-time certified
representation. It is the first nonzero-internal-jet / time-allocation step
toward the full path-plus-time-law architecture, without replacing the trusted
compiled evaluator in the same experiment.

For multiple disconnected unsafe components, family 7 currently falls through
to the already-certified family 6/legacy migration families. This avoids
silently inventing a multi-component joining theorem.

REPLACE ALL THREE PRODUCTION FILES TOGETHER
  REPLANNING_SYSTEM.m
  af7r36_core.hpp
  af7r36_single_core_mex.cpp

NATIVE REVISION
  AF7R27_FLOWING_AFFINE_EXACT_C7_DIRECT_V1

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

TEST ORDER
1. Re-run the radius-5 m cylinder case with the SAME fixed-H trigger.
2. Then run:
     S27 = AF7R27_FLOW_AUDIT;
3. For a single dedicated 3-D plot:
     V27 = AF7R27_3D_VIEW;

The key result is family 7:
  CURVED_C7_FLOW_AFFINE
  exact = 1
  speedPASS = 1
  rejoin = 1
  geometry = 1
  committed = 1

If family 7 is seen/exact but speedFAILs, the next step is NOT to raise the
speed limit. It means the exact flowing shape/time allocation still needs more
degrees of freedom.
