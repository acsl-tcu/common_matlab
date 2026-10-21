AF7R32B PROGRESS-RETIMED PRIMARY + DISPLAY-ONLY VISUALIZATION

This is the corrected package.

IMPORTANT
- The program displays the results.
- It does NOT save PNG, MP4, AVI, or any other visualization file.
- No postrun_media directory is created.
- There is no VideoWriter/exportgraphics/saveas/imwrite path.

Planner / trajectory switch
- Same R32 progress-retimed family-7 production code.
- Fixed H=4 s remains trigger-only.
- For the current EXACT affine straight-line base reference:
    family 7 = CURVED_C7_PROGRESS_RETIMED_AFFINE_BASE
- Progress scale alpha and entry/flow/exit timing are searched as decision
  variables; maneuver times are not fixed to 4/1/8 s.
- No obstacle-radius timing threshold.
- Arbitrary obstacle attitude remains R in SO(3).
- Acceptance remains:
    exact C7 -> finite-sensing speed -> rejoin -> continuous whole-body.
- If the active base is not exactly affine, this family fails closed rather
  than pretending the affine retiming theorem applies.

Visualization ownership
All visualization is in run_flight_headless.m only.

After the simulation:
  Figure 5  : existing trajectory / avoidance dashboard
  Figure 10 : existing timing dashboard
  Figure 12 : added large dedicated 3-D trajectory/payload/cable/obstacle view
  Figure 13 : added adaptive-time allocation + cable swing diagnostics
  Figure 14 : added on-screen 3-D UAV/payload/cable animation

Figure 14 replays already logged data. It is display only and is not recorded.

FILES
Replace:
  REPLANNING_SYSTEM.m
  af7r36_core.hpp
  af7r36_single_core_mex.cpp
  run_flight_headless.m
  AF7R32_FLOW_AUDIT.m

MEX REBUILD
If you are moving from R31 (the current production state before R32), rebuild:

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

The figures appear automatically. Nothing is exported to disk.

Afterward:
  S32 = AF7R32_FLOW_AUDIT;

Key planner outputs:
  affine base detected
  timing trials
  exact timing candidates
  monotone-progress rejects
  nominal progress speed
  best progress scale alpha
  best entry / flow / exit
  best affine velocity-control UB
  speed / rejoin / geometry
