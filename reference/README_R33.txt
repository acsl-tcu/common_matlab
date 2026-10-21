AF7R33 ADAPTIVE EXACT RETIMING + SINGLE SWITCHABLE WINDOW

R32 RESULT THAT MOTIVATED THIS CHANGE
- timing trials                 = 480
- exact timing candidates       = 4
- best alpha                    = 1
- best velocity-control UB      = 22.82 m/s
- hard speed pass               = 0

That means the R32 continuous timing search was mostly incompatible with the
required exact binary64 C7 storage. The few timings that survived did not
actually slow progress.

R33 TIMING
R33 makes exact representability part of the timing decision itself.

1. A binary64 time quantum is derived from the live control dt.
   It is a numerical representation quantum, NOT a physical maneuver time.
2. Entry and exit durations are decision variables over every feasible
   power-of-two multiple of that quantum.
3. Flow gets the remaining available support time.
4. The arbitrary flow duration is decomposed into an exact binary sum of
   power-of-two segments.
5. All nonzero-velocity joins share one common dyadic control lattice.
6. Stored derivatives 0..7 are checked exactly.
7. Exact candidates are Pareto-pruned by:
      total velocity UB / acceleration UB / jerk UB
   with no arbitrary top-N cap.
8. Hard authority is unchanged:
      exact C7 -> finite-sensing speed -> rejoin -> continuous whole-body.

There is no radius-specific timing rule.
H=4 s remains trigger-only.
Obstacle pose remains arbitrary R in SO(3).

DISPLAY
run_flight_headless.m now creates ONE result window only.
Switch tabs inside that one window:

  Trajectory
  Timing
  3-D
  Retiming / Cable
  Animation

The Animation tab includes a Replay animation button.
Nothing is saved to disk.

FILES TO REPLACE
  REPLANNING_SYSTEM.m
  af7r36_core.hpp
  af7r36_single_core_mex.cpp
  run_flight_headless.m
  AF7R33_FLOW_AUDIT.m

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

RUN
Run the normal simulation. One result window should appear with five tabs.

Then:
  S33 = AF7R33_FLOW_AUDIT;

Most important outputs:
  exact timing candidates
  Pareto candidates sent to hard certifier
  best alpha
  best entry / flow / exit
  best velocity / acceleration / jerk UB
  speed / rejoin / geometry
