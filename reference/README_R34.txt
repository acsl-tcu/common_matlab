AF7R34 DELAY-PRESERVING NO-CATCH-UP RETIMER

PURPOSE
R33 proved that slowing progress while forcing rejoin to the OLD CLOCK schedule
creates a catch-up requirement.  If sdot<1 on any nonzero interval but the
planner insists on s(t_r)=t_r, then

    integral (sdot - 1) dt = 0

forces sdot>1 somewhere else.  That is exactly the behavior we do not want for
suspended-load transport.

R34 changes the rejoin theorem for the exact-affine nominal base:

    p_nom(t) = p0 + v_b t.

After the avoidance maneuver, R34 holds a constant correction

    d_hold = -v_b * Delta,

so

    p_nom(t) + d_hold = p_nom(t - Delta).

Therefore the repaired trajectory rejoins the SAME NOMINAL SPATIAL PATH with a
persistent schedule delay Delta.  There is no catch-up acceleration and no
requirement to restore the old clock.

IMPORTANT
- H=4 s remains trigger-only.
- No obstacle-radius timing rule is introduced.
- Obstacle attitude remains arbitrary R in SO(3).
- Speed limits are unchanged.
- The finite hard stack remains:
      exact C7 -> finite-sensing speed -> delayed-path rejoin ->
      continuous whole-body certificate.
- A nonzero terminal correction is accepted only when its derivatives 1..7
  are exactly zero and every stored Bezier join is bitwise exact through C7.
- The terminal correction is certified through the finite source domain and
  then persisted analytically as the affine nominal schedule delay.

TIMING
R34 does not use fixed 4/1/8 s maneuver durations.
For each representable flow duration alpha is induced by the spatial unsafe
span.  The entry duration is derived from the symmetric C7 progress-transition
identity

    T_entry = 2 * progress_lead / (1 + alpha)

and snapped only to the exact binary64 lattice.  Exit uses all remaining source
validity except one representation quantum for the certified terminal hold.

Before building stored polygons, timing points are Pareto-filtered by the exact
Hermite velocity / acceleration / jerk upper bounds.  This is synthesis only;
no candidate bypasses the hard certificates.

CURRENT SCOPE
This closes the NO-CATCH-UP theorem for the currently proven exact-affine
nominal base.  It does not claim the final general curved-base r(s)+s(t)
architecture yet.  If R34 closes speed but geometry/path quality still limits
large obstacles, the next change is the spatial C7 path family itself, not a
new timing constant.

DISPLAY
run_flight_headless.m is NOT changed in this package.
Keep the R33 single-window tabbed display exactly as it is.

FILES TO REPLACE
  REPLANNING_SYSTEM.m
  af7r36_core.hpp
  af7r36_single_core_mex.cpp
  AF7R34_FLOW_AUDIT.m

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
Run the normal simulation using the already-installed R33 run_flight_headless.m.
Then:

  S34 = AF7R34_FLOW_AUDIT;

Most important outputs:
  stored delay candidates
  exact terminal-hold C7 candidates
  monotone-progress candidates
  selected schedule delay
  selected alpha
  selected velocity / acceleration / jerk UB
  speed / rejoin / geometry
