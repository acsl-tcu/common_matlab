AF7R35 PATH-TANGENT YAW C6

run_flight_headless.m is NOT modified.

R34 delay-preserving / no-catch-up retiming is retained. R35 adds a formal
path-tangent yaw layer for the family-7 primary.

THEOREM
For the exact-affine base p_b(t)=p0+v_b t and exact-C7 correction d(t),
let e_h be the nominal horizontal unit travel direction.

On every Bezier segment:
    e_h^T v_xy(t)
is a scalar Bernstein polynomial. Therefore:
    e_h^T v_xy(t) >= min(projected derivative control points) = gamma.

R35 requires gamma>0 as a hard family-7 regularity gate.
Consequently:
- horizontal speed never vanishes;
- the actual heading stays inside one open half-plane centered on the nominal
  travel heading;
- atan2 has one globally unique branch there;
- position C7 => horizontal velocity C6 => path-tangent yaw C6.

YAW DERIVATIVES
No finite differences are used.
With:
    z = vx + i*vy,
    yaw = Im(log z),
yaw derivatives through order 6 are calculated analytically from position
derivatives through order 7 using the log-series recurrence.

The actual controller/reference packet yaw slots:
    yaw, yaw_dot, ..., yaw^(6)
are overwritten by this analytic path-tangent yaw jet after a family-7 commit.

DELAY-PRESERVING COMPATIBILITY
After the R34 terminal schedule-delay hold, correction velocity is exactly
zero. Hence yaw returns to the exact nominal path tangent and its derivatives
1..6 are zero. There is no conflict with persistent schedule delay.

IMPORTANT
R35 deliberately does NOT claim to solve the remaining translational speed
failure. R34 showed that the present affine spatial path remains too aggressive.
The next translational change is the general exact-C7 spatial path. This yaw
layer is installed now because its theorem remains valid for that future path
whenever the same strict horizontal-speed certificate gamma>0 is closed.

FILES TO REPLACE
  REPLANNING_SYSTEM.m
  af7r36_core.hpp
  af7r36_single_core_mex.cpp

Do NOT replace run_flight_headless.m.

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

Then run the normal simulation and:
    S35 = AF7R35_YAW_AUDIT;
