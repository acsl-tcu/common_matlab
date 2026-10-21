AF7 runner-owned 3-D visualization fix V2

This package changes visualization ownership only.

Overwrite:
  run_flight_headless.m
  AF7R31_FLOW_AUDIT.m

No production replanner/core/MEX source is changed.

After simulation:
- Figure 5  = trajectory/avoidance dashboard
- Figure 10 = timing dashboard
- Figure 12 = large dedicated 3-D trajectory/obstacle window

Figure 12 is built directly from the logger/replanner data already available
inside run_flight_headless.m. It does not search for another axes or UIAxes.

AF7R31_FLOW_AUDIT no longer opens or searches for any 3-D window. It only
prints planner diagnostics. This enforces the requested ownership:
visualization belongs to run_flight_headless.m, not to the replanner/audit path.

No MEX rebuild is required for these two visualization/audit files.
