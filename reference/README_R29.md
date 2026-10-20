# AF7R58ZZY-R29 Stage3 Lean Exact

R29 starts from the accepted R28 checkpoint and leaves the four-cycle schedule
unchanged so the timing delta cannot be explained by moving work.

It removes Stage3 implementation overhead only:

1. The AF7R58I resource theorem and exact prune boundary are retained, but
   already-certified historical control-point cells are not revalidated.
   R28's exact native compiled packet supplies the post-prune group/segment
   counts and is bound to the newly appended group.

2. The target obstacle packet uses the same production per-type formulas
   (`sphere`, `cylinder`, all other raw enclosing ellipsoids) in a small MEX.
   Installer selftest requires bitwise equality to the MATLAB formulas.

3. Native/canonical binding consumes the resource metadata already closed by
   the fast canonical append instead of rescanning every historical group.

Unchanged:
- degree-15 C7 trajectory generation
- continuous speed/specific-force/qdot certificate
- continuous payload/UAV/cable geometry certificate
- sensor-horizon theorem
- freshness/stale rejection and transactional rollback
- R28 native D0..D6 representation
- controller / estimator / plant
- four-cycle schedule

Run:
1. `AF7R58ZZY_R29_INSTALL`
2. only if `SELFTEST CLOSED = 1` and `R29 INSTALL COMPLETE`
3. `AF7R58ZZY_R29_RUN_CANONICAL`
4. `AF7R58ZZY_R29_REPORT`
