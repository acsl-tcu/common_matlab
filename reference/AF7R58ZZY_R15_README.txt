AF7R58ZZY-R15 LEAN INTEGRATION

Purpose:
- Stop forensic iterations and install the already-proven exact accelerations.
- Controller timing is explicitly outside this package.

Changes used by the runner:
1) Direct subclass REPLANNING_SYSTEM_R15_FAST < REPLANNING_SYSTEM.
2) Observer one-step/two-event numeric kernels use the R14-R3 bitwise-exact AOT/MEX path.
3) All AF7R33 batch calls in staged postcache / repaired packet batch branches use the R11 bitwise-exact de Casteljau MEX helper.
4) No R5/R8/R13 diagnostic inheritance or Flight snapshot capture.
5) REPLANNING_SYSTEM.m itself remains byte-exact. Only the runner constructor line is switched transactionally.

Run:
  AF7R58ZZY_R15_INSTALL
Then normal use:
  run_flight_headless
Rollback:
  AF7R58ZZY_R15_ROLLBACK

After one normal run, proceed to the next research item; do not reopen controller 24.95 s in this package.
