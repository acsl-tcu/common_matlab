AF7R58ZZY-R27 | EXACT WORK-ELIMINATION PACKAGE
===============================================

Purpose
-------
R27 does NOT improve timing by adding another pipeline stage or by moving the
same expensive operation to another control cycle.  The R26 schedule remains:
  trigger, +dt, +2dt, +3dt.

R27 removes two pieces of duplicate/non-authority work from the real Flight
path while preserving the existing theoretical authority:

1. Final full correction recompilation is removed.
   - At Stage1, the already validated derivative MEX differentiates only the
     newly generated correction group.
   - Existing nonexpired groups are copied bitwise after the same exact
     t1 <= t_commit prune used by the canonical resource model.
   - At Stage3 the incremental compiled packet is rebound to the exact R21
     canonical batch model before it can become commit authority.
   - Installer selftest compares the incremental result bitwise against the
     exact R23 full compiler on a deterministic prune+append case.

2. The 161-point postcommit rolling cache is removed from atomic commit.
   - It is not an acceptance/safety certificate.
   - Commit invalidates that cache.
   - The next real repaired-reference packet request uses the existing exact
     compiled evaluator and installs the ordinary rolling cache on demand.

Preserved without weakening
----------------------------
- H = 4 s collision horizon
- sensing / memory semantics
- degree-15 C7 candidate construction
- continuous-time whole-body obstacle certificate
- speed / specific-force / qdot certificates
- resource prune/cap theorem
- canonical batch model and canonical function-handle authority
- obstacle packet and sensor-horizon certificates
- full source/sensor/dependency freshness checks
- target bundle validation
- fail-closed deadline rejection and transactional rollback
- controller / estimator / plant

Run
---
1) AF7R58ZZY_R27_INSTALL
2) AF7R58ZZY_R27_RUN_CANONICAL
3) AF7R58ZZY_R27_REPORT

Return
------
AF7R58ZZY_R27_REPORT.txt
AF7R58ZZY_R27_OUT.mat

Immediate performance criterion
-------------------------------
The controller is not the research target.  R27 reports both total online time
and controller-excluded time.  The immediate target is:
  replan-window controller-excluded max <= 17 ms
with ~15 ms preferred.

Future standalone MEX-build contract
------------------------------------
This is intentionally NOT activated yet.  After the trajectory/replanner path
is accepted, the replanner package should gain one explicit preflight/build
entrypoint that builds every required MEX from source files shipped with the
replanner itself.  It must not read an audit report, test MAT, captured source,
or external test package.  Flight must never compile a MEX.  Runtime should
only verify that the required binary/source revision is present and fail closed
if it is not.  That packaging step is deferred until R27/R28 timing and safety
behavior are accepted, so packaging changes cannot contaminate this experiment.
