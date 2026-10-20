# AF7R58ZZY-R32 — Direct candidate store

R31 successfully removed the Stage3 large-bundle method-boundary cost:
Stage3 core fell from ~13.08 ms to ~7.91 ms.

R31 then exposed Stage1 as the remaining peak:
- Stage1 generator/core: 4.5562 ms
- Stage1 replanner: 13.9428 ms
- Stage1 controller-excluded cycle: 17.4489 ms

R32 applies the same proven implementation principle to Stage1.

Changes:
1. The unused `P.native_raw=RAW` diagnostic mirror is removed.
2. The successful Stage1 candidate is built and stored directly into
   `obj.staged_replan_candidate` inside the generator method.
3. The caller receives only scalar status/timing outputs.
4. The old large `candidate = ...` method return and subsequent
   `obj.staged_replan_candidate=candidate` assignment are removed.
5. Deadline failure explicitly clears any directly staged candidate.

Unchanged:
- AF7R36 MEX and native trajectory/certificate mathematics
- R28 compiled D0..D6 packet
- exact AF7R58I resource theorem
- continuous C7 / speed / specific-force / qdot / whole-body geometry proofs
- R31 direct Stage3 bundle store
- freshness and rollback
- four-cycle schedule
- controller / estimator / plant
- production `reference/REPLANNING_SYSTEM.m`

Run:
1. `AF7R58ZZY_R32_INSTALL`
2. only after `SELFTEST CLOSED = 1` and `R32 INSTALL COMPLETE`
3. `AF7R58ZZY_R32_RUN_CANONICAL`
4. `AF7R58ZZY_R32_REPORT`
