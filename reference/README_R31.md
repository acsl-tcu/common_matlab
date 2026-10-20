# AF7R58ZZY-R31 — Direct Stage Store

The R30 result identifies a large timing gap that is not certificate math:

- inner R30 prepare total: 6.9180 ms
- outer Stage3 core timer: 13.0778 ms
- gap across the method/output boundary: 6.1598 ms
- Stage3 replanner total: 16.1894 ms

R31 removes the large value-struct `B` return from the Stage3 hot path.
The core builds the already-certified final staged bundle directly into the
replanner handle property exactly once. The caller receives only `(ok, reason)`.

R31 also replaces the Stage3 `onCleanup` token object with explicit
exception-safe cleanup. Detailed substage tic/toc instrumentation is disabled;
the authoritative outer Stage1/2/3/4 and runner timing remains.

Unchanged:
- exact AF7R58I resource theorem
- C7, speed, specific-force, qdot and continuous whole-body geometry certificates
- R28 native compiled representation and R29 packet MEX
- freshness and rollback policy
- four-cycle schedule
- controller / estimator / plant
- production REPLANNING_SYSTEM.m
- all MEX binaries/sources

Run:
1. `AF7R58ZZY_R31_INSTALL`
2. only after `SELFTEST CLOSED = 1` and `R31 INSTALL COMPLETE`
3. `AF7R58ZZY_R31_RUN_CANONICAL`
4. `AF7R58ZZY_R31_REPORT`
