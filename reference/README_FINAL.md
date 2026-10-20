# AF7R58ZZY FINAL INTEGRATED R33

This package flattens the actually-flown R33 replanner path into the production
`reference/REPLANNING_SYSTEM.m`.

Final production source SHA-256:

`643d35abe47f7c79f5ff796ea476988469a02909ebd053bb71c668527790ea9e`

## What is integrated

- Original production `REPLANNING_SYSTEM` authority
- R19 exact nominal acceleration capture and observer native fast path
- R21 exact preflight fallback semantics
- R23/R25 compiled repaired-reference evaluator support
- R28 native-once compiled correction representation
- R29 bitwise-closed obstacle-packet retarget MEX path
- R31 direct Stage3 staged-bundle store
- R33 removal of unused `P.native_raw=RAW`

The R32 direct-candidate experiment is intentionally NOT integrated.

## What is unchanged

- H = 4 s sensing/trigger semantics
- degree-15 C7 candidate generation
- continuous speed / specific-force / qdot certificates
- continuous payload/UAV/cable whole-body geometry certificate
- exact AF7R58I resource theorem and fail-closed behavior
- controller, estimator and plant
- current four-cycle measurement schedule

## Files after integration

Keep in the repository root/reference:

- `reference/REPLANNING_SYSTEM.m`
- `run_flight_headless.m`
- runtime `af7r*` helpers/MEX files
- `AF7R58ZZY_FINAL_SELFTEST.m`
- `AF7R58ZZY_FINAL_RUN_CANONICAL.m`
- `AF7R58ZZY_FINAL_REPORT.m`
- `AF7R58ZZY_FINAL_ARCHIVE_LEGACY.m`

Final outputs are written only to:

`generated/AF7R58ZZY_FINAL/`

Installer backups are written only to:

`_af7_final/backups/<timestamp>/`

## Run sequence

1. `AF7R58ZZY_FINAL_INSTALL`
2. Require `FINAL INTEGRATION COMPLETE`
3. `AF7R58ZZY_FINAL_RUN_CANONICAL`
4. `AF7R58ZZY_FINAL_REPORT`
5. Confirm performance/safety matches R33
6. Optional dry run: `AF7R58ZZY_FINAL_ARCHIVE_LEGACY(false)`
7. Only after validation: `AF7R58ZZY_FINAL_ARCHIVE_LEGACY(true)`

Do not archive or delete `af7r*.m`, `*.cpp`, `*.hpp` or MEX binaries with the
legacy helper; they remain runtime dependencies.
