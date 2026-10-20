# AF7R58ZZY-R33 — clean A/B isolation

R33 exists only to isolate the effect of the unused `P.native_raw=RAW` mirror.

Runtime basis:
- R31 Stage1 candidate-return/store semantics: restored
- R31 direct Stage3 store: retained
- R32 direct-candidate-store experiment: removed
- `P.native_raw=RAW`: removed

Therefore the only intended R31 -> R33 data-path change is deletion of the
unused RAW diagnostic mirror.

This is deliberately conservative before touching deep freshness checks.

Run:
1. `AF7R58ZZY_R33_INSTALL`
2. require `SELFTEST CLOSED = 1` and `R33 INSTALL COMPLETE`
3. `AF7R58ZZY_R33_RUN_CANONICAL`
4. `AF7R58ZZY_R33_REPORT`

Interpretation:
- If R33 beats R31 materially, native_raw retention was contributing.
- If R33 ~= R31, native_raw is not the remaining large cost.
- In that case the next target is source/sensor freshness, but only after a
  complete mutation-point audit and an exact epoch/version replacement design.
