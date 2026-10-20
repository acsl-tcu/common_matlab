AF7R58ZZY-R26-R3 FINAL PACKAGE
===============================

This package replaces the earlier runtime-synthesis recovery.

Verified chain
--------------
Original audited R23 runner:
  SHA 22a4b5fa6d5c2ccfc9a3798780782535e101424173e5eec81c8ddfb9c6f49949

Independent reproduction of the R25 installer transformation:
  SHA 0581116b243151e9b6855aacb676c704ea4b15991620213833f8ba914b9a3451

R26-R3 final runner after:
- R25 -> R26 class binding
- detailed profiling OFF
- light runtime audit OFF
- threshold 2.0 ms
- exact R25 fast merge retained
- empty internal diagnostic fields -> NaN in clean mode

Final runner SHA:
  cfa9f9d61b2762fdb899515b2a6d6ddf582d550c1eaf4046574e360b9d28fbe5

This EXACTLY matches the rebuilt SHA printed by MATLAB in the user's latest
failed recovery. The previous expected 36a5... was stale: it belonged to a
different comment-only variant of the standalone profile repair.

Run only
--------
1) AF7R58ZZY_R26_R3_FINAL_RECOVERY
2) AF7R58ZZY_R26_R3_RUN_CANONICAL
3) AF7R58ZZY_R26_R3_REPORT

Do NOT run the older R26/R26-R2/R26-R3 installers or EOL/profile/full-recovery
scripts again.

The launcher runs the canonical runner by absolute path in the BASE workspace,
so a residual path-shadow copy cannot be selected accidentally.
