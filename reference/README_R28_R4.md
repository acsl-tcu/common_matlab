AF7R58ZZY-R28-R4 FINAL INSTALLER FIX
====================================

R3 already proved on the user's MATLAB machine:
- R28 shadow MEX compiled successfully.
- native revision exact = 1
- derivative D0..D6 bitwise exact = 1
- prune boundary t1==commit exercised = 1
- full prune+append packet bitwise exact = 1
- external R27 runtime compile absent = 1
- exact R21 certificate path retained = 1
- native binding gate present = 1
- measurement schedule unchanged = 1
- native packet authority split bound = 1
- SELFTEST CLOSED = 1

R3 then stopped because the destination struct defined C.launcher but one line used
the nonexistent C.launch. Rollback reported coherent R27 source/binary = PASS.

R4 changes ONLY installer mechanics:
1) uses C.launcher consistently
2) shadow path cleanup executes exactly once and is idempotent
3) destination struct-field references were statically audited
4) every installed payload hash is rechecked
5) report/selftest/launcher active bindings are checked before completion

R28 native payload/theory is byte-for-byte unchanged from R3.

Run:
  AF7R58ZZY_R28_R4_INSTALL

Only after SELFTEST CLOSED=1 and R28-R4 INSTALL COMPLETE:
  AF7R58ZZY_R28_RUN_CANONICAL
  AF7R58ZZY_R28_REPORT
