AF7R58ZZY-R28-R2 SAFE PACKAGE
================================

Why R28 V1 failed before testing
--------------------------------
The V1 ZIP incorrectly placed the modified file
  af7r36_single_core_mex.cpp
at ZIP top level.

If that ZIP was extracted directly into common_matlab/reference, extraction
itself overwrote the canonical R27 source before AF7R58ZZY_R28_INSTALL ran.
The failure log showed canonical cpp SHA:
  bd4474a41ae93eba3e42f152ae8ba582ddf0dd59a89853fa07a90f79fbe5520a
which is EXACTLY the intended R28 payload SHA.

The original exported R27 source SHA was:
  4f9778723aa3aa339d5167b90845fcfcaa8564cae68326ff1e4ed504c7e6daea

Thus this was a packaging/source-authority defect, not an unexplained source
change and not a trajectory/certificate failure.

R2 fix
------
- No live-name cpp/hpp exists at ZIP top level.
- Modified live-name files are under payload/.
- Exact exported R27 baseline cpp is under baseline/.
- Installer accepts either exact baseline cpp or the exact known V1 accidental
  overwrite SHA.
- If the accidental overwrite is present, source is normalized back to exact
  R27 before the isolated shadow MEX build/selftest.
- Only after exact bitwise selftests close are live source/MEX/runner installed.
- Any later failure rolls back to coherent R27 runner + baseline cpp + saved MEX.

The mathematical R28 design is unchanged:
- generator candidate and continuous certificate math unchanged
- compiled evaluator representation generated only after native plan success
- derivative recurrence must be bitwise exact to the already-closed R25 DCP MEX
- full prune+append representation must be bitwise exact to R27 reference helper
- H=4 s and the four-cycle measurement schedule are unchanged for this A/B test
- production REPLANNING_SYSTEM.m and controller are not modified

Run
---
1. Extract this ZIP anywhere. It is safe even if extracted into reference/, but
   a neutral temporary folder is preferable.
2. Run:
     AF7R58ZZY_R28_R2_INSTALL
3. Only if SELFTEST CLOSED=1 and R28-R2 INSTALL COMPLETE:
     AF7R58ZZY_R28_RUN_CANONICAL
     AF7R58ZZY_R28_REPORT

Do not run the old AF7R58ZZY_R28_INSTALL again.
