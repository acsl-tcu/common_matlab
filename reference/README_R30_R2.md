# AF7R58ZZY-R30-R2

This is the same R30 runtime/class/MEX payload as R30.

The only change is the installer source-location logic for protected AF7R58I
helpers. R30 incorrectly assumed these files were directly under
`common_matlab/`. Their installed authority is resolved by MATLAB from the
`reference/` tree.

R30-R2 now resolves each protected helper by:
1. current MATLAB `which(...)` binding,
2. exact expected SHA-256,
3. candidate paths under both `common_matlab/` and `common_matlab/reference/`.

It refuses a shadowed helper whose active SHA differs.

The failed R30 installer stopped during section [B], before shadow build or
live writes, so the repository remains the exact flown R29 checkpoint.

Run:
  AF7R58ZZY_R30_R2_INSTALL

Only after:
  SELFTEST CLOSED = 1
  R30-R2 INSTALL COMPLETE

run:
  AF7R58ZZY_R30_RUN_CANONICAL
  AF7R58ZZY_R30_REPORT
