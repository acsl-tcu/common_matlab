AF7R58ZZY-R28-R3
=================

R3 corrects the R2 INSTALLER PARSE ERROR only.

R2 failed before execution because generated fprintf character vectors were
split across physical lines. Therefore R2 performed no repository writes.

R28 theory/payload is unchanged from R2:
- generator mathematics unchanged
- continuous-time certificate mathematics unchanged
- schedule unchanged for measurement
- controller unchanged
- dependent compiled representation produced by the native generator

The installer recognizes either:
1) exact R27 baseline AF7R36 cpp, or
2) the exact known R28-V1 extraction-collision cpp.

Run:
  AF7R58ZZY_R28_R3_INSTALL

Only if SELFTEST CLOSED=1 and R28-R3 INSTALL COMPLETE are printed:
  AF7R58ZZY_R28_RUN_CANONICAL
  AF7R58ZZY_R28_REPORT
