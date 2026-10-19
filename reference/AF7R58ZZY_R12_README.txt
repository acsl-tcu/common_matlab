AF7R58ZZY-R12 FULL-PATH TIMING FORENSIC AUDIT

Purpose
- Diagnostic only. No production install and no new optimization.
- Corrects the R11 audit metric bug: R5 timing ring column 2 is do() total;
  column 3 is get_load_mass(), not do() total.
- Re-runs the exact R8-R2 baseline and exact R11 shadow.
- Decomposes full do(), Stage1/2/3, observer live-vs-hot replay,
  R10FAST live-vs-same-object replay/profile, post-commit reference packet,
  and unmeasured residuals.
- A local bucket improvement cannot mark overall performance closed.

Safety / semantics
- Production REPLANNING_SYSTEM.m is read-only.
- No Arming/Takeoff/Landing work is added.
- No trajectory math, C6/C7 contract, certificate acceptance, derivative order,
  activation time, stale/atomic gate, or commit policy is changed.
- R10FAST is used only after the same bitwise exactness gates as R11.

Run
  AF7R58ZZY_R12_FULL_PATH_FORENSIC_AUDIT

Return
  AF7R58ZZY_R12_REPORT.txt
  AF7R58ZZY_R12_OUT.mat
  AF7R58ZZY_R12_RAW_CAPTURE.mat

Interpretation
- diagnosticClosed=1 means the forensic measurements closed.
- performanceClosed is separate.  In particular, controlPeriod25msClosed=0
  means the measured target-window do() peak still exceeds 25 ms even if a
  local Stage2 bucket improved.
- timing is empirical; this audit does not claim formal WCET.
