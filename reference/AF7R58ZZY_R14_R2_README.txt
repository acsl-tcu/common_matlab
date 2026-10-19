AF7R58ZZY-R14-R2 | OBSERVER AOT/MEX RUNTIME AUDIT

PURPOSE
  Narrow repair of R14 verification harness only.
  R14 compiled the MEX successfully but its bitwise comparator called
  isdouble(a) on struct outputs; MATLAB therefore stopped before testing
  any snapshot (mode0Tested=0, mode1Tested=0).

ONLY FUNCTIONAL AUDIT CHANGE
  bitwise_equal_r14:
      if isdouble(a)
  ->  if isa(a,'double')

NO observer equations, MEX equations, trajectory mathematics, certificate
acceptance, commit policy, mission-phase work, or production source are changed.
No added warmup calls are introduced.

RUN
  AF7R58ZZY_R14_R2_OBSERVER_AOT_RUNTIME_AUDIT

RETURN
  AF7R58ZZY_R14_R2_REPORT.txt
  AF7R58ZZY_R14_R2_OUT.mat
  AF7R58ZZY_R14_R2_RAW_CAPTURE.mat

NOTE
  The prior R14 MAT files do not contain the baseline observer object/snapshot
  structures needed to re-run the bitwise gate. Therefore R14-R2 deliberately
  re-runs the baseline flight before the corrected exactness gate.
