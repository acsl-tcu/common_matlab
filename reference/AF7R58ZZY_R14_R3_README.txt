AF7R58ZZY-R14-R3 | OBSERVER AOT RUNTIME AUDIT

Purpose:
- Fix R14-R2 live-interface mismatch only.
- Baseline snapshot gate and live optimized observer both feed the MEX contraction through the SAME seven numeric 3-vectors: p,q,pL,pT,vL,wL,w.
- No production install. No added warmup. No Arming/Takeoff/Landing work.
- Preserve trajectory/certificate/observer theorem/commit semantics.
- Save baselineRunError/optimizedRunError explicitly if a flight does not close.

Run:
  AF7R58ZZY_R14_R3_OBSERVER_AOT_RUNTIME_AUDIT

Return:
  AF7R58ZZY_R14_R3_REPORT.txt
  AF7R58ZZY_R14_R3_OUT.mat
  AF7R58ZZY_R14_R3_RAW_CAPTURE.mat
