AF7R58ZZY-R13 | Observer call-ordinal / shape forensic

Purpose:
- Diagnostic only. No repair, no production install.
- No added warmup calls. Existing preflight observer calls are only observed.
- Compare preflight vs live observer input/output type/size/struct shape.
- Record global update/one-step/contraction invocation ordinals.
- Verify exact post-run replay and classify early-call JIT/allocator behavior.

Run:
  AF7R58ZZY_R13_OBSERVER_CALL_ORDINAL_FORENSIC_AUDIT

Return:
  AF7R58ZZY_R13_REPORT.txt
  AF7R58ZZY_R13_OUT.mat
  AF7R58ZZY_R13_RAW_CAPTURE.mat

No C6/C7, trajectory, certificate, sensing, commit-policy, Arming, Takeoff or Landing semantics are changed.
