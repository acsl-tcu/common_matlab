AF7R58ZZY-R13-R2 | Observer call-ordinal / shape forensic

Purpose:
- Syntax-fixed R13 forensic audit. Diagnostic only; no repair or production install.
- No added warmup calls. Existing two-event observer preflight calls are only observed.
- Compare preflight vs live observer input/output type/size/struct shape.
- Record global update/one-step/contraction invocation ordinals.
- Verify exact post-run replay and classify early-call JIT/allocator behavior.

R13-R2 fixes:
- Correct MATLAB quote escaping in the temporary-runner addpath construction.
- Correct MATLAB quote escaping in the run(tempRunner) command construction.
- No observer equation, trajectory, certificate, sensing, commit policy, or mission-phase semantics changed.

Run:
  AF7R58ZZY_R13_R2_OBSERVER_CALL_ORDINAL_FORENSIC_AUDIT

Return:
  AF7R58ZZY_R13_R2_REPORT.txt
  AF7R58ZZY_R13_R2_OUT.mat
  AF7R58ZZY_R13_R2_RAW_CAPTURE.mat
