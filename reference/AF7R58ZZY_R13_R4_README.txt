AF7R58ZZY-R13-R4 | Observer call-ordinal / shape forensic

Purpose:
- Diagnostic only; no performance repair, production install, or added warmup.
- Compare the existing observer preflight calls and live Flight observer calls.
- Record update / one-step / contraction call ordinals and input/output structure shapes.
- Use exact replay to distinguish observer mathematics from live first-use/allocation cost.

R13-R4 corrections after R13-R2/R3 audit failures:
- Removed the false assumption that REPLANNING_SYSTEM.m must be in the same folder as run_flight_headless.m.
- Restored the exact SHA-256 byte-reading implementation already proven by R12 (fread(...,'*uint8') + MessageDigest).
- Authority is the actual MATLAB-resolved which('REPLANNING_SYSTEM') path, exactly as in R12.
- On a real SHA mismatch, prints every which(...,'-all') candidate and SHA; it never silently switches source files.
- R13 observer equations/classes are unchanged from R13-R2.

Run:
  AF7R58ZZY_R13_R4_OBSERVER_CALL_ORDINAL_FORENSIC_AUDIT

Return:
  AF7R58ZZY_R13_R4_REPORT.txt
  AF7R58ZZY_R13_R4_OUT.mat
  AF7R58ZZY_R13_R4_RAW_CAPTURE.mat
