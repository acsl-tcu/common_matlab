AF7R58ZZY-R13-R3 | OBSERVER CALL-ORDINAL / SHAPE FORENSIC

Purpose
- Diagnostic only. No production modification and no added warmup.
- Fixes R13-R2 authority binding failure without weakening source binding.
- Distinguishes RAW_EXACT, TEXT_EXACT_EOL_BOM_ONLY, PATH_SHADOW, and TRUE_CONTENT_MISMATCH.
- Uses run_flight_headless.m sibling REPLANNING_SYSTEM.m as the runner-local authority.
- Bundles a read-only canonical authority snapshot solely for normalized-text comparison.

Run
  AF7R58ZZY_R13_R3_OBSERVER_CALL_ORDINAL_FORENSIC_AUDIT

Expected output
  AF7R58ZZY_R13_R3_REPORT.txt
  AF7R58ZZY_R13_R3_OUT.mat
  AF7R58ZZY_R13_R3_RAW_CAPTURE.mat

Safety / authority
- No source file is modified.
- CRLF/LF and UTF-8 BOM differences are accepted only if normalized source text is byte-for-byte identical to the bundled canonical snapshot.
- Any real source-content difference remains fail-closed and prints the first differing line.
- MATLAB path shadow is corrected only by path priority for the diagnostic run; no file is copied/overwritten.
