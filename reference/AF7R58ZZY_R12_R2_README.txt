AF7R58ZZY R12-R2

Purpose:
- Fix the R12 report-writer struct indexing bug at observer.events.
- Recover the completed R12 forensic analysis from the EXISTING R12 RAW capture.
- No simulation rerun is needed for POSTRUN_RECOVERY.
- No production source modification.
- No Arming/Takeoff/Landing work.

Recommended now:
1. Put AF7R58ZZY_R12_R2_POSTRUN_RECOVERY.m beside AF7R58ZZY_R12_RAW_CAPTURE.mat.
2. Run:
   AF7R58ZZY_R12_R2_POSTRUN_RECOVERY
3. It writes:
   AF7R58ZZY_R12_R2_RECOVERED_REPORT.txt
   AF7R58ZZY_R12_R2_OUT.mat

The FULL_PATH_FORENSIC_AUDIT variant is included only for a future clean rerun if needed.
The only semantic fix to the report writer is robust struct/cell event indexing.
