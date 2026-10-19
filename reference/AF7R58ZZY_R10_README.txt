AF7R58ZZY-R10 | Stage2 direct exact-MEX connection audit

Purpose
- Verify the R9-R3 exact de Casteljau MEX in the REAL Stage2 postcache call.
- Do NOT optimize or change the observer in this run.
- Do NOT modify production REPLANNING_SYSTEM.m or the production AF7R33 helper.
- Do NOT move work into Arming / Takeoff / Landing.

What changes in the shadow class
- REPLANNING_SYSTEM_R10_DIAG inherits REPLANNING_SYSTEM_R8R2_DIAG.
- It overrides only prepare_staged_reference_only_bundle().
- That method is the R5-R4 source-bound diagnostic method, with one computational substitution:
    af7r33_flat_jet_batch_model(obj,tqCache,batchModel)
  becomes
    af7r33_flat_jet_batch_model_R10FAST(obj,tqCache,batchModel)
- R10FAST is generated at runtime from the exact current AF7R33 source.
- The local de Casteljau recurrence uses MEX only after local and full 21xN bitwise equivalence pass.

Run
  AF7R58ZZY_R10_STAGE2_DIRECT_MEX_AUDIT

Expected success status
  STAGE2_DIRECT_MEX_R10_COMPLETE

Return
  AF7R58ZZY_R10_REPORT.txt
  AF7R58ZZY_R10_OUT.mat
  AF7R58ZZY_R10_RAW_CAPTURE.mat

Success requires
- full R10FAST helper bitwise exact
- stage2FastRealCalls >= 1
- Stage2 postcache <= 0.50 of same-run baseline
- Stage2 total <= 0.80 of same-run baseline

Notes
- A configured C++ MEX compiler is required.
- Production installation is forbidden in this package; this is shadow validation only.
