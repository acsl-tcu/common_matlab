AF7R58ZZY-R8-R2 | EXACT KERNEL + C6 FORENSIC AUDIT

Purpose
- Fixes the R8 line-122 MATLAB syntax error only in the runner invocation path.
- Performs no production repair and no performance-changing modification.
- Adds no Arming, Takeoff, Landing, or warmup workload.
- Replays already-captured Flight observer inputs and the committed batch model only AFTER the mission to distinguish cold/JIT/allocation cost from steady kernel cost.

C6 / yaw audit
- Position: staged REFERENCE_ONLY active smoothness must be continuous and continuity_order >= 6; current source constructs order 7.
- Current runner yaw: gen_ref_line with direction [0,0,1] has no defined horizontal heading. Its constant yaw is C-infinity and is treated as a smooth heading hold for the purely vertical nominal segment.
- Repaired-direction yaw: current staged canonical evaluator changes XYZ derivative slots only and does not generate yaw from the repaired horizontal tangent. Therefore the intended direction-facing yaw behavior is reported separately as NOT IMPLEMENTED in the current staged path.
- Generic REFERENCE_ONLY yaw-C6 gate: audited separately and never silently treated as closed.

Files
1) AF7R58ZZY_R8_R2_KERNEL_C6_FORENSIC_AUDIT.m
2) REPLANNING_SYSTEM_R8R2_DIAG.m
3) REPLANNING_SYSTEM_R5R4_DIAG.m

Run
  AF7R58ZZY_R8_R2_KERNEL_C6_FORENSIC_AUDIT

Return
  AF7R58ZZY_R8_R2_REPORT.txt
  AF7R58ZZY_R8_R2_OUT.mat
  AF7R58ZZY_R8_R2_RAW_CAPTURE.mat

Do NOT modify REPLANNING_SYSTEM.m for this audit.
