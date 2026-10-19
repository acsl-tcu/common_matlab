AF7R58ZZY-R8 | EXACT KERNEL + C6 FORENSIC AUDIT

Purpose
- Do NOT modify production REPLANNING_SYSTEM.m.
- Do NOT add work to Arming, Takeoff, or Landing.
- Do NOT add mission-phase warmup.
- Preserve current trajectory/certificate/commit mathematics.
- Diagnose whether observer and AF7R33 batch spikes are cold/JIT/allocation dominated or steady arithmetic dominated.
- Audit C6 closure separately for the current run and for the generic staged REFERENCE_ONLY contract.

Files
- AF7R58ZZY_R8_KERNEL_C6_FORENSIC_AUDIT.m
- REPLANNING_SYSTEM_R8_DIAG.m
- REPLANNING_SYSTEM_R5R4_DIAG.m

Run
1. Put all three .m files in the same reference folder.
2. Do not modify REPLANNING_SYSTEM.m.
3. Run:
     AF7R58ZZY_R8_KERNEL_C6_FORENSIC_AUDIT

R8 behavior
- The mission run uses the same production observer/reference equations, with diagnostic timing and value capture only.
- Exact observer inputs already computed at Flight entry are stored for post-run replay.
- After the 50-s mission has ended, R8 repeats the exact one-step/contraction kernels and the exact AF7R33 161-point batch evaluation.
- MATLAB profiler is also run only after the mission.
- Post-run replay therefore does not shift runtime cost into Arming/Takeoff/Landing.

C6 audit
- Position: active staged REFERENCE_ONLY smoothness must be certified continuous with order >= 6 (current source produces order 7 / C7).
- Current runner yaw: gen_ref_line yaw must be source-constant and the canonical correction evaluator must leave yaw slots untouched.
- Generic staged contract: R8 separately reports whether the REFERENCE_ONLY active gate explicitly revalidates yaw-C6/yaw-policy. A missing generic gate is reported as OPEN; it is not silently treated as closed.
- Bezier endpoint/internal d0..d7 residuals are reported only as supplementary diagnostics, not as replacement for the continuous-time certificate.

Outputs
- AF7R58ZZY_R8_REPORT.txt
- AF7R58ZZY_R8_OUT.mat
- AF7R58ZZY_R8_RAW_CAPTURE.mat

Interpretation
- COLD_JIT_ALLOCATION_DOMINANT: live Flight cost is >=3x post-run hot replay.
- MIXED_COLD_AND_STEADY: live Flight cost is >=1.5x and <3x hot replay.
- STEADY_KERNEL_DOMINANT: live Flight cost is <1.5x hot replay.

No repair is installed by R8. Any later production repair must preserve or improve performance relative to the current baseline and must not weaken C6/safety/certificate contracts.
