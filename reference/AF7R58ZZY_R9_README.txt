AF7R58ZZY-R9 | PERFORMANCE-ONLY SHADOW REPAIR AUDIT

Place all package files together in the reference folder. Do NOT modify REPLANNING_SYSTEM.m.

Run:
    AF7R58ZZY_R9_PERFORMANCE_REPAIR_AUDIT

R9 automatically performs:
  1) one baseline 50-s Flight using the exact current implementation,
  2) extraction of the CURRENT af7r33_flat_jet_batch_model local decasteljau_batch,
  3) runtime MEX compilation and bitwise equivalence checks,
  4) one optimized 50-s shadow Flight only if exactness closes,
  5) observer exact-kernel pre-mission equivalence and post-mission replay checks,
  6) fail-closed performance comparison.

No production source is edited. No Arming/Takeoff/Landing work is added.
The diagnostic adds a one-time pre-mission equivalence check only.

Success status:
    PERFORMANCE_SHADOW_REPAIR_R9_COMPLETE

Return these three files:
    AF7R58ZZY_R9_REPORT.txt
    AF7R58ZZY_R9_OUT.mat
    AF7R58ZZY_R9_RAW_CAPTURE.mat

Important:
- A supported C/C++ MEX compiler must be configured in MATLAB (mex -setup C++).
- If local or full-helper bitwise identity fails, the optimized Flight is NOT run.
- R9 never reduces derivative order and never changes trajectory/certificate/commit equations.
