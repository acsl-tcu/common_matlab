AF7R58ZZY-R11 | PRE-MISSION SAME-CLASS FAST-HELPER SPECIALIZATION

Purpose
- Fix ONLY the R10 first-real-Stage2 cold/JIT specialization spike.
- Preserve R10 exact direct Stage2 MEX connection.
- Preserve trajectory mathematics, derivative order 0..6, certificates, commit policy and observer path.
- Do NOT add work to real Arming, Takeoff or Landing.

Why R11
R10 proved the exact MEX connection is correct but its first real Stage2 call was cold:
  first real postcache ~33 ms, later retry ~3.7 ms.
R11 reuses the production's EXISTING pre-motion successful-candidate/full-packet prime.
It replaces only that already-existing full batch helper call with the SAME R10FAST helper,
thereby specializing the exact helper for the SAME R11 object class before real Arming.
No additional prime loop/call is introduced.

Run
1. Place all files in this package in the same reference folder.
2. Keep production REPLANNING_SYSTEM.m unchanged.
3. Run:
   AF7R58ZZY_R11_PREMISSION_FAST_SPECIALIZATION_AUDIT
4. Return:
   AF7R58ZZY_R11_REPORT.txt
   AF7R58ZZY_R11_OUT.mat
   AF7R58ZZY_R11_RAW_CAPTURE.mat

R11 success requires, among other gates:
- local de Casteljau MEX bitwise exact
- full 21xN AF7R33 helper bitwise exact
- existing pre-motion full-packet prime actually used the fast helper
- real Stage2 actually used the fast helper
- no second Stage2 retry
- Stage2 postcache and total both improve materially
- production remains read-only
