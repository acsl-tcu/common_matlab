AF7R58ZZY-R16-R2

Purpose
- Fix R16 vector-time call into scalar-time symbolic matlabFunction handles.
- Preserve the exact r={0,2} reduced evaluator and existing Stage2/runtime-packet optimizations.
- No controller/HLC/plant/certificate/trajectory-math changes.

Install
1. Put all package files in the reference folder or add this folder to the MATLAB path.
2. Run: AF7R58ZZY_R16_R2_INSTALL
3. Run normally: run_flight_headless

Rollback
AF7R58ZZY_R16_R2_ROLLBACK

The R16-R2 core evaluates nominal flat derivative handles one scalar time at a time, matching the canonical scalar evaluator contract.
