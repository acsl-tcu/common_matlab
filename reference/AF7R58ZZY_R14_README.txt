AF7R58ZZY-R14 | AOT/MEX OBSERVER SHADOW + FULL RUNTIME AUDIT

Purpose
- Test the R13-R4 conclusion with an AOT/MEX observer numeric kernel.
- No production install and no added observer warmup call.
- Preserve exact observer theorem/output contract; require bitwise equality.
- Compare the runner's whole-path runtime_profile before/after.

Run
  AF7R58ZZY_R14_OBSERVER_AOT_RUNTIME_AUDIT

Return
  AF7R58ZZY_R14_REPORT.txt
  AF7R58ZZY_R14_OUT.mat
  AF7R58ZZY_R14_RAW_CAPTURE.mat

Interpretation
- online_before_plant_s is the 25 ms control-period authority.
- full_iteration_s includes numerical plant + logger and is reported as
  simulation context only; it is NOT substituted for the control deadline.
- The audit runs a baseline diagnostic flight, compiles/tests the MEX,
  explicitly unloads the MEX so the tests cannot serve as extra warmup,
  then runs the optimized shadow flight. The existing observer preflight
  sequence remains unchanged.
- If neither compiled mode is bitwise exact, optimized flight is forbidden.
- R14 does not include the R11 Stage2 repair; observer acceleration is being
  isolated first. Global 25 ms closure may therefore remain open near replan.
