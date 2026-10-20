AF7R58ZZY-R18 Stage2 Fine-Grain Diagnostic

Purpose:
- Do NOT optimize.
- Decompose the single R17 Stage2 event into meaningful sub-blocks.
- R17 measured Stage2 remains the performance authority because R18 adds timing calls.

Install:
  AF7R58ZZY_R18_INSTALL
  run_flight_headless

After run:
  S = replanner_instance.get_staged_pipeline_status();
  R = replanner_instance.get_r18_stage2_status();
  disp(R)

Rollback:
  AF7R58ZZY_R18_ROLLBACK

No changes to REPLANNING_SYSTEM.m, controller, HLC, plant, trajectory mathematics,
continuous certificate acceptance, commit policy, or mission-phase work placement.
