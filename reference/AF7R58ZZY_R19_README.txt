AF7R58ZZY-R19 | NATIVE CORRECTION MAPPING + EXACT LEAN STAGE2

Purpose
-------
R18 showed that the dominant postcache cost is MATLAB orchestration, especially
per-point segment mapping, while the de Casteljau numeric work itself is tiny.
R19 moves ONLY correction segment mapping + r=0/r=2 derivative evaluation into
one strict-FP MEX call.

Important
---------
- AF7R36 op=2 is deliberately NOT used. The live check showed position differed
  from the R17 nominal-table authority by 1.1368683772161603e-13 and was slower.
- Nominal position/acceleration still come from the exact R19 retained dt-grid tables.
- Correction groups are accumulated in MATLAB in original group order and only
  on active columns, matching R17 semantics.
- Installer performs multi-group/multi-segment bitwise exactness tests against
  af7r73_posaccel_core_R17 BEFORE switching the runner.
- REPLANNING_SYSTEM.m, controller, plant, certificate acceptance and commit policy
  are unchanged.

Install
-------
AF7R58ZZY_R19_INSTALL
run_flight_headless

After run
---------
S = replanner_instance.get_staged_pipeline_status();
R = replanner_instance.get_r19_performance_status();
fprintf('Stage1 = %.6f ms\n',S.last_stage1_ms);
fprintf('Stage2 = %.6f ms\n',S.last_stage2_ms);
fprintf('Stage3 = %.6f ms\n',S.last_stage3_ms);
fprintf('postcache = %.6f ms\n',R.postcache_ms);
fprintf('packet = %.6f ms\n',R.packet_ms);
fprintf('batch model = %.6f ms\n',R.batch_model_ms);
fprintf('prepare total = %.6f ms\n',R.prepare_total_ms);
fprintf('commit/fail = %g/%g\n',S.commit_count,S.fail_count);

Rollback
--------
AF7R58ZZY_R19_ROLLBACK
