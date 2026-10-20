AF7R58ZZY-R20 | LEAN STAGE2 INTEGRATION

R19 accounting closed exactly:
  measured subtotal  7.2682 ms
  prepare residual   1.4750 ms
  Stage2 wrapper     0.8886 ms
  total              9.6318 ms

R20 addresses the whole remaining Stage2 path rather than one isolated kernel:
- remove unused read-only evaluation struct construction from staged hot path
  while preserving the same reference-layer accept/reject gate;
- memoize the finite resource contract against exact frozen table/time keys;
  candidate prune/count/control-point validation remains live in Stage2;
- when speed bounds differ, skip the production deep reuse test because its
  first necessary equality is already false; rebuild speed-dependent packet
  values from the Stage1-fresh static authority using unchanged formulas;
- direct-accumulate correction groups into the contiguous nominal-table slice
  in one strict-FP MEX call. Installer proves bitwise equivalence to R19 core.

No AF7R36 op=2 evaluator is used.
No controller/HLC/plant/trajectory/certificate/commit-policy change.
REPLANNING_SYSTEM.m is not modified.

Install:
  AF7R58ZZY_R20_INSTALL
  run_flight_headless

After run:
  S = replanner_instance.get_staged_pipeline_status();
  R = replanner_instance.get_r20_performance_status();
  fprintf('Stage1 = %.6f ms\n',S.last_stage1_ms);
  fprintf('Stage2 = %.6f ms\n',S.last_stage2_ms);
  fprintf('Stage3 = %.6f ms\n',S.last_stage3_ms);
  fprintf('prepare = %.6f ms\n',R.prepare_total_ms);
  fprintf('batch = %.6f ms\n',R.batch_model_ms);
  fprintf('packet = %.6f ms\n',R.packet_ms);
  fprintf('handles = %.6f ms\n',R.handles_ms);
  fprintf('postcache = %.6f ms\n',R.postcache_ms);
  fprintf('bundle = %.6f ms\n',R.bundle_ms);
  fprintf('residual = %.6f ms\n',R.prepare_residual_ms);
  fprintf('wrapper = %.6f ms\n',R.stage2_wrapper_ms);
  fprintf('fresh gate = %.6f ms\n',R.fresh_gate_ms);
  fprintf('cache/freshPacket/postFast = %d/%d/%d\n', ...
      R.resource_cache_used,R.fresh_packet_path_used,R.postcache_fast_used);
  fprintf('commit/fail = %g/%g\n',S.commit_count,S.fail_count);

Rollback:
  AF7R58ZZY_R20_ROLLBACK
