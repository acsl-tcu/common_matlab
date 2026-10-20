AF7R58ZZY-R16 | STAGE2 + POSTCOMMIT EXACT LEAN PERFORMANCE IMPLEMENTATION

Purpose
- Continue from installed R15 without touching controller code.
- Reduce staged Stage2 and repaired-reference packet time only.

Exact implementation changes
1) Observer AOT/MEX from R15 is retained unchanged.
2) Staged Stage2 now calls the production exact runtime-packet reuse theorem.
   If the equality gate is false, the original rebuild path is executed.
3) The repaired-reference packet evaluator computes only flat derivative orders
   r=0 (position) and r=2 (acceleration), because only those rows are consumed
   to build pL and q. Degree-15 derivative recursion and de Casteljau arithmetic
   are unchanged. r=1,3,4,5,6 are not evaluated on this packet-only path.
4) The same reduced evaluator is used for Stage2 post-commit cache and rolling
   postcommit tail/full packet updates.

Not changed
- REPLANNING_SYSTEM.m production source
- Stage1 candidate mathematics
- Stage2 certificate acceptance
- Stage3 commit/freshness semantics
- continuous-time geometry/smoothness certificates
- controller / plant / HLC
- Arming / Takeoff / Landing workload

Install from current R15 state:
  AF7R58ZZY_R16_INSTALL

Then run normally:
  run_flight_headless

After the run, print the key times:
  S = replanner_instance.get_staged_pipeline_status();
  fprintf('class=%s\n',class(replanner_instance));
  fprintf('Stage1=%.6f ms | Stage2=%.6f ms | Stage3=%.6f ms\n', ...
      S.last_stage1_ms,S.last_stage2_ms,S.last_stage3_ms);

Rollback to the exact pre-R16 runner state:
  AF7R58ZZY_R16_ROLLBACK

Primary acceptance target for this iteration
- Stage2: improve materially from R15 measured 9.7973 ms.
- Then inspect 9.85--10.025 s Online-before-plant profile.
- Do not use the unrelated controller spike as an R16 acceptance criterion.
