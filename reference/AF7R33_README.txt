AF7R33 / R13C — repeated-replan-safe production integration
==============================================================

Why this stage exists
---------------------
R13A/R13B proved large speedups with exact/equivalent batch geometry and QP,
but the R13A V3 flat evaluator was NOMINAL-only. Promoting that directly would
make a later replan on top of an already REPAIRED reference incorrect.

R13C fixes that architectural issue before production promotion.

Core representation
-------------------
Every committed piecewise repair is stored explicitly as its degree-15 control
points and global segment times.

Because a repair is additive to the previous active flat output and rejoins
with zero endpoint jets, the active flat reference is represented exactly as:

    nominal flat jet + sum(all committed piecewise correction jets)

This supports repeated replanning without silently reverting to NOMINAL.

Production changes after the preinstall gate closes
----------------------------------------------------
- all-obstacle level-batched R10A episode cover
- batched R10B corridor construction
- exact native small-QP fast path, with unchanged quadprog fallback
- batched continuous whole-body theorem, with unchanged legacy fallback on
  helper error/malformed result
- explicit repeated-replan flat model stored at reference-only commit
- flat-batch warmup before Flight

Unchanged
---------
- C7 piecewise towing structure
- affine unsafe correction / towing geometry theorem
- R9 Stage A/B speed theorem
- strict rejoin
- continuous whole-body support theorem
- reason-code semantics
- hard 4 ms modification deadline
- sampled acceptance remains forbidden

Run order
---------
Keep the R31 geometry MEX available:
    af7r31_geometry_batch_mex.mexw64

1) Build the new paired-obstacle episode MEX:
    M33 = AF7R33_BUILD_EPISODE_PAIR_MEX();

2) If the exact QP MEX is no longer present, rebuild it:
    Q32 = AF7R32_BUILD_EXACT_QP_MEX();

3) Run the full preinstall gate on the stored true trigger:
    G33 = AF7R33C_PREINSTALL_GATE(replanner_instance);

Require:
    G33.closed == 1

4) Install:
    I33 = AF7R33_INSTALL_R13C_PRODUCTION();

5) Reload class:
    dbclear all
    clear classes
    clear functions
    rehash

6) Audit source:
    S33 = AF7R33D_R13C_SOURCE_AUDIT();
Require:
    S33.closed == 1

7) NOW run a fresh:
    run_flight_headless

8) Measure:
    T25 = AF7R25A_HIGHRES_TIMING_AUDIT(logger);
    T24 = AF7R24A_R9_SYNTHESIS_TIMING_AUDIT(logger);
    R28 = AF7R28_BLUE_SPIKE_ATTRIBUTION(runtime_profile,logger);
    if exist('AF7R23A_R9_STAGE_TIMING_AUDIT','file')==2
        T23 = AF7R23A_R9_STAGE_TIMING_AUDIT(logger);
    end

Important
---------
The first fresh run after installation is the important result. Shadow replay
timing is no longer enough once production is patched.

If R13C brings the generator close to but not below 4 ms, the next target is
the remaining R9 local-vector + whole-body adaptive engine in one compiled
kernel. At that point QP/rejoin are already too small to matter.
