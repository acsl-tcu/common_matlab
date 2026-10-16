AF7R38 V2 — syntax repair + 10 s attribution
=============================================

Root cause of the syntax error
------------------------------
The first R38 installer used an outer sprintf() to generate MATLAB source.
One inserted source line itself contained the format token "%s":

    assert(okPrime,'... failed: %s',reasonPrime);

The OUTER sprintf consumed that token while generating REPLANNING_SYSTEM.m,
which corrupted/truncated the generated character vector. This directly
matches:
    line 9761: 文字ベクトルの終端が正しくありません

It also explains why the first source audit saw:
    postcallback prime   = 0
    postcallback restore = 0

V2 escapes that inner token as %%s in the installer so the generated source
contains the intended literal %s.

Repair order
------------
1) Extract this package and add it to the path.

2) Run:
    R38fix = AF7R38_REPAIR_V2();

This automatically:
- preserves the malformed R38 source,
- restores the newest clean REPLANNING_SYSTEM_pre_R38_*.m backup,
- verifies that backup is R14D,
- installs the corrected R38,
- installs read-only 10 s commit attribution.

3) Reload:
    dbclear all
    clear classes
    clear functions
    rehash

4) Audit:
    S38v2 = AF7R38A_V2_SOURCE_AUDIT();
    S38v2.closed

Require 1.

5) Fresh run:
    run_flight_headless

6) Then:
    T25 = AF7R25A_HIGHRES_TIMING_AUDIT(logger);
    R28 = AF7R28_BLUE_SPIKE_ATTRIBUTION(runtime_profile,logger);
    T23 = AF7R23A_R9_STAGE_TIMING_AUDIT(logger);
    B38 = AF7R38B_10S_POSTCALLBACK_ATTRIBUTION(replanner_instance);

Why the 10 s problem is already known to be downstream
-------------------------------------------------------
In the previous fresh R14D run:
- callback QPC median was ~2.0 ms and max ~3.17 ms;
- at t=10.000 s callback QPC was ~1.815 ms;
- yet the sensor/replan stage was ~46.2 ms and status was -30.

Therefore the C++ trajectory generator is NOT the remaining 10 s bottleneck.

Source inspection shows two downstream contributors:
A) under the old code, EXECUTION_AWARE diagnostics were still evaluated before
   deadline handling even when REFERENCE_ONLY was the active commit policy;
B) try_commit_reference_only_proposal() rebuilds trigger sweep inflation,
   obstacle packet, sensor-horizon certificate, dependency signature and
   final assertions before it can close the atomic reference commit.

Corrected R38 removes A from the REFERENCE_ONLY <=4 ms path.
B38 directly measures the remaining reference-only commit prepare/rebuild/total
cost. If -30 remains, this tells us whether the actual atomic rebuild—not the
planner—is the remaining target.

No safety theorem, 4 ms threshold, R9 certificate, strict rejoin certificate,
or continuous whole-body certificate is weakened.
