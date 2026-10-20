AF7R58ZZY-R26
==============

Purpose
-------
Reduce the remaining R25 replan spike without weakening trajectory/certificate math.

Changes
-------
1. Keep the R25 exact fast merge.
2. Turn internal detailed profiling and light-runtime audit OFF for the clean timing run.
3. Rebalance the synchronous replanning pipeline over four visits:
   trigger cycle       : exact Stage1 candidate generation
   trigger + 1*dt      : exact reference certificate
   trigger + 2*dt      : exact R21 core bundle preparation
   trigger + 3*dt      : exact R23 correction compile + unchanged R25 transactional activation
4. Scheduled activation is therefore trigger+75 ms at dt=25 ms.
5. Fixed collision-prediction horizon remains H=4 s.
6. Full freshness is checked before each consuming stage.
7. No production REPLANNING_SYSTEM.m modification.

Why
---
R25 reduced steady median from 10.1812 to 6.8827 ms and Stage1/Stage3 online load,
but Stage2 remained 29.3305 ms. R26 redistributes Stage2 work instead of weakening it.

Run
---
AF7R58ZZY_R26_INSTALL
run_flight_headless
AF7R58ZZY_R26_REPORT

Return
------
AF7R58ZZY_R26_REPORT.txt
AF7R58ZZY_R26_OUT.mat
