AF7R25F RT-BOUNDED DIRECT REPLACEMENT

Replace ALL THREE source files together:
  REPLANNING_SYSTEM.m
  af7r36_core.hpp
  af7r36_single_core_mex.cpp

Then rebuild af7r36_single_core_mex exactly as for R25E.

What R25F changes:
1. The online generation+certificate+commit authority is now hard_modify_deadline_s
   (<=4 ms), not prediction_dt=25 ms.
2. R25 bell corridor rows are reduced EXACTLY:
      n'A >= b_i for all 25-ms cells  <=>  n'A >= max_i b_i.
   No feasible trajectory is lost or added by this reduction.
3. The family-6 fixed-sign QP is solved as independent 3D component QPs.
   This is algebraically identical because H25 is block diagonal and each row
   touches only one component amplitude A_j.
4. The unbounded adaptive-rejoin full-certificate loop is REMOVED from the
   synchronous Flight hot path. Flight uses the already-certified RAWbase
   one-shot witness. Compact rejoin must later be reintroduced only with a
   bounded/algebraic construction, never an unbounded repeated search.

Important:
This is a hard-real-time correction, not a claim that all cases are now <4 ms.
If the one-shot certified planner still exceeds 4 ms, R25F fails closed and the
next optimization target is the remaining single native call itself.

Native revision:
  AF7R25F_RT_BOUNDED_ZERO_JET_BELL_DIRECT_V1
