AF7R58ZZY-R7 | NO-PHASE-SHIFT REPAIR VALIDATION
================================================

PURPOSE
- Validate the first implementation repair after R6-R2 root-cause closure.
- Production REPLANNING_SYSTEM.m is NOT modified by this package.
- No additional work is moved into Arming, Takeoff, Landing, do() phase dispatch,
  setup-prime calls, or warm-up calls.

R7 CHANGES UNDER TEST
1) Observer Flight-time implementation:
   - reduce repeated nested-struct copy/write work;
   - reuse scalar trig terms inside the same equations;
   - replace 12 scalar positive nextfloat-up calls by the exact IEEE-754 vector form.
   - observer theorem/equations and hard-support acceptance remain unchanged.

2) Repaired future-reference packet:
   - future collision packet requires pL and q;
   - q is formed from position acceleration plus gravity;
   - therefore cache construction evaluates only derivative orders 0 and 2,
     rather than computing the unused 1,3,4,5,6 derivative orders for all 161 points;
   - same canonical correction support, segment selection, degree-15 derivative
     recursion, de Casteljau order and correction-addition order are retained.

POST-RUN EQUIVALENCE GATE
- R7 compares the p/a-only evaluator against the current full AF7R33 0..6 batch
  evaluator over the full horizon plus correction/segment boundaries.
- p and acceleration must be bitwise identical and max absolute error must be 0.
- optimized observer one-step, measured-error and two-event contraction results
  must isequaln the copied legacy equations.
- vector nextfloat-up must be bitwise identical to production scalar nextfloat-up.
- If any gate is open, status is INCOMPLETE_FAIL_CLOSED and no repair conclusion
  is accepted.

EXACT BINDINGS
Production REPLANNING_SYSTEM.m:
  310cfdd01f9554a80b71e8d90464816a721859709ad7c4b9137cdc0f3a6eaf49
Current run_flight_headless.m:
  d4d15d081ea8a6e3073b0a63ce36e47f983777ac2c81db194384de69f5e5fc37
R5-R4 parent diagnostic:
  2b874c2e92052de1c756c3b00f8f62f5cb08018a496b9c78b981df085f2efb18
R7 diagnostic:
  e9819541c3bdc079cf0700ca1114b145d3d9ce70f5a95ca96fee8f745cc039e1
p/a scalar helper:
  1529ae7774fabd8b7a991dcd449a1548b08a66b6be752553cf81fa6f8f0508c9
p/a batch helper:
  32ed5100e2a35e8dba0d1b9e3ee6613d47875aec6ba69924d90af66e1daa80ac
R7 audit entrypoint:
  a0e2946be9d95a8aaed4c2526f50445ad4d1d74aab2c3b8f3beaa073237f7ea9

INSTALL / RUN
Place these five .m files in the same reference folder (or keep them together
on MATLAB path). Do NOT replace REPLANNING_SYSTEM.m.

  AF7R58ZZY_R7_NO_PHASE_SHIFT_REPAIR_AUDIT.m
  REPLANNING_SYSTEM_R7_DIAG.m
  REPLANNING_SYSTEM_R5R4_DIAG.m
  af7r59_packet_pa_scalar.m
  af7r59_packet_pa_batch.m

Run:
  AF7R58ZZY_R7_NO_PHASE_SHIFT_REPAIR_AUDIT

RETURN
  AF7R58ZZY_R7_REPORT.txt
  AF7R58ZZY_R7_OUT.mat
  AF7R58ZZY_R7_RAW_CAPTURE.mat

INTERPRETATION
This is a shadow validation, not the final production install. If equivalence
closes and the measured Flight-time costs improve, the next step is a
transactional production installer containing only these validated changes.
