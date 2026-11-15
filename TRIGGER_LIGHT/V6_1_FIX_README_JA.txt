V6.1 GOVERNOR TRACE FIX — PATCH for existing common_matlab/TRIGGER_LIGHT

ROOT CAUSE (from user's actual V6 results MAT):
Original 6-second four-case results ALL FAIL after source rollout because
wp_six_trigger_predict_hlc_4s.m lacks field "governorRateState".
V6 baseline package did not include the newer function. The failure does
not establish a physical instability at 6s.

PATCH FILES:
  SIX_WAY/wp_six_trigger_predict_hlc_4s.m (records 6 governor states)
  TESTS/TEST_TRIGGER_V6_HLC_STRESS_6S.m (runtime governor trace contract)
  TESTS/TEST_TRIGGER_V6_SOURCE_CONTRACT.m (static preflight)
  RUN_PURE_TRIGGER_V6_CAUSAL_AUDIT.m (preflight; case error in results)
  PURE_PORT_CUMULATIVE_HANDOFF.txt (accumulating previous work)

Install: unzip to common_matlab (overwrite same relative paths).
Do NOT put files directly at repo root. Do NOT modify original source HLC,
24-state RHS, EKF, Flight, CBF or the V2/V3/V4 geometric alternatives.

MATLAB:
  cd('C:\Users\student\Documents\GitHub\common_matlab')
  addpath(genpath(pwd))
  cd('TRIGGER_LIGHT')
  clear classes
  rehash
  which wp_six_trigger_predict_hlc_4s -all
  C=TEST_TRIGGER_V6_SOURCE_CONTRACT(); disp(C)
  R=RUN_PURE_TRIGGER_V6_CAUSAL_AUDIT(false);
  disp(R.status)
  if isstruct(R.causalForecast) && isfield(R.causalForecast,'records')
      disp(struct2table(R.causalForecast.records))
      disp(struct2table(R.causalForecast.noiseRecords))
  else
      disp(R.errorText)
  end
Results ZIP: TRIGGER_LIGHT/TESTS/RESULTS (unique timestamp).

Boundary: 6s is OFFLINE reference; nominal modal governor 4s is NOT
physically-certified, NOT full EKF, NOT saturated 4s validated,
NOT full 1-2ms geometry+predictor time qualified.
