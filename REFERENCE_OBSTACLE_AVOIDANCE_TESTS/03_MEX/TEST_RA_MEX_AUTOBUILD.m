function R=TEST_RA_MEX_AUTOBUILD()
% Should not fail all testing when no C compiler exists.
agent=RA_TEST_AGENT();cover='multiple_ellipsoids';
obs=RA_TEST_WORLD(cover,'near');
P=reference_avoidance_prepare_observed(agent,'C',cover,obs);
M=P.mex;
assert(strcmp(P.cfg.systemMode,'C'));
assert(isequal(logical(P.cfg.useMexCChainPreclear),logical(M.available)));
M2=reference_avoidance_mex_ensure('ensure');
if M.available
    assert(strcmp(M2.status,'READY_VERIFIED'));
    V=reference_avoidance_mex_verify();
    assert(strcmp(V.status,'PASS_NUMERIC_PARITY'));
else
    V=struct('status','SKIPPED_NO_NATIVE','tested',0);
end
% Execute native-enabled/disabled selected C geometry with equal inputs.
F=RA_TEST_FORECAST(P.cfg);
G1=wp_six_trigger_evaluate_fast_v8(P.known,P.cfg,F);
cfg=P.cfg;cfg.useMexCChainPreclear=false;
G2=wp_six_trigger_evaluate_fast_v8(P.known,cfg,F);
assert(strcmp(G1.status,G2.status),'Native altered selected C classification');
% B never compiles and never enables C-only native code.
B=reference_avoidance_prepare_observed(agent,'B',cover,obs);
assert(~B.cfg.useMexCChainPreclear);
R=struct('status','PASS_WITH_NATIVE_OR_FALLBACK','native',M, ...
    'parity',V,'selectedCClassificationEqual',true, ...
    'safetyCertified',false);
end
