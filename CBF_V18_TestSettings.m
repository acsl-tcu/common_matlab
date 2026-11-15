function s=CBF_V18_TestSettings()
% V18 focus R193 parameter-class DEFAULTS, not proven current flight values.
% Never changes prior V11-V17 test fixtures, generators, or HLC classes.
s=test_CBF_Fixture();
s.physical.massDrone=.762;
s.physical.massLoad=.0556;
s.physical.cableLength=.46;
s.physical.gravity=9.81;
s.physical.inertia=[.02237568;.02985236;.0480374];
% Unverified physical sizing/actuator box: retains V16 test assumptions.
% Before real deployment, replace cableRadius, margin, input limits with
% measured values and confirm live flight's actual configuration.
end
