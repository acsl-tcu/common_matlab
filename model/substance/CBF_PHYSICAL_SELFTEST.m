% CBF_PHYSICAL_SELFTEST.m
% Numerical consistency test of S3 physical HOCBF derivatives only.
% NOT a proof of closed-loop safety and NOT a flight simulation.
clear Generate_CBF_ActuatorAware_6Methods
cfg=struct();
cfg.modes={'physical'};
cfg.methods={'S3'};
cfg.smokeTest=true;
cfg.writeFunctions=false;
T=Generate_CBF_ActuatorAware_6Methods('selftest',cfg);
disp(T);
assert(height(T)==3,'Expected three body components (uav/cable/load).');
assert(all(strcmp(string(T.status),"generated+sample_audited")), ...
    'Some physical/S3 derivative sample audit failed.');
fprintf('[SELFTEST] physical/S3 sample derivative audit PASSED.\n');
fprintf('[SELFTEST] Reminder: no sample-data, feasibility, or safety theorem established.\n');
