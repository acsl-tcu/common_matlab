function R=TEST_TRIGGER_V5_GOVERNOR_EQUIVALENCE(nTests)
% Compare source wp_governor_step and cached-coeff equivalent. This is a
% source-equivalence test, NOT physical prediction error certification.
if nargin<1,nTests=1200;end
assert(exist('wp_governor_step','file')==2,...
    'Original wp_governor_step.m must be on MATLAB path');
rng(20261113,'twister');
maxZ=0;maxTau=0;maxDer=0;
for k=1:nTests
    dt=.025; T=.2+1.5*rand;
    C=wp_six_trigger_governor_fast_coeff(dt,T);
    z=.02+1.8*rand(6,1);tau=5*rand;cmd=rand;
    [za,ta,da]=wp_governor_step(z,tau,cmd,dt,T);
    [zb,tb,db]=wp_six_trigger_governor_fast_step(z,tau,cmd,C);
    maxZ=max(maxZ,max(abs(za-zb)));
    maxTau=max(maxTau,abs(ta-tb));
    maxDer=max(maxDer,max(abs(da-db)));
end
% A tolerance for floating point equivalence ONLY, not a safety margin.
assert(max([maxZ,maxTau,maxDer])<1e-9, ...
    'Cached governor disagrees with original wp_governor_step');
R=struct('status','PASS_SOURCE_GOVERNOR_NUMERIC_EQUIVALENCE', ...
    'n',nTests,'maxStateAbsDifference',maxZ, ...
    'maxTauAbsDifference',maxTau,'maxTauDerAbsDifference',maxDer, ...
    'physicalPredictionCertified',false);
end
