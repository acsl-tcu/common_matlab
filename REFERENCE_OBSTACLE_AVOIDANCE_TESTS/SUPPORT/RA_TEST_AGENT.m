function a=RA_TEST_AGENT()
% Test-only mock of model parameters. loadmass=999 is a sentinel: never use.
a=struct();
a.parameter=struct('mass',.9,'cableL',2.0,'gravity',9.81, ...
    'jx',.01,'jy',.01,'jz',.02,'loadmass',999);
a.estimator.result.state=struct('mL',.14);
end
