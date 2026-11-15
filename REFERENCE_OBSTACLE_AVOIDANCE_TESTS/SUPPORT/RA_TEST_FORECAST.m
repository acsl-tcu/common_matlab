function F=RA_TEST_FORECAST(cfg)
% 4s 41 knots, taut 2m cable. Synthetic geometry-only fixture.
t=linspace(0,4,41);
pD=[linspace(0,4,41);zeros(1,41);3*ones(1,41)];
pL=pD-repmat([0;0;cfg.cableLength],1,41);
F=struct('timeOffsets',t,'pDrone',pD,'pLoad',pL, ...
    'rhoDrone',zeros(1,41),'rhoLoad',zeros(1,41), ...
    'accDrone',zeros(1,40),'accLoad',zeros(1,40), ...
    'modelLoadMass',cfg.currentLoadMass);
end
