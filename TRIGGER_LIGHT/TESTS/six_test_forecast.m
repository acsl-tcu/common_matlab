function F = six_test_forecast(t,kind,offset)
%SIX_TEST_FORECAST Synthetic exact taut-cable motion with ANALYTIC bounds.
% kind='straight' or 'swing'. This is not an HLC simulation.
if nargin<2,kind='straight';end
if nargin<3,offset=0;end
t=reshape(t,1,[]);
if strcmp(kind,'straight')
    pD=[t;offset*ones(1,numel(t));3*ones(1,numel(t))];
    pL=pD+[zeros(1,numel(t));zeros(1,numel(t));-2*ones(1,numel(t))];
    ad=0;al=0;
elseif strcmp(kind,'swing')
    w=1.35;
    amp=0.20;
    theta=amp*sin(w*t+0.27);
    pD=[0.7*t;offset+0.15*sin(w*t);3.2*ones(1,numel(t))];
    pL=pD+2*[sin(theta);zeros(1,numel(t));-cos(theta)];
    ad=0.15*w^2;
    al=ad+2*(amp*w^2+(amp*w)^2);
else
    error('SixTrigger:Fixture','Unsupported synthetic motion');
end
F=struct('timeOffsets',t,'pDrone',pD,'pLoad',pL, ...
    'rhoDrone',zeros(1,numel(t)),'rhoLoad',zeros(1,numel(t)), ...
    'accDrone',ad*ones(1,numel(t)-1), ...
    'accLoad',al*ones(1,numel(t)-1), ...
    'boundSource','analytic_synthetic_motion_only');
end
