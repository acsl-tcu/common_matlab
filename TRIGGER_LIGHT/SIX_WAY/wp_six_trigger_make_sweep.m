function bodies = wp_six_trigger_make_sweep(cfg,forecast,k,mode)
%WP_SIX_TRIGGER_MAKE_SWEEP Guaranteed outer swept bodies, conditioned on bounds.
% For each interval, endpoints pD(t),pL(t) are enclosed by linear segments
% between predicted knots plus rho endpoints + A*dt^2/8.
% True taut cable: (1-s)*pD(t)+s*pL(t), s in [0,1].
%
% C: sweeping each of the 9 cable balls, with a radius valid for ALL times.
% B: sweeping the complete cable capsule, no cable sampling blind spots.
% D: sweeping the uploaded outward ellipsoid. We enclose that ellipsoid by
%    (cable segment + ball(rExtra)), where rExtra >= transverse semiaxis
%    AND >= axial overhang. This is a sufficient (possibly loose) bound,
%    NOT silently substituting the B geometry for D.
%
% Numerical double arithmetic is NOT formally outward-rounded.
dt=forecast.timeOffsets(k+1)-forecast.timeOffsets(k);
D=forecast.pDrone(:,k:k+1);
L=forecast.pLoad(:,k:k+1);
rD=max(forecast.rhoDrone(k:k+1))+forecast.accDrone(k)*dt^2/8;
rL=max(forecast.rhoLoad(k:k+1))+forecast.accLoad(k)*dt^2/8;
rBodyD=cfg.droneRadius+cfg.safetyMargin;
rBodyL=cfg.loadRadius+cfg.safetyMargin;
rCable=cfg.cableRadius+cfg.safetyMargin;

% The true cable length throughout the interval cannot exceed this value,
% provided the endpoint tubes and acceleration bounds are valid.
lengthKnots=sqrt(sum((L-D).^2,1));
lengthUpper=max(lengthKnots)+rD+rL;

emptyBody=struct('name','','vertices',zeros(3,0),'radius',0);
if mode=='C'
    bodies=repmat(emptyBody,1,cfg.cableSegments+3);
else
    bodies=repmat(emptyBody,1,3);
end
bodies(1)=makeBody('drone',D,rD+rBodyD);
bodies(2)=makeBody('payload',L,rL+rBodyL);

switch mode
    case 'C'
        N=cfg.cableSegments;
        % Each cable subsegment (length <= lengthUpper/N) and its physical
        % tube fit inside a sphere of radius hypot(lengthUpper/(2N),rCable).
        chainRadius=hypot(lengthUpper/(2*N),rCable);
        for j=0:N
            s=j/N;
            centers=(1-s)*D+s*L;
            errorRadius=(1-s)*rD+s*rL;
            bodies(j+3)=makeBody(sprintf('cable_%02d',j), ...
                centers,chainRadius+errorRadius);
        end
    case 'B'
        bodies(3)=makeBody('cable',[D,L],max(rD,rL)+rCable);
    case 'D'
        eta=cfg.ellipsoidEta;
        b=sqrt(1+1/eta)*rCable;  % uploaded D: transverse semiaxis
        hUpper=lengthUpper/2;
        aUpper=sqrt((1+eta)*hUpper^2+b^2);
        % For every h in [0,hUpper], max dist(ellipsoid(h), segment(h))
        % <= max(b, sqrt((1+eta)*hUpper^2+b^2)-hUpper).
        % The expression is convex in h, hence its maximum is at an end.
        rExtra=max(b,aUpper-hUpper);
        bodies(3)=makeBody('cable',[D,L],max(rD,rL)+rExtra);
    otherwise
        error('SixTrigger:Mode','Unsupported mode');
end
end
function body=makeBody(name,vertices,radius)
body=struct('name',name,'vertices',vertices,'radius',radius);
end
