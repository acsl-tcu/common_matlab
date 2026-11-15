function out=wp_trigger_swept_gate(known,cfg,forecast)
%WP_TRIGGER_SWEPT_GATE Conditional all-time collision screening over 4 s.
% Geometry proof (assuming externally valid bound inputs):
%  True end positions at knots are within rhoDrone/rhoLoad of forecast.
%  ||true p_D''|| <= accDrone(k), ||true p_L''|| <= accLoad(k)
%  throughout interval I_k. Twice differentiable true end trajectories.
%  Straight taut cable c(s,t)=(1-s)D(t)+sL(t), 0<=s<=1.
%  Then true body swept volumes lie within:
%  drone: conv(D_k,D_{k+1}) + ball(max(rhoD)+AD*dt^2/8+rD)
%  load:  conv(L_k,L_{k+1}) + ball(max(rhoL)+AL*dt^2/8+rL)
%  cable: conv(D_k,D_{k+1},L_k,L_{k+1})
%         + ball(max(maxrhoD,maxrhoL)+max(AD,AL)*dt^2/8+rCable).
%  A containing ball is fast exclusion; support-plane separation against
%  actual outer-cover sphere/ellipsoid is a SOUND SUFFICIENT condition.
%  Failed separation -> POSSIBLE_CONTACT (NOT proven collision).
%
% This floating-point implementation is not outward-rounded. CLEAR is only
% CONDITIONAL on externally VALID truth-enclosing inputs, taut cable, known
% outer covers and proper numerical robustness; no physical safe-to-commit.
% It never simulates future HLC inputs and does not assume they stay fixed.
start=tic;
N=0; checked=0; narrow=0; wholeBalls=0;
out=struct('status','UNKNOWN','reason','','shouldRequestReplan',true, ...
    'firstInterval',[NaN NaN],'firstPartIndex',NaN, ...
    'firstObstacle','','firstBody','','elapsedMs',NaN, ...
    'intervalsChecked',0,'broadphaseComparisons',0, ...
    'broadphaseEliminations',0,'supportChecks',0, ...
    'conditionalOnExternalBounds',true,'safeToCommit',false, ...
    'guaranteesPhysicalSafety',false,'fullyEvaluated',false);
if known.dynamicUnsupported || ~known.groundSupported
    out.reason='unsupported_dynamic_obstacle_or_ground';out.elapsedMs=1000*toc(start);return;
end
if ~isfield(cfg,'droneRadius') || ~isfield(cfg,'loadRadius') || ...
   ~isfield(cfg,'cableRadius') || ~isfield(cfg,'safetyMargin')
    out.reason='missing_geometry_radii';out.elapsedMs=1000*toc(start);return;
end
rad=[cfg.droneRadius,cfg.loadRadius,cfg.cableRadius]+cfg.safetyMargin;
if any(~isfinite(rad)) || any(rad<0)
    out.reason='invalid_geometry_radii';out.elapsedMs=1000*toc(start);return;
end
required={'timeOffsets','pDrone','pLoad','rhoDrone','rhoLoad','accDrone','accLoad'};
for i=1:numel(required)
    if ~isfield(forecast,required{i})
        out.reason=['missing_bound_',required{i}];out.elapsedMs=1000*toc(start);return;
    end
end
t=reshape(forecast.timeOffsets,1,[]);N=numel(t);
if N<2 || abs(t(1))>1e-12 || abs(t(end)-4)>1e-12 || ...
        any(~isfinite(t)) || any(diff(t)<=0) || ...
        ~isequal(size(forecast.pDrone),[3,N]) || ...
        ~isequal(size(forecast.pLoad),[3,N])
    out.reason='invalid_or_incomplete_4s_grid';out.elapsedMs=1000*toc(start);return;
end
rD=reshape(forecast.rhoDrone,1,[]);
rL=reshape(forecast.rhoLoad,1,[]);
aD=reshape(forecast.accDrone,1,[]);
aL=reshape(forecast.accLoad,1,[]);
if numel(rD)~=N || numel(rL)~=N || ...
        numel(aD)~=N-1 || numel(aL)~=N-1 || ...
        any(~isfinite([forecast.pDrone(:);forecast.pLoad(:);rD(:);rL(:);aD(:);aL(:)])) || ...
        any([rD,rL,aD,aL]<0)
    out.reason='invalid_or_unproven_numerical_bounds';out.elapsedMs=1000*toc(start);return;
end
if isfield(cfg,'checkGround') && ~cfg.checkGround
    checkGround=false;
else
    checkGround=known.groundEnabled;
end
% Every interval, even if it contains no fixed-route site, is evaluated.
for k=1:N-1
    dt=t(k+1)-t(k);
    dd=forecast.pDrone(:,k:k+1);
    ll=forecast.pLoad(:,k:k+1);
    eD=max(rD(k:k+1))+aD(k)*dt^2/8;
    eL=max(rL(k:k+1))+aL(k)*dt^2/8;
    % Cable vertices are a time-space convex hull of four nominal endpoints.
    verts={dd,ll,[dd,ll]};
    inflation=[eD+rad(1),eL+rad(2),max(eD,eL)+rad(3)];
    for b=1:3
        v=verts{b}; e=inflation(b);
        if checkGround
            groundDist=min(known.groundNormal'*(v-known.groundPoint),[],2)- ...
                e-known.groundMargin;
            if ~(groundDist>0)
                out=signal_possible(out,k,t,0,'ground',b,'ground_not_separated');
                out=finish(out,start,k,checked,wholeBalls,narrow);return;
            end
        end
        % Broad-phase circumsphere is a proof of no intersection when apart.
        center=mean(v,2);
        rv=max(sqrt(sum((v-center).^2,1)))+e;
        P=known.numParts;
        if P==0,continue;end
        distances=sqrt(sum((known.center-center).^2,1));
        pass=(distances>rv+known.outerRadius);
        checked=checked+P;wholeBalls=wholeBalls+sum(pass);
        if all(pass),continue;end
        candidates=find(~pass);
        for ii=1:numel(candidates)
            j=candidates(ii);
            narrow=narrow+1;
            if ~separated_by_support(v,e,known,j)
                out=signal_possible(out,k,t,j,known.id{j},b, ...
                    'outer_swept_volume_cannot_be_separated');
                out=finish(out,start,k,checked,wholeBalls,narrow);return;
            end
        end
    end
end
out.status='CLEAR_CONDITIONAL';out.reason='all_intervals_separated_given_bounds';
out.shouldRequestReplan=false;out.fullyEvaluated=true;
out=finish(out,start,N-1,checked,wholeBalls,narrow);
end
function yes=separated_by_support(v,r,known,j)
% Condition: min_{x in hull+ball(r)} n*x > max_{o in obstacle} n*o
% for SOME unit normal n. Failed candidate normals do not imply collision.
c=known.center(:,j);
q=mean(v,2)-c;
Ns=[eye(3),known.R(:,:,j),q,v-c];
ns=sqrt(sum(Ns.^2,1));
Ns=Ns(:,ns>1e-14);ns=ns(ns>1e-14);
Ns=bsxfun(@rdivide,Ns,ns);
% ± directions tested; only a proven positive halfspace is accepted.
Ns=[Ns,-Ns];
proj=Ns'*v;
minBody=min(proj,[],2)-r;
R=known.R(:,:,j);
axisProj=R'*Ns;
obsSupport=sqrt(sum(bsxfun(@times,axisProj,known.axes(:,j)).^2,1))';
sep=minBody-(Ns'*c+obsSupport);
% A small positive floating-point guard reduces boundary false clears;
% this is NOT a machine-directed outward interval proof.
scale=max(1,max(abs([proj(:);minBody;obsSupport;Ns'*c])));
yes=any(sep>64*eps(scale));
end
function out=signal_possible(out,k,t,j,id,b,reason)
out.status='POSSIBLE_CONTACT';out.reason=reason;
out.firstInterval=t(k:k+1);out.firstPartIndex=j;
out.firstObstacle=id;
names={'drone','payload','cable'};
out.firstBody=names{b};
end
function out=finish(out,start,k,comparisons,broadphase,narrow)
out.intervalsChecked=k;
out.broadphaseComparisons=comparisons;
out.broadphaseEliminations=broadphase;
out.supportChecks=narrow;
out.elapsedMs=1000*toc(start);
end
