function R=TRAJECTORY_C9_27_INTERVAL_REFERENCE_SEGMENT_INTLAB(piece,t0,t1,g_world,L)
% Full continuous-time degree-27 nominal reference enclosure on [t0,t1].
% Directed INTLAB de Casteljau restricts the *actual Bernstein polynomial*;
% payload, UAV and cable boxes are derived without sampling.  The flat
% taut-cable relation is q=(g-pddot)/||g-pddot||; zero-norm => UNKNOWN.
% Input doubles describe the mathematical candidate; no actual MATLAB
% double execution-roundoff or reference-switch authorization is inferred.
R=struct('status','UNKNOWN_REFERENCE_INTERVAL_INPUT','valid',false, ...
 'payload_3x2',NaN(3,2),'uav_3x2',NaN(3,2), ...
 'taut_nonzero',false,'safe_to_commit',false);
if ~(isstruct(piece)&&isfield(piece,'control_points_m') && ...
     isequal(size(piece.control_points_m),[3 28]) && ...
     all(isfinite(piece.control_points_m(:))) && ...
     all(isfinite([t0,t1,L])) && ...
     t1>t0 && L>0 && isscalar(piece.duration_s) && ...
     isfinite(piece.duration_s) && piece.duration_s>0 && ...
     isscalar(piece.t0_s) && t0>=piece.t0_s && ...
     t1<=piece.t0_s+piece.duration_s && ...
     isequal(size(g_world),[3 1]) && all(isfinite(g_world)))
    return
end
if exist('infsup','file')~=2 || exist('intval','file')~=2
    R.status='UNKNOWN_REFERENCE_INTERVAL_INTLAB_MISSING';return
end
try
    P=infsup(piece.control_points_m,piece.control_points_m);
    T=intval(piece.duration_s);
    D=(27/T)*(P(:,2:end)-P(:,1:end-1));
    D2=(26/T)*(D(:,2:end)-D(:,1:end-1));
    u0=(intval(t0)-intval(piece.t0_s))/T;
    u1=(intval(t1)-intval(piece.t0_s))/T;
    if ~(double(inf(u0))>=0 && double(sup(u1))<=1 && ...
            double(sup(u0))<double(inf(u1)))
        R.status='UNKNOWN_REFERENCE_INTERVAL_PHASE_RANGE';return
    end
    Pg=restrict(P,u0,u1);
    Ag=restrict(D2,u0,u1);
    p=infsup(min(double(inf(Pg)),[],2), ...
             max(double(sup(Pg)),[],2));
    a=infsup(min(double(inf(Ag)),[],2), ...
             max(double(sup(Ag)),[],2));
    W=infsup(g_world,g_world)-a;
    W2=sum(W.^2);
    if ~(isfinite(double(inf(W2))) && double(inf(W2))>0)
        R.status='UNKNOWN_REFERENCE_INTERVAL_TAUT_DEGENERATE';return
    end
    q=W/sqrt(W2);
    pu=p-intval(L)*q;
    plo=double(inf(p));phi=double(sup(p));
    ulo=double(inf(pu));uhi=double(sup(pu));
    if any(~isfinite([plo;phi;ulo;uhi]))
        R.status='UNKNOWN_REFERENCE_INTERVAL_NOT_FINITE';return
    end
    R.payload_3x2=[plo,phi];
    R.uav_3x2=[ulo,uhi];
    R.taut_nonzero=true;
    R.valid=true;
    R.status='CONDITIONAL_DIRECTED_C9_NOMINAL_FULL_SEGMENT_BOX';
catch ME
    R.status=['UNKNOWN_REFERENCE_INTERVAL_EXCEPTION_' ME.identifier];
end
end
function Q=restrict(P,a,b)
% Interval-valued de Casteljau, no double arithmetic in interpolation.
Q=P;
if double(inf(a))>0
    [~,Q]=split(Q,a);
end
if double(sup(b))<1
    local=(b-a)/(intval(1)-a);
    [Q,~]=split(Q,local);
end
end
function [left,right]=split(P,t)
n=size(P,2)-1;
left=P;right=P;z=P;
for j=0:n
    left(:,j+1)=z(:,1);
    right(:,n-j+1)=z(:,end);
    z=(intval(1)-t)*z(:,1:end-1)+t*z(:,2:end);
end
end
