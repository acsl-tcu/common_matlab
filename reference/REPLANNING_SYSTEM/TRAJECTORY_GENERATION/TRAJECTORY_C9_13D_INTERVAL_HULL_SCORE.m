function [S,cache]=TRAJECTORY_C9_13D_INTERVAL_HULL_SCORE(piece,Q,req,cache)
% Conservative, real-arithmetic Bernstein HULL SEARCH SCORE for the SAME
% planes used by the 13D local QP. It NEVER certifies physical safety.
% Each probe plane is scored on its own Voronoi time cell (midpoints of
% adjacent probe phases). The complete payload polynomial and the straight
% cable's selected lambda are enclosed for ALL times within that cell.
% In particular, a zero POINT merit does not imply a zero CELL bound.
%
% For any plane n'*x>=h, x=pL-lambda*L*(g-pL'')/||g-pL''||,
%   h-n'*x <= h-min_B n'*P + lambda*L*upper(n'*w/||w||).
% Every bound follows from the Bernstein convex-hull property. The norm
% denominator uses the exact coordinatewise Bernstein interval box;
% no empirical safety margin or nominal speed threshold is introduced.
% OUTWARD ROUNDED INTERVAL ARITHMETIC IS NOT CLAIMED here. The independent
% continuous-time shape/force oracle and HLC proof remain mandatory.
S=struct('status','UNKNOWN_INTERVAL_HULL_NOT_EVALUATED','valid',false, ...
    'bound_m',Inf,'per_plane_upper_m',[], 'cell_count',0, ...
    'minimum_w_norm_lower_mps2',Inf,'safe_to_commit',false);
% Optional caller-owned cache ONLY for REPEATED evaluations with the same
% probe phases (e.g. line search).  Each Bernstein restriction is LINEAR
% in the control points.  No obstacle, mass, radius, or candidate state
% is cached.  A one-shot call retains the original de Casteljau path.
if nargin<4,cache=[];end
try
    P=double(piece.control_points_m);
    T=double(piece.duration_s);
    n=double(Q.geometry_plane_normals_3xm);
    h=double(Q.geometry_plane_thresholds_m(:));
    lam=double(Q.geometry_plane_lambdas(:));
    phases=double(Q.geometry_plane_phases(:));
    g=double(req.authority.gravity_world_mps2(:));
    L=double(req.authority.cable_length_m);
    M=numel(phases);
    assert(isequal(size(P),[3 24]) && isreal(P)&&all(isfinite(P(:))) ...
        &&isscalar(T)&&isfinite(T)&&T>0 ...
        &&isequal(size(n),[3 M])&&numel(h)==M&&numel(lam)==M ...
        &&isequal(size(g),[3 1])&&numel(L)==1&&L>0 ...
        &&all(isfinite([n(:);h;lam;phases;g;L])) ...
        &&all(phases>=0&phases<=1)&&all(lam>=0&lam<=1), ...
        'C9_INTERVAL_HULL:Input','Invalid finite physical data.');
    if M==0
        S.valid=true;S.bound_m=0;S.per_plane_upper_m=zeros(0,1);
        S.minimum_w_norm_lower_mps2=NaN;
        S.status='NO_PLANES__SEARCH_SCORE_ONLY';return
    end
    [u,~,idx]=unique(phases);
    k=numel(u);
    edges=[0; (u(1:end-1)+u(2:end))/2; 1];
    cache_on=isstruct(cache) && isscalar(cache) && ...
        isfield(cache,'enabled') && isequal(cache.enabled,true);
    if cache_on
        fresh=~all(isfield(cache,{'phase_key','degree23','degree21'})) ...
            || ~isequal(cache.phase_key,phases);
        if fresh
            mats23=cell(k,1);mats21=cell(k,1);
            I23=eye(24);I21=eye(22);
            for cc=1:k
                mats23{cc}=bezier_interval(I23,edges(cc),edges(cc+1));
                mats21{cc}=bezier_interval(I21,edges(cc),edges(cc+1));
            end
            cache=struct('enabled',true,'phase_key',phases, ...
                'degree23',{mats23},'degree21',{mats21});
        end
    end
    % Control points of the physical-time acceleration (degree 21).
    A=(23*22/T^2)*diff(P,2,2);
    out=Inf(M,1);
    w_min=Inf;
    for j=1:k
        ids=find(idx==j);
        a=edges(j);b=edges(j+1);
        if ~(b>a),return;end
        if cache_on
            Pj=P*cache.degree23{j};
            Aj=A*cache.degree21{j};
        else
            Pj=bezier_interval(P,a,b);
            Aj=bezier_interval(A,a,b);
        end
        W=bsxfun(@minus,g,Aj);
        lo=min(W,[],2);hi=max(W,[],2);
        % Distance from 0 to the axis-aligned box enclosing conv(W).
        closest=max(lo,0)+min(hi,0);
        dmin=norm(closest);
        dmax=max(sqrt(sum(W.^2,1)));
        w_min=min(w_min,dmin);
        if ~(isfinite(dmin)&&dmin>0&&isfinite(dmax)&&dmax>=dmin)
            S.status='UNKNOWN_CELL_W_NORM_NOT_SEPARATED_FROM_ZERO';return
        end
        ni=n(:,ids);nlen=sqrt(sum(ni.^2,1)).';
        pmin=min(ni.'*Pj,[],2);
        nwmax=max(ni.'*W,[],2);
        q_up=zeros(numel(ids),1);
        pos=nwmax>=0;
        q_up(pos)=nwmax(pos)/dmin;
        q_up(~pos)=nwmax(~pos)/dmax;
        % An actual unit direction satisfies |n'*q|<=||n||.
        q_up=min(nlen,max(-nlen,q_up));
        out(ids)=h(ids)-pmin+lam(ids)*L.*q_up;
    end
    if any(~isfinite(out)),return;end
    S.valid=true;
    S.per_plane_upper_m=out;
    S.bound_m=max([0;out]);
    S.cell_count=k;
    S.minimum_w_norm_lower_mps2=w_min;
    S.status='VALID_CELL_HULL_UPPER_BOUND__SEARCH_ONLY_NOT_FLIGHT_PROOF';
catch ME
    S.status=['UNKNOWN_INTERVAL_HULL_' ME.identifier];
end
end

function S=bezier_interval(P,a,b)
% Restrict one Bezier polygon exactly in real polynomial arithmetic to
% [a,b] in [0,1], using de Casteljau subdivision. No time samples.
if a==0
    R=P;
else
    [~,R]=bezier_split(P,a);
end
if b==1
    S=R;
else
    t=(b-a)/(1-a);
    [S,~]=bezier_split(R,t);
end
end

function [left,right]=bezier_split(P,t)
n=size(P,2)-1;
D=cell(1,n+1);D{1}=P;
for j=1:n
    D{j+1}=(1-t)*D{j}(:,1:end-1)+t*D{j}(:,2:end);
end
left=zeros(size(P));right=zeros(size(P));
for j=0:n
    left(:,j+1)=D{j+1}(:,1);
    right(:,j+1)=D{n-j+1}(:,j+1);
end
end
