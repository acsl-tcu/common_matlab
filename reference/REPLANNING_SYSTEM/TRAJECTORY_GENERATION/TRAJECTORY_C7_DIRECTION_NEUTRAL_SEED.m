function [seed,report]=TRAJECTORY_C7_DIRECTION_NEUTRAL_SEED(req,shapes,nominal_u,nominal_l,inflation,nleaf)
%TRAJECTORY_C7_DIRECTION_NEUTRAL_SEED Geometry-derived initialization ONLY.
% This is not a second NLP, a collision certificate, or an HLC command.
% It displaces ONLY the eight free Bernstein controls (same displacement
% for UAV/load) along observed-geometry-dependent directions, leaving
% both endpoint 0..7 jets exactly unchanged. It does NOT choose a global
% left/right/up direction or a shape-specific avoidance rule.
%
% A seed is returned ONLY if an independently constructed full-body
% corridor for the SAME temporal leaf grid exists, and the supplied
% derivative/heading bounds pass. Otherwise the old NLP initializer is used.
%
% ALL computational limits are explicit caller inputs:
% req.search_options.geometric_warm_start = struct(...
%   'enabled',true, 'max_attempts',integer, 'max_normal_iterations',integer)
% The ray amplitude is bounded analytically by the caller's continuous-
% time Bernstein 1..6 derivative limits, NOT by a guessed distance.
% More complicated homotopy classes may be missed (fail closed).
start=tic;
seed=struct('available',false,'free_controls',zeros(48,1), ...
    'normals',zeros(3,3*numel(shapes)*nleaf));
report=struct('status','NOT_REQUESTED','attempts',0,'elapsed_ms',0, ...
    'near_obstacle_ids',zeros(1,0),'corridor_certified',false);
s=req.search_options;
w=s.geometric_warm_start;
assert(isstruct(w)&&isscalar(w)&& ...
    all(isfield(w,{'enabled','max_attempts','max_normal_iterations'})), ...
    'TRAJECTORY_C7_SEED:Budget','Explicit warm-start configuration required.');
assert((isequal(w.enabled,true)||isequal(w.enabled,false)) && ...
    isnumeric(w.max_attempts)&&isscalar(w.max_attempts)&& ...
    isfinite(w.max_attempts)&&w.max_attempts>=0&& ...
    w.max_attempts==fix(w.max_attempts)&& ...
    isnumeric(w.max_normal_iterations)&&isscalar(w.max_normal_iterations)&& ...
    isfinite(w.max_normal_iterations)&&w.max_normal_iterations>=1&& ...
    w.max_normal_iterations==fix(w.max_normal_iterations), ...
    'TRAJECTORY_C7_SEED:Budget','Invalid caller-owned search budgets.');
if ~w.enabled || w.max_attempts==0 || isempty(shapes)
    report.status='DISABLED_OR_EMPTY';report.elapsed_ms=toc(start)*1000;return
end
assert(numel(inflation)==3&&all(isfinite(inflation))&&all(inflation>=0) && ...
    isstruct(req.observed)&&isfield(req,'candidate_feasibility'), ...
    'TRAJECTORY_C7_SEED:Input','Invalid shape or live limit input.');
ids=reshape([shapes.id],1,[]);
obsids=reshape([req.observed.id],1,[]);
observed_active=req.observed(ismember(obsids,ids));
assert(isequal(reshape([observed_active.id],1,[]),ids), ...
    'TRAJECTORY_C7_SEED:IDs','Active observed order must match NLP shapes.');

% Evaluate the original degree-23 curves at data-derived points. These
% points determine only an initial search RAY, NEVER collision safety.
P0=double(nominal_u.control_points_m);
Q0=double(nominal_l.control_points_m);
assert(isequal(size(P0),[3 24])&&isequal(size(Q0),[3 24]), ...
    'TRAJECTORY_C7_SEED:Control','Expected degree-23 control polygons.');
nsamples=size(P0,2);
ss=(0:nsamples-1)/(nsamples-1);
U=zeros(3,nsamples);L=U;
for i=1:nsamples
    b=bernstein23(ss(i));
    U(:,i)=P0*b;L(:,i)=Q0*b;
end
chord=U(:,end)-U(:,1);
D=zeros(3,8);
nearids=zeros(1,0);
for j=1:numel(shapes)
    S=shapes(j);
    % Six exact support values give a valid *obstacle-only* bounding
    % sphere for seed filtering (NOT a full-body collision approximation).
    extent=zeros(3,1);
    for a=1:3
        axis=zeros(3,1);axis(a)=1;
        extent(a)=max( ...
            TRAJECTORY_OBSTACLE_SUPPORT.value(S,axis)-S.center(a), ...
            TRAJECTORY_OBSTACLE_SUPPORT.value(S,-axis)+S.center(a));
    end
    Rout=norm(extent);
    if ~(isfinite(Rout)&&Rout>=0),continue;end
    dist=zeros(1,nsamples);closest=zeros(3,nsamples);
    for i=1:nsamples
        cable=L(:,i)-U(:,i);den=cable.'*cable;
        if den>0
            a=max(0,min(1,((S.center-U(:,i)).'*cable)/den));
        else
            a=0;
        end
        q=U(:,i)+a*cable;
        closest(:,i)=q;
        dist(i)=norm(q-S.center);
    end
    [dmin,index]=min(dist);
    if dmin>Rout+max(inflation)
        % Only omit a seed contribution, NOT an NLP or proof constraint.
        continue
    end
    t=ss(index);
    v=chord;
    if index>1&&index<nsamples
        v=U(:,index+1)-U(:,index-1);
    end
    radial=closest(:,index)-S.center;
    if dot(v,v)>0
        radial=radial-v*(dot(radial,v)/dot(v,v));
    end
    % A true zero normal has no preferred direction by rotation symmetry.
    % The tangent x cable direction is covariant under arbitrary SO(3).
    if norm(radial)<=eps(max(1,norm(S.center)+norm(closest(:,index))))
        radial=cross(v,L(:,index)-U(:,index));
    end
    mag=norm(radial);
    if ~(isfinite(mag)&&mag>0),continue;end
    d=radial/mag;
    basis=bernstein23(t);
    gain=sum(basis(9:16));
    if ~(isfinite(gain)&&gain>eps(1)),continue;end
    % No hand-tuned clearance. Exact support of the *actual* shape and
    % caller-provided cable inflation gives the penetration along d.
    gap=TRAJECTORY_OBSTACLE_SUPPORT.value(S,d)+inflation(3) ...
        -d.'*closest(:,index);
    if ~(isfinite(gap)&&gap>0),continue;end
    D=D+(d*(gap/gain))*ones(1,8);
    nearids(end+1)=S.id; %#ok<AGROW>
end
report.near_obstacle_ids=nearids;
if isempty(nearids)||~all(isfinite(D(:)))||norm(D,'fro')==0
    report.status='NO_COORDINATE_FREE_ESCAPE_RAY';
    report.elapsed_ms=toc(start)*1000;return
end

cf=req.candidate_feasibility;
% Since the Bernstein derivative polygons depend AFFINELY on displacement
% lambda, ||D0+lambda*D1|| <= live_limit is a quadratic inequality for
% EVERY derivative control. Intersect all positive feasible intervals.
% Thus the entire ray prefix [0,lambda_max] is rigorously kinematically
% admissible at all times (assuming nominal derivative controls pass).
u1=TRAJECTORY_C7_FREE_BERNSTEIN.build(req.t0_s,req.duration_s, ...
    req.start_jets.uav,req.end_jets.uav,req.nominal_free.uav+D);
l1=TRAJECTORY_C7_FREE_BERNSTEIN.build(req.t0_s,req.duration_s, ...
    req.start_jets.payload,req.end_jets.payload,req.nominal_free.payload+D);
max_amp=Inf;
for body=1:2
    if body==1
        base=nominal_u;incremented=u1;
        maxima=double(cf.derivative_limits.uav.derivative_norm_max(:));
    else
        base=nominal_l;incremented=l1;
        maxima=double(cf.derivative_limits.payload.derivative_norm_max(:));
    end
    for order=1:6
        B=base.derivative_controls{order+1};
        E=incremented.derivative_controls{order+1}-B;
        for i=1:size(B,2)
            b=B(:,i);d=E(:,i);limit=maxima(order);
            A=d.'*d;C=b.'*b-limit^2;
            if C>0
                report.status='NOMINAL_DERIVATIVE_BOUND_NOT_SATISFIED';
                report.elapsed_ms=toc(start)*1000;return
            end
            if A>0
                Bcross=b.'*d;
                root=(-Bcross+sqrt(max(0,Bcross^2-A*C)))/A;
                if ~isfinite(root)||root<0
                    report.status='NO_POSITIVE_DERIVATIVE_FEASIBLE_RAY';
                    report.elapsed_ms=toc(start)*1000;return
                end
                max_amp=min(max_amp,root);
            end
        end
    end
end
if ~(isfinite(max_amp)&&max_amp>0)
    report.status='UNBOUNDED_OR_ZERO_RAY';
    report.elapsed_ms=toc(start)*1000;return
end
co=req.proof_options;
co.max_subdivision_depth=s.time_leaf_depth;
co.max_leaf_count=s.max_leaf_count;
co.max_normal_iterations=w.max_normal_iterations;
for attempt=1:w.max_attempts
    report.attempts=attempt;
    % Start from the analytically permitted envelope, then sample toward
    % the nominal. No arbitrary physical travel cap or clearance factor.
    amp=max_amp*(w.max_attempts-attempt+1)/w.max_attempts;
    FU=req.nominal_free.uav+amp*D;
    FL=req.nominal_free.payload+amp*D;
    if ~all(isfinite(FU(:)))||~all(isfinite(FL(:))),break;end
    Pu=TRAJECTORY_C7_FREE_BERNSTEIN.build(req.t0_s,req.duration_s, ...
        req.start_jets.uav,req.end_jets.uav,FU);
    Pl=TRAJECTORY_C7_FREE_BERNSTEIN.build(req.t0_s,req.duration_s, ...
        req.start_jets.payload,req.end_jets.payload,FL);
    ku=TRAJECTORY_KINEMATIC_CERTIFICATE(Pu,cf.derivative_limits.uav, ...
        struct('max_subdivision_depth',s.time_leaf_depth));
    kl=TRAJECTORY_KINEMATIC_CERTIFICATE(Pl,cf.derivative_limits.payload, ...
        struct('max_subdivision_depth',s.time_leaf_depth));
    if ~(ku.certified&&kl.certified),continue;end
    H=TRAJECTORY_HEADING_CERTIFICATE(Pu,cf.heading_options);
    if ~H.certified,continue;end
    % A corridor is independently reconstructed from observed shapes.
    % The entire swept cable uses a COMMON separating plane. Its strict
    % support inequalities provide a certified FEASIBLE starting point.
    try
        C=TRAJECTORY_SAFE_CORRIDOR( ...
            struct('uav',Pu,'payload',Pl),observed_active, ...
            req.physical_radii,co);
    catch
        continue
    end
    if ~C.certified,continue;end
    NN=zeros(3,3*numel(shapes)*nleaf);
    accepted=true;
    for k=1:nleaf
        a=req.t0_s+(k-1)*req.duration_s/nleaf;
        b=req.t0_s+k*req.duration_s/nleaf;
        found=0;
        for h=1:numel(C.leaves)
            tt=C.leaves(h).time_interval_s;
            tol=eps(max(1,max(abs([tt,a,b]))))*16;
            if tt(1)-tol<=a && b<=tt(2)+tol
                found=h;break
            end
        end
        if found==0,accepted=false;break;end
        leaf=C.leaves(found);
        if ~isequal(reshape(leaf.obstacle_ids,1,[]),ids)
            accepted=false;break
        end
        for j=1:numel(shapes)
            for body=1:3
                switch body
                    case 1,n=leaf.uav_normals(:,j);
                    case 2,n=leaf.payload_normals(:,j);
                    otherwise,n=leaf.cable_normals(:,j);
                end
                if ~(all(isfinite(n))&&norm(n)>0)
                    accepted=false;break
                end
                NN(:,3*((k-1)*numel(shapes)+j-1)+body)=n/norm(n);
            end
            if ~accepted,break;end
        end
        if ~accepted,break;end
    end
    if ~accepted,continue;end
    seed.available=true;
    seed.free_controls=[FU(:);FL(:)];
    seed.normals=NN;
    report.status='CERTIFIED_CORRIDOR_FEASIBLE_INITIALIZATION';
    report.corridor_certified=true;
    report.elapsed_ms=toc(start)*1000;
    return
end
report.status='NO_CERTIFIED_SEED_WITHIN_EXTERNAL_BUDGET';
report.elapsed_ms=toc(start)*1000;
end

function b=bernstein23(t)
% Scale-free, no individual obstacle or axis assumptions.
n=23;i=(0:n).';
b=zeros(24,1);
if t<=0,b(1)=1;return;end
if t>=1,b(end)=1;return;end
for j=0:n
    b(j+1)=nchoosek(n,j)*t^j*(1-t)^(n-j);
end
end
