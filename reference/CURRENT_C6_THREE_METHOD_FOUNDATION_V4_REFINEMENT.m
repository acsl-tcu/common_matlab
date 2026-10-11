function result = CURRENT_C6_THREE_METHOD_FOUNDATION_V4_REFINEMENT(methodSelect,stageSelect)
% CURRENT_C6_THREE_METHOD_FOUNDATION_V3_REJOIN
% ========================================================================
% THEORY-FIRST FOUNDATION VALIDATION FOR C6 SUSPENDED-LOAD AVOIDANCE
%
% Single-file usage:
%   result = CURRENT_C6_THREE_METHOD_FOUNDATION_V2();
%   result = CURRENT_C6_THREE_METHOD_FOUNDATION_V2('BSPLINE_D7');
%   result = CURRENT_C6_THREE_METHOD_FOUNDATION_V2('POLY_D13_MIN7','A');
%   result = CURRENT_C6_THREE_METHOD_FOUNDATION_V2('BEZIER_D13_MIN7','B');
%
% One file contains all three implementations and the common test harness.  The ONLY acceptance requirements in this foundation are:
%   (R1) trajectory is structurally C6 and splice errors d0...d6 are small;
%   (R2) final safety certificate treats Payload / Cable / UAV separately;
%   (R3) obstacles are oriented ellipsoids;
%   (R4) ALL currently known obstacles are included in certification;
%   (R5) collision-free claim is continuous-time via Bernstein convex-hull
%        sufficient conditions, never sampling-only;
%   (R6) Stage C uses a 15 m simulated sensor, persistent obstacle memory,
%        and look-ahead collision trigger before avoidance generation.
%
% Methods:
%   BSPLINE_D7       degree-7 B-spline, simple interior knots -> C6.
%   POLY_D13_MIN7    piecewise degree-13 polynomial, min integral p^(7)^2.
%   BEZIER_D13_MIN7  piecewise degree-13 Bernstein/Bezier, min 7th diff.
%
% IMPORTANT: v/a/j and HLC input bounds are DIAGNOSTICS ONLY here.  They are
% intentionally not part of acceptance until the C6 + geometric-safety base
% is demonstrated robustly.
%
% DESIGN NOTE -- ONE TRAJECTORY, NOT SEPARATE "OUT" AND "RETURN" PLANS
% ------------------------------------------------------------------------
% The avoidance interval is optimized as one trajectory from the current
% splice state to a nominal rejoin state.  Intermediate via points create
% segments/constraints, but those segments are solved JOINTLY in one
% optimization with C0...C6 continuity.  There is no independently solved
% outbound trajectory followed by a separately solved return trajectory.
% This follows the common waypoint/corridor trajectory-optimization pattern
% used by minimum-control-effort polynomial, B-spline replanning, and
% Bernstein/Bezier safe-corridor planners.
%
% FOUNDATION SCOPE
%   Stage A: representation and C6 mathematics only.
%   Stage B: one oriented ellipsoid + separate UAV/Cable/Payload geometry.
%   Stage C: same core plus sensor, persistent memory, trigger, automatic
%            rejoin selection, and common guide-refinement ladder.
%
% IMPORTANT: all candidate generators remain structurally C6.  A candidate
% is never accepted merely because it looks smooth; acceptance additionally
% requires the d0..d6 numerical residual test and the independent continuous
% geometry certificate.
% v/a/j and HLC input bounds remain diagnostic only until all three methods
% can produce a C6 geometrically certified trajectory.
% ========================================================================
if nargin<1 || isempty(methodSelect), methodSelect='ALL'; end
if nargin<2 || isempty(stageSelect), stageSelect='ALL'; end
methodSelect=upper(string(methodSelect));
stageSelect=upper(string(stageSelect));
allMethods=["BSPLINE_D7","POLY_D13_MIN7","BEZIER_D13_MIN7"];
if methodSelect=="ALL", methods=allMethods; else
    if ~any(methodSelect==allMethods), error('C6BASE:BadMethod','Unknown method %s',methodSelect); end
    methods=methodSelect;
end
clc; format compact;
P=c6_params();
scenarios=c6_scenarios(P);
if stageSelect~="ALL"
    keep=arrayfun(@(s) upper(string(s.stage))==stageSelect,scenarios);
    if ~any(keep), error('C6BASE:BadStage','Unknown stage %s. Use A, B, C, or ALL.',stageSelect); end
    scenarios=scenarios(keep);
end

fprintf('\n========================================================================================================================\n');
fprintf(' SINGLE-FILE C6 FOUNDATION -- B-spline / polynomial / Bezier under one common safety test\n');
fprintf(' methods: %s\n',strjoin(cellstr(methods),', '));
fprintf(' Stage A = representation/unit test | Stage B = one ellipsoid | Stage C = sensor+trigger+two ellipsoids\n');
fprintf('========================================================================================================================\n');

out=cell(numel(methods),numel(scenarios));
for im=1:numel(methods)
    fprintf('\n########################################################################################################################\n');
    fprintf(' METHOD: %s\n',methods(im));
    fprintf('########################################################################################################################\n');
    for is=1:numel(scenarios)
        out{im,is}=c6_run_stage(methods(im),scenarios(is),P);
    end
end
result=struct('methods',methods,'scenarios',{scenarios},'runs',{out},'requirements',c6_requirement_summary(methods,out,scenarios,P));
c6_print_cross_method_summary(result);
end

function P=c6_params()
P=struct();
P.p_bs=7; P.n_poly=13; P.n_bez=13;
P.c6_tol=2e-6;
P.d_margin=0.15;
P.R_payload=0.24;
P.R_uav=0.30;
P.R_cable=0.045;
P.L_cable=1.00;
P.g=9.81; P.e3=[0;0;1];
P.sensor_R=15.0; P.dt_sensor=0.05; P.lookahead=5.0; P.pred_dt=0.10;
P.sep_dirs=c6_fibonacci_sphere(362);
P.offsets=[2.5 3.5 4.5 5.5]; % Stage-B simple single-obstacle candidates only.
% Stage-C multi-obstacle front-end: graph search over analytically generated
% support points.  This is deliberately representation-independent: all
% three C6 methods receive exactly the same graph-derived polyline routes.
P.guide_extra=[0.50 1.00 1.50 2.00];
P.guide_dirs=c6_fibonacci_sphere(42);
P.guide_sep_dirs=c6_fibonacci_sphere(98);
% Stage-C smoothing refinement ladder.  Every entry comes from the SAME
% certified graph polyline; only waypoint density changes.  This follows the
% common path->smooth-trajectory pattern while keeping final continuous
% certification independent from the generator.
P.guide_refine_spacings=[1.75 1.00 0.65 0.40]; % [m], coarse -> fine
P.guide_dev_samples=11;       % diagnostic samples per Bezier segment only
P.guide_edge_margin=0.05;     % additional positive guide-edge separation [m]
P.max_graph_routes=16;
% Stage-C nominal rejoin phase candidates [s] measured from the trigger state.
% They are evaluated in ascending order; the earliest candidate that is
% itself safe and graph-reachable is selected.  This is NOT a T escalation:
% rejoin phase and avoidance execution duration remain distinct quantities.
P.rejoin_phase_candidates=[12 14 16 18 20 22 24];
P.T_scales=1.0; % FOUNDATION ONLY: one prescribed duration per stage. No T-escalation/time search yet.
end

function S=c6_scenarios(P)
% Stage A: representation mathematics only, deliberately includes one via
% so internal C6 is tested.
A=struct(); A.name='A_UNIT_C6'; A.stage='A'; A.sensor=false;
A.p0=[0;0;0]; A.pg=[8;0;0]; A.v0=[0.8;0;0]; A.vg=[0.8;0;0];
A.T=10; A.obs=[]; A.fixed_vias=[4;1.0;0.5]; A.world_obs=[];

% Stage B: one oriented ellipsoid. Candidate vias are generated on common
% lateral directions and every method sees exactly the same route set.
% The via is an interpolation/constraint point inside ONE globally solved
% trajectory; it is not a boundary between an independently planned outward
% leg and an independently planned return leg.
B=struct(); B.name='B_ONE_ELLIPSOID'; B.stage='B'; B.sensor=false;
B.p0=[0;0;0]; B.pg=[10;0;0]; B.v0=[0.8;0;0]; B.vg=[0.8;0;0]; B.T=13;
B.obs=c6_make_obs([5;0;0],[1.10;0.85;1.15],c6_Rz(deg2rad(25))*c6_Ry(deg2rad(10)),1);
B.fixed_vias=[]; B.world_obs=B.obs;

% Stage C: system-integration foundation. At t=0 every obstacle surface is
% outside the 15 m sensor sphere around the UAV. Sensor memory persists.
C=struct(); C.name='C_SENSOR_TRIGGER_TWO_OBS'; C.stage='C'; C.sensor=true;
C.p0=[0;0;0]; C.pg=[]; C.v0=[1.0;0;0]; C.vg=[1.0;0;0]; C.T=15;
o1=c6_make_obs([20;0;0],[1.0;0.85;1.15],c6_Rz(deg2rad(20)),1);
o2=c6_make_obs([24;0.4;0.2],[1.15;0.75;1.00],c6_Rz(deg2rad(-30))*c6_Ry(deg2rad(12)),2);
C.obs=[o1 o2]; C.world_obs=C.obs; C.fixed_vias=[];
S=[A B C];
end

function R=c6_run_stage(method,S,P)
fprintf('\n------------------------------------------------------------------------------------------------------------------------\n');
fprintf(' %s / %s\n',S.name,method);
fprintf('------------------------------------------------------------------------------------------------------------------------\n');

if S.sensor
    trig=c6_sensor_trigger(S,P);
    fprintf('[SYS] initialOutside15=%d detectedAt=%.2f triggerAt=%.2f TTC=%.2f knownIDs=%s\n',...
        trig.initial_outside,trig.first_detect,trig.t_trigger,trig.TTC,mat2str(trig.known_ids));
    if ~trig.found
        R=c6_empty_run(method,S.name,'NO_TRIGGER'); return;
    end
    p0=S.p0+S.v0*trig.t_trigger;
    pg=[]; % selected automatically from nominal rejoin candidates below
    obsKnown=S.world_obs(trig.known_ids);
    D0=c6_nominal_jet(p0,S.v0); D1=[];
    T0=S.T;
else
    p0=S.p0; pg=S.pg; obsKnown=S.obs;
    D0=c6_nominal_jet(p0,S.v0); D1=c6_nominal_jet(pg,S.vg);
    T0=S.T;
    trig=struct('found',true,'initial_outside',true,'first_detect',NaN,'t_trigger',0,'TTC',NaN,'known_ids',1:numel(obsKnown));
end

if strcmp(S.stage,'A')
    routeSets={S.fixed_vias}; routeTags={'UNIT_VIA'};
elseif strcmp(S.stage,'B')
    [routeSets,routeTags]=c6_route_candidates_single(p0,pg,obsKnown,P);
else
    [routeSets,routeTags,routeDiag,pg,rejoinPhase]=c6_route_candidates_graph_rejoin(p0,S.vg,obsKnown,P);
    fprintf('[ROUTE-GRAPH] nodes=%d edges=%d rawRoutes=%d uniqueRoutes=%d selectedRejoin=%s\n',...
        routeDiag.nodes,routeDiag.edges,routeDiag.raw_routes,numel(routeSets),c6_num(rejoinPhase));
    c6_print_route_graph_diag(routeDiag);
    if isempty(routeSets)
        R=c6_empty_run(method,S.name,'NO_GRAPH_ROUTE'); R.trigger=trig; R.known_count=numel(obsKnown);
        R.rejoin_phase=NaN; R.rejoin_goal=[];
        c6_print_stage_requirements(R,S,P); return;
    end
    D1=c6_nominal_jet(pg,S.vg);
    fprintf('[REJOIN] selected phase=%.2f s nominalGoal=[%.3f %.3f %.3f]^T ; execution T remains %.2f s\n',...
        rejoinPhase,pg(1),pg(2),pg(3),T0);
end

rows=repmat(c6_row(),1,numel(routeSets)*numel(P.T_scales)); ir=0; best=[];
for its=1:numel(P.T_scales)
    T=T0*P.T_scales(its);
    for iq=1:numel(routeSets)
        vias=routeSets{iq}; ir=ir+1; rr=c6_row(); rr.route=routeTags{iq}; rr.T=T; rr.method=char(method);
        tb=tic;
        try
            seg=c6_build(method,T,D0,D1,vias,P);
            rr.build_ms=1e3*toc(tb); rr.built=true;
            [rr.c6_error,rr.c6_by_order,rr.c6_rel_by_order]=c6_c6_error(seg); [rr.endpoint_error,rr.end_by_order,rr.end_rel_by_order]=c6_endpoint_error(seg);
            rr.condition_metric=seg.condition_metric;
            rr.n_vias=size(vias,2);
            rr.guide_dev=c6_max_guide_deviation(seg,[p0,vias,pg],P);
            tc=tic; cert=c6_certify(seg,obsKnown,P); rr.cert_ms=1e3*toc(tc);
            rr.geom=cert.pass; rr.minG=cert.minG; rr.minG_payload=cert.minG_payload; rr.minG_uav=cert.minG_uav; rr.minG_cable=cert.minG_cable;
            rr.v=cert.v; rr.a=cert.a; rr.j=cert.j; rr.tension_lb=cert.tension_lb;
            rr.known_all=(cert.n_obstacles==numel(obsKnown));
            rr.structural_c6=seg.structural_c6;
            rr.safe=rr.built && rr.structural_c6 && rr.c6_error<=P.c6_tol && rr.endpoint_error<=P.c6_tol && rr.geom && rr.known_all;
            if rr.safe, rr.reason='PASS_CORE'; elseif ~rr.geom, rr.reason='GEOMETRY_FAIL'; elseif rr.c6_error>P.c6_tol, rr.reason='C6_NUMERIC_FAIL'; else, rr.reason='OTHER_FAIL'; end
            rr.total_ms=rr.build_ms+rr.cert_ms;
            if rr.safe && (isempty(best) || rr.total_ms<best.row.total_ms), best=struct('row',rr,'seg',seg,'vias',vias,'cert',cert); end
        catch ME
            rr.build_ms=1e3*toc(tb); rr.reason=['EXCEPTION:' ME.identifier]; rr.message=ME.message;
        end
        rows(ir)=rr;
        fprintf('[%s] route=%-17s T=%5.2f vias=%3d dev=%6.3f build=%d C6=%8.2e end=%8.2e geom=%d minG=%+8.3e [L/U/C]=[%+7.3e %+7.3e %+7.3e] cond=%8.2e ms=%7.2f %s\n',...
            method,rr.route,rr.T,rr.n_vias,rr.guide_dev,rr.built,rr.c6_error,rr.endpoint_error,rr.geom,rr.minG,rr.minG_payload,rr.minG_uav,rr.minG_cable,rr.condition_metric,rr.total_ms,rr.reason);
        if strcmp(S.stage,'A'), break; end
    end
    if strcmp(S.stage,'A'), break; end
end
rows=rows(1:ir);
R=struct('method',char(method),'scenario',S.name,'rows',rows,'best',best,'trigger',trig,'known_count',numel(obsKnown));
if S.sensor, R.rejoin_phase=rejoinPhase; R.rejoin_goal=pg; else, R.rejoin_phase=NaN; R.rejoin_goal=pg; end
c6_print_stage_requirements(R,S,P);
end

function seg=c6_build(method,T,D0,D1,vias,P)
switch method
    case "BSPLINE_D7", seg=c6_build_bspline(T,D0,D1,vias,P);
    case "POLY_D13_MIN7", seg=c6_build_poly(T,D0,D1,vias,P);
    case "BEZIER_D13_MIN7", seg=c6_build_bezier(T,D0,D1,vias,P);
    otherwise, error('C6BASE:Method','Bad method');
end
end

% ================================ B-SPLINE ===============================
function seg=c6_build_bspline(T,D0,D1,vias,P)
p=P.p_bs; nVia=size(vias,2); N=max(22,2*(nVia+1)+14); U=c6_clamped_uniform_knots(N,p);
frac=c6_route_fractions(D0(:,1),vias,D1(:,1));
A=[]; B=zeros(0,3);
for r=0:6
    A=[A;c6_bs_basis_derivative_all(0,U,p,N,r)/T^r]; B=[B;D0(:,r+1)']; %#ok<AGROW>
end
for r=0:6
    A=[A;c6_bs_basis_derivative_all(1,U,p,N,r)/T^r]; B=[B;D1(:,r+1)']; %#ok<AGROW>
end
for k=1:nVia
    A=[A;c6_bs_basis_derivative_all(frac(k),U,p,N,0)]; B=[B;vias(:,k)']; %#ok<AGROW>
end
% Minimize integrated square of 7th derivative in normalized coordinate.
spans=c6_bs_spans(U,p); Q=zeros(N); qn=5;
[xg,wg]=c6_gauss5();
for s=1:size(spans,1)
    ua=spans(s,1); ub=spans(s,2);
    for q=1:qn
        u=(ua+ub)/2+(ub-ua)/2*xg(q); b7=c6_bs_basis_derivative_all(u,U,p,N,7)/T^7;
        Q=Q+(ub-ua)/2*wg(q)*(b7'*b7)*T;
    end
end
[X,condm]=c6_eq_qp(Q,A,B);
% Convert every nonzero knot span exactly (up to 8x8 solve roundoff) to Bezier.
Bez=cell(size(spans,1),1); dt=zeros(1,size(spans,1));
for s=1:size(spans,1)
    ua=spans(s,1); ub=spans(s,2); tau=(0:p)'/p; Mbern=c6_bernstein_matrix(p,tau); V=zeros(p+1,3);
    for k=1:p+1
        u=ua+(ub-ua)*tau(k); V(k,:)=c6_bs_basis_derivative_all(u,U,p,N,0)*X;
    end
    C=Mbern\V; Bez{s}=C'; dt(s)=T*(ub-ua);
end
seg=struct('name','BSPLINE_D7','B',{Bez},'dt',dt,'D0',D0,'D1',D1,'structural_c6',true,'condition_metric',condm);
end

function U=c6_clamped_uniform_knots(N,p)
ni=N-p-1; if ni>0, internal=(1:ni)/(ni+1); else, internal=[]; end
U=[zeros(1,p+1),internal,ones(1,p+1)];
end
function spans=c6_bs_spans(U,p)
spans=[]; for k=p+1:numel(U)-p-1, if U(k+1)>U(k), spans=[spans;U(k) U(k+1)]; end,end %#ok<AGROW>
end
function dN=c6_bs_basis_derivative_all(u,U,p,N,r)
dN=zeros(1,N); for i=1:N,dN(i)=c6_bs_basis_derivative_single(i,p,r,u,U);end
end
function v=c6_bs_basis_derivative_single(i,p,r,u,U)
if r>p,v=0;return,end
if r==0,v=c6_bs_basis(i,p,u,U);return,end
if p==0,v=0;return,end
den1=U(i+p)-U(i); den2=U(i+p+1)-U(i+1); a=0;b=0;
if den1~=0,a=p/den1*c6_bs_basis_derivative_single(i,p-1,r-1,u,U);end
if den2~=0,b=p/den2*c6_bs_basis_derivative_single(i+1,p-1,r-1,u,U);end
v=a-b;
end
function v=c6_bs_basis(i,p,u,U)
if i<1 || i+p+1>numel(U),v=0;return,end
if p==0
    v=double((u>=U(i) && u<U(i+1)) || (abs(u-1)<1e-14 && U(i+1)==1 && U(i)<1)); return
end
a=0;b=0; d1=U(i+p)-U(i); d2=U(i+p+1)-U(i+1);
if d1~=0,a=(u-U(i))/d1*c6_bs_basis(i,p-1,u,U);end
if d2~=0,b=(U(i+p+1)-u)/d2*c6_bs_basis(i+1,p-1,u,U);end
v=a+b;
end

% ============================ POWER POLYNOMIAL ===========================
function seg=c6_build_poly(T,D0,D1,vias,P)
n=P.n_poly; M=size(vias,2)+1; frac=c6_route_fractions(D0(:,1),vias,D1(:,1)); br=[0 frac 1]; dt=T*diff(br);
[A,B]=c6_poly_eq(n,dt,D0,D1,vias); Q=c6_poly_Q(n,dt,7); [X,condm]=c6_eq_qp(Q,A,B);
Bez=cell(M,1);
for s=1:M
    ix=(s-1)*(n+1)+(1:n+1); C=X(ix,:); BB=zeros(3,n+1);
    for ax=1:3, BB(ax,:)=c6_power_to_bernstein(C(:,ax)',n); end
    Bez{s}=BB;
end
seg=struct('name','POLY_D13_MIN7','B',{Bez},'dt',dt,'D0',D0,'D1',D1,'structural_c6',true,'condition_metric',condm);
end
function [A,B]=c6_poly_eq(n,dt,D0,D1,vias)
M=numel(dt); A=[];B=[];
for r=0:6,A=[A;c6_poly_row(n,M,1,0,r,dt(1))];B=[B;D0(:,r+1)'];end %#ok<AGROW>
for r=0:6,A=[A;c6_poly_row(n,M,M,1,r,dt(end))];B=[B;D1(:,r+1)'];end %#ok<AGROW>
for s=1:M-1
    q=vias(:,s)';
    A=[A;c6_poly_row(n,M,s,1,0,dt(s))];B=[B;q]; %#ok<AGROW>
    for r=1:6,A=[A;c6_poly_row(n,M,s,1,r,dt(s))-c6_poly_row(n,M,s+1,0,r,dt(s+1))];B=[B;0 0 0];end %#ok<AGROW>
    A=[A;c6_poly_row(n,M,s+1,0,0,dt(s+1))];B=[B;q]; %#ok<AGROW>
end
end
function row=c6_poly_row(n,M,s,tau,r,dt)
row=zeros(1,(n+1)*M); q=zeros(1,n+1);
for k=r:n,q(k+1)=factorial(k)/factorial(k-r)*tau^(k-r)/dt^r;end
ix=(s-1)*(n+1)+(1:n+1);row(ix)=q;
end
function Q=c6_poly_Q(n,dt,r)
M=numel(dt);Q=zeros((n+1)*M);for s=1:M
 q=zeros(n+1);for i=r:n,for j=r:n
  ci=factorial(i)/factorial(i-r);cj=factorial(j)/factorial(j-r);
  q(i+1,j+1)=ci*cj/(i+j-2*r+1)/dt(s)^(2*r-1);
 end,end
 ix=(s-1)*(n+1)+(1:n+1);Q(ix,ix)=q;
end
end
function b=c6_power_to_bernstein(a,n)
b=zeros(1,n+1);for k=0:n,for j=0:k,b(k+1)=b(k+1)+a(j+1)*nchoosek(k,j)/nchoosek(n,j);end,end
end

% ================================= BEZIER ================================
function seg=c6_build_bezier(T,D0,D1,vias,P)
n=P.n_bez;M=size(vias,2)+1;frac=c6_route_fractions(D0(:,1),vias,D1(:,1));br=[0 frac 1];dt=T*diff(br);
[A,B]=c6_bezier_eq(n,dt,D0,D1,vias);Q=c6_bezier_Q(n,dt,7);[X,condm]=c6_eq_qp(Q,A,B);
Bez=cell(M,1);for s=1:M,ix=(s-1)*(n+1)+(1:n+1);Bez{s}=X(ix,:)';end
seg=struct('name','BEZIER_D13_MIN7','B',{Bez},'dt',dt,'D0',D0,'D1',D1,'structural_c6',true,'condition_metric',condm);
end
function [A,B]=c6_bezier_eq(n,dt,D0,D1,vias)
M=numel(dt);A=[];B=[];
for r=0:6,A=[A;c6_bezier_row(n,M,1,0,r,dt(1))];B=[B;D0(:,r+1)'];end %#ok<AGROW>
for r=0:6,A=[A;c6_bezier_row(n,M,M,1,r,dt(end))];B=[B;D1(:,r+1)'];end %#ok<AGROW>
for s=1:M-1
 q=vias(:,s)';
 A=[A;c6_bezier_row(n,M,s,1,0,dt(s))];B=[B;q]; %#ok<AGROW>
 for r=1:6,A=[A;c6_bezier_row(n,M,s,1,r,dt(s))-c6_bezier_row(n,M,s+1,0,r,dt(s+1))];B=[B;0 0 0];end %#ok<AGROW>
 A=[A;c6_bezier_row(n,M,s+1,0,0,dt(s+1))];B=[B;q]; %#ok<AGROW>
end
end
function row=c6_bezier_row(n,M,s,atEnd,r,dt)
row=zeros(1,(n+1)*M);q=zeros(1,n+1);scale=factorial(n)/factorial(n-r)/dt^r;
if ~atEnd
    for k=0:r,q(k+1)=scale*(-1)^(r-k)*nchoosek(r,k);end
else
    for k=0:r,q(n-r+k+1)=scale*(-1)^(r-k)*nchoosek(r,k);end
end
ix=(s-1)*(n+1)+(1:n+1);row(ix)=q;
end
function Q=c6_bezier_Q(n,dt,r)
M=numel(dt);nv=(n+1)*M;Q=zeros(nv);D=eye(n+1);for k=1:r,D=diff(D,1,1);end
for s=1:M
 scale=(factorial(n)/factorial(n-r)/dt(s)^r)^2*dt(s);ix=(s-1)*(n+1)+(1:n+1);Q(ix,ix)=scale*(D'*D);
end
end

% ========================== EQUALITY-QP SOLVER ===========================
function [X,condm]=c6_eq_qp(Q,A,B)
% Row-normalized equality constrained quadratic minimization:
%   min .5 x'Qx, A x=b.
% Uses a particular solution plus nullspace.  This avoids the badly scaled
% monolithic KKT solve seen in the first prototype.
rowScale=max(vecnorm(A,2,2),1e-14); An=A./rowScale; Bn=B./rowScale;
rankA=rank(An,1e-10); if rankA<size(An,1)
    % Redundant continuity/waypoint rows are normal; use SVD particular solve.
end
X0=lsqminnorm(An,Bn,1e-12);
res=norm(An*X0-Bn,'fro'); if res>1e-7*(1+norm(Bn,'fro')),error('C6BASE:EqualityInfeasible','Equality residual %.3e',res);end
Z=null(An,1e-10);
if isempty(Z),X=X0;else
    H=Z'*Q*Z; G=Z'*Q*X0; reg=1e-11*max(1,norm(H,'fro'));
    X=X0-Z*((H+reg*eye(size(H)))\G);
end
s=svd(An); s=s(s>1e-12); if isempty(s),condm=1;else,condm=max(s)/min(s);end
end

% ======================= COMMON CONTINUOUS CERTIFICATE ==================
function cert=c6_certify(seg,obs,P)
N=P.sep_dirs;K=size(N,2); no=numel(obs);
minG=inf;minL=inf;minU=inf;minC=inf;vmax=0;amax=0;jmax=0;tLB=inf;pass=true;
if no==0
    H=zeros(K,0);
else
    H=zeros(K,no);for o=1:no,for k=1:K,H(k,o)=c6_obstacle_support(obs(o),N(:,k));end,end
end
for s=1:numel(seg.B)
    B=seg.B{s};dt=seg.dt(s);
    V=c6_bern_derivative(B,dt,1);A=c6_bern_derivative(B,dt,2);J=c6_bern_derivative(B,dt,3);
    vmax=max(vmax,max(vecnorm(V,2,1)));amax=max(amax,max(vecnorm(A,2,1)));jmax=max(jmax,max(vecnorm(J,2,1)));
    W=A+P.g*P.e3; Mmax=max(vecnorm(W,2,1)); Mlb=max(0,min(W(3,:))); tLB=min(tLB,Mlb);
    if Mlb<=1e-8, pass=false; end
    plow=min(N'*B,[],2); wmin=min(N'*W,[],2); qlow=-ones(K,1);
    pos=wmin>=0; if Mmax>1e-12, qlow(pos)=wmin(pos)/Mmax;end
    neg=~pos; if Mlb>1e-12, qlow(neg)=max(-1,wmin(neg)/Mlb);end
    dL=P.R_payload*ones(K,1);
    dU=P.R_uav-P.L_cable*qlow;
    dC=P.R_cable+max(0,-P.L_cable*qlow);
    if no>0
        GL=plow-dL-H-P.d_margin; GU=plow-dU-H-P.d_margin; GC=plow-dC-H-P.d_margin;
        bL=max(GL,[],1);bU=max(GU,[],1);bC=max(GC,[],1);
        minL=min(minL,min(bL));minU=min(minU,min(bU));minC=min(minC,min(bC));
        segObs=min([bL;bU;bC],[],1); minG=min(minG,min(segObs));
        if any(bL<0) || any(bU<0) || any(bC<0),pass=false;end
    end
end
if no==0,minG=inf;minL=inf;minU=inf;minC=inf;end
cert=struct('pass',pass,'minG',minG,'minG_payload',minL,'minG_uav',minU,'minG_cable',minC,'v',vmax,'a',amax,'j',jmax,'tension_lb',tLB,'n_obstacles',no);
end
function D=c6_bern_derivative(B,T,r)
D=B;n=size(B,2)-1;for q=1:r,D=(n-q+1)/T*diff(D,1,2);end
end
function h=c6_obstacle_support(ob,n)
h=n'*ob.center+sqrt(max(0,n'*ob.S*n));
end

% ========================== VALIDATION METRICS ===========================
function [e,by,rel]=c6_c6_error(seg)
by=zeros(1,7);rel=zeros(1,7);M=numel(seg.B);
for s=1:M-1
    for r=0:6
        l=c6_bez_end(seg.B{s},seg.dt(s),r,true);
        rr=c6_bez_end(seg.B{s+1},seg.dt(s+1),r,false);
        er=norm(l-rr); sc=max([1,norm(l),norm(rr)]);
        by(r+1)=max(by(r+1),er); rel(r+1)=max(rel(r+1),er/sc);
    end
end
e=max(by);
end
function [e,by,rel]=c6_endpoint_error(seg)
by=zeros(1,7);rel=zeros(1,7);
for r=0:6
    a0=c6_bez_end(seg.B{1},seg.dt(1),r,false); b0=seg.D0(:,r+1);
    a1=c6_bez_end(seg.B{end},seg.dt(end),r,true); b1=seg.D1(:,r+1);
    e0=norm(a0-b0); e1=norm(a1-b1);
    by(r+1)=max(e0,e1);
    rel(r+1)=max(e0/max([1,norm(a0),norm(b0)]),e1/max([1,norm(a1),norm(b1)]));
end
e=max(by);
end
function v=c6_bez_end(B,T,r,atEnd)
D=c6_bern_derivative(B,T,r);if atEnd,v=D(:,end);else,v=D(:,1);end
end

% ====================== SENSOR / COLLISION TRIGGER =======================
function tr=c6_sensor_trigger(S,P)
tr=struct('found',false,'initial_outside',true,'first_detect',NaN,'t_trigger',NaN,'TTC',NaN,'known_ids',[]);
% Nominal payload pL(t)=p0+v*t, a=0; UAV is one cable length above payload.
pQ0=S.p0+P.L_cable*P.e3;
for i=1:numel(S.world_obs)
    if c6_point_ellipsoid_lb(pQ0,S.world_obs(i))<=P.sensor_R,tr.initial_outside=false;end
end
memory=false(1,numel(S.world_obs));
for t=0:P.dt_sensor:40
    pL=S.p0+S.v0*t;pQ=pL+P.L_cable*P.e3;
    for i=1:numel(S.world_obs)
        if c6_point_ellipsoid_lb(pQ,S.world_obs(i))<=P.sensor_R
            if isnan(tr.first_detect),tr.first_detect=t;end
            memory(i)=true;
        end
    end
    known=find(memory); if isempty(known),continue,end
    [hit,ttc]=c6_nominal_collision_prediction(pL,S.v0,S.world_obs(known),P);
    if hit,tr.found=true;tr.t_trigger=t;tr.TTC=ttc;tr.known_ids=known;return,end
end
end
function d=c6_point_ellipsoid_lb(p,ob)
% Safe lower bound using circumscribed radius. Used only by sensor emulation,
% not by the final safety certificate.
d=max(0,norm(p-ob.center)-max(ob.radii));
end
function [hit,ttc]=c6_nominal_collision_prediction(pL,v,obs,P)
hit=false;ttc=inf;for tau=0:P.pred_dt:P.lookahead
    p=pL+v*tau; % nominal acceleration zero -> cable vertical
    pQ=p+P.L_cable*P.e3;
    for i=1:numel(obs)
        % Conservative trigger only: point-to-center minus circumscribed
        % ellipsoid radius and each body radius / cable capsule envelope.
        ro=max(obs(i).radii)+P.d_margin;
        if norm(p-obs(i).center)<=ro+P.R_payload || norm(pQ-obs(i).center)<=ro+P.R_uav || c6_point_segment_distance(obs(i).center,p,pQ)<=ro+P.R_cable
            hit=true;ttc=tau;return
        end
    end
end
end
function d=c6_point_segment_distance(x,a,b)
v=b-a; if dot(v,v)<1e-14,d=norm(x-a);return,end
t=max(0,min(1,dot(x-a,v)/dot(v,v)));d=norm(x-(a+t*v));
end

% =========================== ROUTE CANDIDATES ============================
function [routes,tags]=c6_route_candidates_single(p0,pg,obs,P)
e=pg-p0;e=e/max(norm(e),1e-12);ref=[0;0;1];if abs(e'*ref)>0.9,ref=[0;1;0];end
b1=cross(e,ref);b1=b1/norm(b1);b2=cross(e,b1);b2=b2/norm(b2);
dirs=[b1 -b1 b2 -b2 (b1+b2)/sqrt(2) (b1-b2)/sqrt(2)];
routes={};tags={}; line=pg-p0;den=max(dot(line,line),1e-12);
for io=1:numel(P.offsets)
    off=P.offsets(io);
    for id=1:size(dirs,2)
        d=dirs(:,id);V=zeros(3,numel(obs));ss=zeros(1,numel(obs));
        for k=1:numel(obs)
            ss(k)=dot(obs(k).center-p0,line)/den;
            V(:,k)=obs(k).center+d*(max(obs(k).radii)+P.R_uav+P.L_cable+P.d_margin+off);
        end
        [~,ord]=sort(ss);routes{end+1}=V(:,ord);tags{end+1}=sprintf('d%d_o%.1f',id,off); %#ok<AGROW>
    end
end
end
function [routes,tags,diagout,selectedPg,selectedPhase]=c6_route_candidates_graph_rejoin(p0,vNom,obs,P)
% MULTI-OBSTACLE FRONT-END WITH AUTOMATIC NOMINAL REJOIN SELECTION.
% Rejoin phase candidates are nominal future times, independent of the
% avoidance execution duration T.  Candidates are tested earliest first.
% A candidate is admissible only if (i) the goal point itself has positive
% guide clearance against every known obstacle and (ii) the safe-edge graph
% contains a start-to-goal path.  Once the earliest admissible rejoin phase
% is found, routes for that SAME rejoin point are collected over guide_extra.

routes={}; tags={}; selectedPg=[]; selectedPhase=NaN;
raw=0; totalEdges=0; maxNodes=0;
rejoinDiag=repmat(struct('phase',NaN,'goal',[NaN;NaN;NaN],'point_safe',false,...
    'point_margin',-Inf,'point_worst_obs',0,'reachable',false,...
    'best_goal_degree',0,'best_goal_edge_margin',-Inf),1,numel(P.rejoin_phase_candidates));
selectedByExtra=[];

for irj=1:numel(P.rejoin_phase_candidates)
    phase=P.rejoin_phase_candidates(irj);
    pg=p0+vNom*phase;
    [goalPointSafe,goalPointDiag]=c6_guide_edge_safe_diag(pg,pg,obs,P);
    bestGoalDegree=0; bestGoalEdgeMargin=-Inf; candidateReachable=false;
    candidateRoutes={}; candidateTags={}; candidateRaw=0; candidateEdges=0; candidateMaxNodes=0;
    byExtra=repmat(struct('extra',NaN,'nodes',0,'edges',0,'rejected',0,...
        'start_degree',0,'goal_degree',0,'reachable_start',0,'reachable_goal',0,...
        'same_component',false,'direct_start_goal',false,'raw_route',false,...
        'reject_by_obs',[],'worst_margin',Inf,'worst_obs',0,...
        'start_best_margin',-Inf,'start_best_node',0,'start_best_obs',0,...
        'goal_best_margin',-Inf,'goal_best_node',0,'goal_best_obs',0),1,numel(P.guide_extra));

    % If the point itself is not certified safe, no graph edge ending at it
    % can provide a certified rejoin, so skip graph construction for this
    % candidate while still recording the diagnostic.
    if goalPointSafe
        for ie=1:numel(P.guide_extra)
            extra=P.guide_extra(ie);
            nodes=[p0,pg];
            for io=1:numel(obs)
                ob=obs(io);
                for k=1:size(P.guide_dirs,2)
                    d=P.guide_dirs(:,k); d=d/max(norm(d),1e-12);
                    rho=sqrt(max(0,d'*ob.S*d));
                    delta=c6_vertical_system_support(d,P)+P.d_margin+extra;
                    q=ob.center+d*(rho+delta);
                    nodes(:,end+1)=q; %#ok<AGROW>
                end
            end
            n=size(nodes,2); candidateMaxNodes=max(candidateMaxNodes,n);
            W=inf(n); W(1:n+1:end)=0;
            accepted=0; rejected=0; rejectByObs=zeros(1,numel(obs));
            worstMargin=Inf; worstObs=0;
            startBestMargin=-Inf; startBestNode=0; startBestObs=0;
            goalBestMargin=-Inf; goalBestNode=0; goalBestObs=0;

            for i=1:n-1
                for j=i+1:n
                    [ok,ed]=c6_guide_edge_safe_diag(nodes(:,i),nodes(:,j),obs,P);
                    if ok
                        w=norm(nodes(:,j)-nodes(:,i)); W(i,j)=w; W(j,i)=w;
                        accepted=accepted+1;
                    else
                        rejected=rejected+1;
                        for io=1:numel(obs)
                            if ed.best_by_obs(io)<P.guide_edge_margin
                                rejectByObs(io)=rejectByObs(io)+1;
                            end
                        end
                        if ed.worst_margin<worstMargin
                            worstMargin=ed.worst_margin; worstObs=ed.worst_obs;
                        end
                    end
                    if i==1 || j==1
                        other=j; if j==1,other=i;end
                        if ed.worst_margin>startBestMargin
                            startBestMargin=ed.worst_margin; startBestNode=other; startBestObs=ed.worst_obs;
                        end
                    end
                    if i==2 || j==2
                        other=j; if j==2,other=i;end
                        if ed.worst_margin>goalBestMargin
                            goalBestMargin=ed.worst_margin; goalBestNode=other; goalBestObs=ed.worst_obs;
                        end
                    end
                end
            end
            candidateEdges=candidateEdges+accepted;
            adj=isfinite(W) & W>0;
            startDegree=nnz(adj(1,:)); goalDegree=nnz(adj(2,:));
            reachS=c6_graph_reachable(adj,1); reachG=c6_graph_reachable(adj,2);
            sameComp=reachS(2); directSG=adj(1,2);
            path=c6_dijkstra(W,1,2); rawFound=~isempty(path);
            bestGoalDegree=max(bestGoalDegree,goalDegree);
            bestGoalEdgeMargin=max(bestGoalEdgeMargin,goalBestMargin);

            byExtra(ie)=struct('extra',extra,'nodes',n,'edges',accepted,'rejected',rejected,...
                'start_degree',startDegree,'goal_degree',goalDegree,...
                'reachable_start',nnz(reachS),'reachable_goal',nnz(reachG),...
                'same_component',sameComp,'direct_start_goal',directSG,'raw_route',rawFound,...
                'reject_by_obs',rejectByObs,'worst_margin',worstMargin,'worst_obs',worstObs,...
                'start_best_margin',startBestMargin,'start_best_node',startBestNode,'start_best_obs',startBestObs,...
                'goal_best_margin',goalBestMargin,'goal_best_node',goalBestNode,'goal_best_obs',goalBestObs);

            if isempty(path),continue,end
            candidateReachable=true; candidateRaw=candidateRaw+1;
            Pbase=nodes(:,path);
            Pbase=c6_prune_polyline(Pbase,obs,P);
            % Keep the certified homotopy/graph route fixed and refine only
            % the waypoint density.  This is common to all three C6 methods.
            for ispc=1:numel(P.guide_refine_spacings)
                sp=P.guide_refine_spacings(ispc);
                Pline=c6_densify_polyline(Pbase,sp);
                vias=Pline(:,2:end-1); % empty is valid for a direct certified route
                duplicate=false;
                for r=1:numel(candidateRoutes)
                    if size(candidateRoutes{r},2)==size(vias,2) && norm(candidateRoutes{r}-vias,'fro')<1e-6
                        duplicate=true; break
                    end
                end
                if ~duplicate
                    candidateRoutes{end+1}=vias; %#ok<AGROW>
                    candidateTags{end+1}=sprintf('r%.1f_e%.2f_s%.2f',phase,extra,sp); %#ok<AGROW>
                end
            end
        end
    end

    rejoinDiag(irj)=struct('phase',phase,'goal',pg,'point_safe',goalPointSafe,...
        'point_margin',goalPointDiag.worst_margin,'point_worst_obs',goalPointDiag.worst_obs,...
        'reachable',candidateReachable,'best_goal_degree',bestGoalDegree,...
        'best_goal_edge_margin',bestGoalEdgeMargin);

    if goalPointSafe && candidateReachable && ~isempty(candidateRoutes)
        selectedPg=pg; selectedPhase=phase;
        routes=candidateRoutes(1:min(numel(candidateRoutes),P.max_graph_routes));
        tags=candidateTags(1:min(numel(candidateTags),P.max_graph_routes));
        raw=candidateRaw; totalEdges=candidateEdges; maxNodes=candidateMaxNodes;
        selectedByExtra=byExtra;
        break
    end
end

if isempty(selectedByExtra)
    selectedByExtra=repmat(struct('extra',NaN,'nodes',0,'edges',0,'rejected',0,...
        'start_degree',0,'goal_degree',0,'reachable_start',0,'reachable_goal',0,...
        'same_component',false,'direct_start_goal',false,'raw_route',false,...
        'reject_by_obs',[],'worst_margin',Inf,'worst_obs',0,...
        'start_best_margin',-Inf,'start_best_node',0,'start_best_obs',0,...
        'goal_best_margin',-Inf,'goal_best_node',0,'goal_best_obs',0),1,numel(P.guide_extra));
end

diagout=struct('nodes',maxNodes,'edges',totalEdges,'raw_routes',raw,...
    'by_extra',selectedByExtra,'rejoin_candidates',rejoinDiag,...
    'selected_phase',selectedPhase,'selected_goal',selectedPg);
end

function reach=c6_graph_reachable(adj,s)
n=size(adj,1); reach=false(1,n); q=zeros(1,n); head=1; tail=1; q(1)=s; reach(s)=true;
while head<=tail
    u=q(head); head=head+1;
    nb=find(adj(u,:));
    for v=nb
        if ~reach(v),tail=tail+1;q(tail)=v;reach(v)=true;end
    end
end
end

function c6_print_route_graph_diag(D)
fprintf('[REJOIN-DIAG] nominal candidates, earliest safe+reachable is selected\n');
for k=1:numel(D.rejoin_candidates)
    r=D.rejoin_candidates(k);
    if ~isfinite(r.phase),continue,end
    fprintf('  phase=%5.2f goal=[%7.3f %7.3f %7.3f] pointSafe=%d pointMargin=%+.3e worstObs=%d reachable=%d bestGoalDeg=%d bestGoalEdge=%+.3e%s\n',...
        r.phase,r.goal(1),r.goal(2),r.goal(3),r.point_safe,r.point_margin,r.point_worst_obs,r.reachable,r.best_goal_degree,r.best_goal_edge_margin,...
        c6_selected_mark(r.phase,D.selected_phase));
end
fprintf('[ROUTE-GRAPH-DIAG] per guide_extra for SELECTED rejoin candidate\n');
for ie=1:numel(D.by_extra)
    x=D.by_extra(ie);
    fprintf('  extra=%4.2f nodes=%3d accepted=%4d rejected=%4d startDeg=%3d goalDeg=%3d reachS=%3d reachG=%3d sameComp=%d directSG=%d route=%d\n',...
        x.extra,x.nodes,x.edges,x.rejected,x.start_degree,x.goal_degree,x.reachable_start,x.reachable_goal,x.same_component,x.direct_start_goal,x.raw_route);
    fprintf('             startBestMargin=%+.3e ->node=%d worstObs=%d | goalBestMargin=%+.3e ->node=%d worstObs=%d | globalWorst=%+.3e obs=%d\n',...
        x.start_best_margin,x.start_best_node,x.start_best_obs,x.goal_best_margin,x.goal_best_node,x.goal_best_obs,x.worst_margin,x.worst_obs);
    if isempty(x.reject_by_obs)
        fprintf('             rejectByObs=[]\n');
    else
        fprintf('             rejectByObs=%s (an edge can be counted for multiple obstacles)\n',mat2str(x.reject_by_obs));
    end
end
end

function delta=c6_vertical_system_support(n,P)
% Required payload-center support offset for the three bodies when cable is
% vertical.  Each body remains separate; max() is only used to find a guide
% separator that is simultaneously sufficient for all three.
nzv=n(3);
dL=P.R_payload;
dU=P.R_uav-P.L_cable*nzv;
dC=P.R_cable+max(0,-P.L_cable*nzv);
delta=max([dL,dU,dC]);
end

function ok=c6_guide_edge_safe(a,b,obs,P)
[ok,~]=c6_guide_edge_safe_diag(a,b,obs,P);
end

function [ok,D]=c6_guide_edge_safe_diag(a,b,obs,P)
% Passive diagnostic companion to c6_guide_edge_safe.  best_by_obs(io) is
% the best separating margin for obstacle io over the SAME separator set
% used by the planner.  Therefore no diagnostic approximation is introduced.
bestByObs=-inf(1,numel(obs));
for io=1:numel(obs)
    best=-inf;
    for k=1:size(P.guide_sep_dirs,2)
        n=P.guide_sep_dirs(:,k);
        g=min(n'*a,n'*b)-c6_obstacle_support(obs(io),n)-c6_vertical_system_support(n,P)-P.d_margin;
        if g>best,best=g;end
    end
    bestByObs(io)=best;
end
if isempty(bestByObs)
    worstMargin=Inf; worstObs=0; ok=true;
else
    [worstMargin,worstObs]=min(bestByObs);
    ok=all(bestByObs>=P.guide_edge_margin);
end
D=struct('best_by_obs',bestByObs,'worst_margin',worstMargin,'worst_obs',worstObs);
end

function path=c6_dijkstra(W,s,t)
n=size(W,1);dist=inf(1,n);prev=zeros(1,n);used=false(1,n);dist(s)=0;
for it=1:n
    cand=find(~used);if isempty(cand),break,end
    [~,ii]=min(dist(cand));u=cand(ii);if isinf(dist(u)),break,end
    used(u)=true;if u==t,break,end
    nb=find(isfinite(W(u,:)) & W(u,:)>0);
    for v=nb
        alt=dist(u)+W(u,v);
        if alt<dist(v),dist(v)=alt;prev(v)=u;end
    end
end
if isinf(dist(t)),path=[];return,end
path=t;u=t;while u~=s,u=prev(u);if u==0,path=[];return,end;path=[u path];end %#ok<AGROW>
end

function Pout=c6_prune_polyline(Pin,obs,P)
% Greedy line-of-sight shortcutting, but only when the replacement edge has
% the same continuous guide certificate.  This is therefore not heuristic
% obstacle deletion.
if size(Pin,2)<=2,Pout=Pin;return,end
idx=1;sel=1;
while idx<size(Pin,2)
    j=size(Pin,2);
    while j>idx+1 && ~c6_guide_edge_safe(Pin(:,idx),Pin(:,j),obs,P),j=j-1;end
    sel(end+1)=j;idx=j; %#ok<AGROW>
end
Pout=Pin(:,sel);
end

function Pout=c6_densify_polyline(Pin,spacing)
Pout=Pin(:,1);
for i=1:size(Pin,2)-1
    a=Pin(:,i);b=Pin(:,i+1);L=norm(b-a);m=max(1,ceil(L/spacing));
    for k=1:m,Pout(:,end+1)=a+(k/m)*(b-a);end %#ok<AGROW>
end
end

function dmax=c6_max_guide_deviation(seg,Pline,P)
% Diagnostic only: maximum sampled distance from the generated C6 curve to
% the certified/densified guide polyline.  It is NOT a safety certificate;
% final acceptance remains c6_certify().
dmax=0;
if isempty(Pline) || size(Pline,2)<2,return,end
ns=max(3,P.guide_dev_samples); tau=linspace(0,1,ns)';
for s=1:numel(seg.B)
    B=seg.B{s}; M=c6_bernstein_matrix(size(B,2)-1,tau); X=B*M';
    for k=1:size(X,2)
        d=inf;
        for j=1:size(Pline,2)-1
            d=min(d,c6_point_segment_distance(X(:,k),Pline(:,j),Pline(:,j+1)));
        end
        dmax=max(dmax,d);
    end
end
end

function f=c6_route_fractions(p0,vias,pg)
P=[p0,vias,pg];L=vecnorm(diff(P,1,2),2,1);c=[0 cumsum(L)];
if c(end)<1e-12,f=(1:size(vias,2))/(size(vias,2)+1);else,f=c(2:end-1)/c(end);end
f=max(1e-4,min(1-1e-4,f));
end

% =========================== REQUIREMENT REPORT ==========================
function c6_print_stage_requirements(R,S,P)
rows=R.rows; built=any([rows.built]); core=any([rows.safe]); c6ok=any([rows.built] & [rows.c6_error]<=P.c6_tol & [rows.endpoint_error]<=P.c6_tol);
known=all(arrayfun(@(x) ~x.built || x.known_all,rows));
fprintf('\n[REQUIREMENTS %s / %s]\n',S.name,R.method);
fprintf(' R1 build candidate                         : %s\n',c6_pf(built));
fprintf(' R2 structural/global C6 + splice d0..d6   : %s\n',c6_pf(c6ok));
fprintf(' R3 separate Payload/UAV/Cable geometry    : PASS (certificate architecture)\n');
fprintf(' R4 oriented ellipsoid obstacles           : PASS (support-function model)\n');
fprintf(' R5 all known obstacles INCLUDED/CHECKED   : %s\n',c6_pf(known));
fprintf(' R6 continuous-time certificate USED       : PASS (Bernstein convex-hull sufficient certificate)\n');
if S.sensor
 fprintf(' R7 sensor radius 15 m + persistent memory : %s\n',c6_pf(R.trigger.initial_outside && ~isnan(R.trigger.first_detect)));
 fprintf(' R8 future-collision trigger before replan : %s (trigger=%.2f, TTC=%.2f)\n',c6_pf(R.trigger.found),R.trigger.t_trigger,R.trigger.TTC);
 fprintf(' R9 multi-obstacle route front-end         : %s\n',c6_pf(~strcmp(rows(1).reason,'NO_GRAPH_ROUTE')));
else
 fprintf(' R7/R8/R9 sensor/trigger/graph             : N/A in this stage\n');
end
fprintf(' R10 certified collision-free candidate    : %s\n',c6_pf(core));
fprintf(' CORE ACCEPTANCE (C6 + geometric safety)    : %s\n',c6_pf(core));
% Report d0...d6 separately for the best safe row; if none is safe, use the
% built candidate with the largest geometric margin so numerical smoothness
% remains visible even in a geometry failure.
ib=find([rows.safe],1,'first');
if isempty(ib)
    ibs=find([rows.built]);
    if ~isempty(ibs),[~,q]=max([rows(ibs).minG]);ib=ibs(q);end
end
if ~isempty(ib)
    rr=rows(ib);
    fprintf(' C6 JOIN abs residual d0..d6               : %s\n',c6_vec(rr.c6_by_order));
    fprintf(' C6 JOIN rel residual d0..d6               : %s\n',c6_vec(rr.c6_rel_by_order));
    fprintf(' SPLICE abs residual d0..d6                : %s\n',c6_vec(rr.end_by_order));
    fprintf(' SPLICE rel residual d0..d6                : %s\n',c6_vec(rr.end_rel_by_order));
end
end
function req=c6_requirement_summary(methods,out,scenarios,P)
req=struct();req.methods=methods;req.stage=cell(numel(methods),numel(scenarios));
for im=1:numel(methods),for is=1:numel(scenarios)
 R=out{im,is};if isempty(R),continue,end;rows=R.rows;
 req.stage{im,is}=struct('build',any([rows.built]),'c6',any([rows.built]&[rows.c6_error]<=P.c6_tol&[rows.endpoint_error]<=P.c6_tol),'safe',any([rows.safe]));
end,end
end
function c6_print_cross_method_summary(result)
fprintf('\n========================================================================================================================\n');
fprintf(' CROSS-METHOD FOUNDATION SUMMARY\n');
fprintf('========================================================================================================================\n');
fprintf(' method               StageA build/C6/core   StageB build/C6/core   StageC build/C6/core\n');
for im=1:numel(result.methods)
 fprintf(' %-20s',result.methods(im));
 for is=1:numel(result.scenarios)
  x=result.requirements.stage{im,is};fprintf('      %d/%d/%d         ',x.build,x.c6,x.safe);
 end
 fprintf('\n');
end
fprintf('\nInterpretation rule: do NOT rank methods until each has at least Stage A build+C6 PASS.\n');
fprintf('Current gate: Stage C must produce at least one certified multi-obstacle route for all three methods before adding v/a/j or HLC input constraints.\n');
end
function s=c6_selected_mark(x,sel)
if isfinite(sel) && abs(x-sel)<1e-12,s='  <SELECTED>';else,s='';end
end
function s=c6_num(x)
if isempty(x) || ~isfinite(x),s='NONE';else,s=sprintf('%.2f',x);end
end
function s=c6_vec(v),s=['[' strtrim(sprintf(' %.2e',v)) ' ]'];end
function s=c6_pf(x),if x,s='PASS';else,s='FAIL';end,end
function r=c6_row()
r=struct('method','','route','','T',NaN,'built',false,'build_ms',NaN,'cert_ms',NaN,'total_ms',NaN,'structural_c6',false,...
    'c6_error',NaN,'c6_by_order',nan(1,7),'c6_rel_by_order',nan(1,7),...
    'endpoint_error',NaN,'end_by_order',nan(1,7),'end_rel_by_order',nan(1,7),...
    'condition_metric',NaN,'n_vias',0,'guide_dev',NaN,'geom',false,'known_all',false,'safe',false,'minG',NaN,'minG_payload',NaN,'minG_uav',NaN,'minG_cable',NaN,'v',NaN,'a',NaN,'j',NaN,'tension_lb',NaN,'reason','','message','');
end
function R=c6_empty_run(method,name,reason)
R=struct('method',char(method),'scenario',name,'rows',c6_row(),'best',[],'trigger',[],'known_count',0);R.rows.reason=reason;
end

% ================================ UTILS ==================================
function D=c6_nominal_jet(p,v)
D=zeros(3,7);D(:,1)=p;D(:,2)=v;
end
function ob=c6_make_obs(c,r,R,id)
ob=struct('center',c(:),'radii',r(:),'R',R,'S',R*diag(r(:).^2)*R','id',id);
end
function R=c6_Rz(a),R=[cos(a) -sin(a) 0;sin(a) cos(a) 0;0 0 1];end
function R=c6_Ry(a),R=[cos(a) 0 sin(a);0 1 0;-sin(a) 0 cos(a)];end
function M=c6_bernstein_matrix(n,t)
M=zeros(numel(t),n+1);for i=1:numel(t),for k=0:n,M(i,k+1)=nchoosek(n,k)*t(i)^k*(1-t(i))^(n-k);end,end
end
function [x,w]=c6_gauss5()
x=[-0.906179845938664;-0.538469310105683;0;0.538469310105683;0.906179845938664];
w=[0.236926885056189;0.478628670499366;0.568888888888889;0.478628670499366;0.236926885056189];
end
function D=c6_fibonacci_sphere(N)
k=(0:N-1)';z=1-2*(k+0.5)/N;th=pi*(3-sqrt(5))*k;r=sqrt(max(0,1-z.^2));D=[r.*cos(th),r.*sin(th),z]';D=[D,[1;0;0],[-1;0;0],[0;1;0],[0;-1;0],[0;0;1],[0;0;-1]];D=D./vecnorm(D,2,1);
end
