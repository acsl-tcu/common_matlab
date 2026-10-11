function result = CURRENT_C6_THREE_METHOD_CLEAN_SENSOR_TRIGGER_V5B_BEZIER_EXACT_MIN7_FIX(methodSelect,stageSelect)
% CURRENT_C6_THREE_METHOD_CLEAN_SENSOR_TRIGGER_V5B_BEZIER_EXACT_MIN7_FIX
% =========================================================================
% CLEAN C6 BASELINE FOR THREE TRAJECTORY REPRESENTATIONS.
%
% Purpose of this file:
%   1) Make EACH representation work cleanly as a native C6 trajectory
%      generator before adding sensor/trigger/multi-obstacle/replanning.
%   2) Use the ACTUAL ORIENTED ELLIPSOID in trajectory-generation geometry.
%      No circumscribed sphere / max(radii) is used by the trajectory
%      generator or the final continuous certificate.
%   3) Keep each representation's own mathematics separate:
%        BSPLINE_D7      : degree-7 B-spline, simple interior knots -> C6.
%        POLY_D13_MIN7   : piecewise degree-13 power polynomial,
%                         min integral ||p^(7)||^2.
%        BEZIER_D13_MIN7 : piecewise degree-13 Bernstein/Bezier,
%                         exact min integral ||p^(7)||^2 using the
%                         Bernstein Gram matrix.
%
% Deliberately NOT included yet:
%   - automatic rejoin search
%   - safe-flight corridor
%   - v/a/j or HLC-input acceptance
% These are added only AFTER all three native C6 baselines pass A, B, and C.
%
% Stage A: representation-only C6 unit test, with one interior via point.
% Stage B: one rotated ellipsoid.
% Stage C: two rotated ellipsoids, both known from the start.
% Stage D: same clean two-obstacle problem preceded by one onboard sensor,
%          persistent obstacle memory, and a future-collision trigger.
%          Obstacles are outside the sensor sphere at t=0. Detection alone
%          does NOT trigger replanning. Automatic rejoin/SFC remain absent.
%
% Usage:
%   result = CURRENT_C6_THREE_METHOD_CLEAN_SENSOR_TRIGGER_V4();
%   result = CURRENT_C6_THREE_METHOD_CLEAN_SENSOR_TRIGGER_V4('BSPLINE_D7');
%   result = CURRENT_C6_THREE_METHOD_CLEAN_SENSOR_TRIGGER_V4('POLY_D13_MIN7','D');
% =========================================================================
if nargin<1 || isempty(methodSelect), methodSelect='ALL'; end
if nargin<2 || isempty(stageSelect), stageSelect='ALL'; end
methodSelect=upper(string(methodSelect));
stageSelect=upper(string(stageSelect));
allMethods=["BSPLINE_D7","POLY_D13_MIN7","BEZIER_D13_MIN7"];
if methodSelect=="ALL"
    methods=allMethods;
else
    if ~any(methodSelect==allMethods)
        error('C6CLEAN:BadMethod','Unknown method %s',methodSelect);
    end
    methods=methodSelect;
end
if ~any(stageSelect==["ALL","A","B","C","D"])
    error('C6CLEAN:BadStage','Use A, B, C, D, or ALL.');
end

clc; format compact;
P=c6c_params();
scenarios=c6c_scenarios();
if stageSelect~="ALL"
    scenarios=scenarios(arrayfun(@(s) string(s.stage)==stageSelect,scenarios));
end

fprintf('\n========================================================================================================================\n');
fprintf(' CLEAN C6 THREE-METHOD BASELINE -- native C6 + sensor/memory/trigger front-end\n');
fprintf(' methods: %s\n',strjoin(cellstr(methods),', '));
fprintf(' trajectory obstacle model: ORIENTED ELLIPSOID SUPPORT; circumscribed-sphere generation = OFF\n');
fprintf(' Stage A = native C6 | B = one ellipsoid | C = two known ellipsoids | D = sensor+memory+trigger+two ellipsoids\n');
    fprintf(' solver diagnostic: full svd(A) + rank/nullity/equality residual logging ON\n');
fprintf(' Bezier objective: EXACT integral ||p^(7)||^2 via Bernstein Gram matrix\n');
fprintf('========================================================================================================================\n');

out=cell(numel(methods),numel(scenarios));
for im=1:numel(methods)
    fprintf('\n########################################################################################################################\n');
    fprintf(' METHOD: %s\n',methods(im));
    fprintf('########################################################################################################################\n');
    for is=1:numel(scenarios)
        out{im,is}=c6c_run(methods(im),scenarios(is),P);
    end
end
result=struct('methods',methods,'scenarios',{scenarios},'runs',{out},'params',P);
c6c_summary(result,P);
end

function P=c6c_params()
P=struct();
P.p_bs=7; P.n_poly=13; P.n_bez=13;
% Native construction is mathematically C6. These tolerances only decide
% whether the floating-point verification is numerically acceptable.
P.c6_abs_tol=2e-6;
P.c6_rel_tol=2e-6;
P.endpoint_abs_tol=2e-6;
P.endpoint_rel_tol=2e-6;
P.d_margin=0.15;
P.R_payload=0.24;
P.R_uav=0.30;
P.R_cable=0.045;
P.L_cable=1.00;
P.g=9.81; P.e3=[0;0;1];
% Final certificate direction set. The obstacle support itself is exact for
% every tested direction: h_E(n)=n'c+sqrt(n'Sn).
P.sep_dirs=c6c_add_cardinal(c6c_fibonacci_sphere(722));
% Stage-B ellipsoid-aware route candidates. `extra` is clearance outside
% the exact ellipsoid support plane, NOT outside a bounding sphere.
P.detour_extra=[0.25 0.50 0.85 1.25 1.75];
P.detour_dirs=12;
P.long_extra=0.35;
% Stage-C direction combinations: first same-side bypasses, then nearby
% direction pairs, then the full set. This is search ordering only; every
% candidate is still certified against ALL oriented ellipsoids.
P.multi_neighbor_span=2;
% Stage-D front-end only. Sensor may use a conservative circumscribed sphere
% for cheap detection; trajectory generation/certificate never do.
P.sensor_R=15.0;
P.dt_sensor=0.05;
P.lookahead=5.0;
P.pred_dt=0.10;
% Console logging: keep all candidate data in result.rows, but do not print every failed candidate.
P.compact_log=true;
P.compact_topk=5;
end

function S=c6c_scenarios()
A=struct();
A.name='A_NATIVE_C6'; A.stage='A';
A.p0=[0;0;0]; A.pg=[8;0;0]; A.v0=[0.8;0;0]; A.vg=[0.8;0;0];
A.T=10; A.obs=[]; A.fixed_vias=[4;1.0;0.5];

B=struct();
B.name='B_ONE_ROTATED_ELLIPSOID'; B.stage='B';
B.p0=[0;0;0]; B.pg=[10;0;0]; B.v0=[0.8;0;0]; B.vg=[0.8;0;0];
B.T=13;
B.obs=c6c_make_obs([5;0;0],[1.10;0.85;1.15],c6c_Rz(deg2rad(25))*c6c_Ry(deg2rad(10)),1);
B.fixed_vias=[];

C=struct();
C.name='C_TWO_ROTATED_ELLIPSOIDS'; C.stage='C';
C.p0=[0;0;0]; C.pg=[12;0;0]; C.v0=[0.8;0;0]; C.vg=[0.8;0;0];
C.T=16;
C.obs(1)=c6c_make_obs([4.25; 0.20; 0.05],[1.00;0.72;1.05],c6c_Rz(deg2rad(22))*c6c_Ry(deg2rad(8)),1);
C.obs(2)=c6c_make_obs([7.55;-0.25; 0.18],[0.92;0.78;1.12],c6c_Rz(deg2rad(-31))*c6c_Ry(deg2rad(-12)),2);
C.fixed_vias=[];

D=struct();
D.name='D_SENSOR_TRIGGER_TWO_ELLIPSOIDS'; D.stage='D';
% Nominal payload starts at x=0 and moves at 1 m/s. Both obstacle surfaces
% are outside the 15-m sensor sphere at t=0. Avoidance starts only when the
% future-collision trigger fires; the clean C6 backend then receives only
% the persistently detected obstacles.
D.p0=[0;0;0]; D.pg=[30;0;0]; D.v0=[1.0;0;0]; D.vg=[1.0;0;0];
D.T=20;
D.obs(1)=c6c_make_obs([20.25; 0.20; 0.05],[1.00;0.72;1.05],c6c_Rz(deg2rad(22))*c6c_Ry(deg2rad(8)),1);
D.obs(2)=c6c_make_obs([24.00;-0.25; 0.18],[0.92;0.78;1.12],c6c_Rz(deg2rad(-31))*c6c_Ry(deg2rad(-12)),2);
D.fixed_vias=[];
S=[A B C D];
end

function R=c6c_run(method,S,P)
fprintf('\n------------------------------------------------------------------------------------------------------------------------\n');
fprintf(' %s / %s\n',S.name,method);
fprintf('------------------------------------------------------------------------------------------------------------------------\n');
front=[]; Splan=S;
if string(S.stage)=="D"
    front=c6c_sensor_trigger_frontend(S,P);
    c6c_print_frontend(front,S,P);
    if ~front.triggered
        R=struct('method',char(method),'scenario',S.name,'rows',repmat(c6c_row(),1,0),'best',[], 'front',front);
        c6c_print_requirements(R,S,P);
        return
    end
    Splan.p0=front.payload_at_trigger;
    Splan.v0=S.v0;
    Splan.obs=S.obs(front.known_ids_at_trigger);
end
D0=c6c_nominal_jet(Splan.p0,Splan.v0);
D1=c6c_nominal_jet(Splan.pg,Splan.vg);

if string(S.stage)=="A"
    routes={S.fixed_vias};
    tags={'UNIT_VIA'};
    routeMeta={struct('generator','representation-unit','outer_sphere_used',false,...
        'min_support_clearance',NaN)};
else
    [routes,tags,routeMeta]=c6c_ellipsoid_routes(method,Splan.p0,Splan.pg,Splan.obs,P);
    for io=1:numel(Splan.obs)
        ob=Splan.obs(io);
        fprintf('[ELLIPSOID %d] center=[%.3f %.3f %.3f] radii=[%.3f %.3f %.3f]\n',...
            ob.id,ob.center,ob.radii);
        fprintf('[ELLIPSOID %d] R=\n',ob.id); disp(ob.R);
    end
    fprintf('[GENERATOR] oriented support h_E(n)=n''c+sqrt(n''Sn), outerSphereUsed=0, obstacles=%d, candidates=%d\n',...
        numel(Splan.obs),numel(routes));
end

rows=repmat(c6c_row(),1,numel(routes));
best=[];
for ir=1:numel(routes)
    rr=c6c_row(); rr.tag=tags{ir}; rr.nvias=size(routes{ir},2);
    try
        tc=tic;
        seg=c6c_build(method,Splan.T,D0,D1,routes{ir},P);
        rr.build_ms=1e3*toc(tc); rr.build=true;
        [rr.join_abs_max,rr.join_abs,rr.join_rel]=c6c_c6_error(seg);
        [rr.end_abs_max,rr.end_abs,rr.end_rel]=c6c_endpoint_error(seg);
        rr.c6pass=seg.structural_c6 && max(rr.join_abs)<=P.c6_abs_tol && max(rr.join_rel)<=P.c6_rel_tol && ...
            max(rr.end_abs)<=P.endpoint_abs_tol && max(rr.end_rel)<=P.endpoint_rel_tol;
        tc=tic; rr.cert=c6c_certify(seg,Splan.obs,P); rr.cert_ms=1e3*toc(tc);
        rr.geom=rr.cert.pass;
        rr.safe=rr.build && rr.c6pass && rr.geom;
        rr.condition_metric=seg.condition_metric;
        rr.route_meta=routeMeta{ir};
        rr.reason=c6c_reason(rr);
        if isempty(best) || c6c_better(rr,best), best=rr; end
        % In compact mode, failed candidates stay in result.rows but are not
        % printed one-by-one. A safe selected candidate is printed immediately.
        if ~P.compact_log || rr.safe
            c6c_print_candidate(rr,method,Splan,seg,rr.safe);
        end
        if rr.safe
            % First safe route is sufficient for this foundation stage.
            rows(ir)=rr; rows=rows(1:ir); best=rr; break;
        end
    catch ME
        rr.reason=['BUILD_FAIL:' ME.identifier];
        if ~P.compact_log
            if ~isempty(ME.stack)
                st=ME.stack(1);
                fprintf('[%s] route=%-20s BUILD_FAIL %s : %s | at %s:%d\n',...
                    method,rr.tag,ME.identifier,ME.message,st.name,st.line);
            else
                fprintf('[%s] route=%-20s BUILD_FAIL %s : %s\n',method,rr.tag,ME.identifier,ME.message);
            end
        end
    end
    rows(ir)=rr;
end
if P.compact_log
    c6c_print_search_summary(rows,best,method,Splan,P);
end
R=struct('method',char(method),'scenario',S.name,'rows',rows,'best',best,'front',front);
c6c_print_requirements(R,S,P);
end

function c6c_print_candidate(rr,method,Splan,seg,isSelected)
mark=''; if isSelected, mark=' <SELECTED>'; end
fprintf('[%s] route=%-20s vias=%2d build=1 C6=%d geom=%d minG=%+9.3e [L/U/C]=[%+9.3e %+9.3e %+9.3e] cond=%.2e ms=%7.2f %s%s\n',...
    method,rr.tag,rr.nvias,rr.c6pass,rr.geom,rr.cert.minG,rr.cert.minG_payload,rr.cert.minG_uav,rr.cert.minG_cable,...
    rr.condition_metric,rr.build_ms+rr.cert_ms,rr.reason,mark);
sd=seg.solver_diag;
fprintf('  [EQ-DIAG] A=%dx%d rank=%d null=%d eqRes=%.3e rel=%.3e svdTol=%.3e Qnorm=%.3e\n',...
    sd.m,sd.n,sd.rank,sd.nullity,sd.eq_res_norm,sd.eq_res_rel,sd.svd_tol,sd.q_norm);
if numel(Splan.obs)>1 && ~isempty(rr.cert)
    for io=1:numel(Splan.obs)
        fprintf('  [OBS-DIAG] id=%d minG=%+9.3e [L/U/C]=[%+9.3e %+9.3e %+9.3e] guide=%+9.3e\n',...
            Splan.obs(io).id,rr.cert.minG_per_obstacle(io),rr.cert.minG_payload_per_obstacle(io),...
            rr.cert.minG_uav_per_obstacle(io),rr.cert.minG_cable_per_obstacle(io),...
            rr.route_meta.per_obstacle_support_clearance(io));
    end
end
end

function c6c_print_search_summary(rows,best,method,Splan,P)
if isempty(rows), return; end
nb=numel(rows); nbuild=sum([rows.build]); nc6=sum([rows.c6pass]); ngeom=sum([rows.geom]); nsafe=sum([rows.safe]);
fprintf('[SEARCH-SUMMARY] %s tested=%d build=%d C6=%d geom=%d safe=%d\n',method,nb,nbuild,nc6,ngeom,nsafe);
if nsafe>0
    fprintf('[SEARCH-SUMMARY] first certified route selected above; failed candidates suppressed=%d\n',max(0,nb-1));
    return
end
valid=find([rows.build] & [rows.c6pass] & arrayfun(@(r) ~isempty(r.cert),rows));
if isempty(valid)
    fprintf('[SEARCH-SUMMARY] no built+C6 candidate available for geometry ranking.\n');
    ib=find(~[rows.build],1,'first');
    if ~isempty(ib)
        rr=rows(ib);
        fprintf('[SEARCH-SUMMARY] representative failure: route=%s reason=%s\n',rr.tag,rr.reason);
    end
    return
end
g=arrayfun(@(i) rows(i).cert.minG,valid);
[~,ord]=sort(g,'descend'); ord=ord(1:min(P.compact_topk,numel(ord)));
fprintf('[SEARCH-SUMMARY] no certified route. Top %d candidates by continuous minG:\n',numel(ord));
for k=1:numel(ord)
    rr=rows(valid(ord(k)));
    fprintf('  #%d route=%-20s minG=%+9.3e [L/U/C]=[%+9.3e %+9.3e %+9.3e] cond=%.2e\n',...
        k,rr.tag,rr.cert.minG,rr.cert.minG_payload,rr.cert.minG_uav,rr.cert.minG_cable,rr.condition_metric);
    if numel(Splan.obs)>1
        fprintf('     perObs minG=['); fprintf('%+.3e ',rr.cert.minG_per_obstacle); fprintf(']\n');
    end
end
if ~isempty(best)
    fprintf('[SEARCH-SUMMARY] BEST route=%s minG=%+.3e reason=%s\n',best.tag,best.cert.minG,best.reason);
end
end

function tf=c6c_better(a,b)
if isempty(b), tf=true; return; end
if a.safe~=b.safe, tf=a.safe; return; end
if a.geom~=b.geom, tf=a.geom; return; end
if a.c6pass~=b.c6pass, tf=a.c6pass; return; end
ag=-Inf; bg=-Inf;
if ~isempty(a.cert),ag=a.cert.minG;end
if ~isempty(b.cert),bg=b.cert.minG;end
tf=ag>bg;
end

function s=c6c_reason(rr)
if ~rr.build, s='BUILD_FAIL';
elseif ~rr.c6pass, s='C6_NUMERIC_FAIL';
elseif ~rr.geom, s='GEOMETRY_FAIL';
else, s='PASS_CORE'; end
end

% ======================= SENSOR / MEMORY / TRIGGER =======================
function F=c6c_sensor_trigger_frontend(S,P)
% Sensor is centered at the nominal UAV CG. For the cheap detection gate
% only, obstacle j is conservatively represented by its circumscribed
% sphere max(radii). This approximation NEVER enters trajectory generation.
% Trigger prediction uses the actual oriented ellipsoid support function.
v=S.v0; speed=norm(v); if speed<=0,error('C6CLEAN:ZeroNominalSpeed','Stage D requires nonzero nominal speed.');end
Tnom=norm(S.pg-S.p0)/speed;
no=numel(S.obs); detected=false(1,no); detect_t=nan(1,no);
initial_out=true(1,no);
pU0=S.p0+P.L_cable*P.e3;
for o=1:no
    rs=max(S.obs(o).radii); % SENSOR ONLY, explicitly allowed simplification
    initial_out(o)=norm(S.obs(o).center-pU0) > P.sensor_R+rs;
end
triggered=false; trig_t=NaN; ttc=NaN; trig_obs=0; trig_body=''; known_at=[]; payload_trig=[NaN;NaN;NaN];
first_detect=NaN; memory_pass=true; prev_known=false(1,no);
for t=0:P.dt_sensor:Tnom
    pL=S.p0+v*t; pU=pL+P.L_cable*P.e3;
    for o=1:no
        if ~detected(o)
            rs=max(S.obs(o).radii); % SENSOR ONLY
            if norm(S.obs(o).center-pU)<=P.sensor_R+rs
                detected(o)=true; detect_t(o)=t;
                if isnan(first_detect),first_detect=t;end
            end
        end
    end
    if any(prev_known & ~detected), memory_pass=false; end
    prev_known=detected;
    ids=find(detected);
    if ~isempty(ids)
        [hit,tau,oid,body]=c6c_predict_nominal_collision(pL,v,S.obs(ids),ids,P);
        if hit
            triggered=true; trig_t=t; ttc=tau; trig_obs=oid; trig_body=body; known_at=ids; payload_trig=pL; break
        end
    end
end
F=struct('initial_outside_each',initial_out,'initial_outside_all',all(initial_out),...
    'detected_time',detect_t,'first_detect_time',first_detect,'memory_pass',memory_pass,...
    'triggered',triggered,'trigger_time',trig_t,'ttc',ttc,'trigger_obstacle_id',trig_obs,...
    'trigger_body',trig_body,'known_ids_at_trigger',known_at,'payload_at_trigger',payload_trig,...
    'trigger_after_detection',triggered && ~isnan(first_detect) && trig_t>first_detect,...
    'sensor_model','UAV-CG sphere; circumscribed obstacle sphere for detection only',...
    'trigger_model','sampled nominal prediction with oriented-ellipsoid support margins');
end

function [hit,tau,globalObsId,bodyName]=c6c_predict_nominal_collision(pNow,v,knownObs,globalIds,P)
hit=false; tau=Inf; globalObsId=0; bodyName='';
for dt=0:P.pred_dt:P.lookahead
    pL=pNow+v*dt; pU=pL+P.L_cable*P.e3;
    for io=1:numel(knownObs)
        [g,b]=c6c_point_system_margin(pL,pU,knownObs(io),P);
        if g<=0
            hit=true; tau=dt; globalObsId=globalIds(io); bodyName=b; return
        end
    end
end
end

function [g,bodyName,parts]=c6c_point_system_margin(pL,pU,ob,P)
N=P.sep_dirs; K=size(N,2); H=zeros(K,1);
for k=1:K,H(k)=c6c_obstacle_support(ob,N(:,k));end
nPL=N'*pL; nPU=N'*pU;
gL=max(nPL-H-P.R_payload-P.d_margin);
gU=max(nPU-H-P.R_uav-P.d_margin);
gC=max(min(nPL,nPU)-H-P.R_cable-P.d_margin);
parts=[gL gU gC]; [g,idx]=min(parts);
names={'Payload','UAV','Cable'}; bodyName=names{idx};
end

function c6c_print_frontend(F,S,P)
fprintf('[SENSOR] R=%.2f m center=UAV-CG initialOutsideAll=%d perObs=%s detectionApprox=circumscribed-sphere(SENSOR ONLY)\n',...
    P.sensor_R,F.initial_outside_all,mat2str(F.initial_outside_each));
fprintf('[MEMORY] detectedAt=%s persistent=%d\n',mat2str(F.detected_time,4),F.memory_pass);
if F.triggered
    fprintf('[TRIGGER] detected-alone=NO lookahead=%.2f s triggerAt=%.2f TTC=%.2f obsID=%d body=%s knownIDs=%s\n',...
        P.lookahead,F.trigger_time,F.ttc,F.trigger_obstacle_id,F.trigger_body,mat2str(F.known_ids_at_trigger));
    fprintf('[TRIGGER] payloadAt=[%.3f %.3f %.3f] ; trajectory generation below uses ORIENTED ELLIPSOIDS only\n',F.payload_at_trigger);
else
    fprintf('[TRIGGER] NONE within nominal interval; no avoidance trajectory is generated.\n');
end
for o=1:numel(S.obs)
    rs=max(S.obs(o).radii); d0=norm(S.obs(o).center-(S.p0+P.L_cable*P.e3))-rs;
    fprintf('[SENSOR-DIAG] obsID=%d initial approx surface distance=%.3f m > R? %d\n',S.obs(o).id,d0,d0>P.sensor_R);
end
end

% ========================= ORIENTED-ELLIPSOID ROUTES ====================
function [routes,tags,meta]=c6c_ellipsoid_routes(method,p0,pg,obs,P)
% Ellipsoid-exact guide generation for one OR multiple known obstacles.
% No circumscribed sphere / max(radii) is used.
%
% For obstacle j and unit bypass normal d_j:
%   rho_j(d_j)=sqrt(d_j' S_j d_j)
% and each guide triplet is placed outside the corresponding exact support
% plane by system support + d_margin + extra. For multiple obstacles the
% triplets are ordered along start->goal. Different obstacles may use
% different bypass directions; this avoids forcing one artificial common
% side around all obstacles.
e=pg-p0; e=e/norm(e);
ref=[0;0;1]; if abs(e'*ref)>0.9,ref=[0;1;0];end
b1=cross(e,ref); b1=b1/norm(b1);
b2=cross(e,b1); b2=b2/norm(b2);
ang=linspace(0,2*pi,P.detour_dirs+1); ang(end)=[];
dirs=zeros(3,numel(ang));
for k=1:numel(ang), dirs(:,k)=cos(ang(k))*b1+sin(ang(k))*b2; end

% Search-order direction tuples. With two obstacles, try same direction
% first, then nearby directions, then every remaining pair. This changes
% only candidate ordering, never safety acceptance.
no=numel(obs);
if no==1
    tuples=(1:size(dirs,2))';
elseif no==2
    nd=size(dirs,2); pairs=[];
    for i=1:nd, pairs=[pairs;i i]; end %#ok<AGROW>
    for span=1:P.multi_neighbor_span
        for i=1:nd
            pairs=[pairs;i 1+mod(i-1+span,nd); i 1+mod(i-1-span,nd)]; %#ok<AGROW>
        end
    end
    for i=1:nd, for j=1:nd, pairs=[pairs;i j]; end, end %#ok<AGROW>
    tuples=unique(pairs,'rows','stable');
else
    error('C6CLEAN:MultiObsCount','This clean stage currently supports one or two obstacles.');
end

cproj=arrayfun(@(o) dot(o.center-p0,e),obs);
[~,ord]=sort(cproj);
routes={}; tags={}; meta={};
for ie=1:numel(P.detour_extra)
    extra=P.detour_extra(ie);
    for it=1:size(tuples,1)
        Vraw=cell(1,no); perObsClear=inf(1,no);
        for jo=1:no
            io=ord(jo); ob=obs(io);
            d=dirs(:,tuples(it,io)); d=d/norm(d);
            rhoD=sqrt(max(0,d'*ob.S*d));
            rhoE=sqrt(max(0,e'*ob.S*e));
            sysD=c6c_vertical_system_support(d,P);
            lateral=rhoD+sysD+P.d_margin+extra;
            longitudinal=rhoE+P.long_extra;
            q1=ob.center-e*longitudinal+d*lateral;
            q2=ob.center+d*lateral;
            q3=ob.center+e*longitudinal+d*lateral;
            Vraw{io}=[q1 q2 q3];
            hE=d'*ob.center+rhoD;
            perObsClear(io)=min(d'*[q1 q2 q3]-hE-sysD-P.d_margin);
        end
        V=[];
        for jo=1:no, V=[V Vraw{ord(jo)}]; end %#ok<AGROW>
        routes{end+1}=V; %#ok<AGROW>
        if no==1
            tags{end+1}=sprintf('ell_d%02d_x%.2f',tuples(it,1),extra); %#ok<AGROW>
        else
            tags{end+1}=sprintf('ell_d%02d-d%02d_x%.2f',tuples(it,1),tuples(it,2),extra); %#ok<AGROW>
        end
        meta{end+1}=struct('generator','oriented-ellipsoid-support','outer_sphere_used',false,...
            'direction_indices',tuples(it,:),'extra',extra,...
            'min_support_clearance',min(perObsClear),'per_obstacle_support_clearance',perObsClear,...
            'n_obstacles',no,'method',char(method)); %#ok<AGROW>
    end
end
end

function delta=c6c_vertical_system_support(n,P)
% Support measured from payload-center reference for a vertical cable.
% Separate body models remain separate in the final certificate; max() here
% is used only to place a guide point that is sufficient for all three.
nzv=n(3);
dL=P.R_payload;
dU=P.R_uav-P.L_cable*nzv;
dC=P.R_cable+max(0,-P.L_cable*nzv);
delta=max([dL,dU,dC]);
end

% ============================== BUILD ROUTER =============================
function seg=c6c_build(method,T,D0,D1,vias,P)
switch method
    case "BSPLINE_D7", seg=c6c_build_bspline(T,D0,D1,vias,P);
    case "POLY_D13_MIN7", seg=c6c_build_poly(T,D0,D1,vias,P);
    case "BEZIER_D13_MIN7", seg=c6c_build_bezier(T,D0,D1,vias,P);
    otherwise, error('C6CLEAN:Method','Bad method');
end
end

% ================================ B-SPLINE ===============================
function seg=c6c_build_bspline(T,D0,D1,vias,P)
p=P.p_bs; nVia=size(vias,2);
% Enough free control points to impose endpoint jets and interpolation
% without changing the native degree-7/simple-knot C6 construction.
N=max(22,2*nVia+18);
U=c6c_clamped_uniform_knots(N,p);
frac=c6c_route_fractions(D0(:,1),vias,D1(:,1));
A=[]; B=zeros(0,3);
for r=0:6
    A=[A;c6c_bs_basis_derivative_all(0,U,p,N,r)/T^r]; B=[B;D0(:,r+1)']; %#ok<AGROW>
end
for r=0:6
    A=[A;c6c_bs_basis_derivative_all(1,U,p,N,r)/T^r]; B=[B;D1(:,r+1)']; %#ok<AGROW>
end
for k=1:nVia
    A=[A;c6c_bs_basis_derivative_all(frac(k),U,p,N,0)]; B=[B;vias(:,k)']; %#ok<AGROW>
end
spans=c6c_bs_spans(U,p); Q=zeros(N); [xg,wg]=c6c_gauss5();
for s=1:size(spans,1)
    ua=spans(s,1); ub=spans(s,2);
    for q=1:numel(xg)
        u=(ua+ub)/2+(ub-ua)/2*xg(q);
        b7=c6c_bs_basis_derivative_all(u,U,p,N,7)/T^7;
        Q=Q+(ub-ua)/2*wg(q)*(b7'*b7)*T;
    end
end
[X,condm,solverDiag]=c6c_eq_qp(Q,A,B);
% Convert each native B-spline span to degree-7 Bernstein only for common
% validation/certification. Native C6 comes from simple knots, not from this
% conversion.
Bez=cell(size(spans,1),1); dt=zeros(1,size(spans,1));
for s=1:size(spans,1)
    ua=spans(s,1); ub=spans(s,2); tau=(0:p)'/p;
    MB=c6c_bernstein_matrix(p,tau); V=zeros(p+1,3);
    for k=1:p+1
        u=ua+(ub-ua)*tau(k);
        V(k,:)=c6c_bs_basis_derivative_all(u,U,p,N,0)*X;
    end
    Bez{s}=(MB\V)'; dt(s)=T*(ub-ua);
end
seg=struct('name','BSPLINE_D7','B',{Bez},'dt',dt,'D0',D0,'D1',D1,...
    'structural_c6',true,'condition_metric',condm,'solver_diag',solverDiag,'native',struct('U',U,'X',X,'degree',p));
end

function U=c6c_clamped_uniform_knots(N,p)
ni=N-p-1; if ni>0, internal=(1:ni)/(ni+1); else, internal=[]; end
U=[zeros(1,p+1),internal,ones(1,p+1)];
end
function spans=c6c_bs_spans(U,p)
spans=[];
for k=p+1:numel(U)-p-1
    if U(k+1)>U(k), spans=[spans;U(k) U(k+1)]; end %#ok<AGROW>
end
end
function dN=c6c_bs_basis_derivative_all(u,U,p,N,r)
dN=zeros(1,N); for i=1:N,dN(i)=c6c_bs_basis_derivative_single(i,p,r,u,U);end
end
function v=c6c_bs_basis_derivative_single(i,p,r,u,U)
if r>p,v=0;return,end
if r==0,v=c6c_bs_basis(i,p,u,U);return,end
if p==0,v=0;return,end
den1=U(i+p)-U(i); den2=U(i+p+1)-U(i+1); a=0;b=0;
if den1~=0,a=p/den1*c6c_bs_basis_derivative_single(i,p-1,r-1,u,U);end
if den2~=0,b=p/den2*c6c_bs_basis_derivative_single(i+1,p-1,r-1,u,U);end
v=a-b;
end
function v=c6c_bs_basis(i,p,u,U)
if i<1 || i+p+1>numel(U),v=0;return,end
if p==0
    v=double((u>=U(i) && u<U(i+1)) || (abs(u-1)<1e-14 && U(i+1)==1 && U(i)<1)); return
end
a=0;b=0; d1=U(i+p)-U(i); d2=U(i+p+1)-U(i+1);
if d1~=0,a=(u-U(i))/d1*c6c_bs_basis(i,p-1,u,U);end
if d2~=0,b=(U(i+p+1)-u)/d2*c6c_bs_basis(i+1,p-1,u,U);end
v=a+b;
end

% ============================ POWER POLYNOMIAL ===========================
function seg=c6c_build_poly(T,D0,D1,vias,P)
n=P.n_poly; M=size(vias,2)+1;
frac=c6c_route_fractions(D0(:,1),vias,D1(:,1)); br=[0 frac 1]; dt=T*diff(br);
[A,B]=c6c_poly_eq(n,dt,D0,D1,vias); Q=c6c_poly_Q(n,dt,7);
[X,condm,solverDiag]=c6c_eq_qp(Q,A,B);
Bez=cell(M,1);
for s=1:M
    ix=(s-1)*(n+1)+(1:n+1); C=X(ix,:); BB=zeros(3,n+1);
    for ax=1:3, BB(ax,:)=c6c_power_to_bernstein(C(:,ax)',n); end
    Bez{s}=BB;
end
seg=struct('name','POLY_D13_MIN7','B',{Bez},'dt',dt,'D0',D0,'D1',D1,...
    'structural_c6',true,'condition_metric',condm,'solver_diag',solverDiag,'native',struct('X',X,'degree',n));
end
function [A,B]=c6c_poly_eq(n,dt,D0,D1,vias)
M=numel(dt); A=[];B=[];
for r=0:6,A=[A;c6c_poly_row(n,M,1,0,r,dt(1))];B=[B;D0(:,r+1)'];end %#ok<AGROW>
for r=0:6,A=[A;c6c_poly_row(n,M,M,1,r,dt(end))];B=[B;D1(:,r+1)'];end %#ok<AGROW>
for s=1:M-1
    q=vias(:,s)';
    A=[A;c6c_poly_row(n,M,s,1,0,dt(s))];B=[B;q]; %#ok<AGROW>
    for r=1:6
        A=[A;c6c_poly_row(n,M,s,1,r,dt(s))-c6c_poly_row(n,M,s+1,0,r,dt(s+1))];B=[B;0 0 0]; %#ok<AGROW>
    end
    A=[A;c6c_poly_row(n,M,s+1,0,0,dt(s+1))];B=[B;q]; %#ok<AGROW>
end
end
function row=c6c_poly_row(n,M,s,tau,r,dt)
row=zeros(1,(n+1)*M); q=zeros(1,n+1);
for k=r:n,q(k+1)=factorial(k)/factorial(k-r)*tau^(k-r)/dt^r;end
ix=(s-1)*(n+1)+(1:n+1);row(ix)=q;
end
function Q=c6c_poly_Q(n,dt,r)
M=numel(dt);Q=zeros((n+1)*M);
for s=1:M
 q=zeros(n+1);
 for i=r:n,for j=r:n
  ci=factorial(i)/factorial(i-r);cj=factorial(j)/factorial(j-r);
  q(i+1,j+1)=ci*cj/(i+j-2*r+1)/dt(s)^(2*r-1);
 end,end
 ix=(s-1)*(n+1)+(1:n+1);Q(ix,ix)=q;
end
end
function b=c6c_power_to_bernstein(a,n)
b=zeros(1,n+1);
for k=0:n,for j=0:k,b(k+1)=b(k+1)+a(j+1)*nchoosek(k,j)/nchoosek(n,j);end,end
end

% ================================= BEZIER ================================
function seg=c6c_build_bezier(T,D0,D1,vias,P)
n=P.n_bez;M=size(vias,2)+1;
frac=c6c_route_fractions(D0(:,1),vias,D1(:,1));br=[0 frac 1];dt=T*diff(br);
[A,B]=c6c_bezier_eq(n,dt,D0,D1,vias);Q=c6c_bezier_Q(n,dt,7);
[X,condm,solverDiag]=c6c_eq_qp(Q,A,B);
Bez=cell(M,1);
for s=1:M,ix=(s-1)*(n+1)+(1:n+1);Bez{s}=X(ix,:)';end
seg=struct('name','BEZIER_D13_MIN7','B',{Bez},'dt',dt,'D0',D0,'D1',D1,...
    'structural_c6',true,'condition_metric',condm,'solver_diag',solverDiag,'native',struct('X',X,'degree',n));
end
function [A,B]=c6c_bezier_eq(n,dt,D0,D1,vias)
M=numel(dt);A=[];B=[];
for r=0:6,A=[A;c6c_bezier_row(n,M,1,0,r,dt(1))];B=[B;D0(:,r+1)'];end %#ok<AGROW>
for r=0:6,A=[A;c6c_bezier_row(n,M,M,1,r,dt(end))];B=[B;D1(:,r+1)'];end %#ok<AGROW>
for s=1:M-1
 q=vias(:,s)';
 A=[A;c6c_bezier_row(n,M,s,1,0,dt(s))];B=[B;q]; %#ok<AGROW>
 for r=1:6
  A=[A;c6c_bezier_row(n,M,s,1,r,dt(s))-c6c_bezier_row(n,M,s+1,0,r,dt(s+1))];B=[B;0 0 0]; %#ok<AGROW>
 end
 A=[A;c6c_bezier_row(n,M,s+1,0,0,dt(s+1))];B=[B;q]; %#ok<AGROW>
end
end
function row=c6c_bezier_row(n,M,s,atEnd,r,dt)
row=zeros(1,(n+1)*M);q=zeros(1,n+1);
scale=factorial(n)/factorial(n-r)/dt^r;
if ~atEnd
    for k=0:r,q(k+1)=scale*(-1)^(r-k)*nchoosek(r,k);end
else
    for k=0:r,q(n-r+k+1)=scale*(-1)^(r-k)*nchoosek(r,k);end
end
ix=(s-1)*(n+1)+(1:n+1);row(ix)=q;
end
function Q=c6c_bezier_Q(n,dt,r)
% Exact Hessian for integral ||p^(r)(t)||^2 dt in Bernstein basis.
% For m=n-r,
%   p^(r)(u) = n!/(n-r)! * sum_i Delta^r P_i B_i^m(u) / dt^r,
% and the integral requires the Bernstein Gram matrix
%   G_ij = integral_0^1 B_i^m(u) B_j^m(u) du.
% Using D''*D alone minimizes forward-difference energy, not the requested
% continuous-time derivative integral; that distinction becomes important
% in Stage D when the nonzero trigger-state jet strongly shapes the curve.
M=numel(dt); nv=(n+1)*M; Q=zeros(nv);
D=eye(n+1);
for k=1:r, D=diff(D,1,1); end
m=n-r; G=zeros(m+1);
for i=0:m
    for j=0:m
        G(i+1,j+1)=nchoosek(m,i)*nchoosek(m,j)/((2*m+1)*nchoosek(2*m,i+j));
    end
end
for s=1:M
    scale=(factorial(n)/factorial(n-r)/dt(s)^r)^2*dt(s);
    ix=(s-1)*(n+1)+(1:n+1);
    Q(ix,ix)=scale*(D'*G*D);
end
end

% ========================== EQUALITY-QP SOLVER ===========================
function [X,condm,diagOut]=c6c_eq_qp(Q,A,B)
% Row-normalize then solve equality-constrained quadratic in the nullspace.
% MATLAB default svd(A) returns the full U/S/V matrices needed here.
% Do NOT pass the invalid option 'full'. This keeps the explicit nullspace
% construction independent of null(A,tol) syntax.
rowScale=max(vecnorm(A,2,2),1e-14); An=A./rowScale; Bn=B./rowScale;
[U,S,V]=svd(An); %#ok<ASGLU>
s=diag(S); if isempty(s), smax=0; else, smax=max(s); end
tol=1e-11*max(size(An))*max(1,smax);
r=sum(s>tol);
if r==0
    X0=zeros(size(An,2),size(Bn,2));
else
    X0=V(:,1:r)*(diag(1./s(1:r))*(U(:,1:r)'*Bn));
end
res=norm(An*X0-Bn,'fro');
if res>1e-8*(1+norm(Bn,'fro'))
    error('C6CLEAN:EqualityInfeasible','Normalized equality residual %.3e',res);
end
Z=V(:,r+1:end);
if isempty(Z)
    X=X0;
else
    H=Z'*Q*Z; G=Z'*Q*X0;
    reg=1e-12*max(1,norm(H,'fro'));
    X=X0-Z*((H+reg*eye(size(H)))\G);
end
sn=s(s>tol); if isempty(sn),condm=1;else,condm=max(sn)/min(sn);end
diagOut=struct('m',size(An,1),'n',size(An,2),'rank',r,'nullity',size(An,2)-r,...
    'svd_tol',tol,'eq_res_norm',res,'eq_res_rel',res/max(1,norm(Bn,'fro')),...
    'cond_rank',condm,'q_norm',norm(Q,'fro'));
end

% ======================= CONTINUOUS SAFETY CERTIFICATE ==================
function cert=c6c_certify(seg,obs,P)
% Obstacle geometry is an oriented ellipsoid throughout:
%   E={x:(x-c)'S^{-1}(x-c)<=1}
%   h_E(n)=n'c+sqrt(n'Sn)
% NO max(radii) / bounding sphere is used here.
N=P.sep_dirs;K=size(N,2); no=numel(obs);
minG=inf;minL=inf;minU=inf;minC=inf;vmax=0;amax=0;jmax=0;tLB=inf;pass=true;
perL=inf(1,no); perU=inf(1,no); perC=inf(1,no); perG=inf(1,no);
if no==0
    H=zeros(K,0);
else
    H=zeros(K,no);
    for o=1:no,for k=1:K,H(k,o)=c6c_obstacle_support(obs(o),N(:,k));end,end
end
for is=1:numel(seg.B)
    B=seg.B{is};dt=seg.dt(is);
    V=c6c_bern_derivative(B,dt,1);A=c6c_bern_derivative(B,dt,2);J=c6c_bern_derivative(B,dt,3);
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
        perL=min(perL,bL); perU=min(perU,bU); perC=min(perC,bC);
        perG=min(perG,min([bL(:)';bU(:)';bC(:)'],[],1));
        minG=min(minG,min([bL bU bC]));
        if any(bL<0)||any(bU<0)||any(bC<0),pass=false;end
    end
end
if no==0,minG=inf;minL=inf;minU=inf;minC=inf; perL=[]; perU=[]; perC=[]; perG=[]; end
cert=struct('pass',pass,'minG',minG,'minG_payload',minL,'minG_uav',minU,'minG_cable',minC,...
    'minG_per_obstacle',perG,'minG_payload_per_obstacle',perL,'minG_uav_per_obstacle',perU,...
    'minG_cable_per_obstacle',perC,'v',vmax,'a',amax,'j',jmax,'tension_lb',tLB,...
    'n_obstacles',no,'obstacle_model','ORIENTED_ELLIPSOID_SUPPORT');
end
function D=c6c_bern_derivative(B,T,r)
D=B;n=size(B,2)-1;for q=1:r,D=(n-q+1)/T*diff(D,1,2);end
end
function h=c6c_obstacle_support(ob,n)
h=n'*ob.center+sqrt(max(0,n'*ob.S*n));
end

% ========================== VALIDATION METRICS ===========================
function [e,by,rel]=c6c_c6_error(seg)
by=zeros(1,7);rel=zeros(1,7);M=numel(seg.B);
for s=1:M-1
    for r=0:6
        l=c6c_bez_end(seg.B{s},seg.dt(s),r,true);
        rr=c6c_bez_end(seg.B{s+1},seg.dt(s+1),r,false);
        er=norm(l-rr); sc=max([1,norm(l),norm(rr)]);
        by(r+1)=max(by(r+1),er); rel(r+1)=max(rel(r+1),er/sc);
    end
end
e=max(by);
end
function [e,by,rel]=c6c_endpoint_error(seg)
by=zeros(1,7);rel=zeros(1,7);
for r=0:6
    a0=c6c_bez_end(seg.B{1},seg.dt(1),r,false); b0=seg.D0(:,r+1);
    a1=c6c_bez_end(seg.B{end},seg.dt(end),r,true); b1=seg.D1(:,r+1);
    e0=norm(a0-b0); e1=norm(a1-b1);
    by(r+1)=max(e0,e1);
    rel(r+1)=max(e0/max([1,norm(a0),norm(b0)]),e1/max([1,norm(a1),norm(b1)]));
end
e=max(by);
end
function v=c6c_bez_end(B,T,r,atEnd)
D=c6c_bern_derivative(B,T,r);if atEnd,v=D(:,end);else,v=D(:,1);end
end

% ================================ REPORT =================================
function c6c_print_requirements(R,S,P)
b=R.best;
fprintf('\n[REQUIREMENTS %s / %s]\n',S.name,R.method);
if isempty(b)
    fprintf(' R1 build candidate                         : FAIL\n');
    fprintf(' R2 native structural C6                   : N/A\n');
    fprintf(' R3 numeric d0..d6 continuity              : N/A\n');
    if string(S.stage)=="D" && ~isempty(R.front)
        f=R.front;
        fprintf(' R4 initial obstacles outside 15-m sensor   : %s\n',c6c_pf(f.initial_outside_all));
        fprintf(' R5 persistent detection memory             : %s\n',c6c_pf(f.memory_pass));
        fprintf(' R6 future-collision trigger (not detect)   : %s\n',c6c_pf(f.triggered && f.trigger_after_detection));
    end
    fprintf(' CORE ACCEPTANCE                            : FAIL\n');
    return
end
fprintf(' R1 build candidate                         : %s\n',c6c_pf(b.build));
fprintf(' R2 native structural C6                   : %s\n',c6c_pf(true));
fprintf(' R3 numeric d0..d6 continuity              : %s\n',c6c_pf(b.c6pass));
if string(S.stage)=="D"
    f=R.front;
    fprintf(' R4 initial obstacles outside 15-m sensor   : %s\n',c6c_pf(f.initial_outside_all));
    fprintf(' R5 persistent detection memory             : %s (knownIDs=%s)\n',c6c_pf(f.memory_pass),mat2str(f.known_ids_at_trigger));
    fprintf(' R6 future-collision trigger (not detect)   : %s (t=%.2f, TTC=%.2f)\n',c6c_pf(f.triggered && f.trigger_after_detection),f.trigger_time,f.ttc);
end
if string(S.stage)=="A"
    fprintf(' R4 oriented ellipsoid trajectory geometry : N/A\n');
    fprintf(' R5 all known ellipsoids included           : N/A\n');
    fprintf(' R6 continuous separate L/U/C certificate  : N/A\n');
else
    if string(S.stage)=="D"
        nKnown=numel(R.front.known_ids_at_trigger);
        fprintf(' R7 oriented ellipsoid trajectory geometry : PASS (support-function; no outer sphere)\n');
        fprintf(' R8 all known ellipsoids included           : %s (count=%d)\n',c6c_pf(~isempty(b.cert) && b.cert.n_obstacles==nKnown),nKnown);
        fprintf(' R9 continuous separate L/U/C certificate  : %s\n',c6c_pf(b.geom));
    else
        fprintf(' R4 oriented ellipsoid trajectory geometry : PASS (support-function; no outer sphere)\n');
        fprintf(' R5 all known ellipsoids included           : %s (count=%d)\n',c6c_pf(~isempty(b.cert) && b.cert.n_obstacles==numel(S.obs)),numel(S.obs));
        fprintf(' R6 continuous separate L/U/C certificate  : %s\n',c6c_pf(b.geom));
    end
    if isfield(b,'route_meta') && ~isempty(b.route_meta)
        fprintf('    generator outerSphereUsed               : %d\n',b.route_meta.outer_sphere_used);
        fprintf('    min exact support-plane guide clearance : %+9.3e\n',b.route_meta.min_support_clearance);
        if isfield(b.route_meta,'per_obstacle_support_clearance')
            fprintf('    per-obstacle guide clearance            : %s\n',c6c_vec(b.route_meta.per_obstacle_support_clearance));
        end
    end
end
fprintf(' CORE ACCEPTANCE                            : %s\n',c6c_pf(b.safe));
fprintf(' C6 JOIN abs residual d0..d6               : %s\n',c6c_vec(b.join_abs));
fprintf(' C6 JOIN rel residual d0..d6               : %s\n',c6c_vec(b.join_rel));
fprintf(' SPLICE abs residual d0..d6                : %s\n',c6c_vec(b.end_abs));
fprintf(' SPLICE rel residual d0..d6                : %s\n',c6c_vec(b.end_rel));
fprintf(' numerical thresholds abs/rel              : %.1e / %.1e\n',P.c6_abs_tol,P.c6_rel_tol);
end

function c6c_summary(result,P)
fprintf('\n========================================================================================================================\n');
fprintf(' CLEAN-BASELINE SUMMARY\n');
fprintf('========================================================================================================================\n');
fprintf(' method               StageA build/C6/core   StageB build/C6/core   StageC build/C6/core   StageD build/C6/core\n');
for im=1:numel(result.methods)
    vals=zeros(4,3);
    for is=1:numel(result.scenarios)
        st=string(result.scenarios(is).stage);
        col=find(["A","B","C","D"]==st,1);
        if isempty(col),continue,end
        R=result.runs{im,is};
        if ~isempty(R.best), vals(col,:)=[R.best.build,R.best.c6pass,R.best.safe]; end
    end
    fprintf(' %-22s %d/%d/%d               %d/%d/%d               %d/%d/%d               %d/%d/%d\n',...
        result.methods(im),vals(1,:),vals(2,:),vals(3,:),vals(4,:));
end
fprintf('\nGate rule: keep rejoin/SFC/v-a-j/HLC disabled until all three methods pass A-D.\n');
fprintf('Stage D rule: obstacles start outside 15-m sensor; memory is persistent; detection alone never triggers replanning.\n');
fprintf('Trajectory-generation rule: oriented ellipsoid support only; outer-sphere generation is forbidden in B/C/D.\n');
fprintf('Sensor-only simplification: circumscribed sphere may be used only for detection. Trigger uses oriented-ellipsoid support margins.\n');
fprintf('C6 numeric tolerance: abs %.1e, rel %.1e; d0..d6 are always printed separately.\n',P.c6_abs_tol,P.c6_rel_tol);
end

% ================================ UTILS ==================================
function row=c6c_row()
row=struct('tag','','nvias',0,'build',false,'build_ms',NaN,'cert_ms',NaN,'c6pass',false,'geom',false,'safe',false,...
    'join_abs_max',Inf,'join_abs',inf(1,7),'join_rel',inf(1,7),'end_abs_max',Inf,'end_abs',inf(1,7),'end_rel',inf(1,7),...
    'condition_metric',Inf,'cert',[],'route_meta',[],'reason','');
end
function D=c6c_nominal_jet(p,v)
D=zeros(3,7);D(:,1)=p;D(:,2)=v;
end
function ob=c6c_make_obs(c,r,R,id)
ob=struct('center',c(:),'radii',r(:),'R',R,'S',R*diag(r(:).^2)*R','id',id);
end
function R=c6c_Rz(a),R=[cos(a) -sin(a) 0;sin(a) cos(a) 0;0 0 1];end
function R=c6c_Ry(a),R=[cos(a) 0 sin(a);0 1 0;-sin(a) 0 cos(a)];end
function M=c6c_bernstein_matrix(n,t)
M=zeros(numel(t),n+1);for i=1:numel(t),for k=0:n,M(i,k+1)=nchoosek(n,k)*t(i)^k*(1-t(i))^(n-k);end,end
end
function [x,w]=c6c_gauss5()
x=[-0.906179845938664;-0.538469310105683;0;0.538469310105683;0.906179845938664];
w=[0.236926885056189;0.478628670499366;0.568888888888889;0.478628670499366;0.236926885056189];
end
function f=c6c_route_fractions(p0,vias,pg)
P=[p0 vias pg];d=vecnorm(diff(P,1,2),2,1);L=sum(d);
if L<=1e-12, f=(1:size(vias,2))/(size(vias,2)+1);else,c=cumsum(d);f=c(1:end-1)/L;end
end
function D=c6c_fibonacci_sphere(N)
k=(0:N-1)';z=1-2*(k+0.5)/N;phi=pi*(3-sqrt(5))*k;r=sqrt(max(0,1-z.^2));D=[r.*cos(phi) r.*sin(phi) z]';
end
function D=c6c_add_cardinal(D)
C=[eye(3) -eye(3)]; D=[D C]; D=D./vecnorm(D,2,1);
end
function s=c6c_vec(v),s=['[' strtrim(sprintf(' %.2e',v)) ' ]'];end
function s=c6c_pf(x),if x,s='PASS';else,s='FAIL';end,end
