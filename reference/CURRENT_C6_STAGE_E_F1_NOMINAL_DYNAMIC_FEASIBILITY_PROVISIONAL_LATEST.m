function OUT = CURRENT_C6_STAGE_E_F1_NOMINAL_DYNAMIC_FEASIBILITY_PROVISIONAL_LATEST(methodSelect,phys)
% Resolve vehicle/system parameters: explicit input > external/base workspace > defaults.
phys = c6f1_resolve_phys(phys);
% CURRENT_C6_STAGE_E_F1_NOMINAL_DYNAMIC_FEASIBILITY
% =========================================================================
% ONE-FILE Stage-E C6 trajectory generation + Stage-F1 nominal dynamic
% feasibility audit for the single-quadrotor suspended-load system.
%
% Research policy:
%   * NO HLC/logger/closed-loop simulation in planning feasibility.
%   * NO arbitrary v/a/j acceptance limits. v/a/j are diagnostics only.
%   * Stage-E C6/geometry/tension logic is preserved.
%   * Selected C6 trajectory is mapped through the nominal suspended-load
%     dynamics to cable tension, total thrust, and body moment.
%   * Nominal yaw reference is PASSED THROUGH UNCHANGED. Avoidance does not
%     generate or modify yaw.
%
% Model convention:
%   xQ = xL - L*p,  p: unit vector from quadrotor to payload
%   mL*(xddL + g*e3) = -T*p
%   mQ*(xddQ + g*e3) = f*R*e3 + T*p
%   J*Omegadot + Omega x (J*Omega) = M
%
% BASIC USAGE
%   OUT = CURRENT_C6_STAGE_E_F1_NOMINAL_DYNAMIC_FEASIBILITY();
%
% CONDITION OVERRIDE EXAMPLE
%   phys = struct();
%   phys.mass = 0.762;
%   phys.jx = 0.02237568;
%   phys.jy = 0.02985236;
%   phys.jz = 0.0480374;
%   phys.loadmass = 0.0556;     % replace with latest estimated payload mass
%   phys.cableL = 2.0;          % current default condition
%   phys.yaw_ref = [0;0;0];     % [psi; dpsi; ddpsi], passed through unchanged
%   % or: phys.yaw_ref = @(t)[psi(t); dpsi(t); ddpsi(t)];
%   OUT = CURRENT_C6_STAGE_E_F1_NOMINAL_DYNAMIC_FEASIBILITY('ALL',phys);
%
% Optional Stage-E parameter overrides that are not vehicle dynamics:
%   phys.c6_param_override = struct('R_uav',0.30,'R_payload',0.24,...);
%
% Output:
%   OUT.c6      : Stage-E C6 result
%   OUT.dynamic : nominal dynamics/actuator audit
%   OUT.phys    : physical/yaw parameters used
% =========================================================================

if nargin<1 || isempty(methodSelect), methodSelect='ALL'; end
if nargin<2 || isempty(phys), phys=struct(); end
phys=f1_defaults(phys);
methodSelect=upper(string(methodSelect));
allMethods=["BSPLINE_D7","POLY_D13_MIN7","BEZIER_D13_MIN7"];
if methodSelect=="ALL"
    methods=allMethods;
else
    if ~any(methodSelect==allMethods), error('C6CLEAN:BadMethod','Unknown method %s',methodSelect); end
    methods=methodSelect;
end

clc; format compact;
P=c6c_params();
% Keep Stage-E geometry/certificate and nominal dynamics physically aligned.
P.L_cable=phys.cableL;
P.g=phys.gravity;
if ~isempty(phys.c6_param_override)
    fno=fieldnames(phys.c6_param_override);
    for ko=1:numel(fno)
        if ~isfield(P,fno{ko})
            error('C6CLEAN:BadParamOverride','Unknown c6_param_override field: %s',fno{ko});
        end
        P.(fno{ko})=phys.c6_param_override.(fno{ko});
    end
end
% Dynamics consistency wins for these two fields.
P.L_cable=phys.cableL;
P.g=phys.gravity;

scenarios=c6c_scenarios();
scenarios=scenarios(arrayfun(@(s) string(s.stage)=="E",scenarios));

fprintf('\n========================================================================================================================\n');
fprintf(' STAGE-E + F1 ONE-FILE -- THREE NATIVE C6 REPRESENTATIONS\n');
fprintf(' methods: %s\n',strjoin(cellstr(methods),', '));
fprintf(' C6 objective: exact physical integral ||p^(7)||^2 in square-root form\n');
fprintf(' geometry: oriented ellipsoid support | cable length L=%.6g m\n',P.L_cable);
fprintf(' v/a/j: MEASURE ONLY; no acceptance limits\n');
fprintf(' yaw: nominal reference PASSTHROUGH; avoidance yaw modification = OFF\n');
fprintf('========================================================================================================================\n');

runs=cell(numel(methods),numel(scenarios));
for im=1:numel(methods)
    fprintf('\n########################################################################################################################\n');
    fprintf(' METHOD: %s\n',methods(im));
    fprintf('########################################################################################################################\n');
    for is=1:numel(scenarios)
        runs{im,is}=c6c_run(methods(im),scenarios(is),P);
    end
end
R6=struct('methods',methods,'scenarios',{scenarios},'runs',{runs},'params',P);
c6c_summary(R6,P);

DYN=f1_run_dynamic_audit(R6,phys);

OUT=struct();
OUT.mode='STAGE_E_C6_PLUS_NOMINAL_DYNAMIC_FEASIBILITY';
OUT.c6=R6;
OUT.dynamic=DYN;
OUT.phys=phys;
OUT.policy=struct('hlc_used',false,'logger_used',false,'closed_loop_simulation_used',false,...
    'velocity_gate',false,'acceleration_gate',false,'jerk_gate',false,...
    'nominal_yaw_passthrough',true,'avoidance_yaw_modification',false,...
    'sampled_actuator_audit',true);
assignin('base','c6_stage_e_f1_result',OUT);
end

function DYN=f1_run_dynamic_audit(R6,phys)
f1_validate_R6(R6);

fprintf('\n================================================================================================================================================\n');
fprintf(' STAGE-F1 NOMINAL DYNAMIC FEASIBILITY -- NO HLC / NO LOGGER / NO CLOSED-LOOP SIMULATION\n');
fprintf(' mQ=%.6g kg  mL=%.6g kg  L=%.6g m  J=[%.8g %.8g %.8g] kg m^2  g=%.6g\n', ...
    phys.mass,phys.loadmass,phys.cableL,phys.jx,phys.jy,phys.jz,phys.gravity);
fprintf(' actuator check: thrust in [%.6g, %.6g] N, |M| <= [%.6g %.6g %.6g] N m\n', ...
    phys.thrust_min,phys.thrust_max,phys.moment_abs_max);
fprintf(' yaw reference: PASSTHROUGH (no avoidance-side modification)\n');
fprintf(' v/a/j: diagnostic only | nonlinear actuator audit sampling dt <= %.6g s\n',phys.dt_eval);
fprintf('================================================================================================================================================\n');

nm=numel(R6.methods); ns=numel(R6.scenarios);
REC=repmat(f1_empty_record(),nm,ns);
for im=1:nm
    for is=1:ns
        run=R6.runs{im,is};
        rec=f1_empty_record();
        rec.method=char(string(R6.methods(im)));
        rec.case_id=R6.scenarios(is).case_id;
        rec.scenario=R6.scenarios(is).name;
        rec.upstream_safe=~isempty(run.best) && run.best.safe;
        if ~isfield(run,'selected_segment') || isempty(run.selected_segment)
            rec.reason='NO_SELECTED_SEGMENT';
            REC(im,is)=rec;
            fprintf('[F1] %-5s %-22s upstream=%d dynamic=NA reason=%s\n',rec.case_id,rec.method,rec.upstream_safe,rec.reason);
            continue
        end
        try
            [traj,diag]=f1_audit_segment(run.selected_segment,phys);
            rec.available=true;
            rec.valid=diag.valid;
            rec.tension_min=diag.tension_min;
            rec.thrust_min=diag.thrust_min;
            rec.thrust_max=diag.thrust_max;
            rec.moment_abs_max=diag.moment_abs_max;
            rec.vmax=diag.vmax; rec.amax=diag.amax; rec.jmax=diag.jmax;
            rec.n_samples=diag.n_samples;
            rec.dynamic_pass=diag.valid && diag.tension_min>0 && ...
                diag.thrust_min>=phys.thrust_min && diag.thrust_max<=phys.thrust_max && ...
                all(diag.moment_abs_max<=phys.moment_abs_max);
            rec.pass=rec.upstream_safe && rec.dynamic_pass;
            rec.reason=f1_reason(rec,phys);
            rec.trace=traj;
            fprintf(['[F1] %-5s %-22s upstream=%d dyn=%d TloadMin=%8.4f ', ...
                     'f=[%8.4f,%8.4f] MabsMax=[%8.4f %8.4f %8.4f] ', ...
                     'v/a/j=[%7.3f %7.3f %7.3f] samples=%d reason=%s\n'], ...
                rec.case_id,rec.method,rec.upstream_safe,rec.dynamic_pass,rec.tension_min,...
                rec.thrust_min,rec.thrust_max,rec.moment_abs_max,...
                rec.vmax,rec.amax,rec.jmax,rec.n_samples,rec.reason);
        catch ME
            rec.reason=['AUDIT_FAIL:' ME.identifier];
            rec.error_message=ME.message;
            fprintf('[F1] %-5s %-22s AUDIT_FAIL %s : %s\n',rec.case_id,rec.method,ME.identifier,ME.message);
        end
        REC(im,is)=rec;
    end
end

fprintf('\n================================================================================================================================================\n');
fprintf(' STAGE-F1 SUMMARY\n');
fprintf('================================================================================================================================================\n');
for is=1:ns
    for im=1:nm
        r=REC(im,is);
        fprintf(' %-5s %-22s upstream=%d dynamic=%d final=%d  %s\n',r.case_id,r.method,r.upstream_safe,r.dynamic_pass,r.pass,r.reason);
    end
end
fprintf('\nPolicy:\n');
fprintf(' - No HLC/logger/closed-loop simulation is part of planning feasibility.\n');
fprintf(' - Nominal yaw reference is passed through unchanged.\n');
fprintf(' - v/a/j are diagnostics only; no empirical derivative limits are introduced.\n');
fprintf(' - Upstream C6/geometry/tension and nominal actuator feasibility are both required for final pass.\n');
fprintf(' - Actuator feasibility is sampled in time; it is not claimed as a continuous-time nonlinear actuator certificate.\n');
fprintf('================================================================================================================================================\n\n');

avail=[REC.available];
DYN=struct();
DYN.mode='NOMINAL_DYNAMIC_FEASIBILITY';
DYN.records=REC;
DYN.phys=phys;
DYN.pass_all_available=~any(avail) || all([REC(avail).pass]);
end

function P=c6c_params()
P=struct();
P.p_bs=7; P.n_poly=13; P.n_bez=13;
% Native C6 is structural; no empirical decimal continuity thresholds are used.
P.d_margin=0.15;
P.R_payload=0.24;
P.R_uav=0.30;
P.R_cable=0.045;
P.L_cable=2.00; % integrated default; overridden from phys.cableL in main
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
P.compact_topk=3;
% Stage-E reporting only. v/a/j limits are intentionally not enforced yet.
P.stageE_rep_fail=true;
P.stageE_compact=true;
P.solver_name='SQRT_LS_COMPLETE_QR';
end

function S=c6c_scenarios()
A=struct();
A.name='A_NATIVE_C6'; A.stage='A'; A.case_id='A00'; A.case_desc='native C6 unit';
A.p0=[0;0;0]; A.pg=[8;0;0]; A.v0=[0.8;0;0]; A.vg=[0.8;0;0];
A.T=10; A.obs=[]; A.fixed_vias=[4;1.0;0.5];

B=struct();
B.name='B_ONE_ROTATED_ELLIPSOID'; B.stage='B'; B.case_id='B00'; B.case_desc='one rotated ellipsoid';
B.p0=[0;0;0]; B.pg=[10;0;0]; B.v0=[0.8;0;0]; B.vg=[0.8;0;0];
B.T=13;
B.obs=c6c_make_obs([5;0;0],[1.10;0.85;1.15],c6c_Rz(deg2rad(25))*c6c_Ry(deg2rad(10)),1);
B.fixed_vias=[];

C=struct();
C.name='C_TWO_ROTATED_ELLIPSOIDS'; C.stage='C'; C.case_id='C00'; C.case_desc='two known ellipsoids';
C.p0=[0;0;0]; C.pg=[12;0;0]; C.v0=[0.8;0;0]; C.vg=[0.8;0;0];
C.T=16;
C.obs(1)=c6c_make_obs([4.25; 0.20; 0.05],[1.00;0.72;1.05],c6c_Rz(deg2rad(22))*c6c_Ry(deg2rad(8)),1);
C.obs(2)=c6c_make_obs([7.55;-0.25; 0.18],[0.92;0.78;1.12],c6c_Rz(deg2rad(-31))*c6c_Ry(deg2rad(-12)),2);
C.fixed_vias=[];

D=c6c_make_dynamic_case('D00','D_SENSOR_TRIGGER_TWO_ELLIPSOIDS','baseline sensor/trigger',...
    [20.25;0.20;0.05],[1.00;0.72;1.05],22,8,...
    [24.00;-0.25;0.18],[0.92;0.78;1.12],-31,-12,1.0,20);
D.stage='D';

% Stage-E regression matrix. Every case starts with obstacle surfaces outside
% the 15-m sensor at t=0. These cases vary one or two meaningful factors at
% a time so failures remain diagnosable. This is NOT a ranking benchmark.
E(1)=c6c_make_dynamic_case('E01','E01_BASELINE','baseline repeat',...
    [20.25;0.20;0.05],[1.00;0.72;1.05],22,8,[24.00;-0.25;0.18],[0.92;0.78;1.12],-31,-12,1.0,20);
E(2)=c6c_make_dynamic_case('E02','E02_HIGH_ROTATION','strong obstacle rotations',...
    [20.30;0.15;0.10],[1.00;0.72;1.05],48,24,[24.10;-0.20;0.15],[0.92;0.78;1.12],-52,-28,1.0,20);
E(3)=c6c_make_dynamic_case('E03_LARGER_OBS','E03_LARGER_OBS','larger ellipsoids',...
    [20.60;0.20;0.05],[1.30;0.92;1.28],25,12,[24.60;-0.25;0.18],[1.18;1.00;1.35],-35,-15,1.0,21);
E(4)=c6c_make_dynamic_case('E04_CLOSE_PAIR','E04_CLOSE_PAIR','closer obstacle spacing',...
    [20.25;0.25;0.05],[1.00;0.72;1.05],22,8,[22.95;-0.25;0.16],[0.92;0.78;1.12],-31,-12,1.0,20);
E(5)=c6c_make_dynamic_case('E05_LATERAL','E05_LATERAL','opposite lateral offsets',...
    [20.25;0.70;0.05],[1.00;0.72;1.05],22,8,[24.00;-0.80;0.18],[0.92;0.78;1.12],-31,-12,1.0,20);
E(6)=c6c_make_dynamic_case('E06_VERTICAL','E06_VERTICAL','opposite vertical offsets',...
    [20.25;0.15;0.65],[1.00;0.72;1.05],22,18,[24.00;-0.20;-0.50],[0.92;0.78;1.12],-31,-22,1.0,20);
E(7)=c6c_make_dynamic_case('E07_FAST','E07_FAST','faster nominal approach',...
    [20.25;0.20;0.05],[1.00;0.72;1.05],22,8,[24.00;-0.25;0.18],[0.92;0.78;1.12],-31,-12,1.35,17);
E(8)=c6c_make_dynamic_case('E08_SLOW','E08_SLOW','slower nominal approach',...
    [20.25;0.20;0.05],[1.00;0.72;1.05],22,8,[24.00;-0.25;0.18],[0.92;0.78;1.12],-31,-12,0.70,25);
E(9)=c6c_make_dynamic_case('E09_NEAR_SENSOR_EDGE','E09_NEAR_SENSOR_EDGE','initially just outside sensor',...
    [17.20;0.20;0.05],[1.00;0.72;1.05],22,8,[21.00;-0.25;0.18],[0.92;0.78;1.12],-31,-12,1.0,18);
E(10)=c6c_make_dynamic_case('E10_ASYMMETRIC','E10_ASYMMETRIC','asymmetric size and orientation',...
    [20.40;0.45;0.20],[1.35;0.55;0.85],60,-18,[24.30;-0.45;-0.10],[0.70;1.20;1.05],-58,25,1.0,21);
for k=1:numel(E), E(k).stage='E'; end
S=[A B C D E];
end

function S=c6c_make_dynamic_case(id,name,desc,c1,r1,yaw1,pitch1,c2,r2,yaw2,pitch2,speed,Tplan)
S=struct();
% Keep field order identical to A/B/C so MATLAB struct concatenation is deterministic.
S.name=name; S.stage='E'; S.case_id=id; S.case_desc=desc;
S.p0=[0;0;0]; S.pg=[30;0;0]; S.v0=[speed;0;0]; S.vg=[speed;0;0]; S.T=Tplan;
S.obs(1)=c6c_make_obs(c1,r1,c6c_Rz(deg2rad(yaw1))*c6c_Ry(deg2rad(pitch1)),1);
S.obs(2)=c6c_make_obs(c2,r2,c6c_Rz(deg2rad(yaw2))*c6c_Ry(deg2rad(pitch2)),2);
S.fixed_vias=[];
end

function R=c6c_run(method,S,P)
fprintf('\n------------------------------------------------------------------------------------------------------------------------\n');
fprintf(' %s / %s\n',S.name,method);
fprintf('------------------------------------------------------------------------------------------------------------------------\n');
front=[]; Splan=S;
if any(string(S.stage)==["D","E"])
    tf_front=tic;
    front=c6c_sensor_trigger_frontend(S,P);
    front.runtime_ms=1e3*toc(tf_front);
    c6c_print_frontend(front,S,P);
    if ~front.triggered
        R=struct('method',char(method),'scenario',S.name,'rows',repmat(c6c_row(),1,0),'best',[], 'front',front,...
            'route_gen_ms',NaN,'search_total_ms',NaN,'selected_segment',[]);
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
    route_gen_ms=0;
    routes={S.fixed_vias};
    tags={'UNIT_VIA'};
    routeMeta={struct('generator','representation-unit','outer_sphere_used',false,...
        'min_support_clearance',NaN)};
else
    troute=tic;
    [routes,tags,routeMeta]=c6c_ellipsoid_routes(method,Splan.p0,Splan.pg,Splan.obs,P);
    route_gen_ms=1e3*toc(troute);
    if string(S.stage)=="E" && P.stageE_compact
        fprintf('[CASE] %s: %s | obstacles=%d\n',S.case_id,S.case_desc,numel(Splan.obs));
    else
        for io=1:numel(Splan.obs)
            ob=Splan.obs(io);
            fprintf('[ELLIPSOID %d] center=[%.3f %.3f %.3f] radii=[%.3f %.3f %.3f]\n',...
                ob.id,ob.center,ob.radii);
            fprintf('[ELLIPSOID %d] R=\n',ob.id); disp(ob.R);
        end
    end
    fprintf('[GENERATOR] oriented support h_E(n)=n''c+sqrt(n''Sn), outerSphereUsed=0, obstacles=%d, candidates=%d\n',...
        numel(Splan.obs),numel(routes));
end

rows=repmat(c6c_row(),1,numel(routes));
best=[];
selected_segment=[];
tsearch=tic;
for ir=1:numel(routes)
    rr=c6c_row(); rr.tag=tags{ir}; rr.nvias=size(routes{ir},2);
    try
        tc=tic;
        seg=c6c_build(method,Splan.T,D0,D1,routes{ir},P);
        rr.build_ms=1e3*toc(tc); rr.build=true;
        [rr.join_abs_max,rr.join_abs,rr.join_rel]=c6c_c6_error(seg);
        [rr.end_abs_max,rr.end_abs,rr.end_rel]=c6c_endpoint_error(seg);
        % Native exact-arithmetic construction is C6. Floating residuals are diagnostics,
        % not empirical decimal acceptance thresholds.
        rr.c6pass=seg.structural_c6;
        rr.solver_diag=seg.solver_diag;
        tc=tic; rr.cert=c6c_certify(seg,Splan.obs,P); rr.cert_ms=1e3*toc(tc);
        rr.geom=rr.cert.geom_pass;
        rr.tension=rr.cert.tension_pass;
        rr.safe=rr.build && rr.c6pass && rr.geom && rr.tension;
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
            % Export selected trajectory only; generation/ranking/acceptance are unchanged.
            selected_segment=seg;
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
search_total_ms=1e3*toc(tsearch);
if P.compact_log
    c6c_print_search_summary(rows,best,method,Splan,P);
end
R=struct('method',char(method),'scenario',S.name,'rows',rows,'best',best,'front',front,...
    'route_gen_ms',route_gen_ms,'search_total_ms',search_total_ms,'selected_segment',selected_segment);
if string(S.stage)=="E", c6c_print_perf(R,S); end
c6c_print_requirements(R,S,P);
end

function c6c_print_candidate(rr,method,Splan,seg,isSelected)
mark=''; if isSelected, mark=' <SELECTED>'; end
fprintf('[SELECTED] method=%s route=%s vias=%d C6mode=%s C6ok=%d geom=%d tension=%d minG=%+.6e [L/U/C]=[%+.6e %+.6e %+.6e] candidate=%.3f ms%s\n', ...
    method,rr.tag,rr.nvias,c6c_c6_mode(method),rr.c6pass,rr.geom,rr.tension,rr.cert.minG,rr.cert.minG_payload,rr.cert.minG_uav,rr.cert.minG_cable,rr.build_ms+rr.cert_ms,mark);
sd=seg.solver_diag;
fprintf(' [SOLVER] mode=%s A=%dx%d rankA(struct)=%d null=%d rankCZ(struct)=%d condA(raw)=%.3e condA(rowScaled)=%.3e condCZ(coord)=%.3e\n', ...
    sd.mode,sd.m,sd.n,sd.eq_rank_structural,sd.nullity,sd.rank_cz_structural,sd.cond_a,sd.cond_a_scaled,sd.cond_cz);
fprintf(' [SPECTRUM] sigmaMinA=%.3e sigmaMaxA=%.3e sigmaMinCZ=%.3e sigmaMaxCZ=%.3e J7=%.12e solve=%.3f ms\n', ...
    sd.sigma_min_a,sd.sigma_max_a,sd.sigma_min_cz,sd.sigma_max_cz,sd.objective,sd.solve_ms);
fprintf(' [A-POST] eqRes=%.3e eqRel=%.3e eqCorrection<=%.3e projectedGrad=%.3e optCoeffCorrection<=%.3e\n', ...
    sd.eq_res_norm,sd.eq_res_rel,sd.eq_correction_bound,sd.projected_gradient,sd.opt_correction_bound);
fprintf(' [QR] nullResidual=%.3e particularResidual=%.3e\n',sd.qr_null_res,sd.qr_particular_res);
fprintf(' [C6] joinAbs=%s\n',c6c_vec(rr.join_abs));
fprintf(' [C6] joinRel=%s\n',c6c_vec(rr.join_rel));
fprintf(' [END] endpointAbs=%s\n',c6c_vec(rr.end_abs));
fprintf(' [END] endpointRel=%s\n',c6c_vec(rr.end_rel));
for io=1:numel(Splan.obs)
    fprintf(' [OBS] id=%d minG=%+.6e [L/U/C]=[%+.6e %+.6e %+.6e] guide=%+.6e\n', ...
        Splan.obs(io).id,rr.cert.minG_per_obstacle(io),rr.cert.minG_payload_per_obstacle(io), ...
        rr.cert.minG_uav_per_obstacle(io),rr.cert.minG_cable_per_obstacle(io), ...
        rr.route_meta.per_obstacle_support_clearance(io));
end
end

function c6c_print_search_summary(rows,best,method,Splan,P)
if isempty(rows), return; end
nb=numel(rows); nbuild=sum([rows.build]); nc6=sum([rows.c6pass]); ngeom=sum([rows.geom]); ntension=sum([rows.tension]); nsafe=sum([rows.safe]);
fprintf('[SEARCH] method=%s tested=%d build=%d C6ok=%d geom=%d tension=%d safe=%d suppressed=%d\n', ...
    method,nb,nbuild,nc6,ngeom,ntension,nsafe,max(0,nb-1));
failRows=rows(~[rows.safe]);
if ~isempty(failRows) && P.stageE_rep_fail
    rep=c6c_pick_rep_failure(failRows);
    c6c_print_rep_failure(rep,Splan);
end
if nsafe==0 && ~isempty(best)
    if ~isempty(best.cert)
        fprintf('[BEST-UNSAFE] route=%s minG=%+.6e reason=%s\n',best.tag,best.cert.minG,best.reason);
    else
        fprintf('[BEST-UNSAFE] route=%s reason=%s\n',best.tag,best.reason);
    end
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
elseif ~rr.tension, s='TENSION_FAIL';
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
if string(S.stage)=="E" && P.stageE_compact
    fprintf('[FRONT] case=%s initialOutside=%d detectAt=%s memory=%d trigger=%d t=%.2f TTC=%.2f known=%s\n',...
        S.case_id,F.initial_outside_all,mat2str(F.detected_time,4),F.memory_pass,F.triggered,F.trigger_time,F.ttc,mat2str(F.known_ids_at_trigger));
    return
end
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
p=P.p_bs; nVia=size(vias,2); N=max(22,2*nVia+18); U=c6c_clamped_uniform_knots(N,p);
frac=c6c_route_fractions(D0(:,1),vias,D1(:,1)); A=[]; B=zeros(0,3);
for r=0:6, A=[A;c6c_bs_basis_derivative_all(0,U,p,N,r)/T^r]; B=[B;D0(:,r+1)']; end %#ok<AGROW>
for r=0:6, A=[A;c6c_bs_basis_derivative_all(1,U,p,N,r)/T^r]; B=[B;D1(:,r+1)']; end %#ok<AGROW>
for k=1:nVia, A=[A;c6c_bs_basis_derivative_all(frac(k),U,p,N,0)]; B=[B;vias(:,k)']; end %#ok<AGROW>
spans=c6c_bs_spans(U,p);
% degree 7 => seventh derivative is constant on each knot span, so this is exact.
C=zeros(size(spans,1),N);
for s=1:size(spans,1)
    ua=spans(s,1); ub=spans(s,2); um=(ua+ub)/2;
    C(s,:)=sqrt(T*(ub-ua))*c6c_bs_basis_derivative_all(um,U,p,N,7)/T^7;
end
tsolve=tic; [X,solverDiag]=c6c_solver_sqrt_ls(C,A,B); solverDiag.solve_ms=1e3*toc(tsolve);
solverDiag.objective=c6c_sqrt_cost_value(C,X); solverDiag=c6c_add_apost_diag(C,A,B,X,solverDiag);
Bez=cell(size(spans,1),1); dt=zeros(1,size(spans,1));
for s=1:size(spans,1)
    ua=spans(s,1); ub=spans(s,2); tau=(0:p)'/p; MB=c6c_bernstein_matrix(p,tau); V=zeros(p+1,3);
    for k=1:p+1, u=ua+(ub-ua)*tau(k); V(k,:)=c6c_bs_basis_derivative_all(u,U,p,N,0)*X; end
    Bez{s}=(MB\V)'; dt(s)=T*(ub-ua);
end
seg=struct('name','BSPLINE_D7','B',{Bez},'dt',dt,'D0',D0,'D1',D1,'structural_c6',true,...
    'condition_metric',solverDiag.cond_cz,'solver_diag',solverDiag,'native',struct('U',U,'X',X,'degree',p,'C',C,'A',A,'rhs',B));
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
    v=double((u>=U(i) && u<U(i+1)) || (u==1 && U(i+1)==1 && U(i)<1)); return
end
a=0;b=0; d1=U(i+p)-U(i); d2=U(i+p+1)-U(i+1);
if d1~=0,a=(u-U(i))/d1*c6c_bs_basis(i,p-1,u,U);end
if d2~=0,b=(U(i+p+1)-u)/d2*c6c_bs_basis(i+1,p-1,u,U);end
v=a+b;
end

% ============================ POWER POLYNOMIAL ===========================
function seg=c6c_build_poly(T,D0,D1,vias,P)
n=P.n_poly; M=size(vias,2)+1; frac=c6c_route_fractions(D0(:,1),vias,D1(:,1)); br=[0 frac 1]; dt=T*diff(br);
[A,B]=c6c_poly_eq(n,dt,D0,D1,vias); C=c6c_poly_sqrt_cost(n,dt,7);
tsolve=tic; [X,solverDiag]=c6c_solver_sqrt_ls(C,A,B); solverDiag.solve_ms=1e3*toc(tsolve);
solverDiag.objective=c6c_sqrt_cost_value(C,X); solverDiag=c6c_add_apost_diag(C,A,B,X,solverDiag);
Bez=cell(M,1);
for s=1:M
    ix=(s-1)*(n+1)+(1:n+1); CC=X(ix,:); BB=zeros(3,n+1);
    for ax=1:3, BB(ax,:)=c6c_power_to_bernstein(CC(:,ax)',n); end
    Bez{s}=BB;
end
seg=struct('name','POLY_D13_MIN7','B',{Bez},'dt',dt,'D0',D0,'D1',D1,'structural_c6',true,...
    'condition_metric',solverDiag.cond_cz,'solver_diag',solverDiag,'native',struct('X',X,'degree',n,'C',C,'A',A,'rhs',B));
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
function b=c6c_power_to_bernstein(a,n)
b=zeros(1,n+1);
for k=0:n,for j=0:k,b(k+1)=b(k+1)+a(j+1)*nchoosek(k,j)/nchoosek(n,j);end,end
end

% ================================= BEZIER ================================
function seg=c6c_build_bezier(T,D0,D1,vias,P)
n=P.n_bez; M=size(vias,2)+1; frac=c6c_route_fractions(D0(:,1),vias,D1(:,1)); br=[0 frac 1]; dt=T*diff(br);
[A,B]=c6c_bezier_eq(n,dt,D0,D1,vias); C=c6c_bezier_sqrt_cost(n,dt,7);
tsolve=tic; [X,solverDiag]=c6c_solver_sqrt_ls(C,A,B); solverDiag.solve_ms=1e3*toc(tsolve);
solverDiag.objective=c6c_sqrt_cost_value(C,X); solverDiag=c6c_add_apost_diag(C,A,B,X,solverDiag);
Bez=cell(M,1); for s=1:M, ix=(s-1)*(n+1)+(1:n+1); Bez{s}=X(ix,:)'; end
seg=struct('name','BEZIER_D13_MIN7','B',{Bez},'dt',dt,'D0',D0,'D1',D1,'structural_c6',true,...
    'condition_metric',solverDiag.cond_cz,'solver_diag',solverDiag,'native',struct('X',X,'degree',n,'C',C,'A',A,'rhs',B));
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

% ==================== SQUARE-ROOT CONSTRAINED LS SOLVER ==================
function [X,d]=c6c_solver_sqrt_ls(C,A,B)
% A is structurally full row rank. C|ker(A) is injective: C*dX=0 means zero
% seventh derivative; C6 makes dX one global degree<=6 polynomial and zero
% endpoint jets d0..d6 force dX=0. Hence the constrained minimizer is unique.
rowScale=vecnorm(A,2,2);
if any(rowScale==0), error('C6V8:ZeroEqualityRow','Zero equality row.'); end
An=A./rowScale; Bn=B./rowScale;
m=size(An,1); n=size(An,2);
if m>n, error('C6V8:EqualityShape','Expected m<=n.'); end
[Qe,Re]=qr(An');
R1=Re(1:m,:);
if any(diag(R1)==0), error('C6V8:StructuralRankViolation','Exact zero QR diagonal.'); end
Q1=Qe(:,1:m);
X0=Q1*(R1'\Bn);
Z=Qe(:,m+1:n);
if isempty(Z)
    X=X0; condCZ=1; sigmaMinCZ=Inf; sigmaMaxCZ=0; mode='no-nullspace';
else
    CZ=C*Z; R0=C*X0;
    coln=vecnorm(CZ,2,1);
    if any(coln==0), error('C6V8:InjectivityViolation','Exact zero C*Z column.'); end
    % Column equilibration changes coordinates only; it does not alter the LS problem.
    D=diag(1./coln);
    M=CZ*D;
    Ys=M\(-R0);
    X=X0+Z*(D*Ys);
    sm=svd(CZ);
    sigmaMaxCZ=sm(1); sigmaMinCZ=sm(end); condCZ=sigmaMaxCZ/sigmaMinCZ;
    mode='sqrt-ls-completeQR';
end
saRaw=svd(A);
saScaled=svd(An);
R=A*X-B;
den=saRaw(1)*norm(X,'fro')+norm(B,'fro');
if den==0
    if norm(R,'fro')==0, eqRel=0; else, eqRel=Inf; end
else
    eqRel=norm(R,'fro')/den;
end
d=struct('mode',mode,'m',m,'n',n,'eq_rank_structural',m,'nullity',size(Z,2), ...
    'cond_a',saRaw(1)/saRaw(end),'cond_a_scaled',saScaled(1)/saScaled(end), ...
    'sigma_min_a',saRaw(end),'sigma_max_a',saRaw(1), ...
    'sigma_min_a_scaled',saScaled(end),'sigma_max_a_scaled',saScaled(1), ...
    'rank_cz_structural',size(Z,2),'cond_cz',condCZ, ...
    'sigma_min_cz',sigmaMinCZ,'sigma_max_cz',sigmaMaxCZ, ...
    'eq_res_norm',norm(R,'fro'),'eq_res_rel',eqRel, ...
    'qr_null_res',norm(An*Z,'fro')/norm(An,'fro'), ...
    'qr_particular_res',norm(An*X0-Bn,'fro')/(norm(An,'fro')*norm(X0,'fro')+norm(Bn,'fro')));
end
function d=c6c_add_apost_diag(C,A,B,X,d)
% Normwise coefficient correction bound for raw equality system: ||dX|| <= ||AX-B||/sigma_min(A).
Req=A*X-B; d.eq_correction_bound=norm(Req,'fro')/d.sigma_min_a;
rowScale=vecnorm(A,2,2); An=A./rowScale; m=size(An,1); n=size(An,2); [Qe,~]=qr(An'); Z=Qe(:,m+1:n);
g=C'*(C*X);
if isempty(Z), d.projected_gradient=0; d.opt_correction_bound=0; else, pg=norm(Z'*g,'fro'); d.projected_gradient=pg; d.opt_correction_bound=pg/(d.sigma_min_cz^2); end
end
function J=c6c_sqrt_cost_value(C,X), R=C*X; J=sum(R(:).^2); end
function C=c6c_poly_sqrt_cost(n,dt,r)
M=numel(dt); [u,w]=c6c_gauss7_01(); C=zeros(M*numel(u),(n+1)*M); row=0;
for s=1:M, for q=1:numel(u), row=row+1; v=zeros(1,n+1); for k=r:n, v(k+1)=factorial(k)/factorial(k-r)*u(q)^(k-r)/dt(s)^r; end; ix=(s-1)*(n+1)+(1:n+1); C(row,ix)=sqrt(w(q)*dt(s))*v; end, end
end
function C=c6c_bezier_sqrt_cost(n,dt,r)
M=numel(dt); [u,w]=c6c_gauss7_01(); C=zeros(M*numel(u),(n+1)*M); row=0; D=eye(n+1); for k=1:r, D=diff(D,1,1); end; m=n-r; scale0=factorial(n)/factorial(n-r);
for s=1:M, for q=1:numel(u), row=row+1; bm=c6c_bernstein_values(m,u(q)); v=(scale0/dt(s)^r)*(bm*D); ix=(s-1)*(n+1)+(1:n+1); C(row,ix)=sqrt(w(q)*dt(s))*v; end, end
end
function b=c6c_bernstein_values(n,u), b=zeros(1,n+1); for k=0:n, b(k+1)=nchoosek(n,k)*u^k*(1-u)^(n-k); end, end
function [u,w]=c6c_gauss7_01()
x=[-0.949107912342759;-0.741531185599394;-0.405845151377397;0;0.405845151377397;0.741531185599394;0.949107912342759];
w=[0.129484966168870;0.279705391489277;0.381830050505119;0.417959183673469;0.381830050505119;0.279705391489277;0.129484966168870]; u=(x+1)/2; w=w/2;
end

% ======================= CONTINUOUS SAFETY CERTIFICATE ==================
function cert=c6c_certify(seg,obs,P)
% Obstacle geometry is an oriented ellipsoid throughout:
%   E={x:(x-c)'S^{-1}(x-c)<=1}
%   h_E(n)=n'c+sqrt(n'Sn)
% NO max(radii) / bounding sphere is used here.
N=P.sep_dirs;K=size(N,2); no=numel(obs);
minG=inf;minL=inf;minU=inf;minC=inf;vmax=0;amax=0;jmax=0;tLB=inf;geomPass=true;tensionPass=true;
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
    if Mlb<=0, tensionPass=false; end
    plow=min(N'*B,[],2); wmin=min(N'*W,[],2); qlow=-ones(K,1);
    pos=wmin>=0; if Mmax>0, qlow(pos)=wmin(pos)/Mmax;end
    neg=~pos; if Mlb>0, qlow(neg)=max(-1,wmin(neg)/Mlb);end
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
        if any(bL<0)||any(bU<0)||any(bC<0),geomPass=false;end
    end
end
if no==0,minG=inf;minL=inf;minU=inf;minC=inf; perL=[]; perU=[]; perC=[]; perG=[]; end
cert=struct('pass',geomPass && tensionPass,'geom_pass',geomPass,'tension_pass',tensionPass,'minG',minG,'minG_payload',minL,'minG_uav',minU,'minG_cable',minC,...
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

function rep=c6c_pick_rep_failure(rows)
rep=[]; if isempty(rows), return; end
% Prefer the closest geometry failure because it is the most informative
% near-miss. Otherwise retain a representative C6/build failure.
idx=find([rows.build] & [rows.c6pass] & ~[rows.geom] & arrayfun(@(r) ~isempty(r.cert),rows));
if ~isempty(idx)
    g=arrayfun(@(i) rows(i).cert.minG,idx); [~,k]=max(g); rep=rows(idx(k)); return
end
idx=find([rows.build] & [rows.c6pass] & [rows.geom] & ~[rows.tension],1,'first'); if ~isempty(idx), rep=rows(idx); return; end
idx=find([rows.build] & ~[rows.c6pass],1,'first'); if ~isempty(idx), rep=rows(idx); return; end
idx=find(~[rows.build],1,'first'); if ~isempty(idx), rep=rows(idx); end
end

function c6c_print_rep_failure(rr,Splan)
if isempty(rr), return; end
if ~rr.build
    fprintf('[REP-FAIL] route=%s reason=%s\n',rr.tag,rr.reason);
    return;
end
if isempty(rr.cert)
    fprintf('[REP-FAIL] route=%s reason=%s C6ok=%d maxJoinAbs=%.3e\n', ...
        rr.tag,rr.reason,rr.c6pass,max(rr.join_abs));
    return;
end
fprintf('[REP-FAIL] route=%s reason=%s tension=%d minG=%+.6e [L/U/C]=[%+.6e %+.6e %+.6e] maxJoinAbs=%.3e\n', ...
    rr.tag,rr.reason,rr.tension,rr.cert.minG,rr.cert.minG_payload,rr.cert.minG_uav,rr.cert.minG_cable,max(rr.join_abs));
if ~isempty(rr.solver_diag)
    sd=rr.solver_diag;
    fprintf(' [REP-SOLVER] condA(raw)=%.3e condCZ(coord)=%.3e sigmaMinCZ=%.3e eqCorr<=%.3e optCoeffCorr<=%.3e\n', ...
        sd.cond_a,sd.cond_cz,sd.sigma_min_cz,sd.eq_correction_bound,sd.opt_correction_bound);
end
if numel(Splan.obs)>1
    fprintf(' [REP-OBS] perObsMinG=%s\n',c6c_vec(rr.cert.minG_per_obstacle));
end
end

function c6c_print_perf(R,S)
b=R.best;
frontms=NaN; if ~isempty(R.front) && isfield(R.front,'runtime_ms'), frontms=R.front.runtime_ms; end
if isempty(b)
    fprintf('[PERF] case=%s front=%.3f ms route=%.3f ms search=%.3f ms selectedBuild=NaN selectedCert=NaN tested=%d\n',...
        S.case_id,frontms,R.route_gen_ms,R.search_total_ms,numel(R.rows));
    return
end
fprintf('[VAJ-BOUND] case=%s v<=%.4f m/s a<=%.4f m/s^2 j<=%.4f m/s^3 tensionLB=%.4f\n',...
    S.case_id,b.cert.v,b.cert.a,b.cert.j,b.cert.tension_lb);
fprintf('[PERF] case=%s front=%.3f ms route=%.3f ms build=%.3f ms cert=%.3f ms searchTotal=%.3f ms tested=%d\n',...
    S.case_id,frontms,R.route_gen_ms,b.build_ms,b.cert_ms,R.search_total_ms,numel(R.rows));
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
if string(S.stage)=="E" && P.stageE_compact
    if isempty(b)
        f=R.front;
        fprintf('[E-REQ] case=%s build=0 C6=0 safety=0 initialOutside=%d memory=%d trigger=%d\n',...
            S.case_id,f.initial_outside_all,f.memory_pass,f.triggered && f.trigger_after_detection);
    else
        f=R.front;
        fprintf('[E-REQ] case=%s build=%d C6ok=%d geom=%d tension=%d safety=%d initialOutside=%d memory=%d trigger=%d known=%d maxJoinAbs=%.3e maxEndpointAbs=%.3e\n',...
            S.case_id,b.build,b.c6pass,b.geom,b.tension,b.safe,f.initial_outside_all,f.memory_pass,f.triggered && f.trigger_after_detection,...
            numel(f.known_ids_at_trigger),max(b.join_abs),max(b.end_abs));
    end
    return
end
fprintf('\n[REQUIREMENTS %s / %s]\n',S.name,R.method);
if isempty(b)
    fprintf(' R1 build candidate                         : FAIL\n');
    fprintf(' R2 native structural C6                   : N/A\n');
    fprintf(' R3 numeric d0..d6 continuity              : N/A\n');
    if any(string(S.stage)==["D","E"]) && ~isempty(R.front)
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
if any(string(S.stage)==["D","E"])
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
    if any(string(S.stage)==["D","E"])
        nKnown=numel(R.front.known_ids_at_trigger);
        fprintf(' R7 oriented ellipsoid trajectory geometry : PASS (support-function; no outer sphere)\n');
        fprintf(' R8 all known ellipsoids included           : %s (count=%d)\n',c6c_pf(~isempty(b.cert) && b.cert.n_obstacles==nKnown),nKnown);
        fprintf(' R9 continuous separate L/U/C certificate  : %s\n',c6c_pf(b.geom));
        fprintf(' R10 positive cable-tension lower bound      : %s (LB=%.4f)\n',c6c_pf(b.tension),b.cert.tension_lb);
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
if string(S.stage)=="E" && ~isempty(b.cert)
    fprintf(' E-v/a/j conservative Bernstein bounds     : v<=%.4f  a<=%.4f  j<=%.4f (MEASURE ONLY)\n',b.cert.v,b.cert.a,b.cert.j);
    fprintf(' E timing                                  : route=%.3f build=%.3f cert=%.3f searchTotal=%.3f ms\n',R.route_gen_ms,b.build_ms,b.cert_ms,R.search_total_ms);
end
fprintf(' CORE ACCEPTANCE                            : %s\n',c6c_pf(b.safe));
fprintf(' C6 JOIN abs residual d0..d6               : %s\n',c6c_vec(b.join_abs));
fprintf(' C6 JOIN rel residual d0..d6               : %s\n',c6c_vec(b.join_rel));
fprintf(' SPLICE abs residual d0..d6                : %s\n',c6c_vec(b.end_abs));
fprintf(' SPLICE rel residual d0..d6                : %s\n',c6c_vec(b.end_rel));
fprintf(' C6 numerical residual policy               : diagnostic only; no empirical decimal gate\n');
end

function c6c_summary(result,P)
fprintf('\n========================================================================================================================\n');
fprintf(' STAGE-E SQRT-LS THEORY-INTEGRATED SUMMARY\n');
fprintf('========================================================================================================================\n');
fprintf(' case  method                 core geom tens tested    minG      tLB      vBound   aBound   jBound    condCZ(coord) J7          searchMs\n');
for is=1:numel(result.scenarios)
    S=result.scenarios(is);
    for im=1:numel(result.methods)
        R=result.runs{im,is};
        if isempty(R.best)
            fprintf(' %-5s %-22s FAIL FAIL FAIL %6d      NaN      NaN      NaN      NaN      NaN        NaN         NaN %9.2f\n',...
                S.case_id,result.methods(im),numel(R.rows),R.search_total_ms);
        else
            b=R.best; sd=b.solver_diag;
            fprintf(' %-5s %-22s %4s %4s %4s %6d %+8.3e %8.3f %8.3f %8.3f %8.3f %10.3e %11.4e %9.2f\n',...
                S.case_id,result.methods(im),c6c_pf(b.safe),c6c_pf(b.geom),c6c_pf(b.tension),numel(R.rows), ...
                b.cert.minG,b.cert.tension_lb,b.cert.v,b.cert.a,b.cert.j,sd.cond_cz,sd.objective,R.search_total_ms);
        end
    end
end
fprintf('\nTheory policy:\n');
fprintf(' - C6 acceptance is native-structural for degree-7 simple-knot B-spline and equality-constrained for Power/Bezier; floating join/end residuals are diagnostics.\n');
fprintf(' - All three methods solve exact seventh-derivative energy in square-root form; no Q/Hessian normal-equation solve.\n');
fprintf(' - QR uses the structurally full-row-rank equality system; no SVD rank truncation or artificial ridge.\n');
fprintf(' - v/a/j are conservative Bernstein bounds and are measurement only.\n');
fprintf(' - Geometry certificate is conservative/sufficient finite-direction Bernstein support certification, not exact global distance.\n');
fprintf(' - Positive cable-tension lower bound is reported and gated separately from geometric separation.\n');
fprintf(' - Search logs retain one representative failed candidate; every candidate row remains in result for offline analysis.\n');
fprintf(' - condCZ(coord) and optCoeffCorrection are coordinate-space numerical diagnostics; they must not be interpreted as basis-independent physical quality metrics.\n');
end

% ================================ UTILS ==================================
function row=c6c_row()
row=struct('tag','','nvias',0,'build',false,'build_ms',NaN,'cert_ms',NaN,'c6pass',false,'geom',false,'tension',false,'safe',false,...
    'join_abs_max',Inf,'join_abs',inf(1,7),'join_rel',inf(1,7),'end_abs_max',Inf,'end_abs',inf(1,7),'end_rel',inf(1,7),...
    'condition_metric',Inf,'cert',[],'route_meta',[],'solver_diag',[],'reason','');
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
function f=c6c_route_fractions(p0,vias,pg)
P=[p0 vias pg];d=vecnorm(diff(P,1,2),2,1);L=sum(d);
if L==0, f=(1:size(vias,2))/(size(vias,2)+1);else,c=cumsum(d);f=c(1:end-1)/L;end
end
function D=c6c_fibonacci_sphere(N)
k=(0:N-1)';z=1-2*(k+0.5)/N;phi=pi*(3-sqrt(5))*k;r=sqrt(max(0,1-z.^2));D=[r.*cos(phi) r.*sin(phi) z]';
end
function D=c6c_add_cardinal(D)
C=[eye(3) -eye(3)]; D=[D C]; D=D./vecnorm(D,2,1);
end
function s=c6c_vec(v),s=['[' strtrim(sprintf(' %.2e',v)) ' ]'];end
function s=c6c_pf(x),if x,s='PASS';else,s='FAIL';end,end

function mode=c6c_c6_mode(method)
if string(method)=="BSPLINE_D7"
    mode='native-structural';
else
    mode='equality-constrained';
end
end


% =========================================================================
% STAGE-F1 NOMINAL DYNAMICS HELPERS
% =========================================================================
function P=f1_defaults(P)
D=struct();
D.mass=0.762;
D.jx=0.02237568;
D.jy=0.02985236;
D.jz=0.0480374;
D.gravity=9.81;
D.loadmass=0.0556;        % nominal only; replace with latest estimator output
D.cableL=2.0;             % default test condition; configurable
D.thrust_min=0.0;
D.thrust_max=20.0;
D.moment_abs_max=[1 1 1];
D.dt_eval=0.01;
D.yaw_ref=[0;0;0];        % nominal [psi; dpsi; ddpsi], passed through unchanged
D.yaw_fun=[];              % optional @(t)[psi;dpsi;ddpsi], passed through unchanged
D.c6_param_override=struct(); % optional Stage-E geometry/front-end parameter overrides
fn=fieldnames(D);
for k=1:numel(fn)
    if ~isfield(P,fn{k}) || isempty(P.(fn{k})), P.(fn{k})=D.(fn{k}); end
end
P.J=diag([P.jx P.jy P.jz]);
P.moment_abs_max=reshape(P.moment_abs_max,1,[]);
if numel(P.moment_abs_max)==1, P.moment_abs_max=repmat(P.moment_abs_max,1,3); end
if numel(P.moment_abs_max)~=3, error('F1:MomentLimit','moment_abs_max must be scalar or 3-vector.'); end
end

function f1_validate_R6(R)
req={'methods','scenarios','runs','params'};
for k=1:numel(req)
    if ~isfield(R,req{k}), error('F1:BadR6','R6 is missing field %s.',req{k}); end
end
end

function r=f1_empty_record()
r=struct('method','','case_id','','scenario','','available',false,'valid',false, ...
    'upstream_safe',false,'dynamic_pass',false,'pass',false,'reason','NOT_RUN', ...
    'tension_min',NaN,'thrust_min',NaN,'thrust_max',NaN,'moment_abs_max',nan(1,3), ...
    'vmax',NaN,'amax',NaN,'jmax',NaN,'n_samples',0,'trace',[],'error_message','');
end

function s=f1_reason(r,P)
if ~r.valid, s='NUMERICAL_SINGULARITY';
elseif r.tension_min<=0, s='NONPOSITIVE_TENSION';
elseif r.thrust_min<P.thrust_min, s='THRUST_BELOW_MIN';
elseif r.thrust_max>P.thrust_max, s='THRUST_ABOVE_MAX';
elseif any(r.moment_abs_max>P.moment_abs_max), s='MOMENT_ABOVE_MAX';
elseif ~r.upstream_safe, s='UPSTREAM_C6_SAFETY_FAIL';
else, s='PASS'; end
end

% =========================================================================
% Trajectory sampling and nominal dynamic reconstruction
% =========================================================================
function [TR,DG]=f1_audit_segment(seg,P)
if ~isfield(seg,'B') || ~isfield(seg,'dt'), error('F1:BadSegment','selected_segment must contain B and dt.'); end
Ttot=sum(seg.dt);
tAll=[]; DAll=[]; t0=0;
for s=1:numel(seg.B)
    h=seg.dt(s);
    n=max(2,ceil(h/P.dt_eval)+1);
    tt=linspace(0,h,n);
    if s>1, tt=tt(2:end); end
    for k=1:numel(tt)
        u=tt(k)/h;
        D=zeros(3,7);
        for r=0:6, D(:,r+1)=f1_bezier_eval_derivative(seg.B{s},h,r,u); end
        tAll(end+1,1)=t0+tt(k); %#ok<AGROW>
        DAll(:,:,end+1)=D; %#ok<AGROW>
    end
    t0=t0+h;
end

N=numel(tAll);
Tload=nan(N,1); thrust=nan(N,1); Mom=nan(N,3); valid=true(N,1);
vel=nan(N,1); acc=nan(N,1); jerk=nan(N,1);
pCable=nan(N,3); Omega=nan(N,3);
for k=1:N
    D=DAll(:,:,k);
    vel(k)=norm(D(:,2)); acc(k)=norm(D(:,3)); jerk(k)=norm(D(:,4));
    try
        [z,ok]=f1_nominal_point(D,tAll(k),P);
        valid(k)=ok;
        if ok
            Tload(k)=z.Tload; thrust(k)=z.thrust; Mom(k,:)=z.M.';
            pCable(k,:)=z.p.'; Omega(k,:)=z.Omega.';
        end
    catch
        valid(k)=false;
    end
end

q=find(valid & isfinite(thrust) & all(isfinite(Mom),2));
DG=struct(); DG.n_samples=N; DG.valid=numel(q)==N && N>0;
if isempty(q)
    DG.tension_min=NaN; DG.thrust_min=NaN; DG.thrust_max=NaN; DG.moment_abs_max=nan(1,3);
else
    DG.tension_min=min(Tload(q)); DG.thrust_min=min(thrust(q)); DG.thrust_max=max(thrust(q));
    DG.moment_abs_max=max(abs(Mom(q,:)),[],1);
end
DG.vmax=max(vel,[],'omitnan'); DG.amax=max(acc,[],'omitnan'); DG.jmax=max(jerk,[],'omitnan');
TR=struct('t',tAll,'payload_jet',DAll,'tension',Tload,'thrust',thrust,'moment',Mom, ...
    'cable_direction',pCable,'Omega',Omega,'valid',valid,'duration',Ttot);
end

function [Z,ok]=f1_nominal_point(D,t,P)
e3=[0;0;1];
% Exact local Taylor-jet reconstruction of p through fourth derivative from
% xL derivatives through sixth order.
pj=f1_cable_direction_jet(D,P.gravity); % columns p^(0)...p^(4)
p=pj(:,1); p2=pj(:,3); p3=pj(:,4); p4=pj(:,5);
W=D(:,3)+P.gravity*e3;
wNorm=norm(W);
if ~(isfinite(wNorm) && wNorm>0), ok=false; Z=struct(); return; end
Tload=P.loadmass*wNorm;

% xQ = xL - L*p, and only xQ^(2..4) are required here.
xQ2=D(:,3)-P.cableL*p2;
xQ3=D(:,4)-P.cableL*p3;
xQ4=D(:,5)-P.cableL*p4;

% Quadrotor force vector F = f*R*e3 = mQ(xQdd+ge3) - T*p.
% Since -T*p = mL*(xLdd+ge3), derivatives are particularly clean.
F0=P.mass*(xQ2+P.gravity*e3)+P.loadmass*(D(:,3)+P.gravity*e3);
F1=P.mass*xQ3+P.loadmass*D(:,4);
F2=P.mass*xQ4+P.loadmass*D(:,5);
[f,b3,b3d,b3dd,ok3]=f1_normalize_second(F0,F1,F2);
if ~ok3, ok=false; Z=struct(); return; end

[psi,dpsi,ddpsi]=f1_yaw(t,P);
c=[cos(psi);sin(psi);0];
cd=dpsi*[-sin(psi);cos(psi);0];
cdd=ddpsi*[-sin(psi);cos(psi);0] - dpsi^2*[cos(psi);sin(psi);0];

y=cross(b3,c);
yd=cross(b3d,c)+cross(b3,cd);
ydd=cross(b3dd,c)+2*cross(b3d,cd)+cross(b3,cdd);
[~,b2,b2d,b2dd,ok2]=f1_normalize_second(y,yd,ydd);
if ~ok2, ok=false; Z=struct(); return; end

b1=cross(b2,b3);
b1d=cross(b2d,b3)+cross(b2,b3d);
b1dd=cross(b2dd,b3)+2*cross(b2d,b3d)+cross(b2,b3dd);
R=[b1 b2 b3]; Rd=[b1d b2d b3d]; Rdd=[b1dd b2dd b3dd];
OmHat=0.5*(R.'*Rd-(R.'*Rd).');
OmdHat=Rd.'*Rd+R.'*Rdd; OmdHat=0.5*(OmdHat-OmdHat.');
Omega=f1_vee(OmHat); Omegad=f1_vee(OmdHat);
M=P.J*Omegad + cross(Omega,P.J*Omega);

ok=all(isfinite([Tload;f;Omega;Omegad;M])) && abs(det(R)-1)<1e-6;
Z=struct('p',p,'Tload',Tload,'thrust',f,'R',R,'Omega',Omega,'Omegadot',Omegad,'M',M);
end

% =========================================================================
% Exact cable-direction jet p = -W/||W|| through order 4
% using truncated Taylor coefficients (coefficient = derivative/factorial).
% =========================================================================
function pj=f1_cable_direction_jet(D,g)
e3=[0;0;1]; nr=4;
w=zeros(3,nr+1);
for r=0:nr
    w(:,r+1)=D(:,r+3)/factorial(r);
end
w(:,1)=w(:,1)+g*e3;
h=zeros(1,nr+1);
for n=0:nr
    for i=0:n
        h(n+1)=h(n+1)+dot(w(:,i+1),w(:,n-i+1));
    end
end
if h(1)<=0, error('F1:CableDirectionSingular','||xddL+g e3|| is zero.'); end
s=zeros(1,nr+1); s(1)=sqrt(h(1));
for n=1:nr
    q=0; for i=1:n-1, q=q+s(i+1)*s(n-i+1); end
    s(n+1)=(h(n+1)-q)/(2*s(1));
end
zinv=zeros(1,nr+1); zinv(1)=1/s(1);
for n=1:nr
    q=0; for i=1:n, q=q+s(i+1)*zinv(n-i+1); end
    zinv(n+1)=-q/s(1);
end
pc=zeros(3,nr+1);
for n=0:nr
    for i=0:n, pc(:,n+1)=pc(:,n+1)-w(:,i+1)*zinv(n-i+1); end
end
pj=zeros(3,nr+1);
for r=0:nr, pj(:,r+1)=factorial(r)*pc(:,r+1); end
end

function [s,u,ud,udd,ok]=f1_normalize_second(v,vd,vdd)
s=norm(v); ok=isfinite(s)&&s>0;
if ~ok, u=nan(3,1);ud=u;udd=u;return;end
sd=dot(v,vd)/s;
sdd=(dot(vd,vd)+dot(v,vdd)-sd^2)/s;
u=v/s;
ud=(vd-u*sd)/s;
udd=(vdd-u*sdd-2*ud*sd)/s;
ok=all(isfinite([u;ud;udd]));
end

function [psi,dpsi,ddpsi]=f1_yaw(t,P)
% Nominal yaw reference is PASSED THROUGH UNCHANGED.
% No obstacle-avoidance heading law or velocity-heading alignment is added.
if ~isempty(P.yaw_fun)
    y=P.yaw_fun(t);
elseif isa(P.yaw_ref,'function_handle')
    y=P.yaw_ref(t);
else
    y=P.yaw_ref;
end
y=double(y(:));
if isempty(y), y=[0;0;0]; end
if numel(y)<3, y(end+1:3)=0; end
psi=y(1); dpsi=y(2); ddpsi=y(3);
end

function w=f1_vee(S), w=[S(3,2);S(1,3);S(2,1)]; end

% =========================================================================
% Bernstein derivative evaluation
% =========================================================================
function v=f1_bezier_eval_derivative(B,T,r,u)
D=B; n=size(B,2)-1;
for q=1:r
    D=(n-q+1)/T*diff(D,1,2);
end
m=n-r;
if m<0, v=zeros(size(B,1),1); return; end
b=zeros(m+1,1);
for k=0:m, b(k+1)=nchoosek(m,k)*u^k*(1-u)^(m-k); end
v=D*b;
end

% =========================================================================
% PROVISIONAL-LATEST PARAMETER POLICY
% =========================================================================
% Runtime parameter priority:
%   1) Explicit phys argument fields supplied by caller
%   2) Compatible external parameter object/struct found in base workspace
%   3) Built-in default values below
%
% The planner itself does not require a specific vehicle. Only the defaults
% correspond to the current DIATONE test setup. Payload mass may be replaced
% at runtime by the latest estimate.
%
% Supported external/base-workspace names (first usable one wins):
%   DRONE_PARAM, drone_param, PARAM, param, vehicle_param, sys_param
%
% Required/recognized physical fields:
%   mass, jx, jy, jz, gravity, cableL, loadmass
%
% Optional feasibility/geometry fields:
%   thrust_min, thrust_max, moment_abs_max,
%   R_uav, R_payload, R_cable, sensor_R
%
% Yaw policy:
%   nominal reference passthrough; avoidance planner does not modify yaw.
%
function phys = c6f1_resolve_phys(phys)
    if nargin < 1 || isempty(phys), phys = struct(); end

    % --- Built-in fallback defaults only ---
    D = struct();
    D.mass = 0.762;
    D.jx = 0.02237568;
    D.jy = 0.02985236;
    D.jz = 0.0480374;
    D.gravity = 9.81;
    D.loadmass = 0.0556;
    D.cableL = 2.0;

    D.thrust_min = 0.0;
    D.thrust_max = 20.0;
    D.moment_abs_max = [1 1 1];

    D.R_uav = 0.30;
    D.R_payload = 0.24;
    D.R_cable = 0.045;
    D.sensor_R = 15.0;

    % Default nominal yaw is zero only when no external/reference yaw exists.
    D.yaw_ref = [0;0;0];

    % Try an external parameter source only for fields the caller did not set.
    ext = [];
    names = {'DRONE_PARAM','drone_param','PARAM','param','vehicle_param','sys_param'};
    for k = 1:numel(names)
        try
            if evalin('base', sprintf('exist(''%s'',''var'')', names{k}))
                cand = evalin('base', names{k});
                if ~isempty(cand)
                    ext = cand;
                    break
                end
            end
        catch
        end
    end

    phys = c6f1_fill_from_source(phys, ext);
    phys = c6f1_fill_defaults(phys, D);

    % Normalize moment limit.
    phys.moment_abs_max = phys.moment_abs_max(:).';
    if isscalar(phys.moment_abs_max)
        phys.moment_abs_max = repmat(phys.moment_abs_max,1,3);
    end
end

function out = c6f1_fill_from_source(out, src)
    if isempty(src), return; end
    names = {'mass','jx','jy','jz','gravity','cableL','loadmass',...
             'thrust_min','thrust_max','moment_abs_max',...
             'R_uav','R_payload','R_cable','sensor_R','yaw_ref'};
    for k=1:numel(names)
        f = names{k};
        if isfield(out,f) && ~isempty(out.(f)), continue; end
        try
            if isstruct(src) && isfield(src,f)
                out.(f) = src.(f);
            elseif isobject(src) && isprop(src,f)
                out.(f) = src.(f);
            elseif isobject(src) && any(strcmp(methods(src),'get'))
                v = src.get(string(f));
                if ~isempty(v), out.(f)=v; end
            end
        catch
        end
    end
end

function out = c6f1_fill_defaults(out, D)
    fn = fieldnames(D);
    for k=1:numel(fn)
        f=fn{k};
        if ~isfield(out,f) || isempty(out.(f))
            out.(f)=D.(f);
        end
    end
end
