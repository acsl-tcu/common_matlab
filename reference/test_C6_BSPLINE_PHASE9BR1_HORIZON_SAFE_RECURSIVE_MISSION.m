function result = test_C6_BSPLINE_PHASE9BR1_HORIZON_SAFE_RECURSIVE_MISSION()
% =========================================================================
% Phase 9B-R1: HORIZON-SAFE RECURSIVE MULTI-OBSTACLE REGRESSION
%
% Frozen candidate generator:
%   Phase 9A-G cluster-side compatible corridor
%   + automatic horizon
%   + feasibility QP
%   + at most one verifier-driven cut
%   + unchanged independent continuous-time certificate
%
% Mission logic:
%   sensor -> persistent obstacle memory -> future-risk predictor
%   -> replan from CURRENT EXECUTING CERTIFIED TRAJECTORY
%   -> independent certificate + C6 gate -> atomic switch
%   -> recursive replan if another obstacle threatens current avoidance
%   -> nominal C6 rejoin.
%
% New-obstacle reveal times emulate obstacles becoming newly observable in
% an unknown/occluded environment.  Once observed, static obstacles remain
% permanently in memory.  The sensor itself is the production-shaped
% 15-m ellipsoid range test already used in the Phase-7 production front end.
%
% This is an OFFLINE regression, not the <=3 ms benchmark.
% =========================================================================
clc;
scenario=setup_benchmark();
[cfg,limits,obs0,~,~,~]=unpack_scenario(scenario);
opt=setup_options(); alpha=1/sqrt(3);

% Freeze Phase 9A-G settings.
opt.coherent_lambda=1.25;
opt.coherent_flip_cos=-0.15;
opt.coherent_topK=min(48,opt.separator_grid_N);
opt.compat_enable=true;
opt.compat_min_dot=0.05;
opt.compat_lambda=1.00;
opt.geom_guide_enable=true;
opt.geom_guide_lambda=0.85;
opt.geom_guide_min_dot=0.35;
opt.quality_enable=false;

sensor_range=15.0;
Tlook=5.0; Npred=11; Nsub=4;
dt=0.05; t0=0; tf=72;

% 10 deterministic missions.  No per-mission planner tuning.
Mdef={ ...
 'STRAIGHT',   .75,'SAME_SIDE',3; ...
 'STRAIGHT',  1.25,'ZIGZAG',  3; ...
 'CIRCLE',     .75,'SAME_SIDE',3; ...
 'CIRCLE',    1.00,'ZIGZAG',  3; ...
 'HELIX',      .75,'SAME_SIDE',3; ...
 'HELIX',     1.25,'ZIGZAG',  3; ...
 'S_CURVE',   1.00,'SAME_SIDE',3; ...
 'S_CURVE',   1.25,'ZIGZAG',  3; ...
 'CLIMB_TURN',.75,'SAME_SIDE',3; ...
 'CLIMB_TURN',1.25,'ZIGZAG',  3};

fprintf('================================================================================\n');
fprintf(' Phase 9B-R1: HORIZON-SAFE RECURSIVE MISSIONS -- %d missions\n',size(Mdef,1));
fprintf(' frozen 9A-G corridor + sensor/memory/trigger + current-certified recursive splice\n');
fprintf('================================================================================\n');

R=repmat(p9b_empty_mission(),1,size(Mdef,1));
rep=struct();

for im=1:size(Mdef,1)
    fam=Mdef{im,1}; sf=Mdef{im,2}; layout=Mdef{im,3}; nobs=Mdef{im,4};
    nom_fun=@(t) p9a_nominal(t,fam,sf);
    obs=p9b_make_recursive_obstacles(obs0,nom_fun,cfg,nobs,layout,im);
    reveal=[0 29 41]; reveal=reveal(1:nobs);

    active=struct('valid',false,'t_start',NaN,'t_end',NaN,'plan',p5r2_empty_plan());
    memory=false(1,nobs);
    first_seen=nan(1,nobs);
    gen=0; ntrigger=0; nswitch=0; nrecursive=0; nrefine=0; nqp=0;
    execute=true; reason='MISSION_COMPLETE';
    events=repmat(p9b_empty_event(),1,0);
    tvec=t0:dt:tf; path=nan(3,numel(tvec));
    next_retry=-inf;

    fprintf('\n[%02d/%02d] %-10s sf=%.2f %-9s nobs=%d\n', ...
        im,size(Mdef,1),fam,sf,layout,nobs);

    for kt=1:numel(tvec)
        t=tvec(kt);

        % Certified avoidance automatically rejoins nominal at its C6 endpoint.
        if active.valid && t>active.t_end+1e-9
            active.valid=false;
        end

        exec_fun=@(tt) p2_exec_D(tt,active,nom_fun);
        D=exec_fun(t); path(:,kt)=D(:,1);
        W=D(:,3)+cfg.g_acc*cfg.e3;
        if norm(W)<=opt.a_net_min, nT=cfg.e3; else, nT=W/norm(W); end
        pQ=D(:,1)+cfg.sys.L_cable*nT;

        % Production-shaped sensor. Obstacles not yet revealed emulate
        % previously unobservable static environment; detected static objects
        % are retained permanently in memory.
        avail=find(t>=reveal-1e-12);
        if ~isempty(avail)
            detloc=prod_sensor_detect(pQ,obs(avail),sensor_range);
            if ~isempty(detloc.ids)
                ids=avail(detloc.ids);
                new=ids(~memory(ids));
                for jj=new
                    memory(jj)=true; first_seen(jj)=t;
                    fprintf('  DETECT O%d t=%.2f -> MEMORY RETAINED\n',jj,t);
                end
            end
        end

        known=find(memory);
        if isempty(known) || t<next_retry, continue; end

        % Trigger predicts CURRENT EXECUTING trajectory, never nominal-only
        % while an avoidance is active.
        Tpred=min(Tlook,max(0,tf-t));
        if Tpred<=0, continue; end
        ev=prod_future_trigger_nominal(t,known,obs,exec_fun,cfg,Tpred,Npred,Nsub);
        if ev.found && ev.t_risk>tf+1e-10
            error('Phase9BR1:RiskBeyondMission','Predictor returned t_risk > tf.');
        end
        if ~ev.found, continue; end
        ntrigger=ntrigger+1;

        source='NOMINAL';
        if active.valid, source='CURRENT_CERTIFIED_AVOIDANCE'; end
        fprintf('  TRIGGER #%d t=%.2f risk=%.2f TTC=%.2f O%d source=%s\n', ...
            ntrigger,t,ev.t_risk,ev.TTC,ev.obs_idx,source);

        % Snapshot current certified trajectory BEFORE planning.  This is the
        % immutable reference/start state for this replan attempt.
        old_active=active;
        ref_fun=@(tt) p2_exec_D(tt,old_active,nom_fun);
        Dstart=ref_fun(t);

        knownobs=obs(known);
        risk=audit_nominal_risk_window(ref_fun,knownobs,cfg,opt,t,min(tf,t+35),.04);
        er=p9b_empty_event();
        er.t=t; er.risk_t=ev.t_risk; er.TTC=ev.TTC; er.obs_idx=ev.obs_idx;
        er.source=source; er.generation_before=gen; er.known_count=numel(known);

        if ~isfinite(risk.t_clear)
            er.reason='NO_CLEAR_TIME_IN_LOOKAHEAD_AUDIT';
            events(end+1)=er; %#ok<AGROW>
            execute=false; reason=er.reason;
            fprintf('  EXECUTE=FALSE: %s\n',reason); break;
        end

        H=p6_predict_horizon(t,Dstart,risk,knownobs,ref_fun,cfg,limits,opt);
        T0=H.T_plan;
        Ttry=unique([1.10*T0 1.25*T0 1.50*T0],'stable');
        N=max(30,min(42,ceil(2*T0))); if mod(N,2), N=N+1; end
        cfgN=cfg; cfgN.N_ctrl=N;
        er.T0=T0; er.N=N;

        cand=p5r2_empty_plan(); Tsel=NaN;
        for ih=1:numel(Ttry)
            T=Ttry(ih);
            if T<=0 || t+T>tf, continue; end
            Dend=nom_fun(t+T); % delayed nominal rejoin; exact C6 endpoint
            cand=p5r2_plan_once(cfgN,limits,knownobs,ref_fun,nom_fun, ...
                t,T,Dstart,Dend,opt,alpha);
            er.attempts=er.attempts+1;
            nqp=nqp+cand.n_qp; nrefine=nrefine+cand.n_refine;
            if cand.cert
                Tsel=T; break;
            end
        end

        er.cert=cand.cert; er.reason=cand.reason; er.Tsel=Tsel;
        er.min_g=cand.min_g; er.c6s=cand.c6s; er.c6e=cand.c6e;
        er.v=cand.v; er.a=cand.a; er.j=cand.j;
        er.qp_calls=cand.n_qp; er.refinements=cand.n_refine;
        er.primary_qp_ms=cand.primary_qp_ms;

        if ~cand.cert
            events(end+1)=er; %#ok<AGROW>
            execute=false; reason=['BACKUP_REQUIRED_' cand.reason];
            fprintf('  EXECUTE=FALSE: %s\n',reason); break;
        end

        % Atomic certified switch.  p5r2_plan_once already requires both
        % independent certificate and start/end C6 tolerance.
        was_recursive=old_active.valid;
        gen=gen+1; nswitch=nswitch+1;
        if was_recursive, nrecursive=nrecursive+1; end
        active=struct('valid',true,'t_start',t,'t_end',t+Tsel,'plan',cand);
        er.generation_after=gen; er.recursive=was_recursive;
        events(end+1)=er; %#ok<AGROW>

        fprintf('  SWITCH gen=%d recursive=%d T=%.3f end=%.3f g=%+.4f C6s/e=%.2e/%.2e v/a/j=%.2f/%.2f/%.2f\n', ...
            gen,was_recursive,Tsel,t+Tsel,cand.min_g,cand.c6s,cand.c6e,cand.v,cand.a,cand.j);

        % Do not retrigger on the same instant; next cycles still evaluate
        % the newly executing certified trajectory against persistent memory.
        next_retry=t+2*dt;
    end

    % Final state and mission-level hard gates.
    final_nominal=~active.valid || tf>active.t_end;
    cert_events=[events.cert];
    if isempty(cert_events), all_switch_cert=false; else, all_switch_cert=all(cert_events([events.generation_after]>0)); end
    switched=nswitch>=1;
    recursive_ok=nrecursive>=1;
    c6max=0; ming=inf; vmax=0; amax=0; jmax=0;
    if ~isempty(events)
        ids=find([events.cert]);
        if ~isempty(ids)
            c6max=max([[events(ids).c6s] [events(ids).c6e]]);
            ming=min([events(ids).min_g]);
            vmax=max([events(ids).v]); amax=max([events(ids).a]); jmax=max([events(ids).j]);
        end
    end

    pass=execute && switched && recursive_ok && all_switch_cert && ...
         c6max<=opt.c6_tol && final_nominal;

    rr=p9b_empty_mission();
    rr.id=im; rr.family=fam; rr.speed=sf; rr.layout=layout; rr.nobs=nobs;
    rr.pass=pass; rr.execute=execute; rr.reason=reason;
    rr.detections=nnz(memory); rr.triggers=ntrigger; rr.switches=nswitch;
    rr.recursive_switches=nrecursive; rr.generations=gen;
    rr.refinements=nrefine; rr.qp_calls=nqp; rr.first_seen=first_seen;
    rr.memory_final=memory; rr.final_nominal=final_nominal;
    rr.min_g=ming; rr.max_c6=c6max; rr.max_v=vmax; rr.max_a=amax; rr.max_j=jmax;
    rr.events=events;
    if ~isempty(events)
        rt=[events.risk_t]; rt=rt(isfinite(rt));
        if ~isempty(rt), rr.max_risk_time=max(rt); end
        rr.horizon_violations=sum(rt>tf+1e-10);
    end
    R(im)=rr;

    fprintf('  RESULT pass=%d execute=%d detect=%d/%d trigger=%d switch=%d recursive=%d final_nominal=%d reason=%s\n', ...
        pass,execute,rr.detections,nobs,ntrigger,nswitch,nrecursive,final_nominal,reason);

    if im==2
        rep=struct('t',tvec,'path',path,'obs',obs,'family',fam,'speed',sf,'layout',layout,'events',events);
    end
end

pass=[R.pass];
fprintf('\n================================================================================\n');
fprintf('[PHASE 9B-R1 SUMMARY]\n');
fprintf('================================================================================\n');
fprintf('Mission PASS = %d / %d (%.1f%%)\n',nnz(pass),numel(R),100*mean(pass));
fprintf('Total detections / triggers / switches / recursive switches = %d / %d / %d / %d\n', ...
    sum([R.detections]),sum([R.triggers]),sum([R.switches]),sum([R.recursive_switches]));
fprintf('Mission-horizon predictor violations = %d (required 0)\n',sum([R.horizon_violations]));
if any(pass)
    C=R(pass);
    fprintf('Certified mission envelope: min_g=%+.6f, maxC6=%.3e, max v/a/j=%.3f/%.3f/%.3f\n', ...
        min([C.min_g]),max([C.max_c6]),max([C.max_v]),max([C.max_a]),max([C.max_j]));
end
if any(~pass)
    fprintf('Failed missions retained:\n');
    for k=find(~pass)
        fprintf('  #%02d %-10s sf=%.2f %-9s : %s (switch=%d recursive=%d)\n', ...
            R(k).id,R(k).family,R(k).speed,R(k).layout,R(k).reason,R(k).switches,R(k).recursive_switches);
    end
end

% Exactly one integrated 3-D figure.
if ~isempty(fieldnames(rep))
    figure('Name','Phase 9B-R1 representative recursive mission','Color','w');
    P=rep.path; ok=all(isfinite(P),1);
    plot3(P(1,ok),P(2,ok),P(3,ok),'LineWidth',1.8); hold on; grid on; axis equal;
    for io=1:numel(rep.obs), draw_ellipsoid_local(rep.obs(io)); end
    xlabel('x [m]'); ylabel('y [m]'); zlabel('z [m]');
    title(sprintf('Phase 9B-R1 representative: %s sf=%.2f %s',rep.family,rep.speed,rep.layout));
end

result=struct('pass_all',all(pass),'passed',nnz(pass),'total',numel(R), ...
    'rate',mean(pass),'missions',R,'representative',rep);
end

function r=p9b_empty_mission()
r=struct('id',0,'family','','speed',NaN,'layout','','nobs',0, ...
    'pass',false,'execute',false,'reason','NOT_RUN','detections',0, ...
    'triggers',0,'switches',0,'recursive_switches',0,'generations',0, ...
    'refinements',0,'qp_calls',0,'first_seen',[],'memory_final',[], ...
    'final_nominal',false,'min_g',inf,'max_c6',inf,'max_v',inf,'max_a',inf,'max_j',inf, ...
    'events',repmat(p9b_empty_event(),1,0),'max_risk_time',-inf,'horizon_violations',0);
end

function e=p9b_empty_event()
e=struct('t',NaN,'risk_t',NaN,'TTC',NaN,'obs_idx',NaN,'source','', ...
    'generation_before',0,'generation_after',0,'known_count',0, ...
    'T0',NaN,'Tsel',NaN,'N',0,'attempts',0,'cert',false,'reason','NOT_RUN', ...
    'min_g',-inf,'c6s',inf,'c6e',inf,'v',inf,'a',inf,'j',inf, ...
    'qp_calls',0,'refinements',0,'primary_qp_ms',0,'recursive',false);
end

function obs=p9b_make_recursive_obstacles(obs0,nom_fun,cfg,nobs,layout,mission_id)
% Obstacles lie on later nominal system positions.  Geometry is deterministic.
% Reveal timing is handled by the mission loop, not by moving obstacles.
hit=[30 42 54 64]; hit=hit(1:nobs);
obs=repmat(obs0(1),1,nobs);
for io=1:nobs
    D=nom_fun(hit(io)); W=D(:,3)+cfg.g_acc*cfg.e3;
    if norm(W)<1e-12, nT=cfg.e3; else, nT=W/norm(W); end
    tangent=D(:,2);
    if norm(tangent)<1e-9, tangent=[1;0;0]; else, tangent=tangent/norm(tangent); end
    side=cross(tangent,cfg.e3);
    if norm(side)<1e-9, side=[1;0;0]; else, side=side/norm(side); end
    if strcmp(layout,'ZIGZAG'), sg=(-1)^io; else, sg=1; end
    off=sg*(0.20+0.05*mod(mission_id+io,3))*side;
    obs(io).center=D(:,1)+cfg.sys.L_cable*nT+off;
    scale=0.85+0.10*mod(mission_id+io,3);
    shape=[1.15;0.95;1.25];
    if mod(io,2)==0, shape=[1.35;0.80;1.10]; end
    obs(io).radii=scale*shape;
    ang=[mod(31*mission_id+37*io,180) -20+8*io -15+11*io]*pi/180;
    obs(io).R=eul2rotm_local(ang);
    obs(io).S=obs(io).R*diag(obs(io).radii.^2)*obs(io).R';
end
end

function r=p9ag_density_empty()
r=struct('layout','','nobs',0,'T0',NaN,'Tsel',NaN,'attempts',0, ...
    'cert',false,'min_g',-inf,'reason','NOT_RUN');
end

function obs=p9ag_dense_obstacles(obs0,nom_fun,cfg,no,layout)
obs=repmat(obs0(1),1,no);
switch layout
    case 'CHAIN'
        for k=1:no
            tt=29+2.6*(k-1); D=nom_fun(tt); nT=p9ag_nt(D,cfg);
            obs(k)=p9ag_set_obs(obs(k),D(:,1)+cfg.sys.L_cable*nT, ...
                [1.05;0.85;1.15],[0.17*k -0.08*k 0.11*k]);
        end
    case 'ZIGZAG'
        for k=1:no
            tt=29+2.2*(k-1); D=nom_fun(tt); nT=p9ag_nt(D,cfg);
            tv=D(:,2); tv=tv/max(norm(tv),1e-12);
            side=cross(tv,cfg.e3);
            if norm(side)<1e-9, side=[1;0;0]; else, side=side/norm(side); end
            off=(-1)^k*0.65*side;
            obs(k)=p9ag_set_obs(obs(k),D(:,1)+cfg.sys.L_cable*nT+off, ...
                [1.10;0.80;1.20],[0.23*k 0.09*(-1)^k -0.14*k]);
        end
    case 'SURROUND'
        D=nom_fun(32); nT=p9ag_nt(D,cfg); ctr=D(:,1)+cfg.sys.L_cable*nT;
        phi=(1+sqrt(5))/2;
        for k=1:no
            z=1-2*(k-.5)/no; th=2*pi*k/phi; rr=sqrt(max(0,1-z*z));
            dir=[rr*cos(th);rr*sin(th);z];
            obs(k)=p9ag_set_obs(obs(k),ctr+2.15*dir,[1.00;0.85;1.10], ...
                [0.19*k -0.07*k 0.13*k]);
        end
    otherwise
        error('Unknown density layout');
end
end

function nT=p9ag_nt(D,cfg)
W=D(:,3)+cfg.g_acc*cfg.e3;
if norm(W)<1e-12, nT=cfg.e3; else, nT=W/norm(W); end
end

function o=p9ag_set_obs(o,c,r,eul)
o.center=c; o.radii=r; o.R=eul2rotm_local(eul);
o.S=o.R*diag(o.radii.^2)*o.R';
end

function r=p9a_empty_record()
r=struct('case_id',0,'family','','speed',NaN,'nobs',0,'shape_id',0,'scale',NaN, ...
    'euler_deg',[NaN NaN NaN],'tplan',NaN,'tclear',NaN,'T0',NaN,'Tsel',NaN,'N',NaN, ...
    'attempts',0,'qp_calls',0,'refinements',0,'primary_qp_ms',NaN,'wall_ms',NaN, ...
    'cert',false,'reason','NOT_RUN','min_g',-inf,'c6s',inf,'c6e',inf, ...
    'v',inf,'a',inf,'j',inf,'maxdev',NaN,'geom_required',NaN,'excess_ratio',NaN, ...
    'dom_payload',0,'dom_cable',0,'dom_uav',0);
end

function p9a_print_case(r,ncases)
fprintf('[%02d/%02d] %-10s sf=%.2f nobs=%d shape=%d scale=%.2f ', ...
    r.case_id,ncases,r.family,r.speed,r.nobs,r.shape_id,r.scale);
if r.cert
    fprintf('CERT T0=%.2f T=%.2f N=%d g=%+.3f C6=%.1e/%.1e dev=%.2f geom=%.2f ratio=%.2f QP=%d %.2fms\n', ...
        r.T0,r.Tsel,r.N,r.min_g,r.c6s,r.c6e,r.maxdev,r.geom_required,r.excess_ratio,r.qp_calls,r.primary_qp_ms);
else
    fprintf('FAIL T0=%.2f N=%d QP=%d reason=%s\n',r.T0,r.N,r.qp_calls,r.reason);
end
end

function obs=p9a_make_obstacles(obs0,nom_fun,cfg,nobs,thit,shape_id,scale,eul,case_id)
base_shapes={[1 1 1],[1.6 .65 1.0],[.65 1.65 .85],[.75 .75 1.8]};
shape=base_shapes{shape_id}(:);
obs=repmat(obs0(1),1,nobs);
for io=1:nobs
    ti=thit+4.0*(io-1);
    D=nom_fun(ti);
    W=D(:,3)+cfg.g_acc*cfg.e3;
    if norm(W)<1e-12, nT=cfg.e3; else, nT=W/norm(W); end

    % Deterministic small offsets prevent every test being the same centered
    % collision while retaining genuinely threatening encounters.
    offmag=0.18*mod(case_id+io,3);
    tangent=D(:,2); 
    if norm(tangent)<1e-9, tangent=[1;0;0]; else, tangent=tangent/norm(tangent); end
    side=cross(tangent,cfg.e3);
    if norm(side)<1e-9, side=[1;0;0]; else, side=side/norm(side); end
    sgn=1-2*mod(case_id+io,2);
    offset=sgn*offmag*side;

    obs(io).center=D(:,1)+cfg.sys.L_cable*nT+offset;
    obs(io).radii=scale*[1.15;0.95;1.25].*shape;
    ang=eul+[0.21*(io-1) -0.13*(io-1) 0.17*(io-1)];
    obs(io).R=eul2rotm_local(ang);
    obs(io).S=obs(io).R*diag(obs(io).radii.^2)*obs(io).R';
end
end

function D=p9a_nominal(t,family,sf)
% Analytic C6 nominal families.  Speed factor is implemented by time scaling:
% p_sf(t)=p_base(sf*t), so r-th derivative scales as sf^r.
tau=sf*t;
B=p9a_nominal_base(tau,family);
D=zeros(3,7);
for r=0:6, D(:,r+1)=sf^r*B(:,r+1); end
end

function D=p9a_nominal_base(t,family)
D=zeros(3,7);
switch family
    case 'STRAIGHT'
        D(:,1)=[0.16*t;0;14+0.42*t];
        D(:,2)=[0.16;0;0.42];

    case 'CIRCLE'
        w=.13; R=4.0;
        D(1:2,:)=p9a_xy_harmonic(t,R,w,0);
        D(3,1)=18; % level circle

    case 'HELIX'
        w=.12; R=3.5;
        D(1:2,:)=p9a_xy_harmonic(t,R,w,0);
        D(3,1)=14+.28*t; D(3,2)=.28;

    case 'S_CURVE'
        w=.16;
        D(1,1)=.18*t; D(1,2)=.18;
        for r=0:6
            D(2,r+1)=1.8*w^r*sin(w*t+r*pi/2);
        end
        D(3,1)=14+.35*t; D(3,2)=.35;

    case 'CLIMB_TURN'
        w=.11; R=3.0;
        D(1:2,:)=p9a_xy_harmonic(t,R,w,pi/5);
        % Smooth vertical sinusoid superposed on climb.
        for r=0:6
            if r==0
                D(3,1)=14+.25*t+1.0*sin(.09*t);
            elseif r==1
                D(3,2)=.25+1.0*.09*cos(.09*t);
            else
                D(3,r+1)=1.0*.09^r*sin(.09*t+r*pi/2);
            end
        end
    otherwise
        error('Unknown nominal family %s',family);
end
end

function XY=p9a_xy_harmonic(t,R,w,phase)
XY=zeros(2,7);
for r=0:6
    a=w*t+phase+r*pi/2;
    XY(1,r+1)=R*w^r*cos(a);
    XY(2,r+1)=R*w^r*sin(a);
end
end


function q=p5r2_empty_plan()
% Fixed schema used before the first automatic-horizon attempt.
q=struct('cert',false,'reason','NO_CANDIDATE_ATTEMPTED','P',[],'cache',[], ...
    'obstacles',[],'n_qp',0,'n_refine',0,'min_g',-inf, ...
    'v',inf,'a',inf,'j',inf,'c6s',inf,'c6e',inf,'verify',false, ...
    'quality_applied',false,'quality_lambda',0,'quality_qp',0,'quality_ms',0, ...
    'quality_before',p8_empty_metrics(),'quality_after',p8_empty_metrics(),'primary_qp_ms',0);
end

function q=p5r2_plan_once(cfg,limits,obs,ref_fun,nom_fun,t0,T,Dstart,Dend,opt,alpha)
% One initial combined QP + at most one verifier-driven refinement QP.
q=struct('cert',false,'reason','INIT','P',[],'cache',[],'obstacles',obs, ...
    'n_qp',0,'n_refine',0,'min_g',-inf,'v',inf,'a',inf,'j',inf, ...
    'c6s',inf,'c6e',inf,'verify',false, ...
    'quality_applied',false,'quality_lambda',0,'quality_qp',0,'quality_ms',0, ...
    'quality_before',p8_empty_metrics(),'quality_after',p8_empty_metrics(),'primary_qp_ms',0);
try
    c=build_static_cache_boundary(cfg,t0,t0+T,ref_fun,Dstart,Dend,opt);
    md=build_dyn_E(c); [Dmat,dvec]=dyn_E(md,limits,alpha);
    [G,h,~]=build_coherent_guided_corridor(c,cfg,obs,opt);
    A=G; b=h;

    for it=0:1
        fc=p8r6_feas_online(c,[A;Dmat],[b;dvec],opt); q.n_qp=q.n_qp+1; q.primary_qp_ms=q.primary_qp_ms+fc.solve_ms;
        if fc.flag<=0 || isempty(fc.z)
            q.reason='COMBINED_INFEASIBLE'; return;
        end
        P=apply_z(c,fc.z);
        cert=certify_candidate_global(P,c,cfg,limits,obs,opt);
        [c6s,c6e]=p5r2_c6(P,c,Dstart,Dend);
        q.min_g=cert.min_g_lower; q.v=cert.v_bound; q.a=cert.a_bound; q.j=cert.j_bound;
        q.c6s=c6s; q.c6e=c6e; q.verify=cert.pass;

        if cert.pass && c6s<=opt.c6_tol && c6e<=opt.c6_tol
            [Psel,csel,qinfo]=p8_restore_quality(P,c,[A;Dmat],[b;dvec], ...
                cfg,limits,obs,ref_fun,Dstart,Dend,opt);
            q.quality_applied=qinfo.applied; q.quality_lambda=qinfo.lambda;
            q.quality_qp=qinfo.n_qp; q.quality_ms=qinfo.ms;
            q.n_qp=q.n_qp+qinfo.n_qp;
            q.quality_before=qinfo.before; q.quality_after=qinfo.after;
            q.cert=true;
            if qinfo.applied, q.reason='CERTIFIED_QUALITY_RESTORED_C6';
            else, q.reason='CERTIFIED_SAFE_FALLBACK_C6'; end
            q.P=Psel; q.cache=c; q.min_g=csel.min_g_lower;
            q.v=csel.v_bound; q.a=csel.a_bound; q.j=csel.j_bound;
            [q.c6s,q.c6e]=p5r2_c6(Psel,c,Dstart,Dend);
            q.verify=csel.pass; return;
        end
        if it==1
            if ~cert.pass, q.reason='INDEPENDENT_VERIFIER_REJECT_AFTER_REFINEMENT';
            elseif c6s>opt.c6_tol, q.reason='START_C6_FAIL';
            else, q.reason='END_C6_FAIL'; end
            return;
        end

        % One P5F5 refinement from current candidate's verifier worst pair.
        sp=cert.worst.span; o=cert.worst.obs;
        [cutA,cutb]=p5r2_candidate_cut(c,P,cfg,obs(o),opt,sp);
        A=[A;cutA]; b=[b;cutb]; %#ok<AGROW>
        q.n_refine=1;
    end
catch ME
    q.reason=['ERROR: ' ME.message];
end
end


function [Psel,Csel,info]=p8_restore_quality(Psafe,c,A,b,cfg,limits,obs,ref_fun,D0,D1,opt)
% P8R3: fixed-budget guide-constrained quality restoration.
% Psafe is already independently certified and remains immutable fallback.
ticq=tic;
Csafe=certify_candidate_global(Psafe,c,cfg,limits,obs,opt);
before=p8_metrics(c,Psafe,ref_fun);
info=struct('applied',false,'lambda',0,'n_qp',0,'ms',0, ...
    'before',before,'after',before);
Psel=Psafe; Csel=Csafe;
if ~opt.quality_enable || ~Csafe.pass
    info.ms=1e3*toc(ticq); return;
end

% Construct ONE cheap coherent guide from the already-certified path.
% No graph/topology search: use the dominant lateral displacement already
% selected by the certified coherent-corridor solution.
guide=p8_build_single_hump_guide(c,Psafe,ref_fun,opt);

% Convex quality objective tracks the guide and penalizes jerk/snap.
[Hq,fq]=p8_guided_quality_objective(c,guide,opt);

% Add linear single-hump monotonicity constraints in the guide direction.
[Ag,bg]=p8_single_hump_constraints(c,guide,opt);
Aq=[A;Ag]; bq=[b;bg];

zsafe=control_points_to_z(c,Psafe);
lb=-opt.box_z_max*ones(c.nz,1); ub=opt.box_z_max*ones(c.nz,1);
qo=optimoptions('quadprog','Display','off','Algorithm','interior-point-convex', ...
    'ConstraintTolerance',1e-9,'OptimalityTolerance',1e-9,'MaxIterations',120);
try
    [zq,~,flag]=quadprog(Hq,fq,Aq,bq,[],[],lb,ub,zsafe,qo);
    info.n_qp=1;
catch
    zq=[]; flag=-99;
end
if flag<=0 || isempty(zq)
    info.ms=1e3*toc(ticq); return;
end

% Exactly one quality candidate: fixed online budget.
P=apply_z(c,zq);
C=certify_candidate_global(P,c,cfg,limits,obs,opt);
[es,ee]=p5r2_c6(P,c,D0,D1);
if C.pass && es<=opt.c6_tol && ee<=opt.c6_tol
    M=p8_metrics(c,P,ref_fun);
    smooth_ok = M.jerkE <= opt.quality_jerk_ratio_max*before.jerkE + 1e-10 && ...
                M.snapE <= opt.quality_snap_ratio_max*before.snapE + 1e-10;
    geometric_ok = M.length <= before.length+1e-10 && M.maxdev <= before.maxdev+1e-10;
    if smooth_ok && geometric_ok
        Psel=P; Csel=C; info.applied=true; info.lambda=1.0; info.after=M;
    end
end
info.ms=1e3*toc(ticq);
end

function G=p8_build_single_hump_guide(c,Psafe,ref_fun,opt)
% Fixed-cost guide construction.  It preserves the side already selected by
% the certified trajectory instead of performing expensive topology search.
ns=max(31,2*c.N+1); u=linspace(0,1,ns);
Pref=zeros(3,ns); Ps=zeros(3,ns); E=zeros(3,ns);
for k=1:ns
    D=ref_fun(c.t0+u(k)*c.T); Pref(:,k)=D(:,1);
    Ps(:,k)=eval_spline(c,Psafe,u(k),0); E(:,k)=Ps(:,k)-Pref(:,k);
end
[~,kp]=max(vecnorm(E,2,1));
eg=E(:,kp);
if norm(eg)<1e-9, ng=[1;0;0]; else, ng=eg/norm(eg); end
amp=max(0,ng'*eg)*opt.guide_peak_gain;
up=u(kp);
width=max(opt.guide_min_width_frac,min(opt.guide_max_width_frac, ...
    max(up,1-up)*0.55));
% Compact smooth single hump; zero outside the local avoidance window.
alpha=zeros(1,ns);
for k=1:ns
    r=abs(u(k)-up)/max(width,1e-6);
    if r<1
        % C2 compact bump. The final spline remains C6 because endpoint
        % derivatives are still imposed by the original boundary map.
        alpha(k)=amp*(1-r^2)^3;
    end
end
Pg=Pref+ng*alpha;
G=struct('u',u,'P',Pg,'n',ng,'peak_u',up,'amp',amp);
end

function [H,f]=p8_guided_quality_objective(c,G,opt)
H=opt.qp_reg*eye(c.nz); f=zeros(c.nz,1);
us=linspace(0,1,opt.N_obj);
for k=1:numel(us)
    u=us(k);
    pg=zeros(3,1);
    for d=1:3, pg(d)=interp1(G.u,G.P(d,:),u,'linear'); end
    J0=derivative_map(c,u,0); q0=eval_spline(c,c.Pbase,u,0); e0=q0-pg;
    H=H+2*opt.quality_w_position*(J0'*J0)/numel(us);
    f=f+2*opt.quality_w_position*J0'*e0/numel(us);
    for r=1:4
        J=derivative_map(c,u,r); q=eval_spline(c,c.Pbase,u,r);
        if r==1, w=opt.quality_w_velocity;
        elseif r==2, w=opt.quality_w_acceleration;
        elseif r==3, w=opt.quality_w_jerk;
        else, w=opt.quality_w_snap; end
        H=H+2*w*(J'*J)/numel(us); f=f+2*w*J'*q/numel(us);
    end
end
H=(H+H')/2;
end

function [A,b]=p8_single_hump_constraints(c,G,opt)
% Enforce monotonic projected displacement at a small fixed grid:
% rise to the guide peak, then fall. These are affine inequalities in z.
ug=linspace(0,1,min(17,max(9,c.N-8)));
A=[]; b=[];
prevJ=[]; prevq=[]; prevu=NaN;
for k=1:numel(ug)
    u=ug(k);
    Dg=zeros(3,1);
    for d=1:3, Dg(d)=interp1(G.u,G.P(d,:),u,'linear'); end %#ok<NASGU>
    J=G.n'*derivative_map(c,u,0);
    q=G.n'*eval_spline(c,c.Pbase,u,0);
    if k>1
        um=0.5*(prevu+u);
        if um<=G.peak_u
            % d_k >= d_{k-1}
            A(end+1,:)=prevJ-J; b(end+1)=q-prevq+opt.guide_monotone_tol; %#ok<AGROW>
        else
            % d_k <= d_{k-1}
            A(end+1,:)=J-prevJ; b(end+1)=prevq-q+opt.guide_monotone_tol; %#ok<AGROW>
        end
    end
    prevJ=J; prevq=q; prevu=u;
end
b=b(:);
end

function M=p8_metrics(c,P,ref_fun)
ns=240; u=linspace(0,1,ns); dt=c.T/(ns-1);
Q=zeros(3,ns); J=zeros(3,ns); S=zeros(3,ns); dev=zeros(1,ns);
for k=1:ns
    Q(:,k)=eval_spline(c,P,u(k),0);
    J(:,k)=eval_spline(c,P,u(k),3);
    S(:,k)=eval_spline(c,P,u(k),4);
    D=ref_fun(c.t0+u(k)*c.T); dev(k)=norm(Q(:,k)-D(:,1));
end
M.length=sum(vecnorm(diff(Q,1,2),2,1));
M.maxdev=max(dev); M.rmsdev=sqrt(mean(dev.^2));
M.jerkE=sum(vecnorm(J,2,1).^2)*dt; M.snapE=sum(vecnorm(S,2,1).^2)*dt;
M.score=0.45*M.maxdev+0.10*M.length+0.025*sqrt(max(M.jerkE,0))+0.005*sqrt(max(M.snapE,0));
end

function M=p8_empty_metrics()
M=struct('length',NaN,'maxdev',NaN,'rmsdev',NaN,'jerkE',NaN,'snapE',NaN,'score',inf);
end

function [A,b]=p5r2_candidate_cut(c,P,cfg,ob,opt,sp)
ua=c.spans(sp,1); ub=c.spans(sp,2);
Pbez=span_bezier_points(c,P,ua,ub,0);
Abez=span_bezier_points(c,P,ua,ub,2);
Wbez=Abez+repmat(cfg.g_acc*cfg.e3,1,size(Abez,2));
Mlb=compute_certified_tension_lower_bound_cols(Wbez);
dirs=fibonacci_sphere(max(162,opt.separator_grid_N));
[~,n]=global_separator_grid(Pbez,Wbez,Mlb,ob,cfg,dirs);

wmin=min(n'*Wbez); Mmax=max(vecnorm(Wbez,2,1));
if Mmax<=1e-12
    qlower=-1;
elseif wmin>=0
    qlower=wmin/Mmax;
elseif Mlb>0
    qlower=max(-1,wmin/Mlb);
else
    qlower=-1;
end
Delta=delta_sep_from_q_lower(qlower,cfg.sys);
rhs=obstacle_support(ob,n)+Delta+cfg.d_margin+opt.guide_extra;
B0=c.B0{sp}; Bz=c.Bz{sp};
A=zeros(c.p+1,c.nz); b=zeros(c.p+1,1);
for j=1:c.p+1
    A(j,:)=-n'*Bz(:,:,j);
    b(j)=n'*B0(:,j)-rhs;
end
end

function [es,ee]=p5r2_c6(P,c,D0,D1)
a=zeros(7,1); b=zeros(7,1);
for r=0:6
    a(r+1)=norm(eval_spline(c,P,0,r)-D0(:,r+1),inf);
    b(r+1)=norm(eval_spline(c,P,1,r)-D1(:,r+1),inf);
end
es=max(a); ee=max(b);
end

function e=p5r2_event(t,type,obs,generation,detail)
e=struct('t',t,'type',type,'obs',obs,'generation',generation,'detail',detail);
end

function ev=p5_future_trigger_current(t,ids,obs_world,nom_fun,active,cfg,Tlook,Npred,Nsub)
% Predictor only, never the safety certificate.
if ~active.valid
    ev=prod_future_trigger_nominal(t,ids,obs_world,nom_fun,cfg,Tlook,Npred,Nsub);
else
    exec_fun=@(tt) p5_exec_fun(tt,active,nom_fun);
    ev=prod_future_trigger_nominal(t,ids,obs_world,exec_fun,cfg,Tlook,Npred,Nsub);
end
end

function D=p5_exec_fun(t,active,nom_fun)
% Use the same certified-plan schema as p2_exec_D:
% Certified plan fields are stored directly as active.plan.cache and active.plan.P.
D=p2_exec_D(t,active,nom_fun);
end

function q=run_phase7p2_candidate(cfg0,N,limits,obs,nom_fun,t0,T,Dstart,Dend,opt,alpha)
% Production-shaped Phase-7 candidate:
% exact C6 endpoints -> certified corridor + dynamic inequalities
% -> ONE combined QP -> independent global verifier -> C6 execution gate.
q=empty_phase719_record('INIT');
q.N=N; q.T=T;
try
    cfg=cfg0; cfg.N_ctrl=N;
    c=build_static_cache_boundary(cfg,t0,t0+T,nom_fun,Dstart,Dend,opt);

    % Endpoint construction audit.
    es0=zeros(7,1); ee0=zeros(7,1);
    for r=0:6
        es0(r+1)=norm(eval_spline(c,c.Pbase,0,r)-Dstart(:,r+1),inf);
        ee0(r+1)=norm(eval_spline(c,c.Pbase,1,r)-Dend(:,r+1),inf);
    end
    q.base_c6s=max(es0); q.base_c6e=max(ee0);
    if q.base_c6s>opt.c6_tol || q.base_c6e>opt.c6_tol
        q.reason='BOUNDARY_CONSTRUCTION_ERROR'; return;
    end

    [G,h,~]=build_certified_active_corridor(c,cfg,obs,opt);
    m=build_dyn_E(c); [D,d]=dyn_E(m,limits,alpha);

    % Only one online QP: combined hard feasibility.
    fc=feas_E(c,[G;D],[h;d],opt);
    q.GD=fc.flag>0; q.rho=fc.rho;
    if fc.flag<=0 || isempty(fc.z)
        q.reason='COMBINED_INFEASIBLE'; return;
    end

    P=apply_z(c,fc.z);

    % Independent continuous-time execution certificate.
    cert=certify_candidate_global(P,c,cfg,limits,obs,opt);
    q.verify=cert.pass; q.min_g=cert.min_g_lower;
    q.v=cert.v_bound; q.a=cert.a_bound; q.j=cert.j_bound;

    es=zeros(7,1); ee=zeros(7,1);
    for r=0:6
        es(r+1)=norm(eval_spline(c,P,0,r)-Dstart(:,r+1),inf);
        ee(r+1)=norm(eval_spline(c,P,1,r)-Dend(:,r+1),inf);
    end
    q.c6s=max(es); q.c6e=max(ee);

    if ~q.verify, q.reason='INDEPENDENT_VERIFIER_REJECT'; return; end
    if q.c6s>opt.c6_tol, q.reason='START_C6_FAIL'; return; end
    if q.c6e>opt.c6_tol, q.reason='END_C6_FAIL'; return; end

    q.cert=1; q.reason='CERTIFIED_RECURSIVE_C6';
    q.P=P; q.cache=c; q.obstacles=obs;
catch ME
    q.reason=['ERROR: ' ME.message];
end
end


function D=p2_exec_D(t,active,nom_fun)
% Current executing trajectory. During certified avoidance use the spline;
% after its certified C6 rejoin time use nominal again.
if active.valid && t>=active.t_start-1e-12 && t<=active.t_end+1e-12
    c=active.plan.cache; P=active.plan.P;
    u=(t-c.t0)/c.T; u=max(0,min(1,u));
    D=zeros(3,7);
    for r=0:6, D(:,r+1)=eval_spline(c,P,u,r); end
else
    D=nom_fun(t);
end
end

function ev=p2_future_trigger_current(tnow,ids,obs,active,nom_fun,cfg,Tlook,N,Nsub)
ev=struct('found',false,'t_now',NaN,'t_risk',NaN,'TTC',NaN,'obs_idx',NaN, ...
    'min_future_dist',inf,'threat_source','');
taus=linspace(0.1,Tlook,max(5,N));
pL=zeros(3,numel(taus)); pQ=pL;
for k=1:numel(taus)
    D=p2_exec_D(tnow+taus(k),active,nom_fun); pL(:,k)=D(:,1); a=D(:,3);
    W=a+cfg.g_acc*cfg.e3;
    if norm(W)<=1e-12, nT=cfg.e3; else, nT=W/norm(W); end
    pQ(:,k)=pL(:,k)+cfg.sys.L_cable*nT;
end
for k=1:numel(taus)-1
    for io=ids
        c0=obs(io).center; rmax=max(obs(io).radii);
        dQb=point_segment_dist(c0,pQ(:,k),pQ(:,k+1))-rmax-cfg.sys.R_uav;
        dLb=point_segment_dist(c0,pL(:,k),pL(:,k+1))-rmax-cfg.sys.R_payload;
        if min(dQb,dLb)>(cfg.d_margin+1.2), continue; end
        aa=linspace(0,1,Nsub+2);
        for j=1:numel(aa)
            a=aa(j); L=(1-a)*pL(:,k)+a*pL(:,k+1); Q=(1-a)*pQ(:,k)+a*pQ(:,k+1);
            [d,src]=prod_system_exact_dist(L,Q,obs(io),cfg);
            if d<=cfg.d_margin
                tr=tnow+(1-a)*taus(k)+a*taus(k+1);
                ev=struct('found',true,'t_now',tnow,'t_risk',tr,'TTC',tr-tnow, ...
                    'obs_idx',io,'min_future_dist',d,'threat_source',src);
                return;
            end
        end
    end
end
end

function e=p2_event_template()
e=struct('t',NaN,'type','','detail','','obs_idx',NaN,'t_risk',NaN,'TTC',NaN,'min_g',NaN);
end

function e=p2_make_event(t,type,detail,obs_idx,t_risk,TTC,min_g)
e=struct('t',t,'type',type,'detail',detail,'obs_idx',obs_idx, ...
    't_risk',t_risk,'TTC',TTC,'min_g',min_g);
end

function s=p2_segment_template()
s=struct('kind','','t0',NaN,'t1',NaN,'plan',empty_phase719_record('NONE'));
end

function make_phase7p2_single_3d(t,p,obs,nom_fun,events)
PN=zeros(3,numel(t));
for k=1:numel(t), D=nom_fun(t(k)); PN(:,k)=D(:,1); end
figure('Name','Phase 7-P5R1 Recursive Multi-Obstacle Failure-Safe Validation','Color','w');
h1=plot3(p(1,:),p(2,:),p(3,:),'LineWidth',2.0); hold on;
h2=plot3(PN(1,:),PN(2,:),PN(3,:),'--','LineWidth',1.2);
for i=1:numel(obs)
    [x,y,z]=sphere(24); E=obs(i).R*diag(obs(i).radii)*[x(:)';y(:)';z(:)'];
    X=reshape(E(1,:)+obs(i).center(1),size(x));
    Y=reshape(E(2,:)+obs(i).center(2),size(y));
    Z=reshape(E(3,:)+obs(i).center(3),size(z));
    surf(X,Y,Z,'FaceAlpha',0.15,'EdgeAlpha',0.12,'HandleVisibility','off');
end
sw=find(strcmp({events.type},'SWITCH'));
if ~isempty(sw)
    hs=[];
    for j=sw
        [~,k]=min(abs(t-events(j).t));
        h=plot3(p(1,k),p(2,k),p(3,k),'o','MarkerSize',7,'LineWidth',1.5,'HandleVisibility','off'); %#ok<NASGU>
    end
    h3=plot3(nan,nan,nan,'o','MarkerSize',7,'LineWidth',1.5);
    legend([h1 h2 h3],{'Executed trajectory','Nominal','Certified switch'},'Location','best');
else
    legend([h1 h2],{'Executed trajectory','Nominal'},'Location','best');
end
grid on; axis equal; xlabel('x [m]'); ylabel('y [m]'); zlabel('z [m]');
title('Phase 7-P5R1 recursive multi-obstacle failure-safe mission'); view(3);
end

function O=run_phase720_event_driven(sel,cfg,limits,obs,nom_fun,opt,trig)
ce=sel.splice.cache; Pe=sel.splice.P;
alpha=1/sqrt(3);

% Three deterministic scenarios.  The fractions define only when a NEW
% obstacle becomes observable and where it lies on the CURRENT certified
% trajectory.  They do NOT directly command a replan.
detect_frac=[0.05 0.10 0.15];
risk_frac  =[0.42 0.55 0.68];
events=struct([]);

for k=1:numel(detect_frac)
    td=ce.t0+detect_frac(k)*ce.T;
    ur=risk_frac(k);
    pr=eval_spline(ce,Pe,ur,0);
    vr=eval_spline(ce,Pe,ur,1);

    obnew=obs(1);
    obnew.center=pr;
    if isfield(obnew,'radii')
        % Moderate obstacle, smaller than the original benchmark obstacle.
        obnew.radii=obs(1).radii.*[0.42;0.42;0.42];
        obnew.R=eul2rotm_local([0.08*k,-0.04*k,0.06*k]);
        obnew.S=obnew.R*diag(obnew.radii.^2)*obnew.R';
    end

    % Offset slightly across the path so the test is not a degenerate
    % obstacle exactly centered on the load trajectory.
    if norm(vr)>1e-9
        et=vr/norm(vr);
        side=[0;1;0]-et*(et'*[0;1;0]);
        if norm(side)<1e-8, side=[1;0;0]-et*(et'*[1;0;0]); end
        side=side/norm(side);
        obnew.center=obnew.center+0.20*(-1)^k*side;
    end

    oo=[obs obnew];

    % Production-style trigger: after detection, scan the currently executing
    % trajectory every control step.  Detection by itself is not a trigger.
    ev=find_future_collision_trigger_spline(ce,Pe,oo,cfg,opt,td,ce.t1,0.025,5.0);
    q=empty_phase720_event();
    q.t_detect=td;
    q.new_obstacle=obnew;

    if ~ev.found
        q.reason='DETECTED_BUT_CURRENT_TRAJECTORY_SAFE';
        q.replan_requested=0;
    else
        q.replan_requested=1;
        q.t_trigger=ev.t_now; q.t_risk=ev.t_risk; q.TTC=ev.TTC;
        ut=(ev.t_now-ce.t0)/ce.T;
        Dstart=zeros(3,7);
        for r=0:6, Dstart(:,r+1)=eval_spline(ce,Pe,ut,r); end

        Ntry=unique([sel.N 34 38 42 46]);
        % Rejoin duration is searched adaptively; no fixed 5 s rejoin.
        extraT=[0 0.5 1 2 3 5 7 9];
        qr=search_phase719_case(cfg,Ntry,extraT,limits,oo,nom_fun, ...
            ev.t_now,sel.T,Dstart,opt,alpha);

        q.cert=qr.cert; q.G=qr.G; q.D=qr.D; q.GD=qr.GD; q.verify=qr.verify;
        q.c6s=qr.c6s; q.c6e=qr.c6e; q.min_g=qr.min_g;
        q.N=qr.N; q.T=qr.T; q.reason=qr.reason;
        q.P=qr.P; q.cache=qr.cache; q.obstacles=oo;
    end

    if k==1, events=q; else, events(k)=q; end
end
O.events=events;

% Hot-path timing: quantify what remains after removing the two diagnostic
% feasibility QPs and after moving obstacle-independent cache/D construction
% out of the repeated online solve.
O.timing=profile_phase720_hot_path(sel,cfg,limits,obs,opt);
end

function ev=find_future_collision_trigger_spline(c,P,obs,cfg,opt,t_detect,t_end,dt,Tlook)
ev=struct('found',false,'t_now',NaN,'t_risk',NaN,'TTC',NaN,'g_risk',inf,'obs_idx',NaN);
dirs=fibonacci_sphere(opt.separator_grid_N);
if size(dirs,1)==3, dirs3=dirs; else, dirs3=dirs'; end

times=t_detect:dt:t_end;
for it=1:numel(times)
    tn=times(it);
    tf=min(tn+Tlook,t_end);
    future=tn+dt:dt:tf;
    for kk=1:numel(future)
        t=future(kk);
        u=(t-c.t0)/c.T;
        if u<0 || u>1, continue; end
        p=eval_spline(c,P,u,0);
        a=eval_spline(c,P,u,2);
        W=a+cfg.g_acc*cfg.e3; M=norm(W);
        if M<=1e-9, continue; end
        nT=W/M;

        for io=1:numel(obs)
            best=-inf;
            for id=1:size(dirs3,2)
                n=dirs3(:,id);
                Delta=relative_system_support_minus(n,nT,cfg);
                hE=ellipsoid_support(n,obs(io));
                g=n'*p-Delta-hE-cfg.d_margin;
                if g>best, best=g; end
            end
            if best<0
                ev.found=true; ev.t_now=tn; ev.t_risk=t; ev.TTC=t-tn;
                ev.g_risk=best; ev.obs_idx=io; return;
            end
        end
    end
end
end

function q=empty_phase720_event()
q=struct('t_detect',NaN,'t_trigger',NaN,'t_risk',NaN,'TTC',NaN, ...
    'replan_requested',0,'cert',0,'G',0,'D',0,'GD',0,'verify',0, ...
    'c6s',inf,'c6e',inf,'min_g',-inf,'N',NaN,'T',NaN,'reason','', ...
    'P',[],'cache',[],'obstacles',[],'new_obstacle',[]);
end

function P=profile_phase720_hot_path(sel,cfg,limits,obs,opt)
c=sel.splice.cache;
alpha=1/sqrt(3);
m=build_dyn_E(c); [D,d]=dyn_E(m,limits,alpha);
ntrial=50;

tc=zeros(ntrial,1); tq=zeros(ntrial,1); ta=zeros(ntrial,1);
tv=zeros(ntrial,1); tt=zeros(ntrial,1);

% Warm-up calls are intentionally excluded.
[G,h,~]=build_certified_active_corridor(c,cfg,obs,opt);
fc=feas_E(c,[G;D],[h;d],opt);
if fc.flag>0 && ~isempty(fc.z)
    Pw=apply_z(c,fc.z); certify_candidate_global(Pw,c,cfg,limits,obs,opt);
end

for k=1:ntrial
    tall=tic;

    t=tic; [G,h,~]=build_certified_active_corridor(c,cfg,obs,opt); tc(k)=toc(t)*1e3;

    t=tic; fc=feas_E(c,[G;D],[h;d],opt); tq(k)=toc(t)*1e3;

    if fc.flag>0 && ~isempty(fc.z)
        t=tic; Pc=apply_z(c,fc.z); ta(k)=toc(t)*1e3;
        t=tic; certify_candidate_global(Pc,c,cfg,limits,obs,opt); tv(k)=toc(t)*1e3;
    else
        ta(k)=NaN; tv(k)=NaN;
    end
    tt(k)=toc(tall)*1e3;
end

P=struct(); P.trials=ntrial;
P.corridor=timing_stats720(tc);
P.combined=timing_stats720(tq);
P.apply=timing_stats720(ta);
P.verify=timing_stats720(tv);
P.total=timing_stats720(tt);
P.median_gap_to_3ms=P.total.median/3.0;
end

function S=timing_stats720(x)
x=x(isfinite(x));
if isempty(x)
    S=struct('median',NaN,'p95',NaN,'p99',NaN,'max',NaN); return;
end
x=sort(x(:)); n=numel(x);
S.median=median(x);
S.p95=x(max(1,ceil(0.95*n)));
S.p99=x(max(1,ceil(0.99*n)));
S.max=max(x);
end

function print_phase720_timing(name,S)
fprintf('%-28s %9.3f %9.3f %9.3f %9.3f\n', ...
    name,S.median,S.p95,S.p99,S.max);
end

function make_phase720_figures(O,sel,obs,nom_fun)
ce=sel.splice.cache; Pe=sel.splice.P;
u=linspace(0,1,500); P0=zeros(3,numel(u)); PN=zeros(3,numel(u));
for k=1:numel(u)
    P0(:,k)=eval_spline(ce,Pe,u(k),0);
    D=nom_fun(ce.t0+u(k)*ce.T); PN(:,k)=D(:,1);
end

figure('Name','Phase 7-20: event-driven recursive replanning','Color','w');
h0=plot3(P0(1,:),P0(2,:),P0(3,:),'LineWidth',2.0); hold on;
hn=plot3(PN(1,:),PN(2,:),PN(3,:),'--','LineWidth',1.1);
for io=1:numel(obs), draw_ellipsoid_local(obs(io)); end
handles=[h0 hn]; labels={'Current certified avoidance','Nominal'};

for k=1:numel(O.events)
    q=O.events(k);
    if q.replan_requested
        % trigger marker is always meaningful
        ut=(q.t_trigger-ce.t0)/ce.T;
        pt=eval_spline(ce,Pe,ut,0);
        hm=plot3(pt(1),pt(2),pt(3),'o','MarkerSize',7,'LineWidth',1.5);
        handles(end+1)=hm; labels{end+1}=sprintf('Trigger %d',k); %#ok<AGROW>
    end
    if q.cert
        ur=linspace(0,1,350); Pr=zeros(3,numel(ur));
        for j=1:numel(ur), Pr(:,j)=eval_spline(q.cache,q.P,ur(j),0); end
        hp=plot3(Pr(1,:),Pr(2,:),Pr(3,:),'LineWidth',1.5);
        handles(end+1)=hp; labels{end+1}=sprintf('Certified replan %d',k); %#ok<AGROW>
        draw_ellipsoid_local(q.new_obstacle);
    end
end
grid on; axis equal; xlabel('x [m]'); ylabel('y [m]'); zlabel('z [m]');
legend(handles,labels,'Location','best');
title('Event-driven replanning: only future-risk triggers and certified replans');

figure('Name','Phase 7-20: hot-path timing','Color','w');
S=O.timing;
vals=[S.corridor.median S.combined.median S.apply.median S.verify.median];
bar(vals); grid on;
set(gca,'XTick',1:4,'XTickLabel',{'corridor','combined QP','apply','verifier'});
ylabel('median time [ms]');
title(sprintf('Hot online path; total median %.2f ms (target 3 ms)',S.total.median));
end


% =========================================================================
% Phase 7-19: recursive C6 replanning without experimental shape optimization
% =========================================================================
function O=run_phase719_recursive_validation(sel,cfg,limits,obs,nom_fun,opt,trig)
alpha=1/sqrt(3);
Pexec=sel.splice.P; cexec=sel.splice.cache;
fractions=[0.15 0.25 0.35 0.50 0.65 0.80];
Ntry=unique([sel.N 34 38 42 46]);
extraT=[0 0.5 1 2 3 5 7];

B=struct([]); W=struct([]);
for k=1:numel(fractions)
    frac=fractions(k);
    tr=trig.t_now+frac*sel.T;
    ur=(tr-cexec.t0)/cexec.T;

    Dstart=zeros(3,7);
    for r=0:6
        Dstart(:,r+1)=eval_spline(cexec,Pexec,ur,r);
    end

    % A: original obstacle set only. This must be understood before adding
    % another obstacle.
    qb=search_phase719_case(cfg,Ntry,extraT,limits,obs,nom_fun,tr,sel.T,Dstart,opt,alpha);
    qb.fraction=frac;
    if k==1, B=qb; else, B(k)=qb; end

    % B: only attempt a new obstacle after the arbitrary C6-boundary case.
    if qb.cert
        obnew=obs(1);
        pnow=Dstart(:,1); vnow=Dstart(:,2);
        if norm(vnow)>1e-9, edir=vnow/norm(vnow); else, edir=[0;0;1]; end

        % A moderate future obstacle. This is deliberately not placed almost
        % on top of the current state; the purpose is recursive functionality.
        lateral=[0;(-1)^k;0];
        if abs(edir'*lateral)>0.8, lateral=[(-1)^k;0;0]; end
        lateral=lateral-edir*(edir'*lateral);
        if norm(lateral)>1e-9, lateral=lateral/norm(lateral); end

        obnew.center=pnow+3.5*edir+1.25*lateral;
        if isfield(obnew,'radii')
            obnew.radii=obs(1).radii.*[0.50;0.50;0.50];
            obnew.R=eul2rotm_local([0.10*k,-0.05*k,0.04*k]);
            obnew.S=obnew.R*diag(obnew.radii.^2)*obnew.R';
        end
        oo=[obs obnew];
        qw=search_phase719_case(cfg,Ntry,extraT,limits,oo,nom_fun,tr,sel.T,Dstart,opt,alpha);
    else
        qw=empty_phase719_record('BOUNDARY_ONLY_FAILED_FIRST');
    end
    qw.fraction=frac;
    if k==1, W=qw; else, W(k)=qw; end
end
O.boundary=B; O.newobs=W;
end

function q=search_phase719_case(cfg,Ntry,extraT,limits,obs,nom_fun,tr,Torig,Dstart,opt,alpha)
best=empty_phase719_record('NO_TEST');
remaining=max(2.0,Torig-tr);

for ie=1:numel(extraT)
    Tnew=remaining+extraT(ie);
    Dend=nom_fun(tr+Tnew);

    for in=1:numel(Ntry)
        q=run_phase719_case(cfg,Ntry(in),limits,obs,nom_fun,tr,Tnew,Dstart,Dend,opt,alpha);

        % Keep the deepest diagnostic result even when it fails.
        if diagnostic_depth(q)>diagnostic_depth(best), best=q; end
        if q.cert
            best=q; return;
        end
    end
end
q=best;
end

function q=run_phase719_case(cfg0,N,limits,obs,nom_fun,t0,T,Dstart,Dend,opt,alpha)
q=empty_phase719_record('INIT');
q.N=N; q.T=T;
try
    cfg=cfg0; cfg.N_ctrl=N;
    c=build_static_cache_boundary(cfg,t0,t0+T,nom_fun,Dstart,Dend,opt);

    % Audit endpoint construction BEFORE any optimization.
    es0=zeros(7,1); ee0=zeros(7,1);
    for r=0:6
        es0(r+1)=norm(eval_spline(c,c.Pbase,0,r)-Dstart(:,r+1),inf);
        ee0(r+1)=norm(eval_spline(c,c.Pbase,1,r)-Dend(:,r+1),inf);
    end
    q.base_c6s=max(es0); q.base_c6e=max(ee0);
    if q.base_c6s>opt.c6_tol || q.base_c6e>opt.c6_tol
        q.reason='BOUNDARY_CONSTRUCTION_ERROR'; return;
    end

    [G,h,~]=build_certified_active_corridor(c,cfg,obs,opt);
    m=build_dyn_E(c); [D,d]=dyn_E(m,limits,alpha);

    % Separate solves are diagnostic only.
    fg=feas_E(c,G,h,opt);
    fd=feas_E(c,D,d,opt);
    q.G=fg.flag>0; q.D=fd.flag>0;

    % The actual online-relevant candidate is the combined hard-feasible one.
    fc=feas_E(c,[G;D],[h;d],opt);
    q.GD=fc.flag>0; q.rho=fc.rho;
    if fc.flag<=0 || isempty(fc.z)
        if ~q.G, q.reason='GEOMETRY_INFEASIBLE';
        elseif ~q.D, q.reason='DYNAMICS_INFEASIBLE';
        else, q.reason='COMBINED_INFEASIBLE';
        end
        return;
    end

    % IMPORTANT FIX: do NOT replace fc.z by the experimental shape-QP result.
    P=apply_z(c,fc.z);

    cert=certify_candidate_global(P,c,cfg,limits,obs,opt);
    q.verify=cert.pass; q.min_g=cert.min_g_lower;
    q.v=cert.v_bound; q.a=cert.a_bound; q.j=cert.j_bound;

    es=zeros(7,1); ee=zeros(7,1);
    for r=0:6
        es(r+1)=norm(eval_spline(c,P,0,r)-Dstart(:,r+1),inf);
        ee(r+1)=norm(eval_spline(c,P,1,r)-Dend(:,r+1),inf);
    end
    q.c6s=max(es); q.c6e=max(ee);

    if ~q.verify
        q.reason='INDEPENDENT_VERIFIER_REJECT'; return;
    end
    if q.c6s>opt.c6_tol
        q.reason='START_C6_FAIL'; return;
    end
    if q.c6e>opt.c6_tol
        q.reason='END_C6_FAIL'; return;
    end

    q.cert=1; q.reason='CERTIFIED_RECURSIVE_C6';
    q.P=P; q.cache=c; q.obstacles=obs;
catch ME
    q.reason=['ERROR: ' ME.message];
end
end

function q=empty_phase719_record(reason)
q=struct('fraction',NaN,'cert',0,'G',0,'D',0,'GD',0,'verify',0, ...
    'rho',inf,'min_g',-inf,'v',inf,'a',inf,'j',inf, ...
    'c6s',inf,'c6e',inf,'base_c6s',inf,'base_c6e',inf, ...
    'N',NaN,'T',NaN,'reason',reason,'P',[],'cache',[],'obstacles',[]);
end

function d=diagnostic_depth(q)
d=0;
if isfinite(q.base_c6s), d=1; end
if q.G || q.D, d=2; end
if q.GD, d=3; end
if q.verify, d=4; end
if isfinite(q.c6s), d=5; end
if q.cert, d=6; end
end

function make_phase719_figures(O,sel,obs,nom_fun)
% Only certified trajectories are drawn. No orange/unverified shape candidate.
ce=sel.splice.cache; Pe=sel.splice.P;
u=linspace(0,1,500); P0=zeros(3,numel(u)); PN=zeros(3,numel(u));
for k=1:numel(u)
    P0(:,k)=eval_spline(ce,Pe,u(k),0);
    D=nom_fun(ce.t0+u(k)*ce.T); PN(:,k)=D(:,1);
end

figure('Name','Phase 7-19: certified recursive replanning','Color','w');
h0=plot3(P0(1,:),P0(2,:),P0(3,:),'LineWidth',1.8); hold on;
hn=plot3(PN(1,:),PN(2,:),PN(3,:),'--','LineWidth',1.1);
for io=1:numel(obs), draw_ellipsoid_local(obs(io)); end

handles=[h0 hn]; labels={'Original certified avoidance','Nominal'};
for k=1:numel(O.boundary)
    q=O.boundary(k);
    if q.cert
        ur=linspace(0,1,300); Pr=zeros(3,numel(ur));
        for j=1:numel(ur), Pr(:,j)=eval_spline(q.cache,q.P,ur(j),0); end
        hp=plot3(Pr(1,:),Pr(2,:),Pr(3,:),'LineWidth',1.2);
        handles(end+1)=hp; %#ok<AGROW>
        labels{end+1}=sprintf('Recursive %.0f%%',100*q.fraction); %#ok<AGROW>
    end
end
grid on; axis equal; xlabel('x [m]'); ylabel('y [m]'); zlabel('z [m]');
legend(handles,labels,'Location','best');
title('Only independently certified recursive trajectories are displayed');

figure('Name','Phase 7-19: recursive diagnostic','Color','w');
f=[O.boundary.fraction];
M=zeros(numel(f),5);
for k=1:numel(f)
    M(k,:)=[O.boundary(k).G O.boundary(k).D O.boundary(k).GD ...
            O.boundary(k).verify O.boundary(k).cert];
end
bar(f,M,'grouped'); grid on; ylim([0 1.2]);
xlabel('Replan fraction of executing avoidance trajectory');
ylabel('pass = 1');
legend('G','D','G+D','Verifier','Final cert','Location','best');
title('Recursive replanning failure-stage diagnosis');
end


% =========================================================================
% Phase 7-18: certified smooth arc + recursive replanning diagnosis
% =========================================================================
function O=run_phase718_suite(sel,cfg,limits,obs,nom_fun,opt,trig)
alpha=1/sqrt(3);
O=struct();

c=sel.splice.cache; P0=sel.splice.P;
[G,h,~]=build_certified_active_corridor(c,cfg,obs,opt);
m=build_dyn_E(c); [D,d]=dyn_E(m,limits,alpha);
Ahd=[G;D]; bhd=[h;d];

% Baseline metrics.
M0=trajectory_metrics(c,P0,nom_fun);
C0=certify_candidate_global(P0,c,cfg,limits,obs,opt);

% -------------------------------------------------------------------------
% Build two SOFT objectives:
%   1) nominal tracking, but with much stronger derivative regularization;
%   2) smooth one-sided arc guide around the obstacle.
% Neither guide is a safety proof. Every accepted result must pass the
% independent global verifier.
% -------------------------------------------------------------------------
z0=control_points_to_z(c,P0);

% Smooth nominal objective: do not allow position tracking to dominate jerk.
optS=opt;
optS.w_position=0.20;
optS.w_jerk=5e-2;
optS.w_snap=5e-3;
[Hs,fs]=build_objective_cache_with_weights(c,nom_fun,optS);
[zs,fls]=solve_custom_qp(c,Ahd,bhd,z0,Hs,fs,opt);
if fls<=0, zs=z0; end

% Arc guide objective.  Guide is generated geometrically from start/end and
% obstacle center, then blended to zero at both ends so C6 endpoint equations
% remain exactly those already embedded in the spline parameterization.
guide_fun=make_arc_guide(c,P0,obs(1),nom_fun,cfg);
[Ha,fa]=build_guide_objective(c,guide_fun,optS);
[za,fla]=solve_custom_qp(c,Ahd,bhd,z0,Ha,fa,opt);
if fla<=0, za=z0; end

% Independent verifier may reject the nonlinear support geometry even when
% the fixed-corridor QP is feasible.  Since z0 is certified, search along the
% segment z(lambda)=z0+lambda(z*-z0).  Linear QP constraints remain feasible
% by convexity; certification is checked independently at every lambda.
[Ps,rs]=certified_segment_search(c,z0,zs,cfg,limits,obs,opt,nom_fun,'smooth');
[Pa,ra]=certified_segment_search(c,z0,za,cfg,limits,obs,opt,nom_fun,'arc');

r0=metric_record('baseline',P0,c,C0,M0,1.0);
records=[r0 rs ra];

% Select among CERTIFIED candidates using a shape score.  Safety is a gate,
% never a term that can be traded against appearance.
score=inf(1,numel(records));
for k=1:numel(records)
    if records(k).cert
        score(k)=0.45*records(k).maxdev + 0.10*records(k).length + ...
                 0.025*sqrt(max(records(k).jerkE,0)) + ...
                 0.005*sqrt(max(records(k).snapE,0));
    end
end
[~,ib]=min(score);
best=records(ib);
if strcmp(best.name,'baseline'), Pbest=P0;
elseif strcmp(best.name,'smooth'), Pbest=Ps;
else, Pbest=Pa; end

D0=nom_fun(c.t0); D1=nom_fun(c.t1);
es=zeros(7,1); ee=zeros(7,1);
for r=0:6
    es(r+1)=norm(eval_spline(c,Pbest,0,r)-D0(:,r+1),inf);
    ee(r+1)=norm(eval_spline(c,Pbest,1,r)-D1(:,r+1),inf);
end
Cb=certify_candidate_global(Pbest,c,cfg,limits,obs,opt);

O.arc.records=records;
O.arc.selected_name=best.name;
O.arc.selected_cert=Cb.pass;
O.arc.P=Pbest; O.arc.cache=c;
O.arc.c6_start=max(es); O.arc.c6_end=max(ee);
O.arc.v=Cb.v_bound; O.arc.a=Cb.a_bound; O.arc.j=Cb.j_bound;

% -------------------------------------------------------------------------
% Recursive replanning diagnosis.
% First test arbitrary C6 boundary with NO newly added obstacle.
% Only if that works, test the same boundary after adding a new obstacle.
% This distinguishes boundary-generalization failure from obstacle difficulty.
% -------------------------------------------------------------------------
fractions=[0.20 0.35 0.50 0.65];
Ntry=unique([sel.N 34 38 42]);
extraT=[0 1 2 3 5];
rec=struct([]);

for k=1:numel(fractions)
    frac=fractions(k);
    tr=trig.t_now+frac*sel.T;
    ur=(tr-c.t0)/c.T;
    Dstart=zeros(3,7);
    for r=0:6, Dstart(:,r+1)=eval_spline(c,Pbest,ur,r); end

    % Stage A: arbitrary C6 boundary, original obstacles only.
    [okA,rrA,cA,PA]=search_recursive_case(cfg,Ntry,extraT,limits,obs,nom_fun,tr,sel.T,Dstart,opt,alpha);

    c6err=inf;
    if okA
        ee=zeros(7,1);
        for r=0:6, ee(r+1)=norm(eval_spline(cA,PA,0,r)-Dstart(:,r+1),inf); end
        c6err=max(ee);
    end

    % Stage B: add a less arbitrary obstacle only after Stage A.
    okB=false; rrB=[];
    if okA
        obnew=obs(1);
        pnow=Dstart(:,1); vnow=Dstart(:,2);
        if norm(vnow)>1e-9, edir=vnow/norm(vnow); else, edir=[0;0;1]; end
        % Put obstacle farther ahead than 7-17 and offset it laterally.
        side=[0;(-1)^k;0];
        obnew.center=pnow+3.0*edir+1.15*side;
        if isfield(obnew,'radii')
            obnew.radii=obs(1).radii.*[0.55;0.55;0.55];
            obnew.R=eul2rotm_local([0.12*k,-0.06*k,0.05*k]);
            obnew.S=obnew.R*diag(obnew.radii.^2)*obnew.R';
        end
        oo=[obs obnew];
        [okB,rrB,~,~]=search_recursive_case(cfg,Ntry,extraT,limits,oo,nom_fun,tr,sel.T,Dstart,opt,alpha);
    end

    if ~okA
        reason='BOUNDARY_REPLAN_FAILED';
    elseif ~okB
        reason='NEW_OBSTACLE_CASE_FAILED';
    else
        reason='CERTIFIED_RECURSIVE_REPLAN';
    end
    x=struct('fraction',frac,'boundary_cert',okA,'newobs_cert',okB, ...
        'c6_error',c6err,'reason',reason,'boundary_result',rrA,'newobs_result',rrB);
    if k==1, rec=x; else, rec(k)=x; end
end
O.replan.records=rec; O.replan.total=numel(rec);
O.replan.boundary_certified=sum([rec.boundary_cert]);
O.replan.newobs_certified=sum([rec.newobs_cert]);
end

function [ok,rrbest,cbest,Pbest]=search_recursive_case(cfg,Ntry,extraT,limits,obs,nom_fun,tr,Torig,Dstart,opt,alpha)
ok=false; rrbest=[]; cbest=[]; Pbest=[];
remaining=max(2.0,Torig-tr);
for ie=1:numel(extraT)
    Tnew=remaining+extraT(ie);
    Dend=nom_fun(tr+Tnew);
    for in=1:numel(Ntry)
        [rr,cc,PP]=run_recursive_case(cfg,Ntry(in),limits,obs,nom_fun,tr,Tnew,Dstart,Dend,opt,alpha);
        if rr.cert
            ok=true; rrbest=rr; cbest=cc; Pbest=PP; return;
        end
    end
end
end

function [Pbest,rec]=certified_segment_search(c,z0,zstar,cfg,limits,obs,opt,nom_fun,name)
% Search from the optimized endpoint back toward the known-certified z0.
% This fixes the Phase-7-17 issue where a visually better but uncertified
% nonlinear-support candidate was plotted as if executable.
lams=[1.0 0.9 0.8 0.7 0.6 0.5 0.4 0.3 0.2 0.1 0.0];
Pbest=apply_z(c,z0); Cbest=certify_candidate_global(Pbest,c,cfg,limits,obs,opt);
bestlam=0;
for lam=lams
    z=z0+lam*(zstar-z0);
    P=apply_z(c,z);
    C=certify_candidate_global(P,c,cfg,limits,obs,opt);
    if C.pass
        Pbest=P; Cbest=C; bestlam=lam; break;
    end
end
M=trajectory_metrics(c,Pbest,nom_fun);
rec=metric_record(name,Pbest,c,Cbest,M,bestlam);
end

function r=metric_record(name,P,c,C,M,lambda)
r=struct('name',name,'cert',C.pass,'lambda',lambda,'min_g',C.min_g_lower, ...
    'length',M.length,'maxdev',M.max_nom_dev,'rmsdev',M.rms_nom_dev, ...
    'jerkE',M.jerk_energy,'snapE',M.snap_energy);
end

function [H,f]=build_objective_cache_with_weights(cache,nom_fun,opt)
H=opt.qp_reg*eye(cache.nz); f=zeros(cache.nz,1);
us=linspace(0,1,opt.N_obj);
for k=1:numel(us)
    u=us(k); t=cache.t0+u*cache.T; DD=nom_fun(t);

    J0=derivative_map(cache,u,0); q0=eval_spline(cache,cache.Pbase,u,0);
    e0=q0-DD(:,1);
    H=H+2*opt.w_position*(J0'*J0)/numel(us);
    f=f+2*opt.w_position*J0'*e0/numel(us);

    % Velocity and acceleration energies suppress lateral waviness.
    J1=derivative_map(cache,u,1); q1=eval_spline(cache,cache.Pbase,u,1);
    J2=derivative_map(cache,u,2); q2=eval_spline(cache,cache.Pbase,u,2);
    H=H+2*2e-3*(J1'*J1)/numel(us); f=f+2*2e-3*J1'*q1/numel(us);
    H=H+2*1e-2*(J2'*J2)/numel(us); f=f+2*1e-2*J2'*q2/numel(us);

    J3=derivative_map(cache,u,3); q3=eval_spline(cache,cache.Pbase,u,3);
    H=H+2*opt.w_jerk*(J3'*J3)/numel(us);
    f=f+2*opt.w_jerk*J3'*q3/numel(us);

    J4=derivative_map(cache,u,4); q4=eval_spline(cache,cache.Pbase,u,4);
    H=H+2*opt.w_snap*(J4'*J4)/numel(us);
    f=f+2*opt.w_snap*J4'*q4/numel(us);
end
H=(H+H')/2;
end

function guide_fun=make_arc_guide(cache,Pbase,o,nom_fun,cfg)
% Geometric one-sided half-ellipse-like guide in the plane formed by the
% nominal travel direction and the obstacle lateral direction.  It is only
% a generator preference, never a certificate.
tm=0.5*(cache.t0+cache.t1);
Dm=nom_fun(tm);
v=Dm(:,2);
if norm(v)<1e-9, v=[0;0;1]; end
eT=v/norm(v);
pm=Dm(:,1);
dc=o.center-pm;
eL=dc-eT*(eT'*dc);
if norm(eL)<1e-8
    tmp=[1;0;0]; if abs(eT'*tmp)>0.9, tmp=[0;1;0]; end
    eL=tmp-eT*(eT'*tmp);
end
eL=eL/norm(eL);

% Choose the side opposite the obstacle-center lateral projection.
if eL'*(o.center-pm)>0, eL=-eL; end

% Radius is geometry-based, not a fixed path shape.
if isfield(o,'radii'), ro=max(o.radii); else, ro=sqrt(max(eig(o.S))); end
Rlat=ro+cfg.d_margin+0.55;
guide_fun=@guide_eval;

    function pg=guide_eval(t)
        tau=(t-cache.t0)/cache.T;
        tau=min(max(tau,0),1);
        D=nom_fun(t);
        % Smooth half-sine lateral bypass; zero at both ends and one-sided.
        % Endpoint C6 is still imposed exactly by the spline fixed controls.
        bump=sin(pi*tau)^2;
        pg=D(:,1)+Rlat*bump*eL;
    end
end

function [H,f]=build_guide_objective(cache,guide_fun,opt)
H=opt.qp_reg*eye(cache.nz); f=zeros(cache.nz,1);
us=linspace(0,1,opt.N_obj);
for k=1:numel(us)
    u=us(k); t=cache.t0+u*cache.T;
    J0=derivative_map(cache,u,0); q0=eval_spline(cache,cache.Pbase,u,0);
    e=q0-guide_fun(t);
    % Arc guide is soft; smoothness has substantial weight.
    H=H+2*0.35*(J0'*J0)/numel(us); f=f+2*0.35*J0'*e/numel(us);

    J2=derivative_map(cache,u,2); q2=eval_spline(cache,cache.Pbase,u,2);
    H=H+2*2e-2*(J2'*J2)/numel(us); f=f+2*2e-2*J2'*q2/numel(us);
    J3=derivative_map(cache,u,3); q3=eval_spline(cache,cache.Pbase,u,3);
    H=H+2*8e-2*(J3'*J3)/numel(us); f=f+2*8e-2*J3'*q3/numel(us);
    J4=derivative_map(cache,u,4); q4=eval_spline(cache,cache.Pbase,u,4);
    H=H+2*8e-3*(J4'*J4)/numel(us); f=f+2*8e-3*J4'*q4/numel(us);
end
H=(H+H')/2;
end

function [z,flag]=solve_custom_qp(cache,A,b,z0,H,f,opt)
lb=-opt.box_z_max*ones(cache.nz,1); ub=opt.box_z_max*ones(cache.nz,1);
qo=optimoptions('quadprog','Display','off','Algorithm','interior-point-convex', ...
    'ConstraintTolerance',1e-9,'OptimalityTolerance',1e-9,'MaxIterations',500);
try
    [z,~,flag]=quadprog(H,f,A,b,[],[],lb,ub,z0,qo);
catch
    z=[]; flag=-99;
end
end

function make_phase718_figures(O,obs,nom_fun)
A=O.arc; c=A.cache; ns=500; u=linspace(0,1,ns); t=c.t0+u*c.T;
Pbase=zeros(3,ns); Pbest=zeros(3,ns); Pnom=zeros(3,ns);
for k=1:ns
    % baseline is recoverable from the first metric only through selected cache;
    % use cache.Pbase only for nominal fitting is not the certified baseline, so
    % reconstruct selected and nominal here; Phase 7-17 figure already retains
    % the exact old baseline comparison.
    Pbest(:,k)=eval_spline(c,A.P,u(k),0);
    DD=nom_fun(t(k)); Pnom(:,k)=DD(:,1);
end

figure('Name','Phase 7-18: selected certified smooth trajectory','Color','w');
hs=plot3(Pbest(1,:),Pbest(2,:),Pbest(3,:),'LineWidth',2.2); hold on;
hn=plot3(Pnom(1,:),Pnom(2,:),Pnom(3,:),'--','LineWidth',1.3);
for io=1:numel(obs), draw_ellipsoid_local(obs(io)); end
grid on; axis equal; xlabel('x [m]'); ylabel('y [m]'); zlabel('z [m]');
legend([hs hn],{['Selected certified: ' A.selected_name],'Nominal'},'Location','best');
title('Phase 7-18: only independently certified shape candidates are selectable');

figure('Name','Phase 7-18: certified candidate metrics','Color','w');
R=A.records;
vals=[[R.maxdev]' [R.length]' sqrt([R.jerkE]')];
bar(vals); grid on;
set(gca,'XTick',1:numel(R),'XTickLabel',{R.name});
legend('Max nominal deviation [m]','Path length [m]','sqrt(jerk energy)','Location','best');
title('Certified shape alternatives (baseline retained)');
end


% =========================================================================
% Phase 7-17: true recursive replanning + certified shape optimization
% =========================================================================
function O=run_phase717_suite(sel,cfg,limits,obs,nom_fun,opt,trig)
alpha=1/sqrt(3);
O=struct();

% -------------------------------------------------------------------------
% J1. Improve trajectory shape INSIDE the same hard certified corridor.
% Safety and dynamics remain hard constraints.  The objective already built
% in build_objective_cache is:
%   nominal deviation + jerk energy + snap energy + tiny regularization.
% The old Phase-7 feasibility point is retained as a feasible warm start.
% -------------------------------------------------------------------------
c=sel.splice.cache;
Pbase=sel.splice.P;
[G,h,~]=build_certified_active_corridor(c,cfg,obs,opt);
m=build_dyn_E(c); [D,d]=dyn_E(m,limits,alpha);
A=[G;D]; b=[h;d];

% Recover the z corresponding to the selected feasible trajectory.
z0=control_points_to_z(c,Pbase);
tt=tic;
[zopt,flag,~]=solve_shape_qp_from_feasible(c,A,b,z0,opt);
solve_ms=toc(tt)*1000;

if flag>0 && ~isempty(zopt)
    Popt=apply_z(c,zopt);
    certopt=certify_candidate_global(Popt,c,cfg,limits,obs,opt);
else
    Popt=Pbase;
    certopt=certify_candidate_global(Pbase,c,cfg,limits,obs,opt);
end

Mb=trajectory_metrics(c,Pbase,nom_fun);
Mo=trajectory_metrics(c,Popt,nom_fun);
D0=nom_fun(c.t0); D1=nom_fun(c.t1);
es=zeros(7,1); ee=zeros(7,1);
for r=0:6
    es(r+1)=norm(eval_spline(c,Popt,0,r)-D0(:,r+1),inf);
    ee(r+1)=norm(eval_spline(c,Popt,1,r)-D1(:,r+1),inf);
end

S=struct();
S.baseline_cert=certify_candidate_global(Pbase,c,cfg,limits,obs,opt).pass;
S.optimized_cert=certopt.pass && max(es)<=opt.c6_tol && max(ee)<=opt.c6_tol;
S.baseline_length=Mb.length; S.optimized_length=Mo.length;
S.baseline_max_nom_dev=Mb.max_nom_dev; S.optimized_max_nom_dev=Mo.max_nom_dev;
S.baseline_rms_nom_dev=Mb.rms_nom_dev; S.optimized_rms_nom_dev=Mo.rms_nom_dev;
S.baseline_jerk_energy=Mb.jerk_energy; S.optimized_jerk_energy=Mo.jerk_energy;
S.baseline_snap_energy=Mb.snap_energy; S.optimized_snap_energy=Mo.snap_energy;
S.optimized_min_g=certopt.min_g_lower;
S.optimized_v=certopt.v_bound; S.optimized_a=certopt.a_bound; S.optimized_j=certopt.j_bound;
S.c6_start=max(es); S.c6_end=max(ee); S.solve_ms=solve_ms;
S.P_baseline=Pbase; S.P_optimized=Popt; S.cache=c;
O.shape=S;

% Use optimized trajectory only if it independently certifies; otherwise the
% already-certified baseline remains the executing trajectory.
if S.optimized_cert, Pexec=Popt; else, Pexec=Pbase; end

% -------------------------------------------------------------------------
% J2. TRUE recursive replanning.
% Start derivatives 0..6 come from the CURRENT certified trajectory at tr.
% End derivatives 0..6 match the nominal trajectory at the new rejoin time.
% A newly detected obstacle is added ahead of the executing trajectory.
% -------------------------------------------------------------------------
fractions=[0.20 0.35 0.50 0.65];
Ntry=unique([sel.N 34 38 42]);
extraT=[0 1 2 3];
rec=struct([]);

for k=1:numel(fractions)
    frac=fractions(k);
    tr=trig.t_now+frac*sel.T;
    ur=(tr-c.t0)/c.T;
    Dstart=zeros(3,7);
    for r=0:6, Dstart(:,r+1)=eval_spline(c,Pexec,ur,r); end

    % New obstacle ahead of the current trajectory. Alternating lateral side
    % avoids validating only one preferred bypass direction.
    obnew=obs(1);
    pnow=Dstart(:,1);
    vnow=Dstart(:,2);
    if norm(vnow)>1e-9, edir=vnow/norm(vnow); else, edir=[0;0;1]; end
    side=[0;(-1)^k;0];
    obnew.center=pnow+1.6*edir+0.75*side;
    if isfield(obnew,'radii')
        obnew.radii=obs(1).radii.*[0.72;0.72;0.72];
        obnew.R=eul2rotm_local([0.18*k,-0.10*k,0.08*k]);
        obnew.S=obnew.R*diag(obnew.radii.^2)*obnew.R';
    end
    oo=[obs obnew];

    found=false; best=[]; bestc=[]; bestP=[]; bestms=NaN;
    for ie=1:numel(extraT)
        Tnew=max(2.0,(sel.T-(tr-trig.t_now))+extraT(ie));
        tend=tr+Tnew;
        Dend=nom_fun(tend);
        for in=1:numel(Ntry)
            tt=tic;
            [rr,cc,PP]=run_recursive_case(cfg,Ntry(in),limits,oo,nom_fun,tr,Tnew,Dstart,Dend,opt,alpha);
            ms=toc(tt)*1000;
            if rr.cert
                found=true; best=rr; bestc=cc; bestP=PP; bestms=ms; break;
            end
        end
        if found, break; end
    end

    if found
        estart=zeros(7,1);
        for r=0:6
            estart(r+1)=norm(eval_spline(bestc,bestP,0,r)-Dstart(:,r+1),inf);
        end
        x=struct('fraction',frac,'cert',1,'c6_start_error',max(estart), ...
            'min_g',best.min_g,'N',best.N,'T',best.T,'ms',bestms, ...
            'P',bestP,'cache',bestc,'obstacles',oo);
    else
        x=struct('fraction',frac,'cert',0,'c6_start_error',inf, ...
            'min_g',-inf,'N',NaN,'T',NaN,'ms',NaN, ...
            'P',[],'cache',[],'obstacles',oo);
    end
    if k==1, rec=x; else, rec(k)=x; end
end
R.records=rec; R.total=numel(rec);
R.certified=sum([rec.cert]);
R.c6_pass=sum([rec.c6_start_error]<=opt.c6_tol);
O.recursive=R;
end

function [rr,c,P]=run_recursive_case(cfg0,N,limits,obs,nom_fun,t0,T,Dstart,Dend,opt,alpha)
rr=struct('N',N,'T',T,'G',0,'D',0,'GD',0,'cert',0,'rho',inf, ...
    'min_g',-inf,'vUB',inf,'aUB',inf,'jUB',inf,'error','');
c=[]; P=[];
try
    cfg=cfg0; cfg.N_ctrl=N;
    c=build_static_cache_boundary(cfg,t0,t0+T,nom_fun,Dstart,Dend,opt);
    [G,h,~]=build_certified_active_corridor(c,cfg,obs,opt);
    m=build_dyn_E(c); [D,d]=dyn_E(m,limits,alpha);

    % Online-relevant combined feasibility first.
    fc=feas_E(c,[G;D],[h;d],opt);
    rr.GD=fc.flag>0; rr.rho=fc.rho;
    if fc.flag<=0 || isempty(fc.z), return; end

    % Shape-optimize from the known feasible point, preserving hard constraints.
    [zopt,fl,~]=solve_shape_qp_from_feasible(c,[G;D],[h;d],fc.z,opt);
    if fl>0 && ~isempty(zopt), z=zopt; else, z=fc.z; end
    P=apply_z(c,z);

    cert=certify_candidate_global(P,c,cfg,limits,obs,opt);
    rr.cert=cert.pass; rr.min_g=cert.min_g_lower;
    rr.vUB=cert.v_bound; rr.aUB=cert.a_bound; rr.jUB=cert.j_bound;

    % Exact recursive start and nominal end C6 gates.
    es=zeros(7,1); ee=zeros(7,1);
    for r=0:6
        es(r+1)=norm(eval_spline(c,P,0,r)-Dstart(:,r+1),inf);
        ee(r+1)=norm(eval_spline(c,P,1,r)-Dend(:,r+1),inf);
    end
    rr.cert=rr.cert && max(es)<=opt.c6_tol && max(ee)<=opt.c6_tol;
    rr.G=rr.cert; rr.D=rr.cert; % diagnostic fields not separately solved here
catch ME
    rr.error=ME.message;
end
end

function cache=build_static_cache_boundary(cfg,t0,t1,nom_fun,D0,D1,opt)
% Same cache mathematics as build_static_cache, except endpoint derivatives
% are supplied explicitly.  This is what enables recursive C6 replanning.
p=cfg.p; N=cfg.N_ctrl; T=t1-t0;
if p~=7, error('Phase7-17 recursive boundary implementation assumes p=7.'); end
if N<=2*p, error('N_ctrl must be > 2*p.'); end

knots=clamped_uniform_knots(N,p);
free_idx=(p+1):(N-p); nf=numel(free_idx); nz=3*nf;
Pfixed=solve_endpoint_fixed(p,N,knots,T,D0,D1);

u_fit=linspace(0,1,max(80,5*N))';
A=zeros(numel(u_fit),nf); Y=zeros(numel(u_fit),3);
for k=1:numel(u_fit)
    B=basis_derivative_all(u_fit(k),knots,p,N,0);
    Dn=nom_fun(t0+u_fit(k)*T);
    A(k,:)=B(free_idx);
    Y(k,:)=Dn(:,1)'-B*Pfixed;
end
Pbase=Pfixed; Pbase(free_idx,:)=A\Y;

cache=struct('p',p,'N',N,'T',T,'t0',t0,'t1',t1,'knots',knots, ...
    'free_idx',free_idx,'nf',nf,'nz',nz,'Pbase',Pbase);
uk=unique(knots);
cache.spans=[uk(1:end-1)',uk(2:end)'];
cache.spans=cache.spans(cache.spans(:,2)>cache.spans(:,1),:);
cache.nspans=size(cache.spans,1);

xi=linspace(0,1,p+1)'; Bern=zeros(p+1);
for j=1:p+1, Bern(j,:)=bernstein_row(p,xi(j)); end
cache.Bern_inv=inv(Bern);

cache.B0=cell(cache.nspans,1); cache.Bz=cell(cache.nspans,1);
for sp=1:cache.nspans
    [B0,Bz]=span_bezier_affine(cache,cache.spans(sp,1),cache.spans(sp,2));
    cache.B0{sp}=B0; cache.Bz{sp}=Bz;
end
[cache.H,cache.f]=build_objective_cache(cache,nom_fun,opt);

cache.mid=zeros(cache.nspans,1); cache.pmid=zeros(3,cache.nspans); cache.amid=zeros(3,cache.nspans);
for sp=1:cache.nspans
    u=mean(cache.spans(sp,:)); cache.mid(sp)=u;
    cache.pmid(:,sp)=eval_spline(cache,Pbase,u,0);
    cache.amid(:,sp)=eval_spline(cache,Pbase,u,2);
end
end

function z=control_points_to_z(cache,P)
z=zeros(cache.nz,1);
for j=1:cache.nf
    ii=cache.free_idx(j);
    dz=(P(ii,:)-cache.Pbase(ii,:))';
    z(3*(j-1)+(1:3))=dz;
end
end

function [z,flag,info]=solve_shape_qp_from_feasible(cache,A,b,z0,opt)
lb=-opt.box_z_max*ones(cache.nz,1); ub=opt.box_z_max*ones(cache.nz,1);
% interior-point-convex is used here for robust offline validation of the
% convex quadratic objective. Deployment solver choice is a later timing phase.
qo=optimoptions('quadprog','Display','off','Algorithm','interior-point-convex', ...
    'ConstraintTolerance',1e-9,'OptimalityTolerance',1e-9,'MaxIterations',500);
try
    [z,~,flag,out]=quadprog(cache.H,cache.f,A,b,[],[],lb,ub,z0,qo);
    info=out;
catch ME
    z=[]; flag=-99; info=struct('message',ME.message);
end
end

function M=trajectory_metrics(cache,P,nom_fun)
ns=500; u=linspace(0,1,ns); t=cache.t0+u*cache.T;
Q=zeros(3,ns); J=zeros(3,ns); S=zeros(3,ns); dev=zeros(1,ns);
for k=1:ns
    Q(:,k)=eval_spline(cache,P,u(k),0);
    J(:,k)=eval_spline(cache,P,u(k),3);
    S(:,k)=eval_spline(cache,P,u(k),4);
    D=nom_fun(t(k)); dev(k)=norm(Q(:,k)-D(:,1));
end
ds=vecnorm(diff(Q,1,2),2,1);
M.length=sum(ds);
M.max_nom_dev=max(dev);
M.rms_nom_dev=sqrt(mean(dev.^2));
M.jerk_energy=trapz(t,sum(J.^2,1));
M.snap_energy=trapz(t,sum(S.^2,1));
end

function make_phase717_figures(O,sel,obs,nom_fun,opt,trig)
S=O.shape; c=S.cache;
ns=500; u=linspace(0,1,ns); t=c.t0+u*c.T;
Pb=zeros(3,ns); Po=zeros(3,ns); Pn=zeros(3,ns);
for k=1:ns
    Pb(:,k)=eval_spline(c,S.P_baseline,u(k),0);
    Po(:,k)=eval_spline(c,S.P_optimized,u(k),0);
    D=nom_fun(t(k)); Pn(:,k)=D(:,1);
end

figure('Name','Phase 7-17: trajectory shape comparison','Color','w');
hb=plot3(Pb(1,:),Pb(2,:),Pb(3,:),'--','LineWidth',1.2); hold on;
ho=plot3(Po(1,:),Po(2,:),Po(3,:),'LineWidth',2.0);
hn=plot3(Pn(1,:),Pn(2,:),Pn(3,:),':','LineWidth',1.4);
for io=1:numel(obs), draw_ellipsoid_local(obs(io)); end
grid on; axis equal; xlabel('x [m]'); ylabel('y [m]'); zlabel('z [m]');
legend([hb ho hn],{'Certified baseline','Unverified shape-QP candidate','Nominal'},'Location','best');
title(sprintf('Certified shape optimization: max nominal deviation %.2f -> %.2f m', ...
    S.baseline_max_nom_dev,S.optimized_max_nom_dev));

figure('Name','Phase 7-17: nominal deviation comparison','Color','w');
db=vecnorm(Pb-Pn,2,1); do=vecnorm(Po-Pn,2,1);
plot(t,db,'--','LineWidth',1.2); hold on;
plot(t,do,'LineWidth',1.8); grid on;
xlabel('t [s]'); ylabel('||p-p_{nom}|| [m]');
legend('Feasible baseline','Optimized certified','Location','best');
title('Nominal-deviation reduction inside the certified feasible set');

R=O.recursive;
figure('Name','Phase 7-17: recursive replanning','Color','w');
tl=tiledlayout(2,2,'TileSpacing','compact','Padding','compact');
for k=1:min(4,R.total)
    nexttile;
    q=R.records(k);
    plot3(Po(1,:),Po(2,:),Po(3,:),'LineWidth',1.2); hold on;
    if q.cert
        ur=linspace(0,1,350); Pr=zeros(3,numel(ur));
        for j=1:numel(ur), Pr(:,j)=eval_spline(q.cache,q.P,ur(j),0); end
        plot3(Pr(1,:),Pr(2,:),Pr(3,:),'LineWidth',2);
    end
    for io=1:numel(q.obstacles), draw_ellipsoid_local(q.obstacles(io)); end
    grid on; axis equal; xlabel('x'); ylabel('y'); zlabel('z');
    title(sprintf('replan %.0f%%: cert=%d',100*q.fraction,q.cert));
end
title(tl,'Recursive C6 replanning from the currently executing certified trajectory');
end


% =========================================================================
% Phase 7-16: failed-case boundary diagnosis + internal timing profile
% =========================================================================
function O=run_phase716_suite(sel,cfg,limits,obs,nom_fun,opt,trig,P715)
alpha=1/sqrt(3);
O=struct();

% ---- Failed-case boundary diagnosis --------------------------------------
failed=find(~[P715.static_records.cert]);
Nbase=sel.N; Tbase=sel.T;
Ntest=unique([Nbase 34 38 42]);
dTtest=[0 0.5 1.0 2.0 3.0];
frec=struct([]);

for ii=1:numel(failed)
    k=failed(ii);
    obcase=P715.static_cases{k};
    base=P715.static_records(k);

    % Test T-only first (same N), then N-only (same T), then N+T.
    recovered=false; rN=NaN; rT=NaN; rdT=NaN; rrbest=[]; cls='UNRECOVERED_TESTED_SET';

    for jt=2:numel(dTtest)
        rr=run_plan_case(cfg,Nbase,limits,obcase,nom_fun,trig.t_now,Tbase+dTtest(jt),opt,alpha);
        if rr.cert
            recovered=true; rN=Nbase; rT=Tbase+dTtest(jt); rdT=dTtest(jt); rrbest=rr;
            cls='RECOVERED_BY_T_ONLY'; break;
        end
    end

    if ~recovered
        for jn=2:numel(Ntest)
            rr=run_plan_case(cfg,Ntest(jn),limits,obcase,nom_fun,trig.t_now,Tbase,opt,alpha);
            if rr.cert
                recovered=true; rN=Ntest(jn); rT=Tbase; rdT=0; rrbest=rr;
                cls='RECOVERED_BY_N_ONLY'; break;
            end
        end
    end

    if ~recovered
        % Search smallest dT first, then smallest N.
        for jt=2:numel(dTtest)
            for jn=2:numel(Ntest)
                rr=run_plan_case(cfg,Ntest(jn),limits,obcase,nom_fun,trig.t_now,Tbase+dTtest(jt),opt,alpha);
                if rr.cert
                    recovered=true; rN=Ntest(jn); rT=Tbase+dTtest(jt); rdT=dTtest(jt); rrbest=rr;
                    cls='RECOVERED_BY_N_AND_T'; break;
                end
            end
            if recovered, break; end
        end
    end

    if recovered
        rrho=rrbest.rho; rG=rrbest.G; rD=rrbest.D; rGD=rrbest.GD;
    else
        rrho=inf; rG=0; rD=0; rGD=0;
    end

    x=struct('name',P715.static_names{k}, ...
        'base_G',base.G,'base_D',base.D,'base_GD',base.GD,'base_cert',base.cert, ...
        'base_rho',base.rho,'classification',cls,'recovered',recovered, ...
        'recovery_N',rN,'recovery_T',rT,'recovery_dT',rdT,'recovery_rho',rrho, ...
        'recovery_G',rG,'recovery_D',rD,'recovery_GD',rGD);
    if ii==1, frec=x; else, frec(ii)=x; end
end

O.failure_records=frec; O.n_failed=numel(failed);
if isempty(frec)
    O.n_T_only=0; O.n_N_only=0; O.n_NT=0; O.n_unrecovered=0;
else
    C={frec.classification};
    O.n_T_only=sum(strcmp(C,'RECOVERED_BY_T_ONLY'));
    O.n_N_only=sum(strcmp(C,'RECOVERED_BY_N_ONLY'));
    O.n_NT=sum(strcmp(C,'RECOVERED_BY_N_AND_T'));
    O.n_unrecovered=sum(strcmp(C,'UNRECOVERED_TESTED_SET'));
end

% ---- Internal timing profile --------------------------------------------
% Warm up first so JIT/first-call setup does not contaminate the distribution.
for k=1:3
    run_plan_case_profiled(cfg,sel.N,limits,obs,nom_fun,trig.t_now,sel.T,opt,alpha);
end

ntrial=50;
names={'cache','corridor','dynamic_matrix','geometry_feas','dynamics_feas', ...
       'combined_feas','apply','verifier','total'};
raw=struct();
for j=1:numel(names), raw.(names{j})=nan(ntrial,1); end
meta=[];
for k=1:ntrial
    [~,tp,mm]=run_plan_case_profiled(cfg,sel.N,limits,obs,nom_fun,trig.t_now,sel.T,opt,alpha);
    for j=1:numel(names), raw.(names{j})(k)=tp.(names{j}); end
    meta=mm;
end
prof=struct(); prof.n=ntrial; prof.samples=raw;
for j=1:numel(names)
    prof.(names{j})=stats_local(raw.(names{j}));
end
medtot=max(prof.total.median,eps);
prof.share.cache=100*prof.cache.median/medtot;
prof.share.corridor=100*prof.corridor.median/medtot;
prof.share.dynamic_matrix=100*prof.dynamic_matrix.median/medtot;
prof.share.qps=100*(prof.geometry_feas.median+prof.dynamics_feas.median+prof.combined_feas.median)/medtot;
prof.share.verifier=100*prof.verifier.median/medtot;
prof.problem=meta;
O.profile=prof;
end

function [rr,tp,meta]=run_plan_case_profiled(cfg0,N,limits,obs,nom_fun,t0,T,opt,alpha)
rr=struct('N',N,'T',T,'policy','','risk_fraction',NaN,'G',0,'D',0,'GD',0, ...
    'cert',0,'rho',inf,'z_inf',inf,'min_g',-inf,'vUB',inf,'aUB',inf,'jUB',inf,'error','');
tp=struct('cache',0,'corridor',0,'dynamic_matrix',0,'geometry_feas',0, ...
    'dynamics_feas',0,'combined_feas',0,'apply',0,'verifier',0,'total',0);
meta=struct('nz',NaN,'nspans',NaN,'active_pairs',NaN,'G_rows',NaN,'D_rows',NaN);
if T<=0, rr.error='NONPOSITIVE_HORIZON'; return; end

Tall=tic;
try
    cfg=cfg0; cfg.N_ctrl=N;

    q=tic; c=build_static_cache(cfg,t0,t0+T,nom_fun,opt); tp.cache=toc(q)*1000;

    q=tic; [G,h,ci]=build_certified_active_corridor(c,cfg,obs,opt); tp.corridor=toc(q)*1000;

    q=tic; m=build_dyn_E(c); [D,d]=dyn_E(m,limits,alpha); tp.dynamic_matrix=toc(q)*1000;

    q=tic; fg=feas_E(c,G,h,opt); tp.geometry_feas=toc(q)*1000;
    q=tic; fd=feas_E(c,D,d,opt); tp.dynamics_feas=toc(q)*1000;
    q=tic; fc=feas_E(c,[G;D],[h;d],opt); tp.combined_feas=toc(q)*1000;

    rr.G=fg.flag>0; rr.D=fd.flag>0; rr.GD=fc.flag>0; rr.rho=fc.rho; rr.z_inf=fc.z_inf;

    meta.nz=c.nz; meta.nspans=c.nspans; meta.active_pairs=ci.n_active_pairs;
    meta.G_rows=size(G,1); meta.D_rows=size(D,1);

    if fc.flag>0 && ~isempty(fc.z)
        q=tic; P=apply_z(c,fc.z); tp.apply=toc(q)*1000;
        q=tic; cert=certify_candidate_global(P,c,cfg,limits,obs,opt); tp.verifier=toc(q)*1000;
        rr.cert=cert.pass; rr.min_g=cert.min_g_lower;
        rr.vUB=cert.v_bound; rr.aUB=cert.a_bound; rr.jUB=cert.j_bound;
    end
catch ME
    rr.error=ME.message;
end
tp.total=toc(Tall)*1000;
end

function S=stats_local(x)
x=x(isfinite(x));
if isempty(x)
    S=struct('median',NaN,'p95',NaN,'p99',NaN,'max',NaN); return;
end
S=struct('median',median(x),'p95',percentile_local(x,95), ...
    'p99',percentile_local(x,99),'max',max(x));
end

function make_phase716_figures(O)
P=O.profile;
figure('Name','Phase 7-16: internal timing profile','Color','w');
labels={'Cache','Corridor','Dyn matrix','G feas','D feas','G+D feas','Apply','Verifier'};
med=[P.cache.median P.corridor.median P.dynamic_matrix.median P.geometry_feas.median ...
     P.dynamics_feas.median P.combined_feas.median P.apply.median P.verifier.median];
bar(med); grid on; ylabel('Median time [ms]');
set(gca,'XTick',1:numel(labels),'XTickLabel',labels,'XTickLabelRotation',30);
title(sprintf('Internal timing breakdown, total median %.2f ms',P.total.median));

figure('Name','Phase 7-16: timing distribution','Color','w');
histogram(P.samples.total); grid on;
xlabel('Profiled online path [ms]'); ylabel('Count');
title(sprintf('median %.1f / P95 %.1f / P99 %.1f / max %.1f ms', ...
    P.total.median,P.total.p95,P.total.p99,P.total.max));

if O.n_failed>0
    figure('Name','Phase 7-16: failed-case recovery','Color','w');
    vals=zeros(O.n_failed,1);
    for k=1:O.n_failed
        c=O.failure_records(k).classification;
        if strcmp(c,'RECOVERED_BY_T_ONLY'), vals(k)=1;
        elseif strcmp(c,'RECOVERED_BY_N_ONLY'), vals(k)=2;
        elseif strcmp(c,'RECOVERED_BY_N_AND_T'), vals(k)=3;
        else, vals(k)=4;
        end
    end
    bar(vals); ylim([0 4.5]); grid on;
    set(gca,'YTick',1:4,'YTickLabel',{'T only','N only','N+T','Unrecovered'});
    xlabel('Failed structured case'); ylabel('Recovery class');
    title('Why the original fixed N,T test failed');
end
end


% =========================================================================
% Phase 7-15 structured validation suite
% =========================================================================
function O=run_phase715_suite(sel,cfg,limits,obs,nom_fun,opt,trig,risk)
alpha=1/sqrt(3);
O=struct();

% -------------------------------------------------------------------------
% H1. Structured obstacle stress matrix.
% Each case changes ONE interpretable geometric feature where possible.
% A failed case is recorded, not silently discarded.
% -------------------------------------------------------------------------
cases={}; names={};
add=@(name,oo) assignin_local(name,oo); %#ok<NASGU>

% lateral / longitudinal / vertical translations
shiftset={[0;-1;0],[0;-0.5;0],[0;0.5;0],[0;1;0], ...
          [-0.75;0;0],[0.75;0;0],[0;0;-0.75],[0;0;0.75]};
for k=1:numel(shiftset)
    oo=obs; oo(1).center=obs(1).center+shiftset{k};
    cases{end+1}=oo; names{end+1}=sprintf('shift_%02d',k); %#ok<AGROW>
end

% size / aspect-ratio changes
scales=[0.75 0.9 1.1 1.25];
for q=scales
    oo=obs;
    if isfield(oo(1),'radii')
        oo(1).radii=obs(1).radii*q;
        oo(1).S=oo(1).R*diag(oo(1).radii.^2)*oo(1).R';
    end
    cases{end+1}=oo; names{end+1}=sprintf('scale_%.2f',q); %#ok<AGROW>
end
aspect={[1.6;0.7;0.7],[0.7;1.6;0.7],[0.7;0.7;1.6]};
for k=1:numel(aspect)
    oo=obs;
    if isfield(oo(1),'radii')
        oo(1).radii=obs(1).radii.*aspect{k};
        oo(1).S=oo(1).R*diag(oo(1).radii.^2)*oo(1).R';
    end
    cases{end+1}=oo; names{end+1}=sprintf('aspect_%d',k); %#ok<AGROW>
end

% attitude changes
angles=[-pi/3 -pi/6 pi/6 pi/3];
for q=angles
    oo=obs;
    if isfield(oo(1),'radii')
        oo(1).R=eul2rotm_local([pi/7+q,pi/10-q/2,pi/8+q/3]);
        oo(1).S=oo(1).R*diag(oo(1).radii.^2)*oo(1).R';
    end
    cases{end+1}=oo; names{end+1}=sprintf('att_%+.2f',q); %#ok<AGROW>
end

rec=struct([]);
for k=1:numel(cases)
    tic;
    rr=run_plan_case(cfg,sel.N,limits,cases{k},nom_fun,trig.t_now,sel.T,opt,alpha);
    ms=toc*1000;
    x=struct('name',names{k},'cert',rr.cert,'G',rr.G,'D',rr.D,'GD',rr.GD, ...
        'rho',rr.rho,'min_g',rr.min_g,'ms',ms);
    if k==1, rec=x; else, rec(k)=x; end
end
O.static_records=rec;
O.static_cases=cases;
O.static_names=names;
O.static_total=numel(rec); O.static_certified=sum([rec.cert]);

% -------------------------------------------------------------------------
% H2. Diverse multiple-obstacle layouts: 2, 3, 4, 6 obstacles.
% Separate ellipsoids are retained; no enclosing system/obstacle mega-sphere.
% -------------------------------------------------------------------------
counts=[2 3 4 6];
layouts=cell(size(counts));
mrec=struct([]);
for ic=1:numel(counts)
    m=counts(ic); oo=obs;
    for j=2:m
        ob2=obs(1);
        side=(-1)^j;
        ob2.center=obs(1).center + [0.55*(j-1); side*(1.1+0.35*mod(j,2)); 0.35*(j-1)];
        if isfield(ob2,'radii')
            ob2.radii=obs(1).radii.*[0.70+0.04*j;0.62+0.03*j;0.72];
            ob2.R=eul2rotm_local([pi/7+0.13*j,pi/10-0.09*j,pi/8+0.07*j]);
            ob2.S=ob2.R*diag(ob2.radii.^2)*ob2.R';
        end
        oo(j)=ob2;
    end
    layouts{ic}=oo;
    tic; rr=run_plan_case(cfg,sel.N,limits,oo,nom_fun,trig.t_now,sel.T,opt,alpha); ms=toc*1000;
    x=struct('nobs',m,'cert',rr.cert,'G',rr.G,'D',rr.D,'GD',rr.GD, ...
        'rho',rr.rho,'min_g',rr.min_g,'ms',ms);
    if ic==1, mrec=x; else, mrec(ic)=x; end
end
O.multi_records=mrec; O.multi_total=numel(mrec); O.multi_certified=sum([mrec.cert]);

% -------------------------------------------------------------------------
% H3. Replan-during-avoidance splice test.
% This validates the C6 splice mathematics against the CURRENT certified
% trajectory.  A new obstacle is introduced at several replanning times.
% Full successful replanning requires a generator accepting current-trajectory
% boundary data; if the inherited run_plan_case is nominal-boundary-only, the
% test explicitly records that limitation instead of claiming success.
% -------------------------------------------------------------------------
fractions=[0.20 0.35 0.50 0.65];
rrec=struct([]);
P=sel.splice.P; c=sel.splice.cache;
for k=1:numel(fractions)
    tr=trig.t_now+fractions(k)*sel.T;
    ur=(tr-trig.t_now)/sel.T;
    boundary=zeros(3,7);
    for r=0:6, boundary(:,r+1)=eval_spline(c,P,ur,r); end

    % Numerically re-evaluate the same current certified trajectory at splice.
    berr=zeros(7,1);
    for r=0:6
        q=eval_spline(c,P,ur,r);
        berr(r+1)=norm(q-boundary(:,r+1),inf);
    end
    c6ok=max(berr)<=opt.c6_tol;

    % New obstacle placed ahead of current payload position, alternating side.
    obnew=obs(1);
    pnow=boundary(:,1);
    obnew.center=pnow+[0.4;(-1)^k*0.9;1.2];
    if isfield(obnew,'radii')
        obnew.radii=obs(1).radii.*[0.75;0.75;0.75];
        obnew.R=eul2rotm_local([0.2*k,-0.12*k,0.1*k]);
        obnew.S=obnew.R*diag(obnew.radii.^2)*obnew.R';
    end
    oo=[obs obnew];

    % Current inherited generator cannot impose arbitrary current-trajectory
    % 0..6 boundary values.  Therefore do NOT mislabel nominal-boundary solve
    % as a recursive-replan proof.  We only check whether a nominal-boundary
    % candidate exists and flag generator_boundary_supported=false.
    Tleft=max(2.0,sel.T-(tr-trig.t_now));
    tic; rr=run_plan_case(cfg,sel.N,limits,oo,nom_fun,tr,Tleft,opt,alpha); ms=toc*1000;
    x=struct('fraction',fractions(k),'t_replan',tr,'c6_boundary_error',max(berr), ...
        'c6_boundary_pass',c6ok,'candidate_cert',rr.cert,'candidate_ms',ms, ...
        'generator_boundary_supported',false);
    if k==1, rrec=x; else, rrec(k)=x; end
end
O.replan_records=rrec; O.replan_total=numel(rrec);
O.replan_c6_pass=sum([rrec.c6_boundary_pass]);
% Deliberately zero until arbitrary current-trajectory boundary conditions are
% implemented. This prevents a false recursive-replanning claim.
O.replan_certified=sum([rrec.candidate_cert] & [rrec.generator_boundary_supported]);

% -------------------------------------------------------------------------
% H4. Timing distribution for one ONLINE candidate path.
% No plots, fprintf, stress sweep, or coarse/fine horizon search inside timer.
% -------------------------------------------------------------------------
ntrial=50;
tm=zeros(ntrial,1);
for k=1:ntrial
    tic;
    run_plan_case(cfg,sel.N,limits,obs,nom_fun,trig.t_now,sel.T,opt,alpha);
    tm(k)=toc*1000;
end
O.timing.samples_ms=tm; O.timing.n=ntrial;
O.timing.median=median(tm); O.timing.p95=percentile_local(tm,95);
O.timing.p99=percentile_local(tm,99); O.timing.max=max(tm);

% Categorize failures for interpretation.
O.failure_static_geometry=sum(~[rec.G]);
O.failure_static_combined=sum([rec.G] & ~[rec.GD]);
O.failure_static_certificate=sum([rec.GD] & ~[rec.cert]);
end

function make_phase715_figures(O)
figure('Name','Phase 7-15: structured stress','Color','w');
tl=tiledlayout(2,2,'TileSpacing','compact','Padding','compact');
nexttile;
bar(double([O.static_records.cert])); ylim([0 1.2]); grid on;
xlabel('Structured obstacle case'); ylabel('Certified'); title('Static stress matrix');
nexttile;
bar([O.multi_records.nobs],[O.multi_records.cert]); ylim([0 1.2]); grid on;
xlabel('Number of obstacles'); ylabel('Certified'); title('Multiple-obstacle layouts');
nexttile;
plot([O.multi_records.nobs],[O.multi_records.ms],'-o','LineWidth',1.5); grid on;
xlabel('Number of obstacles'); ylabel('Diagnostic solve time [ms]');
title('Obstacle-count timing');
nexttile;
histogram(O.timing.samples_ms); grid on;
xlabel('Single candidate online-path time [ms]'); ylabel('Count');
title(sprintf('Timing: med %.1f / P99 %.1f / max %.1f ms', ...
    O.timing.median,O.timing.p99,O.timing.max));
title(tl,'Phase 7-15 stress and timing validation');

figure('Name','Phase 7-15: failure classification','Color','w');
bar([O.failure_static_geometry,O.failure_static_combined,O.failure_static_certificate]);
set(gca,'XTickLabel',{'Geometry','G+D intersection','Independent cert'});
ylabel('Failed cases'); grid on; title('Why structured cases fail');

figure('Name','Phase 7-15: recursive splice diagnostic','Color','w');
plot([O.replan_records.t_replan],[O.replan_records.c6_boundary_error],'-o','LineWidth',1.5);
yline(1e-7,'--'); set(gca,'YScale','log'); grid on;
xlabel('Replan time [s]'); ylabel('C6 boundary error');
title('C6 state available for recursive replanning');
end

function y=percentile_local(x,p)
x=sort(x(:)); n=numel(x);
if n==1, y=x; return; end
q=1+(n-1)*p/100; i=floor(q); f=q-i;
if i>=n, y=x(end); else, y=x(i)*(1-f)+x(i+1)*f; end
end

function z=assignin_local(name,oo) %#ok<INUSD>
z=[];
end


% =========================================================================
% Phase 7-14 integrated validation / visualization
% =========================================================================
function out=evaluate_selected_integrated(sel,cfg,limits,obs,nom_fun,opt,trig)
P=sel.splice.P; c=sel.splice.cache;
tt=linspace(trig.t_now,trig.t_now+sel.T,801);
uu=(tt-trig.t_now)/sel.T;
Q=cell(7,1);
for r=0:6
    Q{r+1}=zeros(3,numel(uu));
    for k=1:numel(uu), Q{r+1}(:,k)=eval_spline(c,P,uu(k),r); end
end
finite_all=all(cellfun(@(x)all(isfinite(x(:))),Q));
% C6 is structural inside a degree-7 spline with simple internal knots; splice
% equality was independently checked at both endpoints.
out.c6_all_pass=sel.splice.pass;
out.derivative_sample_pass=finite_all;

vnorm=sqrt(sum(Q{2}.^2,1)); anorm=sqrt(sum(Q{3}.^2,1)); jnorm=sqrt(sum(Q{4}.^2,1));
out.tracking_pass=max(vnorm)<=limits.v_max+opt.cert_tol && ...
    max(anorm)<=limits.a_max+opt.cert_tol && max(jnorm)<=limits.j_max+opt.cert_tol;
out.safety_pass=sel.plan.cert;

% Translational feed-forward force/thrust-direction proxy only.
W=Q{3}+cfg.g_acc*cfg.e3;
Fmag=sqrt(sum(W.^2,1));
nT=W./max(Fmag,eps);
out.input_proxy_pass=all(isfinite(Fmag)) && all(isfinite(nT(:)));
out.t=tt; out.Q=Q; out.vnorm=vnorm; out.anorm=anorm; out.jnorm=jnorm;
out.Fmag=Fmag; out.nT=nT;

% Deterministic robustness sweep: obstacle location, size, and attitude.
cases={};
dy=[-1 -0.5 0.5 1.0];
sc=[0.8 0.9 1.1 1.2];
ang=[-pi/6 pi/6];
for x=dy
    oo=obs; oo(1).center=oo(1).center+[0;x;0]; cases{end+1}=oo; %#ok<AGROW>
end
for x=sc
    oo=obs;
    if isfield(oo(1),'radii')
        oo(1).radii=obs(1).radii*x;
        oo(1).S=oo(1).R*diag(oo(1).radii.^2)*oo(1).R';
        cases{end+1}=oo; %#ok<AGROW>
    end
end
for x=ang
    oo=obs;
    if isfield(oo(1),'radii')
        oo(1).R=eul2rotm_local([pi/7+x,pi/10,pi/8]);
        oo(1).S=oo(1).R*diag(oo(1).radii.^2)*oo(1).R';
        cases{end+1}=oo; %#ok<AGROW>
    end
end
pass=false(1,numel(cases)); rho=inf(1,numel(cases));
for i=1:numel(cases)
    rr=run_plan_case(cfg,sel.N,limits,cases{i},nom_fun,trig.t_now,sel.T,opt,1/sqrt(3));
    pass(i)=rr.cert; rho(i)=rr.rho;
end
out.robust_total=numel(cases); out.robust_certified=sum(pass);
out.robust_pass=pass; out.robust_rho=rho;

% Multiple-obstacle stress test. These are intentionally separate convex
% ellipsoids; no enclosing "whole-system sphere" is introduced.
multi=cell(1,3);
for m=1:3
    oo=obs;
    for j=2:m
        ob2=obs(1);
        ob2.center=obs(1).center+[0.35*(j-1); (-1)^j*1.8; 0.55*(j-1)];
        if isfield(ob2,'radii')
            ob2.radii=obs(1).radii.*[0.75;0.70;0.80];
            ob2.R=eul2rotm_local([pi/7+0.18*j,pi/10-0.12*j,pi/8]);
            ob2.S=ob2.R*diag(ob2.radii.^2)*ob2.R';
        end
        oo(j)=ob2;
    end
    multi{m}=oo;
end
mpass=false(1,3); mtime=nan(1,3);
for m=1:3
    tic;
    rr=run_plan_case(cfg,sel.N,limits,multi{m},nom_fun,trig.t_now,sel.T,opt,1/sqrt(3));
    mtime(m)=toc*1000; mpass(m)=rr.cert;
end
out.multi_total=3; out.multi_certified=sum(mpass);
out.multi_pass=mpass; out.multi_ms=mtime;
end

function make_phase714_figures(sel,cfg,limits,obs,nom_fun,opt,trig,risk,I)
P=sel.splice.P; c=sel.splice.cache; t=I.t; Q=I.Q;

% Figure 1: 3-D geometry and trajectory.
figure('Name','Phase 7-14: 3D trajectory','Color','w');
hAvoid=plot3(Q{1}(1,:),Q{1}(2,:),Q{1}(3,:),'LineWidth',2); hold on;
Pn=zeros(3,numel(t));
for k=1:numel(t), D=nom_fun(t(k)); Pn(:,k)=D(:,1); end
hNom=plot3(Pn(1,:),Pn(2,:),Pn(3,:),'--','LineWidth',1.5);
for io=1:numel(obs), draw_ellipsoid_local(obs(io)); end
hStart=plot3(Q{1}(1,1),Q{1}(2,1),Q{1}(3,1),'o','MarkerSize',8,'LineWidth',1.5);
hEnd=plot3(Q{1}(1,end),Q{1}(2,end),Q{1}(3,end),'s','MarkerSize',8,'LineWidth',1.5);
grid on; axis equal; xlabel('x [m]'); ylabel('y [m]'); zlabel('z [m]');
legend([hAvoid hNom hStart hEnd],{'Certified avoidance','Nominal','Start','C6 rejoin'},'Location','best');
title(sprintf('Certified trajectory: N=%d, T=%.3f s',sel.N,sel.T));

% Figure 2: safety + dynamics.
figure('Name','Phase 7-14: safety and tracking','Color','w');
tl=tiledlayout(2,2,'TileSpacing','compact','Padding','compact');
g=zeros(1,numel(t));
for k=1:numel(t)
    a=Q{3}(:,k); W=a+cfg.g_acc*cfg.e3; nT=W/norm(W);
    gg=inf(1,numel(obs));
    for io=1:numel(obs), gg(io)=best_nominal_point_g(Q{1}(:,k),nT,obs(io),cfg,opt); end
    g(k)=min(gg);
end
nexttile; plot(t,g,'LineWidth',1.5); yline(0,'--'); grid on;
xlabel('t [s]'); ylabel('g sample [m]'); title('Sampled safety margin (certificate is separate)');
nexttile; plot(t,I.vnorm,'LineWidth',1.5); hold on; yline(limits.v_max,'--'); grid on;
xlabel('t [s]'); ylabel('|v| [m/s]'); title('Velocity');
nexttile; plot(t,I.anorm,'LineWidth',1.5); hold on; yline(limits.a_max,'--'); grid on;
xlabel('t [s]'); ylabel('|a| [m/s^2]'); title('Acceleration');
nexttile; plot(t,I.jnorm,'LineWidth',1.5); hold on; yline(limits.j_max,'--'); grid on;
xlabel('t [s]'); ylabel('|j| [m/s^3]'); title('Jerk');
title(tl,'Safety and tracking diagnostics');

% Figure 3: derivatives 0..6 and feed-forward proxy.
figure('Name','Phase 7-14: C6 derivatives and input proxy','Color','w');
tl=tiledlayout(4,2,'TileSpacing','compact','Padding','compact');
for r=0:6
    nexttile; plot(t,Q{r+1}','LineWidth',1.0); grid on;
    xlabel('t [s]'); ylabel(sprintf('p^{(%d)}',r)); title(sprintf('Derivative order %d',r));
end
nexttile; plot(t,I.Fmag,'LineWidth',1.5); grid on;
xlabel('t [s]'); ylabel('|a+ge_3|'); title('Feed-forward thrust/force proxy');
title(tl,'C6 trajectory and translational input proxy');

% Figure 4: robustness and obstacle-count timing.
figure('Name','Phase 7-14: robustness and timing','Color','w');
tl=tiledlayout(1,2,'TileSpacing','compact','Padding','compact');
nexttile; bar(1:I.robust_total,double(I.robust_pass)); ylim([0 1.2]); grid on;
xlabel('Stress case'); ylabel('Certified'); title('Pose / size / location robustness');
nexttile; plot(1:I.multi_total,I.multi_ms,'-o','LineWidth',1.5); grid on;
xlabel('Number of obstacles'); ylabel('Diagnostic time [ms]');
title('Obstacle-count scaling (not WCET)');
title(tl,'Robustness validation');

fprintf('[G: FIGURES]\n');
fprintf('Phase 7-22 suppresses legacy figures; exactly one final 3-D figure is generated.\n');
fprintf('Risk window: first=%.3f, worst=%.3f, clear=%.3f s\n', ...
    risk.t_first_bad,risk.t_worst,risk.t_clear);
end

function draw_ellipsoid_local(ob)
if ~isfield(ob,'radii') || ~isfield(ob,'R'), return; end
[x,y,z]=sphere(24);
X=[x(:)';y(:)';z(:)'];
Y=ob.R*diag(ob.radii)*X+ob.center;
surf(reshape(Y(1,:),size(x)),reshape(Y(2,:),size(y)),reshape(Y(3,:),size(z)), ...
    'FaceAlpha',0.18,'EdgeAlpha',0.12,'HandleVisibility','off');
end


% =========================================================================
% Phase 7-13 wrapper: generate exactly as the validated both-end-C6 generator,
% then independently audit both splice points r=0..6.
% =========================================================================
function [rr,splice]=run_plan_case_with_splice_audit(cfg0,N,limits,obs,nom_fun,t0,T,opt,alpha)
rr=struct('N',N,'T',T,'policy','','risk_fraction',NaN,'G',0,'D',0,'GD',0, ...
    'cert',0,'rho',inf,'z_inf',inf,'min_g',-inf,'vUB',inf,'aUB',inf,'jUB',inf,'error','');
splice=struct('pass',false,'start_max',inf,'end_max',inf,'start_by_order',inf(7,1),'end_by_order',inf(7,1),'P',[],'cache',[]);
if T<=0, rr.error='NONPOSITIVE_HORIZON'; return; end
try
    cfg=cfg0; cfg.N_ctrl=N;
    c=build_static_cache(cfg,t0,t0+T,nom_fun,opt);
    [G,h,~]=build_certified_active_corridor(c,cfg,obs,opt);
    m=build_dyn_E(c); [D,d]=dyn_E(m,limits,alpha);
    fg=feas_E(c,G,h,opt); fd=feas_E(c,D,d,opt); fc=feas_E(c,[G;D],[h;d],opt);
    rr.G=fg.flag>0; rr.D=fd.flag>0; rr.GD=fc.flag>0; rr.rho=fc.rho; rr.z_inf=fc.z_inf;
    if fc.flag>0 && ~isempty(fc.z)
        P=apply_z(c,fc.z);
        cert=certify_candidate_global(P,c,cfg,limits,obs,opt);
        rr.cert=cert.pass; rr.min_g=cert.min_g_lower;
        rr.vUB=cert.v_bound; rr.aUB=cert.a_bound; rr.jUB=cert.j_bound;
        D0=nom_fun(t0); D1=nom_fun(t0+T);
        for r=0:6
            qs=eval_spline(c,P,0,r); qe=eval_spline(c,P,1,r);
            splice.start_by_order(r+1)=norm(qs-D0(:,r+1),inf);
            splice.end_by_order(r+1)=norm(qe-D1(:,r+1),inf);
        end
        splice.start_max=max(splice.start_by_order);
        splice.end_max=max(splice.end_by_order);
        splice.pass=(splice.start_max<=opt.c6_tol)&&(splice.end_max<=opt.c6_tol);
        splice.P=P; splice.cache=c;
        rr.cert=rr.cert && splice.pass;
    end
catch ME
    rr.error=ME.message;
end
end

function validate_phase712e_inputs(cfg,limits,obs,opt)
req_cfg={'p','N_ctrl','g_acc','e3','sys','d_margin'};
for k=1:numel(req_cfg)
    assert(isfield(cfg,req_cfg{k}), 'Phase7-12E:MissingCfgField', ...
        'cfg.%s is required.',req_cfg{k});
end
req_sys={'L_cable','R_payload','R_cable','R_uav'};
for k=1:numel(req_sys)
    assert(isfield(cfg.sys,req_sys{k}), 'Phase7-12E:MissingSysField', ...
        'cfg.sys.%s is required.',req_sys{k});
end
req_lim={'v_max','a_max','j_max'};
for k=1:numel(req_lim)
    assert(isfield(limits,req_lim{k}), 'Phase7-12E:MissingLimitField', ...
        'limits.%s is required.',req_lim{k});
end
req_opt={'box_z_max','guide_extra','active_margin_trigger','separator_grid_N', ...
         'cert_tol','c6_tol','a_net_min'};
for k=1:numel(req_opt)
    assert(isfield(opt,req_opt{k}), 'Phase7-12E:MissingOptField', ...
        'opt.%s is required.',req_opt{k});
end
assert(~isempty(obs),'Phase7-12E:NoObstacle','At least one obstacle is required.');
for i=1:numel(obs)
    assert(isfield(obs(i),'center'),'Phase7-12E:MissingObstacleCenter', ...
        'obs(%d).center is required.',i);
    assert(isfield(obs(i),'S') || isfield(obs(i),'support_fun'), ...
        'Phase7-12E:MissingObstacleSupport', ...
        'obs(%d) requires S or support_fun.',i);
end
end

% =========================================================================
% Trigger reconstruction using the same research-level whole-system
% fixed-normal certificate geometry used by the audit pipeline.
% =========================================================================
function trig=reconstruct_first_trigger(nom_fun,obs,cfg,opt,tmin,tmax,Tlook,dt)
trig=struct('found',false,'t_now',NaN,'t_risk',NaN,'TTC',NaN, ...
    'g_risk',inf,'obs_idx',NaN);
% Search at control-cycle resolution. At each t_now, scan future from near to far.
times=tmin:dt:tmax;
for it=1:numel(times)
    tn=times(it);
    tf=min(tn+Tlook,tmax);
    if tf<=tn+dt, continue; end
    future=tn+dt:dt:tf;
    hit=false;
    for k=1:numel(future)
        t=future(k);
        D=nom_fun(t);
        p=D(:,1); a=D(:,3);
        W=a+cfg.g_acc*cfg.e3;
        M=norm(W);
        if M<=1e-9, continue; end
        nT=W/M;
        dirs=fibonacci_sphere(opt.separator_grid_N);
        if size(dirs,1)==3
            dirs3=dirs;
        elseif size(dirs,2)==3
            dirs3=dirs';
        else
            error('fibonacci_sphere must return 3xN or Nx3 directions.');
        end
        for io=1:numel(obs)
            best=-inf;
            for id=1:size(dirs3,2)
                n=dirs3(:,id);
                Delta=relative_system_support_minus(n,nT,cfg);
                hE=ellipsoid_support(n,obs(io));
                g=n'*p-Delta-hE-cfg.d_margin;
                if g>best, best=g; end
            end
            if best<0
                trig.found=true; trig.t_now=tn; trig.t_risk=t;
                trig.TTC=t-tn; trig.g_risk=best; trig.obs_idx=io;
                hit=true; break;
            end
        end
        if hit, break; end
    end
    if hit, return; end
end
end

function tl=build_timeline_diagnostics(nom_fun,obs,cfg,opt,trig,tmin,Tlook)
% Approximate the production 15 m sensing timeline using obstacle surface
% distance from the nominal UAV position. The trigger itself remains the
% audit whole-system future-collision trigger, so both times are reported
% separately rather than treated as identical.
dt=0.025;
sensor_radius=15.0;
ts=tmin:dt:trig.t_now;
t_sensor=NaN; range_sensor=inf;
for k=1:numel(ts)
    D=nom_fun(ts(k));
    pL=D(:,1); aL=D(:,3);
    W=aL+cfg.g_acc*cfg.e3;
    nT=W/norm(W);
    pQ=pL+cfg.sys.L_cable*nT;
    rr=ellipsoid_surface_distance_approx(pQ,obs(trig.obs_idx));
    if rr<=sensor_radius
        t_sensor=ts(k); range_sensor=rr; break;
    end
end
D=nom_fun(trig.t_now); pL=D(:,1); aL=D(:,3);
nT=(aL+cfg.g_acc*cfg.e3)/norm(aL+cfg.g_acc*cfg.e3);
range_trigger=ellipsoid_surface_distance_approx(pL+cfg.sys.L_cable*nT,obs(trig.obs_idx));
g_now=best_nominal_point_g(pL,nT,obs(trig.obs_idx),cfg,opt);

D=nom_fun(trig.t_risk); pLr=D(:,1); aLr=D(:,3);
nTr=(aLr+cfg.g_acc*cfg.e3)/norm(aLr+cfg.g_acc*cfg.e3);
range_risk=ellipsoid_surface_distance_approx(pLr+cfg.sys.L_cable*nTr,obs(trig.obs_idx));
g_risk=best_nominal_point_g(pLr,nTr,obs(trig.obs_idx),cfg,opt);

tl=struct();
tl.t_sensor=t_sensor; tl.range_sensor=range_sensor;
tl.range_trigger=range_trigger; tl.range_risk=range_risk;
tl.g_now=g_now; tl.g_risk=g_risk;
if isnan(t_sensor)
    tl.sensor_to_trigger=NaN; tl.sensor_to_risk=NaN;
else
    tl.sensor_to_trigger=trig.t_now-t_sensor;
    tl.sensor_to_risk=trig.t_risk-t_sensor;
end
tl.Tlook=Tlook;
end

function g=best_nominal_point_g(p,nT,o,cfg,opt)
dirs=fibonacci_sphere(opt.separator_grid_N);
if size(dirs,1)==3, dirs3=dirs; else, dirs3=dirs'; end
g=-inf;
for k=1:size(dirs3,2)
    n=dirs3(:,k);
    val=n'*p-relative_system_support_minus(n,nT,cfg)-ellipsoid_support(n,o)-cfg.d_margin;
    g=max(g,val);
end
end

function d=ellipsoid_surface_distance_approx(p,o)
% Fast radial surface-distance diagnostic only (not a certificate).
% Exact production code may use its iterative point-to-ellipsoid routine.
p=p(:); c=o.center(:);
v=p-c; nv=norm(v);
if nv<1e-12, d=-min(sqrt(eig(o.S))); return; end
u=v/nv;
rad=sqrt(max(0,u'*o.S*u));
d=nv-rad;
end

function policies=make_policies(TTC)
names={'FIXED_5','TTC','TTC_PLUS_1','TTC_PLUS_2','FIXED_7_5','FIXED_10'};
vals=[5,max(TTC,0.5),max(TTC+1,0.5),max(TTC+2,0.5),7.5,10];
for k=1:numel(names)
    policies(k)=struct('name',names{k},'T',vals(k)); %#ok<AGROW>
end
end

function rr=run_plan_case(cfg0,N,limits,obs,nom_fun,t0,T,opt,alpha)
rr=struct('N',N,'T',T,'policy','','risk_fraction',NaN,'G',0,'D',0,'GD',0, ...
    'cert',0,'rho',inf,'z_inf',inf,'min_g',-inf,'vUB',inf,'aUB',inf,'jUB',inf,'error','');
if T<=0, rr.error='NONPOSITIVE_HORIZON'; return; end
try
    cfg=cfg0; cfg.N_ctrl=N;
    c=build_static_cache(cfg,t0,t0+T,nom_fun,opt);
    [G,h,~]=build_certified_active_corridor(c,cfg,obs,opt);
    m=build_dyn_E(c);
    [D,d]=dyn_E(m,limits,alpha);
    fg=feas_E(c,G,h,opt); fd=feas_E(c,D,d,opt); fc=feas_E(c,[G;D],[h;d],opt);
    rr.G=fg.flag>0; rr.D=fd.flag>0; rr.GD=fc.flag>0; rr.rho=fc.rho; rr.z_inf=fc.z_inf;
    if fc.flag>0 && ~isempty(fc.z)
        P=apply_z(c,fc.z);
        cert=certify_candidate_global(P,c,cfg,limits,obs,opt);
        rr.cert=cert.pass;
        rr.min_g=cert.min_g_lower;
        rr.vUB=cert.v_bound; rr.aUB=cert.a_bound; rr.jUB=cert.j_bound;
    end
catch ME
    rr.error=ME.message;
end
end

function print_plan(r)
fprintf('N=%d T=%.3f | G=%d D=%d G+D=%d cert=%d rho*=%.3e zinf=%.3f min_g=%+.4f v/a/j=%.3f/%.3f/%.3f\n', ...
    r.N,r.T,r.G,r.D,r.GD,r.cert,r.rho,r.z_inf,r.min_g,r.vUB,r.aUB,r.jUB);
if ~isempty(r.error), fprintf('error: %s\n',r.error); end
end

function m=build_dyn_E(cache)
P0=cache.Pbase; [A0,V0,J0]=derivative_control_polygons(P0,cache);
nz=cache.nz; VM=zeros(numel(V0),nz); AM=zeros(numel(A0),nz); JM=zeros(numel(J0),nz);
for q=1:nz
    e=zeros(nz,1); e(q)=1; Pq=apply_z(cache,e);
    [Aq,Vq,Jq]=derivative_control_polygons(Pq,cache);
    VM(:,q)=Vq(:)-V0(:); AM(:,q)=Aq(:)-A0(:); JM(:,q)=Jq(:)-J0(:);
end
m.VM=VM; m.V0=V0(:); m.AM=AM; m.A0=A0(:); m.JM=JM; m.J0=J0(:);
end

function [A,b]=dyn_E(m,lim,alpha)
[A1,b1]=two_E(m.VM,m.V0,alpha*lim.v_max);
[A2,b2]=two_E(m.AM,m.A0,alpha*lim.a_max);
[A3,b3]=two_E(m.JM,m.J0,alpha*lim.j_max);
A=[A1;A2;A3]; b=[b1;b2;b3];
end
function [A,b]=two_E(M,d0,L), A=[M;-M]; b=[L-d0;L+d0]; end

function r=p8r6_feas_online(cache,A,b,opt)
% Production-shaped feasibility solve: ONE QP only.
% No slack-diagnostic QP in the online path.  Objective is intentionally the
% proven P7/P8 feasibility objective; trajectory shaping is done by corridor
% geometry, not by replacing this objective.
r=struct('flag',-99,'rho',NaN,'z_inf',inf,'z',[],'solve_ms',NaN);
lb=-opt.box_z_max*ones(cache.nz,1); ub=opt.box_z_max*ones(cache.nz,1);
qo=optimoptions('quadprog','Display','off','Algorithm','interior-point-convex', ...
    'ConstraintTolerance',1e-9,'OptimalityTolerance',1e-9,'MaxIterations',1500);
H=1e-10*eye(cache.nz); f=zeros(cache.nz,1);
tq=tic;
try
    [z,~,r.flag]=quadprog(H,f,A,b,[],[],lb,ub,[],qo);
    r.solve_ms=1e3*toc(tq);
    if r.flag>0, r.z=z; r.z_inf=norm(z,inf); end
catch
    r.solve_ms=1e3*toc(tq);
end
end

function r=feas_E(cache,A,b,opt)
r=struct('flag',-99,'rho',inf,'z_inf',inf,'z',[]);
lb=-opt.box_z_max*ones(cache.nz,1); ub=opt.box_z_max*ones(cache.nz,1);
qo=optimoptions('quadprog','Display','off','Algorithm','interior-point-convex', ...
    'ConstraintTolerance',1e-9,'OptimalityTolerance',1e-9,'MaxIterations',1500);
H=1e-10*eye(cache.nz); f=zeros(cache.nz,1);
try
    [z,~,r.flag]=quadprog(H,f,A,b,[],[],lb,ub,[],qo);
    if r.flag>0, r.z=z; r.z_inf=norm(z,inf); end
catch
end
Hs=1e-12*eye(cache.nz+1); fs=[zeros(cache.nz,1);1];
try
    [xs,~,fl]=quadprog(Hs,fs,[A,-ones(size(A,1),1)],b,[],[],[lb;0],[ub;inf],[],qo);
    if fl>0, r.rho=max(0,xs(end)); end
catch
end
end

function d=relative_system_support_minus(n,nT,cfg)
% lower-support loss in direction n: Delta_h(-n,nT)
n=n(:); nT=nT(:);
q=(-n)'*nT;
vL=cfg.sys.R_payload;
vQ=cfg.sys.L_cable*q+cfg.sys.R_uav;
vC=max(0,cfg.sys.L_cable*q)+cfg.sys.R_cable;
d=max([vL,vQ,vC]);
end

function h=ellipsoid_support(n,o)
% obstacle support h_E(n) = n'c + sqrt(n'Sn)
n=n(:); c=o.center(:);
h=n'*c+sqrt(max(0,n'*o.S*n));
end


% =========================================================================
% OPTIONS
% =========================================================================
function opt = setup_options()
opt.time_budget_typical_ms = 1.0;
opt.time_budget_worst_ms   = 2.0;

% Generator
opt.qp_reg       = 1e-8;
opt.w_position   = 1.0;
opt.w_jerk       = 2e-3;
opt.w_snap       = 2e-4;
opt.box_z_max    = 8.0;
opt.guide_extra  = 0.02;
opt.active_margin_trigger = 0.35; % generator heuristic only; verifier remains global
opt.separator_grid_N      = 162;  % candidate normal search, not a safety proof

% Corridor/certificate
opt.cert_tol     = 1e-10;
opt.c6_tol       = 1e-7;
opt.a_net_min    = 1e-5;

% Small fixed number of objective samples; matrices are cached.
opt.N_obj = 17;

% quadprog settings.  "active-set" accepts x0 and is useful for small dense
% online QPs.  For deployment, replace this call with a generated/warm-start
% QP solver if profiling shows quadprog dominates the 2 ms budget.
opt.qp_options = optimoptions('quadprog', ...
    'Display','off', ...
    'Algorithm','active-set', ...
    'ConstraintTolerance',1e-8, ...
    'OptimalityTolerance',1e-8, ...
    'MaxIterations',40);
end

% =========================================================================
% STATIC CACHE
% =========================================================================
function cache = build_static_cache(cfg, t0, t1, nom_fun, opt)
p = cfg.p;
N = cfg.N_ctrl;
T = t1-t0;

if p ~= 7
    error('This Phase-7 implementation assumes p=7 for C6 endpoint matching.');
end
if N <= 2*p
    error('N_ctrl must be > 2*p.');
end

knots = clamped_uniform_knots(N,p);
free_idx = (p+1):(N-p);
nf = numel(free_idx);
nz = 3*nf;

D0 = nom_fun(t0);
D1 = nom_fun(t1);
Pfixed = solve_endpoint_fixed(p,N,knots,T,D0,D1);

% Fit free baseline CPs to nominal trajectory.
u_fit = linspace(0,1,max(80,5*N))';
A = zeros(numel(u_fit),nf);
Y = zeros(numel(u_fit),3);
for k=1:numel(u_fit)
    B = basis_derivative_all(u_fit(k),knots,p,N,0);
    D = nom_fun(t0+u_fit(k)*T);
    A(k,:) = B(free_idx);
    Y(k,:) = D(:,1)' - B*Pfixed;
end
Pbase = Pfixed;
Pbase(free_idx,:) = A\Y;

cache = struct();
cache.p = p;
cache.N = N;
cache.T = T;
cache.t0 = t0;
cache.t1 = t1;
cache.knots = knots;
cache.free_idx = free_idx;
cache.nf = nf;
cache.nz = nz;
cache.Pbase = Pbase;

% Nonzero knot spans.
uk = unique(knots);
cache.spans = [uk(1:end-1)', uk(2:end)'];
cache.spans = cache.spans(cache.spans(:,2)>cache.spans(:,1),:);
cache.nspans = size(cache.spans,1);

% Exact polynomial interpolation matrix used only during cache construction.
xi = linspace(0,1,p+1)';
Bern = zeros(p+1);
for j=1:p+1
    Bern(j,:) = bernstein_row(p,xi(j));
end
cache.Bern_inv = inv(Bern);

% Precompute affine Bezier maps for every position span:
% B_j(z) = B0(:,j) + Bz(:,:,j)*z.
cache.B0 = cell(cache.nspans,1);
cache.Bz = cell(cache.nspans,1);
for s=1:cache.nspans
    ua=cache.spans(s,1); ub=cache.spans(s,2);
    [B0,Bz] = span_bezier_affine(cache,ua,ub);
    cache.B0{s}=B0;
    cache.Bz{s}=Bz;
end

% Objective Hessian/f vector cached.
[cache.H, cache.f] = build_objective_cache(cache,nom_fun,opt);

% Nominal midpoint geometry cache.
cache.mid = zeros(cache.nspans,1);
cache.pmid = zeros(3,cache.nspans);
cache.amid = zeros(3,cache.nspans);
for s=1:cache.nspans
    u=mean(cache.spans(s,:));
    cache.mid(s)=u;
    cache.pmid(:,s)=eval_spline(cache,Pbase,u,0);
    cache.amid(:,s)=eval_spline(cache,Pbase,u,2);
end
end

function [H,f] = build_objective_cache(cache,nom_fun,opt)
H = opt.qp_reg*eye(cache.nz);
f = zeros(cache.nz,1);
us = linspace(0,1,opt.N_obj);
for k=1:numel(us)
    u=us(k);
    t=cache.t0+u*cache.T;
    D=nom_fun(t);

    J0=derivative_map(cache,u,0);
    q0=eval_spline(cache,cache.Pbase,u,0);
    e0=q0-D(:,1);
    H=H+2*opt.w_position*(J0'*J0)/numel(us);
    f=f+2*opt.w_position*J0'*e0/numel(us);

    J3=derivative_map(cache,u,3);
    q3=eval_spline(cache,cache.Pbase,u,3);
    H=H+2*opt.w_jerk*(J3'*J3)/numel(us);
    f=f+2*opt.w_jerk*J3'*q3/numel(us);

    J4=derivative_map(cache,u,4);
    q4=eval_spline(cache,cache.Pbase,u,4);
    H=H+2*opt.w_snap*(J4'*J4)/numel(us);
    f=f+2*opt.w_snap*J4'*q4/numel(us);
end
H=(H+H')/2;
end


% =========================================================================
% PHASE 7-11 ACTIVE CORRIDOR GENERATOR
% =========================================================================
function [A,b,info] = build_certified_active_corridor(cache,cfg,obs,opt)
% Candidate generation only.
% A span/obstacle pair is activated when the nominal trajectory is not
% comfortably separated.  Normal selection searches a small deterministic
% sphere and chooses the best nominal certified lower-bound direction.
%
% SAFETY DOES NOT DEPEND ON THIS PRUNING:
% certify_candidate_global() later checks ALL spans x ALL obstacles.

dirs=fibonacci_sphere(opt.separator_grid_N);
rowsA={}; rowsb={};
info.normal=zeros(3,cache.nspans,numel(obs));
info.active=false(cache.nspans,numel(obs));
info.nominal_g=-inf(cache.nspans,numel(obs));
info.n_constraints=0;

for sp=1:cache.nspans
    ua=cache.spans(sp,1); ub=cache.spans(sp,2);
    Pbez=cache.B0{sp};
    Abez=span_bezier_points(cache,cache.Pbase,ua,ub,2);
    Wbez=Abez+repmat(cfg.g_acc*cfg.e3,1,size(Abez,2));
    Mlb=compute_certified_tension_lower_bound_cols(Wbez);

    for o=1:numel(obs)
        [g_best,n_best]=best_separator_grid(Pbez,Wbez,Mlb,obs(o),cfg,dirs);
        info.normal(:,sp,o)=n_best;
        info.nominal_g(sp,o)=g_best;

        % Generator heuristic: activate unsafe/near obstacle spans.
        % Omitted spans are still checked by the independent verifier.
        active = (g_best < opt.active_margin_trigger);
        info.active(sp,o)=active;
        if ~active, continue; end

        % Freeze nominal cable direction ONLY for convex candidate generation.
        um=mean([ua ub]);
        a0=eval_spline(cache,cache.Pbase,um,2);
        w0=a0+cfg.g_acc*cfg.e3;
        if norm(w0)<=opt.a_net_min
            nT0=cfg.e3;
        else
            nT0=w0/norm(w0);
        end
        rhs=obstacle_support(obs(o),n_best)+ ...
            delta_sep_exact(n_best,nT0,cfg.sys)+cfg.d_margin+opt.guide_extra;

        B0=cache.B0{sp}; Bz=cache.Bz{sp};
        for j=1:cache.p+1
            rowsA{end+1,1}=-n_best'*Bz(:,:,j); %#ok<AGROW>
            rowsb{end+1,1}= n_best'*B0(:,j)-rhs; %#ok<AGROW>
        end
    end
end

if isempty(rowsA)
    A=zeros(0,cache.nz); b=zeros(0,1);
else
    A=vertcat(rowsA{:}); b=vertcat(rowsb{:});
end
info.n_constraints=size(A,1);
info.n_active_pairs=nnz(info.active);
end

function [bg,bn]=best_separator_grid(Pbez,Wbez,Mlb,ob,cfg,dirs)
bg=-inf; bn=dirs(:,1);
for k=1:size(dirs,2)
    n=dirs(:,k);
    g=span_system_g_lower_M(n,Pbez,Wbez,Mlb,ob,cfg);
    if g>bg
        bg=g; bn=n;
    end
end
end

% =========================================================================
% PHASE 7-11 INDEPENDENT GLOBAL CONTINUOUS-TIME VERIFIER
% =========================================================================
function cert=certify_candidate_global(P,cache,cfg,limits,obs,opt)
% Independent execution gate.
% - checks EVERY span x EVERY obstacle
% - does not trust active-span pruning
% - does not trust frozen n_T from generator
% - uses Phase-5 M_LB to certify ||a_L+g e3|| away from zero
% - separator directions are candidate witnesses; for each chosen fixed n,
%   the resulting g_LB is a continuous-time sufficient certificate.

[Actrl,Vctrl,Jctrl]=derivative_control_polygons(P,cache);
v_bound=max(vecnorm(Vctrl,2,2));
a_bound=max(vecnorm(Actrl,2,2));
j_bound=max(vecnorm(Jctrl,2,2));
dyn_pass=(v_bound<=limits.v_max)&&(a_bound<=limits.a_max)&&(j_bound<=limits.j_max);

dirs=fibonacci_sphere(max(162,opt.separator_grid_N));
all_safe=true; cable_safe=true; min_g=inf; min_M=inf;
worst=struct('span',0,'obs',0,'g_lower',inf,'M_LB',inf,'n',[0;0;0]);

for sp=1:cache.nspans
    ua=cache.spans(sp,1); ub=cache.spans(sp,2);
    Pbez=span_bezier_points(cache,P,ua,ub,0);
    Abez=span_bezier_points(cache,P,ua,ub,2);
    Wbez=Abez+repmat(cfg.g_acc*cfg.e3,1,size(Abez,2));

    Mlb=compute_certified_tension_lower_bound_cols(Wbez);
    min_M=min(min_M,Mlb);
    if Mlb<=opt.a_net_min
        cable_safe=false; all_safe=false;
    end

    for o=1:numel(obs)
        [g,n]=best_separator_grid(Pbez,Wbez,Mlb,obs(o),cfg,dirs);
        if g < -opt.cert_tol
            all_safe=false;
        end
        if g<min_g
            min_g=g;
            worst=struct('span',sp,'obs',o,'g_lower',g,'M_LB',Mlb,'n',n);
        end
    end
end

cert=struct();
cert.pass=all_safe && cable_safe && dyn_pass;
cert.geometry_pass=all_safe;
cert.dynamic_pass=dyn_pass;
cert.cable_attitude_defined=cable_safe;
cert.min_g_lower=min_g;
cert.min_M_LB=min_M;
cert.worst=worst;
cert.v_bound=v_bound;
cert.a_bound=a_bound;
cert.j_bound=j_bound;
end

% =========================================================================
% FAST SYSTEM-AWARE CORRIDOR GENERATOR
% =========================================================================
function [A,b,info] = build_fast_system_corridor(cache,cfg,obs,opt)
% One separating normal per span/obstacle.
% No hard-coded +Y direction.  Normal comes from local nominal geometry.
%
% The QP uses n_T frozen at the nominal span midpoint:
%   n'p_L >= h_O(n) + Delta_sep(n,n_T_nom) + margin.
%
% This frozen model is ONLY a candidate generator.  Safety is established
% later by certify_candidate(), which bounds n_T continuously.

max_rows = cache.nspans*numel(obs)*(cache.p+1);
A = zeros(max_rows,cache.nz);
b = zeros(max_rows,1);
row=0;

info.normal = zeros(3,cache.nspans,numel(obs));
info.h_obs = zeros(cache.nspans,numel(obs));
info.delta_frozen = zeros(cache.nspans,numel(obs));

for s=1:cache.nspans
    p0=cache.pmid(:,s);
    a0=cache.amid(:,s);
    w0=a0+cfg.g_acc*cfg.e3;
    if norm(w0) <= opt.a_net_min
        nT0=cfg.e3;
    else
        nT0=w0/norm(w0);
    end

    for o=1:numel(obs)
        n = choose_separating_normal(p0,obs(o));
        h = obstacle_support(obs(o),n);
        Delta = delta_sep_exact(n,nT0,cfg.sys);
        rhs = h + Delta + cfg.d_margin + opt.guide_extra;

        info.normal(:,s,o)=n;
        info.h_obs(s,o)=h;
        info.delta_frozen(s,o)=Delta;

        B0=cache.B0{s};
        Bz=cache.Bz{s};
        for j=1:cache.p+1
            % n'*(B0+Bz*z) >= rhs  ->  -n'Bz*z <= n'B0-rhs
            row=row+1;
            A(row,:)=-n'*Bz(:,:,j);
            b(row)=n'*B0(:,j)-rhs;
        end
    end
end

A=A(1:row,:);
b=b(1:row);
info.n_constraints=row;
end

function n = choose_separating_normal(p,ob)
% General convex obstacle interface still needs a reference center to choose
% a useful candidate normal.  Safety itself uses only support_fun.
if ~isfield(ob,'center')
    error('Each obstacle needs center for normal generation.');
end
v=p-ob.center;
if norm(v)<1e-12
    if isfield(ob,'R')
        v=ob.R(:,1);
    else
        v=[1;0;0];
    end
end
n=v/norm(v);
end


% =========================================================================
% PHASE 7-09 FEASIBILITY DIAGNOSTIC
% =========================================================================
function d = diagnose_qp_feasibility(cache,A,b,cfg,obs,corridor,opt)
% Diagnose the SAME linear corridor used by Phase 7-08.
% This function is diagnostic/offline: its runtime is NOT part of the final
% 1 ms / 2 ms target.
%
% Tests:
% D1 row-wise reachability under |z_i| <= box_z_max
% D2 zero/near-zero authority rows
% D3 global feasibility of A*z<=b with box
% D4 minimum uniform slack rho*
% D5 box-size sensitivity
% D6 separator-normal quality on problematic spans (Phase-6 idea)

nz=cache.nz;
zmax=opt.box_z_max;

% ----- D1: exact row-wise minimum over a box ------------------------------
% min a_i*z = -zmax*sum(abs(a_i))
row_min = -zmax*sum(abs(A),2);
row_margin = b-row_min; % >=0 means row can be satisfied individually
bad_row = row_margin < -1e-10;

authority = sum(abs(A),2);
zero_auth = authority < 1e-12;
zero_auth_bad = zero_auth & (b < -1e-10);

d.n_rows=size(A,1);
d.n_row_unreachable=sum(bad_row);
d.n_zero_authority=sum(zero_auth);
d.n_zero_authority_violated=sum(zero_auth_bad);
d.worst_row_margin=min(row_margin);

% Map row -> span, obstacle, Bezier point.
p1=cache.p+1;
nobs=numel(obs);
d.row_span=zeros(d.n_rows,1);
d.row_obs=zeros(d.n_rows,1);
d.row_bez=zeros(d.n_rows,1);
for r=1:d.n_rows
    block=ceil(r/p1);
    d.row_bez(r)=r-(block-1)*p1;
    d.row_span(r)=ceil(block/nobs);
    d.row_obs(r)=block-(d.row_span(r)-1)*nobs;
end

% ----- D2/D3: pure feasibility QP ----------------------------------------
H=1e-10*eye(nz);
f=zeros(nz,1);
lb=-zmax*ones(nz,1);
ub= zmax*ones(nz,1);
qopt=optimoptions('quadprog','Display','off','Algorithm','interior-point-convex', ...
    'ConstraintTolerance',1e-9,'OptimalityTolerance',1e-9,'MaxIterations',200);
try
    [zf,~,flagf,outf]=quadprog(H,f,A,b,[],[],lb,ub,[],qopt);
catch
    zf=[]; flagf=-99; outf=struct();
end
d.global_feasible=(flagf>0);
d.feasibility_flag=flagf;
if isempty(zf)
    d.max_violation_at_feas=NaN;
else
    d.max_violation_at_feas=max(A*zf-b);
end
if isfield(outf,'iterations'), d.feasibility_iterations=outf.iterations; else, d.feasibility_iterations=NaN; end

% ----- D4: minimum uniform relaxation rho* -------------------------------
% min rho
% s.t. A*z - rho <= b, box(z), rho>=0.
Hsl=1e-12*eye(nz+1);
fsl=[zeros(nz,1);1];
Asl=[A,-ones(size(A,1),1)];
lbsl=[lb;0];
ubsl=[ub;inf];
try
    [xs,~,flags]=quadprog(Hsl,fsl,Asl,b,[],[],lbsl,ubsl,[],qopt);
catch
    xs=[]; flags=-99;
end
d.slack_flag=flags;
if isempty(xs)
    d.rho_star=inf;
else
    d.rho_star=xs(end);
end

% ----- D5: box sensitivity -----------------------------------------------
box_list=[0.5 1 2 4 8 12 20 40];
d.box_list=box_list;
d.box_row_unreachable=zeros(size(box_list));
d.box_global_flag=zeros(size(box_list));
for k=1:numel(box_list)
    zm=box_list(k);
    rm=b-(-zm*sum(abs(A),2));
    d.box_row_unreachable(k)=sum(rm < -1e-10);
    lbk=-zm*ones(nz,1); ubk=zm*ones(nz,1);
    try
        [~,~,fk]=quadprog(H,f,A,b,[],[],lbk,ubk,[],qopt);
    catch
        fk=-99;
    end
    d.box_global_flag(k)=fk;
end

% ----- D6: separator diagnostic on only the worst/problematic spans -------
% Phase 6 used a global sphere diagnostic.  Here we use a compact Fibonacci
% sphere (162 directions) only on spans implicated by unreachable rows.
bad_spans=unique(d.row_span(bad_row));
if isempty(bad_spans)
    % If rows are individually reachable but globally incompatible, inspect
    % the spans with the smallest row margins.
    [~,idx]=sort(row_margin,'ascend');
    bad_spans=unique(d.row_span(idx(1:min(16,numel(idx)))));
end
bad_spans=bad_spans(1:min(4,numel(bad_spans)));
d.separator=struct('span',{},'obs',{},'g_current',{},'g_best_nominal',{}, ...
    'improvement',{},'n_current',{},'n_best',{});

dirs=fibonacci_sphere(162);
for ss=1:numel(bad_spans)
    s=bad_spans(ss);
    Pbez0=cache.B0{s}; % baseline trajectory
    Abez0=span_bezier_points(cache,cache.Pbase,cache.spans(s,1),cache.spans(s,2),2);
    Wbez0=Abez0+repmat(cfg.g_acc*cfg.e3,1,size(Abez0,2));
    for o=1:nobs
        nc=corridor.normal(:,s,o);
        gc=span_system_g_lower(nc,Pbez0,Wbez0,obs(o),cfg);
        gb=-inf; nb=nc;
        for q=1:size(dirs,2)
            n=dirs(:,q);
            gq=span_system_g_lower(n,Pbez0,Wbez0,obs(o),cfg);
            if gq>gb, gb=gq; nb=n; end
        end
        e=struct();
        e.span=s; e.obs=o; e.g_current=gc; e.g_best_nominal=gb;
        e.improvement=gb-gc; e.n_current=nc; e.n_best=nb;
        d.separator(end+1)=e; %#ok<AGROW>
    end
end

% Diagnosis label.
if d.n_zero_authority_violated>0
    d.primary_cause='FIXED_C6_ZERO_AUTHORITY';
elseif d.n_row_unreachable>0
    d.primary_cause='ROW_UNREACHABLE_WITHIN_Z_BOX';
elseif ~d.global_feasible && isfinite(d.rho_star) && d.rho_star>1e-8
    d.primary_cause='COLLECTIVE_HALFSPACE_CONFLICT';
elseif ~d.global_feasible
    d.primary_cause='SOLVER_OR_NUMERICAL_FAILURE';
else
    d.primary_cause='LINEAR_CORRIDOR_FEASIBLE';
end
end

function g=span_system_g_lower(n,Pbez,Wbez,ob,cfg)
n=n/norm(n);
p_low=min(n'*Pbez);
proj=n'*Wbez;
wmin=min(proj);
Mmax=max(vecnorm(Wbez,2,1));
if Mmax<=1e-12
    qlow=-1;
elseif wmin>=0
    qlow=wmin/Mmax;
else
    qlow=-1;
end
Dup=delta_sep_from_q_lower(qlow,cfg.sys);
h=obstacle_support(ob,n);
g=p_low-Dup-h-cfg.d_margin;
end

function D=fibonacci_sphere(N)
% Nearly uniform deterministic unit directions.
D=zeros(3,N);
phi=(1+sqrt(5))/2;
for k=0:N-1
    z=1-2*(k+0.5)/N;
    r=sqrt(max(0,1-z*z));
    th=2*pi*k/phi;
    D(:,k+1)=[r*cos(th);r*sin(th);z];
end
end

function print_diagnostic(d)
fprintf('\n================================================================================\n');
fprintf('[Phase 7-10 FULL MATHEMATICAL AUDIT]\n');
fprintf('================================================================================\n');
fprintf('rows                              : %d\n',d.n_rows);
fprintf('individually unreachable rows     : %d\n',d.n_row_unreachable);
fprintf('zero-authority rows               : %d\n',d.n_zero_authority);
fprintf('violated zero-authority rows      : %d\n',d.n_zero_authority_violated);
fprintf('worst row reachability margin     : %+.6e m\n',d.worst_row_margin);
fprintf('pure feasibility QP flag          : %d\n',d.feasibility_flag);
fprintf('global linear corridor feasible   : %d\n',d.global_feasible);
fprintf('minimum uniform slack rho*        : %.6e m\n',d.rho_star);
fprintf('primary diagnosis                 : %s\n',d.primary_cause);

fprintf('\n[box sensitivity]\n');
fprintf(' zmax [m] | unreachable rows | feasibility flag\n');
for k=1:numel(d.box_list)
    fprintf(' %8.2f | %16d | %d\n',d.box_list(k),d.box_row_unreachable(k),d.box_global_flag(k));
end

if ~isempty(d.separator)
    fprintf('\n[separator diagnostic: baseline trajectory]\n');
    fprintf(' span | obs | current g_LB | best g_LB(162 dirs) | improvement\n');
    for k=1:numel(d.separator)
        e=d.separator(k);
        fprintf(' %4d | %3d | %+12.6f | %+19.6f | %+11.6f\n', ...
            e.span,e.obs,e.g_current,e.g_best_nominal,e.improvement);
    end
end

if d.n_row_unreachable>0
    fprintf('\n[worst unreachable rows]\n');
    % compact list from already stored row metadata is intentionally omitted
    % from runtime state; cause is summarized above.
end
fprintf('================================================================================\n');
end


% =========================================================================
% PHASE 7-10 FULL MATHEMATICAL AUDIT
% =========================================================================
function a = run_full_mathematical_audit(cache,A,b,cfg,limits,obs,corridor,opt)
% OFFLINE diagnostic.  Not part of final online latency.
%
% A1 exact row metadata and top violated/reachability rows
% A2 unconstrained-box feasibility (tests whether z-box is the only cause)
% A3 per-span authority / nominal certified geometry / tension M_LB
% A4 1250-direction global separator diagnostic (Phase-6 inheritance)
% A5 active-span necessity classification
% A6 N_ctrl feasibility sweep with the SAME endpoint-C6 construction
%
% IMPORTANT:
% "safe_nominal" below means certified by at least one tested separating
% direction.  A negative grid maximum means NOT CERTIFIED by this finite
% diagnostic search; it is not a collision proof.

zmax=opt.box_z_max;
nr=size(A,1);
row_min=-zmax*sum(abs(A),2);
margin=b-row_min;
authority=sum(abs(A),2);

p1=cache.p+1; nobs=numel(obs);
meta=struct('row',{},'span',{},'obs',{},'bez',{},'b',{},'authority',{}, ...
    'row_min',{},'margin',{},'normal',{});
for r=1:nr
    block=ceil(r/p1);
    bez=r-(block-1)*p1;
    sp=ceil(block/nobs);
    ob=block-(sp-1)*nobs;
    e.row=r; e.span=sp; e.obs=ob; e.b=b(r); e.bez=bez;
    e.authority=authority(r); e.row_min=row_min(r); e.margin=margin(r);
    e.normal=corridor.normal(:,sp,ob);
    meta(end+1)=e; %#ok<AGROW>
end
[~,ord]=sort(margin,'ascend');
a.worst_rows=meta(ord(1:min(20,nr)));
a.n_unreachable=sum(margin<0);

% A2: remove z box entirely.
H=1e-10*eye(cache.nz); f=zeros(cache.nz,1);
qopt=optimoptions('quadprog','Display','off','Algorithm','interior-point-convex', ...
    'ConstraintTolerance',1e-9,'OptimalityTolerance',1e-9,'MaxIterations',500);
try
    [zu,~,fu]=quadprog(H,f,A,b,[],[],[],[],[],qopt);
catch
    zu=[]; fu=-99;
end
a.unboxed_flag=fu;
a.unboxed_feasible=fu>0;
if isempty(zu), a.unboxed_z_inf=inf; else, a.unboxed_z_inf=norm(zu,inf); end

% A3-A5 span audit.
a.span=struct('span',{},'u0',{},'u1',{},'authority_sum',{}, ...
    'M_LB',{},'v_UB',{},'a_UB',{},'j_UB',{},'snap_UB',{},'aQ_UB',{}, ...
    'g_current',{},'g_global_grid',{},'n_global',{}, ...
    'nominal_certified_grid',{},'constraint_needed_nominal',{});

dirs=phase6_sphere_grid();
for sp=1:cache.nspans
    ua=cache.spans(sp,1); ub=cache.spans(sp,2);
    Pbez=span_bezier_points(cache,cache.Pbase,ua,ub,0);
    Vbez=span_bezier_points(cache,cache.Pbase,ua,ub,1);
    Abez=span_bezier_points(cache,cache.Pbase,ua,ub,2);
    Jbez=span_bezier_points(cache,cache.Pbase,ua,ub,3);
    Sbez=span_bezier_points(cache,cache.Pbase,ua,ub,4);
    Wbez=Abez+repmat(cfg.g_acc*cfg.e3,1,size(Abez,2));

    Mlb=compute_certified_tension_lower_bound_cols(Wbez);
    vub=max(vecnorm(Vbez,2,1));
    aub=max(vecnorm(Abez,2,1));
    jub=max(vecnorm(Jbez,2,1));
    sub=max(vecnorm(Sbez,2,1));
    if Mlb>0
        omega_up=jub/Mlb;
        alpha_up=(sub/Mlb)+2*(jub/Mlb)*omega_up+omega_up^2;
        aQub=aub+cfg.sys.L_cable*alpha_up;
    else
        aQub=inf;
    end

    % Report worst obstacle in this span.
    gcur=inf; gglob=inf; nglob=[1;0;0];
    for ob=1:nobs
        nc=corridor.normal(:,sp,ob);
        gc=span_system_g_lower_M(nc,Pbez,Wbez,Mlb,obs(ob),cfg);
        [gg,ng]=global_separator_grid(Pbez,Wbez,Mlb,obs(ob),cfg,dirs);
        if gc<gcur, gcur=gc; end
        if gg<gglob, gglob=gg; nglob=ng; end
    end

    rows_sp=find([meta.span]==sp);
    auth_sp=sum(authority(rows_sp));

    e.span=sp; e.u0=ua; e.u1=ub; e.authority_sum=auth_sp;
    e.M_LB=Mlb; e.v_UB=vub; e.a_UB=aub; e.j_UB=jub; e.snap_UB=sub; e.aQ_UB=aQub;
    e.g_current=gcur; e.g_global_grid=gglob; e.n_global=nglob;
    e.nominal_certified_grid=(gglob>=0);
    % If nominal is already certified, forcing the QP to move that span to a
    % newly chosen halfspace is not necessary for nominal safety.
    e.constraint_needed_nominal=~e.nominal_certified_grid;
    a.span(end+1)=e; %#ok<AGROW>
end

a.min_M_LB=min([a.span.M_LB]);
a.min_g_current=min([a.span.g_current]);
a.min_g_global_grid=min([a.span.g_global_grid]);
a.n_nominal_certified=sum([a.span.nominal_certified_grid]);
a.n_nominal_uncertified=cache.nspans-a.n_nominal_certified;

% A6: identify whether fixed dimension is structurally too small.
Nlist=unique([18 22 26 30 34]);
a.Nctrl=struct('N',{},'nz',{},'rows',{},'unreachable',{},'feas_flag',{},'rho',{});
for k=1:numel(Nlist)
    N=Nlist(k);
    if N<=2*cfg.p, continue; end
    cfgk=cfg; cfgk.N_ctrl=N;
    try
        ck=build_static_cache(cfgk,cache.t0,cache.t1,@benchmark_nominal,opt);
        [Ak,bk,~]=build_fast_system_corridor(ck,cfgk,obs,opt);
        rm=bk-(-zmax*sum(abs(Ak),2));
        Hk=1e-10*eye(ck.nz); fk=zeros(ck.nz,1);
        lb=-zmax*ones(ck.nz,1); ub=zmax*ones(ck.nz,1);
        [~,~,fl]=quadprog(Hk,fk,Ak,bk,[],[],lb,ub,[],qopt);

        Hs=1e-12*eye(ck.nz+1); fs=[zeros(ck.nz,1);1];
        As=[Ak,-ones(size(Ak,1),1)];
        [xs,~,~]=quadprog(Hs,fs,As,bk,[],[],[lb;0],[ub;inf],[],qopt);
        rho=xs(end);
        ek.N=N; ek.nz=ck.nz; ek.rows=size(Ak,1);
        ek.unreachable=sum(rm<0); ek.feas_flag=fl; ek.rho=rho;
    catch
        ek.N=N; ek.nz=NaN; ek.rows=NaN; ek.unreachable=NaN; ek.feas_flag=-99; ek.rho=NaN;
    end
    a.Nctrl(end+1)=ek; %#ok<AGROW>
end

% Overall interpretation.
if ~a.unboxed_feasible
    a.interpretation='HALFSPACE_SET_INCOMPATIBLE_EVEN_WITHOUT_Z_BOX';
elseif a.n_unreachable>0
    a.interpretation='BOX_AND_CORRIDOR_GEOMETRY_LIMIT_REACHABILITY';
elseif a.n_nominal_uncertified>0
    a.interpretation='CORRIDOR_FEASIBLE_BUT_NOMINAL_HAS_UNCERTIFIED_SPANS';
else
    a.interpretation='LINEAR_CORRIDOR_AND_NOMINAL_CERTIFICATE_DIAGNOSTIC_PASS';
end
end

function M=compute_certified_tension_lower_bound_cols(Wc)
% Phase-5 formula, adapted from row-wise W to columns Wc.
W=Wc';
Q=W*W';
nv=size(W,1);
lam=ones(nv,1)/nv;
y=lam;
L=max(eig(Q));
t_step=1/(L+1e-6);
for iter=1:20
    prev=lam;
    grad=Q*y;
    lam=project_to_simplex_local(y-t_step*grad);
    y=lam+((iter-1)/(iter+2))*(lam-prev);
end
what=W'*lam;
Mhat=norm(what);
if Mhat>0
    v=what/Mhat;
    M=max(0,min(W*v));
else
    M=0;
end
end

function x=project_to_simplex_local(v)
n=length(v); u=sort(v,'descend'); cssv=cumsum(u);
rho=find(u>(cssv-1)./(1:n)',1,'last');
theta=(cssv(rho)-1)/rho;
x=max(v-theta,0);
end

function g=span_system_g_lower_M(n,Pbez,Wbez,Mlb,ob,cfg)
n=n/norm(n);
p_low=min(n'*Pbez);
wmin=min(n'*Wbez);
Mmax=max(vecnorm(Wbez,2,1));
if Mmax<=1e-12
    qlow=-1;
elseif wmin>=0
    qlow=wmin/Mmax;
elseif Mlb>0
    qlow=max(-1,wmin/Mlb);
else
    qlow=-1;
end
Dup=delta_sep_from_q_lower(qlow,cfg.sys);
g=p_low-Dup-obstacle_support(ob,n)-cfg.d_margin;
end

function [bg,bn]=global_separator_grid(Pbez,Wbez,Mlb,ob,cfg,dirs)
bg=-inf; bn=[1;0;0];
for k=1:size(dirs,2)
    n=dirs(:,k);
    g=span_system_g_lower_M(n,Pbez,Wbez,Mlb,ob,cfg);
    if g>bg, bg=g; bn=n; end
end
end

function D=phase6_sphere_grid()
Ntheta=25; Nphi=50;
th=linspace(0.01,pi-0.01,Ntheta);
ph=linspace(0,2*pi,Nphi+1); ph(end)=[];
D=zeros(3,Ntheta*Nphi); k=0;
for i=1:Ntheta
    for j=1:Nphi
        k=k+1;
        D(:,k)=[sin(th(i))*cos(ph(j));sin(th(i))*sin(ph(j));cos(th(i))];
    end
end
end

function print_full_audit(a)
fprintf('\n================================================================================\n');
fprintf('[Phase 7-10 FULL MATHEMATICAL AUDIT]\n');
fprintf('================================================================================\n');
fprintf('unreachable rows @ current box : %d\n',a.n_unreachable);
fprintf('unboxed feasibility flag       : %d\n',a.unboxed_flag);
fprintf('unboxed feasible               : %d\n',a.unboxed_feasible);
fprintf('minimum ||z||_inf unboxed sol. : %.6f m\n',a.unboxed_z_inf);
fprintf('min Phase-5 M_LB               : %.6e m/s^2\n',a.min_M_LB);
fprintf('min current-normal g_LB        : %+.6e m\n',a.min_g_current);
fprintf('min global-grid g_LB           : %+.6e m\n',a.min_g_global_grid);
fprintf('nominal certified spans        : %d\n',a.n_nominal_certified);
fprintf('nominal uncertified spans      : %d\n',a.n_nominal_uncertified);
fprintf('interpretation                 : %s\n',a.interpretation);

fprintf('\n[20 worst linear rows]\n');
fprintf(' row | span | obs | bez | authority | row_min | b | reach.margin\n');
for k=1:numel(a.worst_rows)
    e=a.worst_rows(k);
    fprintf('%4d | %4d | %3d | %3d | %9.3e | %+8.4f | %+8.4f | %+10.6f\n', ...
        e.row,e.span,e.obs,e.bez,e.authority,e.row_min,e.b,e.margin);
end

fprintf('\n[span-by-span mathematical audit]\n');
fprintf('span | [u0,u1] | M_LB | g_current | g_grid1250 | vUB | aUB | jUB | aQUB | nominal cert\n');
for k=1:numel(a.span)
    e=a.span(k);
    fprintf('%4d | [%.3f,%.3f] | %.4f | %+8.4f | %+9.4f | %.3f | %.3f | %.3f | %.3f | %d\n', ...
        e.span,e.u0,e.u1,e.M_LB,e.g_current,e.g_global_grid,e.v_UB,e.a_UB,e.j_UB,e.aQ_UB,e.nominal_certified_grid);
end

fprintf('\n[N_ctrl structural sweep]\n');
fprintf(' Nctrl | nz | rows | unreachable | feas.flag | rho*\n');
for k=1:numel(a.Nctrl)
    e=a.Nctrl(k);
    fprintf('%6d | %3.0f | %4.0f | %11.0f | %9d | %.6e\n', ...
        e.N,e.nz,e.rows,e.unreachable,e.feas_flag,e.rho);
end
fprintf('================================================================================\n');
end

% =========================================================================
% SMALL QP
% =========================================================================
function [z,flag,info] = solve_fast_qp(cache,A,b,cfg,opt)
lb=-opt.box_z_max*ones(cache.nz,1);
ub= opt.box_z_max*ones(cache.nz,1);

% Warm-start hook.  In the test function use zero; online integration should
% replace x0 by the previous cycle's certified/solved z after horizon shift.
x0=zeros(cache.nz,1);

try
    [z,~,flag,out]=quadprog(cache.H,cache.f,A,b,[],[],lb,ub,x0,opt.qp_options);
catch ME
    z=[];
    flag=-99;
    info=struct('message',ME.message,'iterations',NaN);
    return;
end

info=struct();
if isfield(out,'iterations'), info.iterations=out.iterations; else, info.iterations=NaN; end
if isfield(out,'message'), info.message=out.message; else, info.message=''; end
end

% =========================================================================
% INDEPENDENT CONTINUOUS-TIME CERTIFICATE
% =========================================================================
function cert = certify_candidate(P,cache,cfg,limits,obs,corridor,opt)
% Sound sufficient certificate for each span/obstacle and the exact normal
% selected by the corridor generator.
%
% For a fixed n:
%   g(t;n)=n'p_L(t)-Delta_sep(n,n_T(t))-h_O(n)-d_margin.
%
% Position:
%   p_low = min BezierCP(n'p_L) <= n'p_L(t), all t in span.
%
% Cable attitude:
%   W(t)=a_L(t)+g e3.
%   W(t) lies in convex hull of acceleration Bezier CPs.
%   If w_min=min n'W_i >=0:
%       n'n_T = n'W/||W|| >= w_min/M_max.
%   Otherwise q_lower=-1 is always valid.
%
% Delta_sep is monotonically non-increasing in q=n'n_T:
%   Delta <= Delta_from_q(q_lower).
%
% Hence g_lower is a continuous-time lower bound.

[Actrl, Vctrl, Jctrl] = derivative_control_polygons(P,cache);

% Continuous-time dynamic feasibility by convex hull + convexity of norm.
v_bound=max(vecnorm(Vctrl,2,2));
a_bound=max(vecnorm(Actrl,2,2));
j_bound=max(vecnorm(Jctrl,2,2));

dyn_pass=(v_bound<=limits.v_max) && ...
         (a_bound<=limits.a_max) && ...
         (j_bound<=limits.j_max);

min_g=inf;
worst=struct('span',0,'obs',0,'g_lower',inf,'q_lower',NaN,'Delta_upper',NaN);
all_safe=true;
singular_safe=true;

% Build Bezier polygons of candidate position and acceleration span-by-span.
for s=1:cache.nspans
    ua=cache.spans(s,1); ub=cache.spans(s,2);
    Pbez=span_bezier_points(cache,P,ua,ub,0);
    Abez=span_bezier_points(cache,P,ua,ub,2);
    Wbez=Abez + repmat(cfg.g_acc*cfg.e3,1,size(Abez,2));

    % Sufficient non-singularity check:
    % if origin cannot be excluded by this simple lower bound, certificate
    % refuses execution.  It never silently normalizes a near-zero vector.
    Wnorm_upper=max(vecnorm(Wbez,2,1));
    if Wnorm_upper <= opt.a_net_min
        singular_safe=false;
        all_safe=false;
    end

    for o=1:numel(obs)
        n=corridor.normal(:,s,o);
        h=obstacle_support(obs(o),n);

        p_low=min(n'*Pbez);
        projW=n'*Wbez;
        w_min=min(projW);
        M_max=max(vecnorm(Wbez,2,1));

        if M_max <= opt.a_net_min
            q_lower=-1.0;
            singular_safe=false;
        elseif w_min>=0
            q_lower=w_min/M_max;
        else
            q_lower=-1.0;
        end

        Delta_up=delta_sep_from_q_lower(q_lower,cfg.sys);
        g_low=p_low-Delta_up-h-cfg.d_margin;

        if g_low < -opt.cert_tol
            all_safe=false;
        end
        if g_low < min_g
            min_g=g_low;
            worst=struct('span',s,'obs',o,'g_lower',g_low, ...
                'q_lower',q_lower,'Delta_upper',Delta_up);
        end
    end
end

cert=struct();
cert.pass=all_safe && dyn_pass && singular_safe;
cert.geometry_pass=all_safe;
cert.dynamic_pass=dyn_pass;
cert.cable_attitude_defined=singular_safe;
cert.min_g_lower=min_g;
cert.worst=worst;
cert.v_bound=v_bound;
cert.a_bound=a_bound;
cert.j_bound=j_bound;
end

% =========================================================================
% P8R4 OFFLINE GEOMETRY-REQUIREMENT FORENSIC
% =========================================================================
function F=p8r4_geometry_requirement_forensic(P,c,ref_fun,obs,cfg,opt)
% Diagnostic only; never used as an execution certificate.
%
% For each time, search fixed directions n and compute the smallest
% translation d*n that would satisfy the instantaneous support half-space
% against ALL currently known obstacles:
%
%   n'*(p_ref+d*n) >= h_O(n)+Delta_component(n,nT)+margin.
%
% This is a certificate-compatible INSTANTANEOUS geometric requirement on a
% finite direction grid.  It is NOT the globally minimum C6 trajectory
% deviation because it deliberately omits temporal continuity and v/a/j.
%
% Component results use the same payload / cable / UAV support terms as the
% planner.  The system requirement is their max, exactly matching
% delta_sep_exact().

ns=121;
dirs=fibonacci_sphere(max(642,opt.separator_grid_N));
u=linspace(0,1,ns);

exec_dev=zeros(1,ns);
reqSys=zeros(1,ns);
reqL=zeros(1,ns);
reqC=zeros(1,ns);
reqQ=zeros(1,ns);
dom=zeros(1,ns);

for k=1:ns
    tt=c.t0+u(k)*c.T;
    D=ref_fun(tt);
    pr=D(:,1);
    ar=D(:,3);
    w=ar+cfg.g_acc*cfg.e3;
    if norm(w)<=opt.a_net_min
        nT=cfg.e3;
    else
        nT=w/norm(w);
    end

    pe=eval_spline(c,P,u(k),0);
    exec_dev(k)=norm(pe-pr);

    best=inf; bestL=inf; bestC=inf; bestQ=inf; bestDom=0;

    for id=1:size(dirs,2)
        n=dirs(:,id);
        q=n'*nT;

        dL=cfg.sys.R_payload;
        dQ=cfg.sys.R_uav-cfg.sys.L_cable*q;
        dC=cfg.sys.R_cable+max(0,-cfg.sys.L_cable*q);

        % One translation direction must clear every known obstacle.
        rL=0; rC=0; rQ=0;
        for io=1:numel(obs)
            h=obstacle_support(obs(io),n)+cfg.d_margin-n'*pr;
            rL=max(rL,h+dL);
            rC=max(rC,h+dC);
            rQ=max(rQ,h+dQ);
        end
        rL=max(0,rL); rC=max(0,rC); rQ=max(0,rQ);
        rs=max([rL rC rQ]);

        if rs<best
            best=rs; bestL=rL; bestC=rC; bestQ=rQ;
            [~,bestDom]=max([rL rC rQ]);
        end
    end

    reqSys(k)=best; reqL(k)=bestL; reqC(k)=bestC; reqQ(k)=bestQ;
    dom(k)=bestDom;
end

F=struct();
F.executed_maxdev=max(exec_dev);
F.executed_rmsdev=sqrt(mean(exec_dev.^2));
F.required_system_max=max(reqSys);
F.required_system_rms=sqrt(mean(reqSys.^2));
F.required_payload_max=max(reqL);
F.required_cable_max=max(reqC);
F.required_uav_max=max(reqQ);
F.dominant_counts=[nnz(dom==1) nnz(dom==2) nnz(dom==3)];
F.excess_abs=F.executed_maxdev-F.required_system_max;
F.excess_ratio=F.executed_maxdev/max(F.required_system_max,1e-9);
F.u=u;
F.executed_dev=exec_dev;
F.required_system=reqSys;
F.required_payload=reqL;
F.required_cable=reqC;
F.required_uav=reqQ;
end

% =========================================================================
% SYSTEM GEOMETRY / OBSTACLE SUPPORT
% =========================================================================
function d = delta_sep_exact(n,nT,sys)
% Phase-3 lower-support separation envelope:
% Delta_sep(n,nT)=Delta_h(-n,nT)
q=n'*nT;
d=delta_sep_from_q_lower(q,sys);
end

function d = delta_sep_from_q_lower(q,sys)
% q is a certified lower bound on n'*nT.
val_L=sys.R_payload;
val_Q=sys.R_uav - sys.L_cable*q;
val_C=sys.R_cable + max(0.0,-sys.L_cable*q);
d=max([val_L,val_Q,val_C]);
end

function h = obstacle_support(ob,n)
% Generic convex obstacle support.
if isfield(ob,'support_fun') && ~isempty(ob.support_fun)
    h=ob.support_fun(n);
elseif isfield(ob,'S') && isfield(ob,'center')
    % Rotated ellipsoid E={c+S^(1/2)u: ||u||<=1}
    h=n'*ob.center + sqrt(max(0,n'*ob.S*n));
else
    error('Obstacle requires support_fun or {center,S}.');
end
end

% =========================================================================
% C6 AUDIT
% =========================================================================
function [err,by] = audit_c6(P,cache,nom_fun,opt)
D0=nom_fun(cache.t0);
D1=nom_fun(cache.t1);
by=zeros(7,1);
for r=0:6
    qs=eval_spline(cache,P,0,r);
    qe=eval_spline(cache,P,1,r);
    by(r+1)=max(norm(qs-D0(:,r+1),inf),norm(qe-D1(:,r+1),inf));
end
err=max(by);
end

% =========================================================================
% B-SPLINE / BEZIER
% =========================================================================
function P = apply_z(cache,z)
P=cache.Pbase;
nf=cache.nf;
P(cache.free_idx,1)=P(cache.free_idx,1)+z(1:nf);
P(cache.free_idx,2)=P(cache.free_idx,2)+z(nf+(1:nf));
P(cache.free_idx,3)=P(cache.free_idx,3)+z(2*nf+(1:nf));
end

function J = derivative_map(cache,u,r)
B=basis_derivative_all(u,cache.knots,cache.p,cache.N,r)/cache.T^r;
bf=B(cache.free_idx);
J=zeros(3,cache.nz);
nf=cache.nf;
J(1,1:nf)=bf;
J(2,nf+(1:nf))=bf;
J(3,2*nf+(1:nf))=bf;
end

function q = eval_spline(cache,P,u,r)
B=basis_derivative_all(u,cache.knots,cache.p,cache.N,r)/cache.T^r;
q=(B*P)';
end

function [B0,Bz] = span_bezier_affine(cache,ua,ub)
p=cache.p;
xi=linspace(0,1,p+1)';
Q0=zeros(p+1,3);
Qz=zeros(p+1,3,cache.nz);
for j=1:p+1
    u=ua+(ub-ua)*xi(j);
    Q0(j,:)=eval_spline(cache,cache.Pbase,u,0)';
    J=derivative_map(cache,u,0);
    for iz=1:cache.nz
        Qz(j,:,iz)=J(:,iz)';
    end
end
Bez0=cache.Bern_inv*Q0;
B0=Bez0';
Bz=zeros(3,cache.nz,p+1);
for iz=1:cache.nz
    tmp=cache.Bern_inv*squeeze(Qz(:,:,iz));
    for j=1:p+1
        Bz(:,iz,j)=tmp(j,:)';
    end
end
end

function Bez = span_bezier_points(cache,P,ua,ub,r)
% Exact degree-(p-r) polynomial reconstruction on one original knot span.
deg=cache.p-r;
xi=linspace(0,1,deg+1)';
M=zeros(deg+1);
Q=zeros(deg+1,3);
for j=1:deg+1
    u=ua+(ub-ua)*xi(j);
    M(j,:)=bernstein_row(deg,xi(j));
    Q(j,:)=eval_spline(cache,P,u,r)';
end
Bez=(M\Q)';
end

function b = bernstein_row(p,x)
b=zeros(1,p+1);
for k=0:p
    b(k+1)=nchoosek(p,k)*x^k*(1-x)^(p-k);
end
end

function [Actrl,Vctrl,Jctrl] = derivative_control_polygons(P,cache)
% Derivative B-spline control polygons in physical time.
Vctrl=derive_ctrl(P,cache.knots,cache.p)/cache.T;
U1=cache.knots(2:end-1);
Actrl=derive_ctrl(Vctrl,U1,cache.p-1)/cache.T;
U2=U1(2:end-1);
Jctrl=derive_ctrl(Actrl,U2,cache.p-2)/cache.T;
end

function D = derive_ctrl(P,U,p)
N=size(P,1);
D=zeros(N-1,3);
for i=1:N-1
    den=U(i+p+1)-U(i+1);
    D(i,:)=p*(P(i+1,:)-P(i,:))/den;
end
end

function P = solve_endpoint_fixed(p,N,U,T,D0,D1)
P=zeros(N,3);
A0=zeros(p); A1=zeros(p);
for r=0:p-1
    q0=basis_derivative_all(0,U,p,N,r)/T^r;
    q1=basis_derivative_all(1,U,p,N,r)/T^r;
    A0(r+1,:)=q0(1:p);
    A1(r+1,:)=q1(end-p+1:end);
end
for ax=1:3
    P(1:p,ax)=A0\D0(ax,1:p)';
    P(end-p+1:end,ax)=A1\D1(ax,1:p)';
end
end

function U = clamped_uniform_knots(N,p)
ni=N-p-1;
if ni>0
    internal=(1:ni)/(ni+1);
else
    internal=[];
end
U=[zeros(1,p+1),internal,ones(1,p+1)];
end

function dN = basis_derivative_all(u,U,p,N,r)
% Cox-de Boor derivative recursion.  r<=p.
if r>p
    dN=zeros(1,N);
    return;
end

% degree-0 basis
M=length(U)-1;
B=zeros(p+1,M);
for i=1:M
    if (u>=U(i) && u<U(i+1)) || (u==1 && U(i+1)==1 && U(i)<1)
        B(1,i)=1;
    end
end

% build basis degrees
for d=1:p
    for i=1:(M-d)
        a=0; c=0;
        den1=U(i+d)-U(i);
        den2=U(i+d+1)-U(i+1);
        if den1~=0, a=(u-U(i))/den1*B(d,i); end
        if den2~=0, c=(U(i+d+1)-u)/den2*B(d,i+1); end
        B(d+1,i)=a+c;
    end
end

if r==0
    dN=B(p+1,1:N);
    return;
end

% Recursive derivative coefficients using lower-degree bases.
dN=zeros(1,N);
for i=1:N
    dN(i)=basis_derivative_single(i,p,r,u,U,B);
end
end

function v = basis_derivative_single(i,p,r,u,U,B)
if r==0
    v=B(p+1,i);
    return;
end
if p==0
    v=0;
    return;
end

% derivative identity:
% d^r N_{i,p} = p/(U_{i+p}-U_i) d^(r-1)N_{i,p-1}
%             - p/(U_{i+p+1}-U_{i+1}) d^(r-1)N_{i+1,p-1}
den1=U(i+p)-U(i);
den2=U(i+p+1)-U(i+1);
a=0; b=0;
if den1~=0
    a=p/den1*basis_derivative_single_general(i,p-1,r-1,u,U);
end
if den2~=0
    b=p/den2*basis_derivative_single_general(i+1,p-1,r-1,u,U);
end
v=a-b;
end

function v = basis_derivative_single_general(i,p,r,u,U)
if r>p
    v=0;
    return;
end
if r==0
    v=basis_single(i,p,u,U);
    return;
end
den1=U(i+p)-U(i);
den2=U(i+p+1)-U(i+1);
a=0; b=0;
if den1~=0
    a=p/den1*basis_derivative_single_general(i,p-1,r-1,u,U);
end
if den2~=0
    b=p/den2*basis_derivative_single_general(i+1,p-1,r-1,u,U);
end
v=a-b;
end

function v = basis_single(i,p,u,U)
if p==0
    v=double((u>=U(i) && u<U(i+1)) || ...
        (u==1 && U(i+1)==1 && U(i)<1));
    return;
end
a=0; b=0;
den1=U(i+p)-U(i);
den2=U(i+p+1)-U(i+1);
if den1~=0
    a=(u-U(i))/den1*basis_single(i,p-1,u,U);
end
if den2~=0
    b=(U(i+p+1)-u)/den2*basis_single(i+1,p-1,u,U);
end
v=a+b;
end


% =========================================================================
% PHASE 7-12F DIAGNOSTIC HELPERS
% =========================================================================
function r=audit_nominal_risk_window(nom_fun,obs,cfg,opt,t0,t1,dt)
ts=t0:dt:t1;
gv=inf(size(ts));
for k=1:numel(ts)
    D=nom_fun(ts(k)); p=D(:,1); a=D(:,3);
    W=a+cfg.g_acc*cfg.e3;
    if norm(W)<=opt.a_net_min
        gv(k)=-inf; continue;
    end
    nT=W/norm(W);
    gobs=inf(1,numel(obs));
    for io=1:numel(obs)
        dirs=fibonacci_sphere(opt.separator_grid_N);
        best=-inf;
        for q=1:size(dirs,2)
            n=dirs(:,q);
            val=n'*p-relative_system_support_minus(n,nT,cfg)-obstacle_support(obs(io),n)-cfg.d_margin;
            best=max(best,val);
        end
        gobs(io)=best;
    end
    gv(k)=min(gobs);
end
bad=gv<0;
ib=find(bad,1,'first');
[gw,iw]=min(gv);
r=struct('t_first_bad',NaN,'t_worst',ts(iw),'g_worst',gw,'t_clear',NaN, ...
    'unsafe_duration',NaN,'t',ts,'g',gv);
if isempty(ib), return; end
r.t_first_bad=ts(ib);
ic=find(~bad & ((1:numel(ts))>ib),1,'first');
if ~isempty(ic), r.t_clear=ts(ic); r.unsafe_duration=r.t_clear-r.t_first_bad; end
end

function d=diagnose_geometry_case(cfg0,N,obs,nom_fun,t0,T,opt)
cfg=cfg0; cfg.N_ctrl=N;
d=struct('N',N,'T',T,'n_active',0,'n_rows',0,'feasible',0,'rho',inf, ...
    'worst_span',0,'worst_row',0,'worst_row_margin',-inf,'after_clear',false);
try
    c=build_static_cache(cfg,t0,t0+T,nom_fun,opt);
    [A,b,info]=build_certified_active_corridor(c,cfg,obs,opt);
    d.n_active=info.n_active_pairs; d.n_rows=size(A,1);
    if isempty(A), d.feasible=1; d.rho=0; return; end
    zmax=opt.box_z_max;
    row_min=-zmax*sum(abs(A),2);
    margin=b-row_min;
    [d.worst_row_margin,d.worst_row]=min(margin);
    % Active-corridor rows are appended in groups of p+1 per active pair.
    d.worst_span=0;
    cnt=0;
    for sp=1:c.nspans
        for io=1:numel(obs)
            if info.active(sp,io)
                rows=cnt+(1:(c.p+1));
                if any(rows==d.worst_row), d.worst_span=sp; end
                cnt=cnt+c.p+1;
            end
        end
    end
    f=feas_E(c,A,b,opt);
    d.feasible=f.flag>0; d.rho=f.rho;
catch
end
end

function a=audit_span_authority(cfg0,N,nom_fun,t0,T,opt)
cfg=cfg0; cfg.N_ctrl=N;
c=build_static_cache(cfg,t0,t0+T,nom_fun,opt);
auth=zeros(c.nspans,1);
for sp=1:c.nspans
    Bz=c.Bz{sp};
    auth(sp)=sum(abs(Bz(:)));
end
[~,ord]=sort(auth,'ascend');
a=struct('N',N,'T',T,'span_authority',auth,'low_spans',ord(1:min(5,numel(ord)))');
end


% =========================================================================
% BENCHMARK / I/O
% =========================================================================

% =========================================================================
% Phase 7-21: production front-end integration
% =========================================================================
function O=run_phase721_production_frontend(cfg,limits,obs,nom_fun,opt,t0,t1,trig7,selected)
dt=0.025; sensor_range=15.0; Tlook=5.0; Npred=11; Nsub=4;
known=false(1,numel(obs)); first_seen=nan(1,numel(obs));
sensor_acc=0; trigger_acc=0;
prod=struct('found',false,'t_now',NaN,'t_risk',NaN,'TTC',NaN,'obs_idx',NaN, ...
    'min_future_dist',inf,'threat_source','');

for t=t0:dt:t1
    D=nom_fun(t); pL=D(:,1); aL=D(:,3);
    W=aL+cfg.g_acc*cfg.e3; 
    if norm(W)<=1e-12, nT=cfg.e3; else, nT=W/norm(W); end
    pQ=pL+cfg.sys.L_cable*nT;

    tt=tic;
    det=prod_sensor_detect(pQ,obs,sensor_range);
    sensor_acc=sensor_acc+toc(tt)*1e3;
    for j=det.ids
        if ~known(j), known(j)=true; first_seen(j)=t; end
    end

    % Permanent static-obstacle memory: once observed, retain it.
    ids=find(known);
    if isempty(ids), continue; end

    tt=tic;
    ev=prod_future_trigger_nominal(t,ids,obs,nom_fun,cfg,Tlook,Npred,Nsub);
    trigger_acc=trigger_acc+toc(tt)*1e3;
    if ev.found
        prod=ev; break;
    end
end

O.sensor=struct('t_detect',min(first_seen,[],'omitnan'),'first_seen',first_seen);
if isempty(O.sensor.t_detect), O.sensor.t_detect=NaN; end
O.memory=struct('known_ids',find(known),'first_seen',first_seen);
O.prod_trigger=prod;
if prod.found && trig7.found, O.trigger_delta=prod.t_now-trig7.t_now; else, O.trigger_delta=NaN; end

q=struct('requested',0,'cert',0,'c6s',inf,'c6e',inf,'min_g',-inf,'N',NaN,'T',NaN,'reason','NO_PRODUCTION_TRIGGER');
planner_ms=0;
if prod.found
    q.requested=1;
    Dstart=nom_fun(prod.t_now);
    Ntry=unique([selected.N 34 38 42 46]);
    extraT=[0 0.5 1 2 3 5 7 9];
    tp=tic;
    qr=search_phase719_case(cfg,Ntry,extraT,limits,obs,nom_fun, ...
        prod.t_now,selected.T,Dstart,opt,1/sqrt(3));
    planner_ms=toc(tp)*1e3;
    q.cert=qr.cert; q.c6s=qr.c6s; q.c6e=qr.c6e; q.min_g=qr.min_g;
    q.N=qr.N; q.T=qr.T; q.reason=qr.reason;
end
O.planner=q;
O.timing=struct('sensor_ms',sensor_acc,'trigger_ms',trigger_acc, ...
    'planner_ms',planner_ms,'total_ms',sensor_acc+trigger_acc+planner_ms);
end

function det=prod_sensor_detect(pQ,obs,R)
% Faithful standalone equivalent of REPLANNING_BSPLINE_NEW.detect_obstacles_fast:
% outer-sphere reject -> exact point/ellipsoid signed distance -> sort.
ids=[]; ranges=[];
for i=1:numel(obs)
    rmax=max(obs(i).radii);
    dc=pQ-obs(i).center;
    if dot(dc,dc)>(R+rmax)^2, continue; end
    d=point_ellipsoid_signed_distance_40(pQ,obs(i).center,obs(i).radii,obs(i).R);
    if d<=R
        ids(end+1)=i; ranges(end+1)=d; %#ok<AGROW>
    end
end
if ~isempty(ids)
    [ranges,k]=sort(ranges,'ascend'); ids=ids(k);
end
det=struct('ids',ids,'ranges',ranges);
end

function ev=prod_future_trigger_nominal(tnow,ids,obs,nom_fun,cfg,Tlook,N,Nsub)
% Standalone equivalent of the production predictor:
% 11 future samples, interval scan, outer-sphere pruning, local subdivision,
% exact ellipsoid distance for UAV/payload/cable.  Predictor only.
ev=struct('found',false,'t_now',NaN,'t_risk',NaN,'TTC',NaN,'obs_idx',NaN, ...
    'min_future_dist',inf,'threat_source','');
taus=linspace(0.1,Tlook,max(5,N));
pL=zeros(3,numel(taus)); pQ=pL;
for k=1:numel(taus)
    D=nom_fun(tnow+taus(k)); pL(:,k)=D(:,1); a=D(:,3);
    W=a+cfg.g_acc*cfg.e3;
    if norm(W)<=1e-12, nT=cfg.e3; else, nT=W/norm(W); end
    pQ(:,k)=pL(:,k)+cfg.sys.L_cable*nT;
end
for k=1:numel(taus)-1
    for io=ids
        c=obs(io).center; rmax=max(obs(io).radii);
        dQb=point_segment_dist(c,pQ(:,k),pQ(:,k+1))-rmax-cfg.sys.R_uav;
        dLb=point_segment_dist(c,pL(:,k),pL(:,k+1))-rmax-cfg.sys.R_payload;
        if min(dQb,dLb)>(cfg.d_margin+1.2), continue; end
        aa=linspace(0,1,Nsub+2);
        for j=1:numel(aa)
            a=aa(j);
            L=(1-a)*pL(:,k)+a*pL(:,k+1);
            Q=(1-a)*pQ(:,k)+a*pQ(:,k+1);
            [d,src]=prod_system_exact_dist(L,Q,obs(io),cfg);
            if d<=cfg.d_margin
                tr=tnow+(1-a)*taus(k)+a*taus(k+1);
                ev=struct('found',true,'t_now',tnow,'t_risk',tr,'TTC',tr-tnow, ...
                    'obs_idx',io,'min_future_dist',d,'threat_source',src);
                return;
            end
        end
    end
end
end

function [d,src]=prod_system_exact_dist(pL,pQ,o,cfg)
dQ=point_ellipsoid_signed_distance_40(pQ,o.center,o.radii,o.R)-cfg.sys.R_uav;
dL=point_ellipsoid_signed_distance_40(pL,o.center,o.radii,o.R)-cfg.sys.R_payload;
% Use the production representative cable ratios; diagnostic predictor only.
ratios=linspace(0,1,9); dC=inf;
for a=ratios
    p=(1-a)*pL+a*pQ;
    dC=min(dC,point_ellipsoid_signed_distance_40(p,o.center,o.radii,o.R)-cfg.sys.R_cable);
end
[d,j]=min([dQ dL dC]); names={'UAV','Payload','Cable'}; src=names{j};
end

function d=point_segment_dist(p,a,b)
ab=b-a; den=dot(ab,ab);
if den<1e-12, d=norm(p-a); return; end
u=max(0,min(1,dot(p-a,ab)/den));
d=norm(p-(a+u*ab));
end

function d=point_ellipsoid_signed_distance_40(p,c,r,R)
p=p(:); c=c(:); r=r(:); y=R'*(p-c); r2=r.^2;
q=sum((y./r).^2); inside=q<1;
if norm(y)<1e-14, d=-min(r); return; end
f=@(lam) sum(r2.*(y.^2)./((lam+r2).^2))-1;
if q>1, lo=0; hi=max(r)*norm(y);
else, lo=-min(r2)*(1-1e-12); hi=0; end
for k=1:40
    mid=.5*(lo+hi);
    if f(mid)>0, lo=mid; else, hi=mid; end
end
lam=.5*(lo+hi); cp=r2.*y./(lam+r2); da=norm(cp-y);
if inside, d=-da; else, d=da; end
end


% =========================================================================
% Phase 7-22 comprehensive production front-end regression
% =========================================================================
function O=run_phase722_frontend_regression(cfg,limits,obs,nom_fun,opt,selected) %#ok<INUSD>
R=15.0; Tlook=5.0; Npred=11; Nsub=4; dt=0.05;

% Baseline, continue after first trigger so memory semantics are observable.
O.baseline=audit_frontend_full(0,20,dt,obs,nom_fun,cfg,R,Tlook,Npred,Nsub);

% Sensor range-entry case. Place an obstacle on the future nominal UAV path
% far enough ahead that it starts outside the 15 m sensor surface range.
t_hit=36;
D=nom_fun(t_hit); W=D(:,3)+cfg.g_acc*cfg.e3;
if norm(W)<=1e-12, nT=cfg.e3; else, nT=W/norm(W); end
pQhit=D(:,1)+cfg.sys.L_cable*nT;
oe=obs(1); oe.center=pQhit; oe.S=oe.R*diag(oe.radii.^2)*oe.R';
A=audit_frontend_full(0,42,dt,oe,nom_fun,cfg,R,Tlook,Npred,Nsub);
D0=nom_fun(0); W0=D0(:,3)+cfg.g_acc*cfg.e3;
if norm(W0)<=1e-12, nT0=cfg.e3; else, nT0=W0/norm(W0); end
pQ0=D0(:,1)+cfg.sys.L_cable*nT0;
d0=point_ellipsoid_signed_distance_40(pQ0,oe.center,oe.radii,oe.R);
O.range_entry=A;
O.range_entry.initially_outside=(d0>R);

% Static-memory test: extend long enough that a once-seen obstacle becomes
% outside sensor range again; known memory must remain true.
om=obs(1);
M=audit_frontend_full(0,50,dt,om,nom_fun,cfg,R,Tlook,Npred,Nsub);
O.memory=struct('seen',~isnan(M.first_detect), ...
    'later_outside',M.seen_then_outside,'retained',any(M.known_final));

% Detected but safely offset obstacle: inside sensor range, no future collision.
os=obs(1);
D0=nom_fun(0); os.center=D0(:,1)+[0;10;0];
os.S=os.R*diag(os.radii.^2)*os.R';
S=audit_frontend_full(0,15,dt,os,nom_fun,cfg,R,Tlook,Npred,Nsub);
safe_min=phase722_sampled_min_distance(0,15,dt,os,nom_fun,cfg,Tlook,Npred);
O.safe_detect=struct('detected',~isnan(S.first_detect), ...
    'triggered',~isnan(S.first_trigger),'min_dist',safe_min);

% Sequential multiple obstacles on the future UAV path.
th=[34 48 62];
oseq=repmat(obs(1),1,numel(th));
for i=1:numel(th)
    D=nom_fun(th(i)); W=D(:,3)+cfg.g_acc*cfg.e3;
    if norm(W)<=1e-12, nT=cfg.e3; else, nT=W/norm(W); end
    oseq(i).center=D(:,1)+cfg.sys.L_cable*nT;
    oseq(i).radii=obs(1).radii.*[1; 0.85+0.1*i; 1];
    oseq(i).R=eul2rotm_local([0.25*i 0.08*i -0.05*i]);
    oseq(i).S=oseq(i).R*diag(oseq(i).radii.^2)*oseq(i).R';
end
O.sequential=audit_frontend_full(0,70,dt,oseq,nom_fun,cfg,R,Tlook,Npred,Nsub);

% Generality regression: trajectory family x obstacle size x pose x location.
funs={nom_fun,@phase722_line,@phase722_arc,@phase722_scurve};
names={'baseline','line','arc','scurve'};
scales=[0.75 1.00 1.25];
yaws=[0 pi/4];
shifts=[-1.0 1.0];
case_template=struct('trajectory','','scale',NaN,'yaw_deg',NaN,'shift',NaN, ...
    'detected',false,'triggered',false,'first_detect',NaN,'first_trigger',NaN,'TTC',NaN);
C=repmat(case_template,1,numel(funs)*numel(scales)*numel(yaws)*numel(shifts)); k=0;
for f=1:numel(funs)
    tfun=funs{f};
    for sc=scales
        for yw=yaws
            for sh=shifts
                oo=obs(1);
                % Put the test obstacle near that trajectory at t=8, then shift.
                Dt=tfun(8); W=Dt(:,3)+cfg.g_acc*cfg.e3;
                if norm(W)<=1e-12, nT=cfg.e3; else, nT=W/norm(W); end
                oo.center=Dt(:,1)+cfg.sys.L_cable*nT+[sh;0;0];
                oo.radii=obs(1).radii*sc;
                oo.R=eul2rotm_local([yw pi/10 pi/8]);
                oo.S=oo.R*diag(oo.radii.^2)*oo.R';
                A=audit_frontend_full(0,16,dt,oo,tfun,cfg,R,Tlook,Npred,Nsub);
                k=k+1;
                C(k)=struct('trajectory',names{f},'scale',sc,'yaw_deg',yw*180/pi, ...
                    'shift',sh,'detected',~isnan(A.first_detect), ...
                    'triggered',~isnan(A.first_trigger),'first_detect',A.first_detect, ...
                    'first_trigger',A.first_trigger,'TTC',A.first_TTC); %#ok<AGROW>
            end
        end
    end
end
O.cases=C;

% Production-trigger discretization sensitivity against the baseline.
pairs=[7 2; 11 2; 11 4; 17 4; 21 6];
trigger_template=struct('Npred',NaN,'Nsub',NaN,'t_trigger',NaN, ...
    't_risk',NaN,'TTC',NaN,'delta_risk',NaN);
TS=repmat(trigger_template,1,size(pairs,1)); refRisk=NaN;
for i=1:size(pairs,1)
    A=audit_frontend_full(0,15,dt,obs,nom_fun,cfg,R,Tlook,pairs(i,1),pairs(i,2));
    if pairs(i,1)==11 && pairs(i,2)==4, refRisk=A.first_risk; end
    TS(i)=struct('Npred',pairs(i,1),'Nsub',pairs(i,2), ...
        't_trigger',A.first_trigger,'t_risk',A.first_risk,'TTC',A.first_TTC, ...
        'delta_risk',NaN); %#ok<AGROW>
end
for i=1:numel(TS), TS(i).delta_risk=TS(i).t_risk-refRisk; end
O.trigger_sensitivity=TS;

% Planner integration checks: baseline plus two modest geometry variants.
% The independent Phase-7 verifier remains the execution gate.
planner_template=struct('name','','requested',0,'cert',0,'N',NaN,'T',NaN, ...
    'min_g',-inf,'c6s',inf,'c6e',inf,'reason','');
PC=repmat(planner_template,1,3);
variants={obs,obs,obs};
variants{2}(1).radii=obs(1).radii*0.9;
variants{2}(1).S=variants{2}(1).R*diag(variants{2}(1).radii.^2)*variants{2}(1).R';
variants{3}(1).center=obs(1).center+[0.25;0;0];
pnames={'baseline','scale_0.9','shift_x0.25'};
for i=1:3
    A=audit_frontend_full(0,15,dt,variants{i},nom_fun,cfg,R,Tlook,Npred,Nsub);
    q=struct('name',pnames{i},'requested',0,'cert',0,'N',NaN,'T',NaN, ...
        'min_g',-inf,'c6s',inf,'c6e',inf,'reason','NO_TRIGGER');
    if ~isnan(A.first_trigger)
        q.requested=1;
        Dstart=nom_fun(A.first_trigger);
        % Preserve the proven Phase-7 delayed-rejoin search logic.
        qr=search_phase719_case(cfg,unique([selected.N 34 38 42]), ...
            [0 0.5 1 2 3 5 7],limits,variants{i},nom_fun, ...
            A.first_trigger,selected.T,Dstart,opt,1/sqrt(3));
        q.cert=qr.cert; q.N=qr.N; q.T=qr.T; q.min_g=qr.min_g;
        q.c6s=qr.c6s; q.c6e=qr.c6e; q.reason=qr.reason;
    end
    PC(i)=q; %#ok<AGROW>
end
O.planner_checks=PC;

% Front-end stage timing only, measured separately from offline sweeps.
ns=100; sm=zeros(ns,1); tm=zeros(ns,1);
D=nom_fun(0); W=D(:,3)+cfg.g_acc*cfg.e3;
if norm(W)<=1e-12, nT=cfg.e3; else, nT=W/norm(W); end
pQ=D(:,1)+cfg.sys.L_cable*nT;
for i=1:ns
    tt=tic; det=prod_sensor_detect(pQ,obs,R); sm(i)=toc(tt)*1e3;
    ids=det.ids; if isempty(ids), ids=1:numel(obs); end
    tt=tic; prod_future_trigger_nominal(0,ids,obs,nom_fun,cfg,Tlook,Npred,Nsub);
    tm(i)=toc(tt)*1e3;
end
O.timing=struct('sensor_median',median(sm),'sensor_p95',local_pct(sm,95),'sensor_max',max(sm), ...
    'trigger_median',median(tm),'trigger_p95',local_pct(tm,95),'trigger_max',max(tm));
end

function A=audit_frontend_full(t0,t1,dt,obs,nom_fun,cfg,R,Tlook,Npred,Nsub)
n=numel(obs); known=false(1,n); first_seen=nan(1,n); first_trig=nan(1,n); first_risk=nan(1,n);
seen_then_out=false; minpd=inf;
for t=t0:dt:t1
    D=nom_fun(t); W=D(:,3)+cfg.g_acc*cfg.e3;
    if norm(W)<=1e-12, nT=cfg.e3; else, nT=W/norm(W); end
    pQ=D(:,1)+cfg.sys.L_cable*nT;
    det=prod_sensor_detect(pQ,obs,R);
    seen=false(1,n); seen(det.ids)=true;
    for i=1:n
        if seen(i) && isnan(first_seen(i)), first_seen(i)=t; end
        if known(i) && ~seen(i), seen_then_out=true; end
    end
    known=known|seen;
    ids=find(known);
    if isempty(ids), continue; end

    % One event per obstacle for diagnostics; memory remains active afterwards.
    for i=ids
        ev=prod_future_trigger_nominal(t,i,obs,nom_fun,cfg,Tlook,Npred,Nsub);
        if isfinite(ev.min_future_dist), minpd=min(minpd,ev.min_future_dist); end
        if ev.found && isnan(first_trig(i))
            first_trig(i)=t; first_risk(i)=ev.t_risk;
        end
    end
end
fd=min(first_seen,[],'omitnan'); ft=min(first_trig,[],'omitnan');
if isempty(fd), fd=NaN; end
if isempty(ft), ft=NaN; fr=NaN; else
    jj=find(first_trig==ft,1); fr=first_risk(jj);
end
if isnan(ft), TTC=NaN; else, TTC=fr-ft; end
P=repmat(struct('first_detect',NaN,'first_trigger',NaN,'first_risk',NaN,'retained_end',false),1,n);
for i=1:n
    P(i).first_detect=first_seen(i); P(i).first_trigger=first_trig(i);
    P(i).first_risk=first_risk(i); P(i).retained_end=known(i);
end
A=struct('first_detect',fd,'first_trigger',ft,'first_risk',fr,'first_TTC',TTC, ...
    'known_final',find(known),'first_seen_all',first_seen,'first_trigger_all',first_trig, ...
    'first_risk_all',first_risk,'seen_then_outside',seen_then_out, ...
    'min_predictor_distance',minpd,'per_obstacle',P);
end

function D=phase722_line(t)
D=zeros(3,7);
D(:,1)=[0.20*t;0;15+0.42*t];
D(:,2)=[0.20;0;0.42];
end

function D=phase722_arc(t)
D=zeros(3,7); R=3.0; w=0.10;
D(:,1)=[R*sin(w*t);R*(1-cos(w*t));15+0.38*t];
D(:,2)=[R*w*cos(w*t);R*w*sin(w*t);0.38];
D(:,3)=[-R*w^2*sin(w*t);R*w^2*cos(w*t);0];
D(:,4)=[-R*w^3*cos(w*t);-R*w^3*sin(w*t);0];
D(:,5)=[R*w^4*sin(w*t);-R*w^4*cos(w*t);0];
D(:,6)=[R*w^5*cos(w*t);R*w^5*sin(w*t);0];
D(:,7)=[-R*w^6*sin(w*t);R*w^6*cos(w*t);0];
end

function D=phase722_scurve(t)
D=zeros(3,7); A=0.8; w=0.22;
D(:,1)=[0.16*t;A*sin(w*t);15+0.40*t];
D(:,2)=[0.16;A*w*cos(w*t);0.40];
D(:,3)=[0;-A*w^2*sin(w*t);0];
D(:,4)=[0;-A*w^3*cos(w*t);0];
D(:,5)=[0;A*w^4*sin(w*t);0];
D(:,6)=[0;A*w^5*cos(w*t);0];
D(:,7)=[0;-A*w^6*sin(w*t);0];
end


function dmin=phase722_sampled_min_distance(t0,t1,dt,obs,nom_fun,cfg,Tlook,Npred)
% Offline diagnostic only. This is NOT a certificate and is not used to
% trigger replanning. It exists only so "detected but safe" does not report
% a meaningless +Inf diagnostic distance.
dmin=inf;
for t=t0:dt:t1
    taus=linspace(0.1,Tlook,max(5,Npred));
    for k=1:numel(taus)
        D=nom_fun(t+taus(k)); pL=D(:,1); a=D(:,3);
        W=a+cfg.g_acc*cfg.e3;
        if norm(W)<=1e-12, nT=cfg.e3; else, nT=W/norm(W); end
        pQ=pL+cfg.sys.L_cable*nT;
        for io=1:numel(obs)
            [d,~]=prod_system_exact_dist(pL,pQ,obs(io),cfg);
            dmin=min(dmin,d);
        end
    end
end
end

function v=local_pct(x,p)
x=sort(x(:)); n=numel(x);
if n==0, v=NaN; return; end
q=1+(n-1)*p/100; a=floor(q); b=ceil(q);
if a==b, v=x(a); else, v=x(a)+(q-a)*(x(b)-x(a)); end
end

function make_phase722_single_3d(sel,obs,nom_fun,trig,O) %#ok<INUSD>
% Exactly one figure in Phase 7-22, and it is 3-D.
c=sel.splice.cache; P=sel.splice.P;
u=linspace(0,1,400); Q=zeros(3,numel(u));
for k=1:numel(u), Q(:,k)=eval_spline(c,P,u(k),0); end
tn=linspace(trig.t_now,trig.t_now+sel.T,400); PN=zeros(3,numel(tn));
for k=1:numel(tn), D=nom_fun(tn(k)); PN(:,k)=D(:,1); end

figure('Name','Phase 7-22: certified trajectory + production front-end','Color','w');
h1=plot3(Q(1,:),Q(2,:),Q(3,:),'LineWidth',2.0); hold on;
h2=plot3(PN(1,:),PN(2,:),PN(3,:),'--','LineWidth',1.2);
for i=1:numel(obs)
    [x,y,z]=sphere(24);
    E=obs(i).R*diag(obs(i).radii)*[x(:)';y(:)';z(:)'];
    X=reshape(E(1,:)+obs(i).center(1),size(x));
    Y=reshape(E(2,:)+obs(i).center(2),size(y));
    Z=reshape(E(3,:)+obs(i).center(3),size(z));
    surf(X,Y,Z,'FaceAlpha',0.15,'EdgeAlpha',0.12,'HandleVisibility','off');
end
h3=plot3(Q(1,1),Q(2,1),Q(3,1),'o','MarkerSize',7,'LineWidth',1.5);
h4=plot3(Q(1,end),Q(2,end),Q(3,end),'s','MarkerSize',7,'LineWidth',1.5);
grid on; axis equal; xlabel('x [m]'); ylabel('y [m]'); zlabel('z [m]');
title('Phase 7-22 certified avoidance trajectory');
legend([h1 h2 h3 h4],{'Certified avoidance','Nominal','Replan start','C6 rejoin'},'Location','best');
view(3);
end

function s = setup_benchmark()
cfg.p=7;
cfg.N_ctrl=26;
cfg.g_acc=9.80665;
cfg.e3=[0;0;1];
cfg.sys.L_cable=1.0;
cfg.sys.R_payload=0.15;
cfg.sys.R_cable=0.02;
cfg.sys.R_uav=0.30;
cfg.d_margin=0.20;

limits.v_max=4.5;
limits.a_max=3.5;
limits.j_max=5.0;

obs(1).center=[1;0.3;18.5];
obs(1).radii=[1.3;1.0;1.6];
obs(1).R=eul2rotm_local([pi/7,pi/10,pi/8]);
obs(1).S=obs(1).R*diag(obs(1).radii.^2)*obs(1).R';

s=struct();
s.cfg=cfg;
s.limits=limits;
s.obs=obs;
s.nom_fun=@benchmark_nominal;
s.t_start=0.0;
s.t_end=15.26;
end

function D = benchmark_nominal(t)
% D(:,r+1) = r-th physical-time derivative, r=0..6.
D=zeros(3,7);
D(:,1)=[0.12*t; 0.15*sin(0.35*t); 15+0.48*t];
D(:,2)=[0.12; 0.0525*cos(0.35*t); 0.48];
D(:,3)=[0; -0.018375*sin(0.35*t); 0];
D(:,4)=[0; -0.00643125*cos(0.35*t); 0];
D(:,5)=[0; 0.0022509375*sin(0.35*t); 0];
D(:,6)=[0; 0.000787828125*cos(0.35*t); 0];
D(:,7)=[0; -0.00027573984375*sin(0.35*t); 0];
end

function [cfg,limits,obs,nom_fun,t0,t1] = unpack_scenario(s)
cfg=s.cfg;
limits=s.limits;
obs=s.obs;
nom_fun=s.nom_fun;
t0=s.t_start;
t1=s.t_end;

if ~isfield(cfg,'p'), cfg.p=7; end
if ~isfield(cfg,'N_ctrl'), cfg.N_ctrl=22; end
if ~isfield(cfg,'g_acc'), cfg.g_acc=9.80665; end
if ~isfield(cfg,'e3'), cfg.e3=[0;0;1]; end

for i=1:numel(obs)
    if ~isfield(obs(i),'support_fun') || isempty(obs(i).support_fun)
        if ~isfield(obs(i),'S')
            if isfield(obs(i),'R') && isfield(obs(i),'radii')
                obs(i).S=obs(i).R*diag(obs(i).radii.^2)*obs(i).R';
            else
                error('Obstacle %d lacks support_fun or ellipsoid S.',i);
            end
        end
    end
end
end

function R = eul2rotm_local(eul)
y=eul(1); p=eul(2); r=eul(3);
Rz=[cos(y) -sin(y) 0; sin(y) cos(y) 0; 0 0 1];
Ry=[cos(p) 0 sin(p); 0 1 0; -sin(p) 0 cos(p)];
Rx=[1 0 0; 0 cos(r) -sin(r); 0 sin(r) cos(r)];
R=Rz*Ry*Rx;
end

function result = make_failure_result(reason,tms,flag)
result=struct('execute',false,'reason',reason,'qpflag',flag, ...
    'time_generate_ms',tms,'time_cert_ms',0,'time_total_ms',tms, ...
    'time_typical_target_pass',false,'time_hard_budget_pass',false);
end

function print_result(r,corridor,cert)
fprintf('\n================================================================================\n');
fprintf('[Phase 7-11 RESULT]\n');
fprintf('================================================================================\n');
fprintf('corridor constraints : %d\n',corridor.n_constraints);
if isfield(corridor,'n_active_pairs')
    fprintf('active span/obs pairs : %d\n',corridor.n_active_pairs);
end
fprintf('QP flag              : %d\n',r.qpflag);
fprintf('generator time       : %.4f ms\n',r.time_generate_ms);

if ~isempty(cert)
    fprintf('certificate time     : %.4f ms\n',r.time_cert_ms);
    fprintf('total online time    : %.4f ms\n',r.time_total_ms);
    fprintf('C6 max error         : %.3e\n',r.c6_err);
    fprintf('certificate min g_LB : %+.6e m\n',cert.min_g_lower);
if isfield(cert,'min_M_LB')
    fprintf('certificate min M_LB : %.6e m/s^2\n',cert.min_M_LB);
end
    fprintf('certified v bound    : %.6f m/s\n',cert.v_bound);
    fprintf('certified a bound    : %.6f m/s^2\n',cert.a_bound);
    fprintf('certified j bound    : %.6f m/s^3\n',cert.j_bound);
    fprintf('geometry certificate : %d\n',cert.geometry_pass);
    fprintf('dynamic certificate  : %d\n',cert.dynamic_pass);
    fprintf('hard 2ms budget      : %d\n',r.time_hard_budget_pass);
end

fprintf('reason               : %s\n',r.reason);
fprintf('EXECUTE              : %d\n',r.execute);
if ~r.execute
    fprintf('online action        : KEEP PREVIOUS CERTIFIED BACKUP\n');
end
fprintf('================================================================================\n');
end


% =========================================================================
% P7 AUTOMATIC HORIZON + COHERENT CORRIDOR HELPERS (from validated P6R5)
% =========================================================================
function H=p6_predict_horizon(t,D0,risk,ob,nom_fun,cfg,limits,opt)
% Automatic geometry/clear-time horizon predictor.
% Supports one or multiple known obstacles.  This is a candidate-time
% predictor, NOT the independent safety certificate.
Tclear=max(0,risk.t_clear-t);

ta=max(t,risk.t_first_bad);
tb=max(ta,risk.t_clear);
tt=linspace(ta,tb,max(25,ceil((tb-ta)/.1)+1));
dirs=fibonacci_sphere(max(162,opt.separator_grid_N));

dneed=0; ro=0; ncrit=[1;0;0]; pcrit=D0(:,1); ocrit=1;

for ii=1:numel(tt)
    D=nom_fun(tt(ii));
    W=D(:,3)+cfg.g_acc*cfg.e3;
    if norm(W)<=1e-12, nT=cfg.e3; else, nT=W/norm(W); end

    % Each obstacle may have a different best separating normal.  The
    % horizon must cover the hardest currently known obstacle.
    for io=1:numel(ob)
        gb=-inf; nb=dirs(:,1);
        for kk=1:size(dirs,2)
            n=dirs(:,kk);
            g=n'*D(:,1)-delta_sep_exact(n,nT,cfg.sys)- ...
              obstacle_support(ob(io),n)-cfg.d_margin;
            if g>gb, gb=g; nb=n; end
        end

        dn=max(0,-gb);
        if isfield(ob(io),'S')
            roi=sqrt(max(0,nb'*ob(io).S*nb));
        else
            roi=0;
        end

        if dn>dneed
            dneed=dn; ncrit=nb; pcrit=D(:,1); ro=roi; ocrit=io;
        elseif abs(dn-dneed)<=1e-12
            ro=max(ro,roi);
        end
    end
end

% Spatial scale -> characteristic dynamic time scales.
davoid=1.10*dneed;             % dimensionless spatial reserve
Lgeom=max(davoid,ro);
Tv=Lgeom/max(limits.v_max,1e-12);
Ta=sqrt(2*Lgeom/max(limits.a_max,1e-12));
Tj=(6*Lgeom/max(limits.j_max,1e-12))^(1/3);

% Nominal cannot be rejoined before ALL known obstacles are clear.
Tgeom=max([Tclear,Tv,Ta,Tj]);

H=struct('T_clear',Tclear,'d_avoid',davoid,'T_geom',Tgeom, ...
    'T_dyn',NaN,'T_rejoin',Tclear,'T_raw',Tgeom,'factor',1, ...
    'T_plan',Tgeom,'T_cap',max(Tgeom,1e-6)*3, ...
    'ncrit',ncrit,'pcrit',pcrit,'critical_obstacle',ocrit, ...
    'obs_support_radius',ro);
end

function [A,b,info]=build_coherent_guided_corridor(cache,cfg,obs,opt)
% Phase 9A-G candidate generator only. Independent verifier is unchanged.
%
% Key change from P8R6:
%   P8R6 enforced temporal coherence independently for each obstacle.
%   With multiple simultaneously active obstacles this can select nearly
%   opposite half-space normals and destroy the corridor.
%
% Here obstacles whose active-span intervals overlap are grouped into a
% local interaction cluster.  Each cluster chooses ONE shared avoidance
% hemisphere from the same separator grid.  Each obstacle still has its own
% span-wise DP sequence, but active normals must remain in that shared
% hemisphere.  Thus temporal coherence and multi-obstacle side coherence
% are imposed without adding another QP.
%
% This remains candidate generation. Safety is certified only by
% certify_candidate_global() over all spans x all obstacles.

dirs=fibonacci_sphere(opt.separator_grid_N);
K=size(dirs,2); ns=cache.nspans; no=numel(obs);

Gall=-inf(ns,K,no);
bestg=-inf(ns,no);
bestidx=ones(ns,no);

for sp=1:ns
    ua=cache.spans(sp,1); ub=cache.spans(sp,2);
    Pbez=cache.B0{sp};
    Abez=span_bezier_points(cache,cache.Pbase,ua,ub,2);
    Wbez=Abez+repmat(cfg.g_acc*cfg.e3,1,size(Abez,2));
    Mlb=compute_certified_tension_lower_bound_cols(Wbez);
    for o=1:no
        for k=1:K
            Gall(sp,k,o)=span_system_g_lower_M(dirs(:,k),Pbez,Wbez,Mlb,obs(o),cfg);
        end
        [bestg(sp,o),bestidx(sp,o)]=max(Gall(sp,:,o));
    end
end

info.normal=zeros(3,ns,no);
info.active=(bestg<opt.active_margin_trigger);
info.nominal_g=bestg;
info.n_constraints=0;
info.method='CLUSTER_SIDE_COMPATIBLE_DP';
info.cluster_id=zeros(1,no);
info.cluster_guide=zeros(3,no);
info.cluster_count=0;

for o=1:no
    for sp=1:ns
        info.normal(:,sp,o)=dirs(:,bestidx(sp,o));
    end
end

% ---- Build obstacle interaction graph from overlapping active spans -------
adj=false(no,no);
for o1=1:no
    adj(o1,o1)=true;
    a1=find(info.active(:,o1));
    if isempty(a1), continue; end
    for o2=o1+1:no
        a2=find(info.active(:,o2));
        if isempty(a2), continue; end
        % One-span dilation catches immediately adjacent interacting blocks.
        ov = max(min(a1)-1,min(a2)-1) <= min(max(a1)+1,max(a2)+1);
        if ov
            adj(o1,o2)=true; adj(o2,o1)=true;
        end
    end
end

% Connected components; inactive obstacles remain unclustered.
seen=false(1,no);
clusters={};
for seed=1:no
    if seen(seed) || ~any(info.active(:,seed)), continue; end
    q=seed; seen(seed)=true; members=[];
    while ~isempty(q)
        u=q(1); q(1)=[]; members(end+1)=u; %#ok<AGROW>
        nb=find(adj(u,:) & ~seen);
        nb=nb(arrayfun(@(x) any(info.active(:,x)),nb));
        seen(nb)=true; q=[q nb]; %#ok<AGROW>
    end
    clusters{end+1}=members; %#ok<AGROW>
end
info.cluster_count=numel(clusters);

% ---- Shared guide for each interacting cluster ---------------------------
clusterGuide=cell(1,numel(clusters));
for ic=1:numel(clusters)
    members=clusters{ic};
    for o=members, info.cluster_id(o)=ic; end

    if numel(members)==1 || ~isfield(opt,'compat_enable') || ~opt.compat_enable
        % Preserve the P8R6 local guide behavior for isolated obstacles.
        o=members(1); ng=zeros(3,1);
        ids=find(info.active(:,o))';
        for sp=ids
            demand=max(0,opt.active_margin_trigger-bestg(sp,o))+0.05;
            ng=ng+demand*dirs(:,bestidx(sp,o));
        end
        if norm(ng)<1e-9, ng=dirs(:,bestidx(ids(1),o)); end
        ng=ng/norm(ng);
        clusterGuide{ic}=ng;
        info.cluster_guide(:,o)=ng;
        continue;
    end

    % Select a common hemisphere by aggregate normalized separator loss.
    % This is deterministic and obstacle-order independent.
    score=zeros(1,K);
    wsum=0;
    for o=members
        ids=find(info.active(:,o))';
        for sp=ids
            gv=Gall(sp,:,o);
            gb=bestg(sp,o);
            scale=max(0.25,abs(gb)+0.5);
            demand=max(0,opt.active_margin_trigger-gb)+0.05;
            score=score+demand*(gb-gv)/scale;
            wsum=wsum+demand;
        end
    end
    if wsum>0, score=score/wsum; end

    % Mild preference for nominal travel-side consistency: avoid choosing a
    % cluster guide opposite to the weighted local best directions.
    navg=zeros(3,1);
    for o=members
        ids=find(info.active(:,o))';
        for sp=ids
            demand=max(0,opt.active_margin_trigger-bestg(sp,o))+0.05;
            navg=navg+demand*dirs(:,bestidx(sp,o));
        end
    end
    if norm(navg)>1e-9
        navg=navg/norm(navg);
        score=score+0.20*(1-navg'*dirs);
    end

    [~,kg]=min(score);
    ng=dirs(:,kg);
    clusterGuide{ic}=ng;
    for o=members, info.cluster_guide(:,o)=ng; end
end

% ---- Per-obstacle temporal DP constrained by cluster side ----------------
for o=1:no
    blocks=p6r4_true_blocks(info.active(:,o));
    if isempty(blocks), continue; end
    ic=info.cluster_id(o);
    if ic>0, ng_cluster=clusterGuide{ic}; else, ng_cluster=[]; end

    for ib=1:size(blocks,1)
        ids=blocks(ib,1):blocks(ib,2);

        cand=false(1,K);
        for sp=ids
            [~,ord]=sort(Gall(sp,:,o),'descend');
            cand(ord(1:min(opt.coherent_topK,K)))=true;
        end
        if ~isempty(ng_cluster)
            [~,ordg]=sort(ng_cluster'*dirs,'descend');
            cand(ordg(1:min(opt.coherent_topK,K)))=true;
        end
        kk=find(cand); D=dirs(:,kk); nk=numel(kk);

        stage=zeros(numel(ids),nk);
        for ii=1:numel(ids)
            gv=Gall(ids(ii),kk,o);
            gb=max(gv); scale=max(0.25,abs(gb)+0.5);
            stage(ii,:)=(gb-gv)/scale;
        end

        if ~isempty(ng_cluster)
            gd=ng_cluster'*D;
            if numel(clusters{ic})>1
                stage=stage+opt.compat_lambda*repmat(1-gd,numel(ids),1);
                stage(:,gd<opt.compat_min_dot)=inf;
            elseif opt.geom_guide_enable
                stage=stage+opt.geom_guide_lambda*repmat(1-gd,numel(ids),1);
                stage(:,gd<opt.geom_guide_min_dot)=inf;
            end
        end

        % If a coarse grid leaves no direction in the requested hemisphere,
        % relax only the generator-side hemisphere gate. Certificate is not
        % relaxed. This avoids an artificial empty candidate set.
        if all(all(~isfinite(stage)))
            for ii=1:numel(ids)
                gv=Gall(ids(ii),kk,o);
                gb=max(gv); scale=max(0.25,abs(gb)+0.5);
                stage(ii,:)=(gb-gv)/scale;
            end
        end

        J=inf(numel(ids),nk); parent=zeros(numel(ids),nk);
        J(1,:)=stage(1,:);
        dots=D'*D;
        trans=opt.coherent_lambda*(1-dots);
        trans(dots<opt.coherent_flip_cos)=inf;
        for ii=2:numel(ids)
            for k=1:nk
                [v,ip]=min(J(ii-1,:)+trans(:,k)');
                J(ii,k)=stage(ii,k)+v;
                parent(ii,k)=ip;
            end
        end
        [~,kcur]=min(J(end,:));
        seq=zeros(1,numel(ids)); seq(end)=kcur;
        for ii=numel(ids):-1:2, seq(ii-1)=parent(ii,seq(ii)); end
        for ii=1:numel(ids), info.normal(:,ids(ii),o)=D(:,seq(ii)); end
    end
end

% ---- Build affine Bezier half-space rows ---------------------------------
rowsA={}; rowsb={};
for sp=1:ns
    ua=cache.spans(sp,1); ub=cache.spans(sp,2);
    for o=1:no
        if ~info.active(sp,o), continue; end
        n=info.normal(:,sp,o);
        um=mean([ua ub]);
        a0=eval_spline(cache,cache.Pbase,um,2);
        w0=a0+cfg.g_acc*cfg.e3;
        if norm(w0)<=opt.a_net_min, nT0=cfg.e3; else, nT0=w0/norm(w0); end
        rhs=obstacle_support(obs(o),n)+ ...
            delta_sep_exact(n,nT0,cfg.sys)+cfg.d_margin+opt.guide_extra;
        B0=cache.B0{sp}; Bz=cache.Bz{sp};
        for j=1:cache.p+1
            rowsA{end+1,1}=-n'*Bz(:,:,j); %#ok<AGROW>
            rowsb{end+1,1}= n'*B0(:,j)-rhs; %#ok<AGROW>
        end
    end
end
if isempty(rowsA)
    A=zeros(0,cache.nz); b=zeros(0,1);
else
    A=vertcat(rowsA{:}); b=vertcat(rowsb{:});
end
info.n_constraints=size(A,1);
info.n_active_pairs=nnz(info.active);

% Diagnostics: actual cross-obstacle angle after compatibility selection.
info.max_cross_angle_deg=0;
for sp=1:ns
    ids=find(info.active(sp,:));
    for i=1:numel(ids)
        for j=i+1:numel(ids)
            d=max(-1,min(1,info.normal(:,sp,ids(i))'*info.normal(:,sp,ids(j))));
            info.max_cross_angle_deg=max(info.max_cross_angle_deg,acosd(d));
        end
    end
end
end


function B=p6r4_true_blocks(x)
x=logical(x(:));
d=diff([false;x;false]);
a=find(d==1); b=find(d==-1)-1;
B=[a b];
end

