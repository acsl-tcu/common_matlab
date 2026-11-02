function A=CHECK_TRIGGER_NATIVE_MODEL_PARITY(logger,agent)
% Complete offline test wrapper. Compilation/source failures also produce
% transferable .mat/.txt diagnostics; Flight/sensor state is never changed.
try
    A=local_run_parity(logger,agent);
catch ME
    folder=fileparts(mfilename('fullpath'));
    A=struct('status','NATIVE_AUDIT_ERROR__FAIL_CLOSED', ...
        'parity_pass',false,'error_identifier',ME.identifier, ...
        'error_message',ME.message,'error_report',getReport(ME,'extended', ...
        'hyperlinks','off'),'continuous_time_safe',false);
    save(fullfile(folder,'TRIGGER_NATIVE_PARITY_RESULT.mat'),'A','-v7');
    fid=fopen(fullfile(folder,'TRIGGER_NATIVE_PARITY_RESULT.txt'),'w');
    if fid~=-1
        fprintf(fid,'status=%s\nidentifier=%s\nmessage=%s\n%s\n', ...
            A.status,A.error_identifier,A.error_message,A.error_report);
        fclose(fid);
    end
    fprintf(2,'[NATIVE PARITY] FAIL_CLOSED: %s | %s\n', ...
        ME.identifier,ME.message);
end
end

function A=local_run_parity(logger,agent)
% Offline source pinning / HLC parity / short-time native-vs-ode15s audit.
% Run AFTER an existing completed Flight; does NOT re-run Flight.
% Does not touch the real agent, sensor, EKF, reference or plant state.
% Outputs MAT/TXT beside this file. Passing is NOT a continuous safety proof.
assert(nargin==2&&isobject(agent)&&isobject(logger), ...
    'CHECK_TRIGGER_NATIVE_MODEL_PARITY:Inputs',...
    'Supply existing logger and agent from a completed Flight.');
folder=fileparts(mfilename('fullpath'));
cpp=fullfile(folder,'TRIGGER_SYMBOLIC_NATIVE_CORE.cpp');
assert(isfile(cpp),'CHECK_TRIGGER_NATIVE_MODEL_PARITY:Source','Native C++ source is missing.');
% Compilation is explicitly OFFLINE: NEVER take Flight startup/arming time.
before=pwd;
cd(folder);
back=onCleanup(@()cd(before));
mexpath=which('TRIGGER_SYMBOLIC_NATIVE_CORE');
need_build=isempty(mexpath);
if ~need_build
    compiled_stamp=dir(mexpath);
    source_stamp=dir(cpp);
    need_build=compiled_stamp.datenum<source_stamp.datenum;
end
if need_build
    fprintf('[NATIVE] Building source with current MATLAB compiler outside Flight...\n');
    mex('-R2018a','-O','-output','TRIGGER_SYMBOLIC_NATIVE_CORE',cpp);
    rehash;
end
source=TRIGGER_NATIVE_MODEL_SOURCE_CHECK(false);
assert(source.source_match && source.native_exists, ...
    'CHECK_TRIGGER_NATIVE_MODEL_PARITY:SourceMismatch',...
    'Original HLC / 24-state RHS SHA does not match native derivation: %s',source.status);
D=logger.Data;
assert(isfield(D,'agent') && ~isempty(D.agent) && ...
    isfield(D.agent(1),'estimator') && isfield(D.agent(1),'reference'),...
    'CHECK_TRIGGER_NATIVE_MODEL_PARITY:History','Existing Sim LOGGER lacks EKF/reference history.');
phases=char(D.phase(1:logger.k));
frames=find(phases=='f');
assert(~isempty(frames),'CHECK_TRIGGER_NATIVE_MODEL_PARITY:NoFlight',...
    'Cannot sample Flight states; use a LOGGER containing flight phase f.');
% Diverse, deterministic Flight samples: this is A/B regression, not safety proof.
ix=unique(frames(round(linspace(1,numel(frames),min(12,numel(frames))))));
P=agent.parameter;
names=["mass","jx","jy","jz","gravity","loadmass","cableL"];
params=double(P.get(names));
assert(numel(params)==7,'CHECK_TRIGGER_NATIVE_MODEL_PARITY:Params','HLC parameter order differs.');
plant=agent.plant;
pp_original=double(plant.param(:));
assert(numel(pp_original)==21 && strcmp(func2str(plant.method),...
    'with_load_model_euler_for_HL') && strcmp(func2str(plant.solver),'ode15s'),...
    'CHECK_TRIGGER_NATIVE_MODEL_PARITY:Plant','Actual plant method/solver not pinned.');
gain=agent.controller.hlc_suspended.param;
dt=double(gain.dt);
tol=[1e-10;1e-8];  % Numeric solver targets, NOT a physical error tube.
state=STATE_CLASS(struct('state_list', ...
    ["p","q","v","w","pL","vL","pT","wL","mL"],...
    'num_list',[3 3 3 3 3 3 3 3 1]));
refstate=STATE_CLASS(struct('state_list',"xd",'num_list',28));
params_shadow=TRIGGER_SHADOW_PARAMETER(names,params);
shadow=TRIGGER_SHADOW_AGENT(state,refstate,params_shadow);
hlc=HLC_SUSPENDED_LOAD(shadow,gain);
hlc.result=struct();
F=struct('sample_index',{},'time_s',{},'loadmass_kg',{},...
    'input_max_error',{},'uav_one_step_max_error_m',{},...
    'payload_one_step_max_error_m',{},'native_elapsed_ms',{},...
    'rhs_evaluations',{},'rejects',{},'status',{},'reason',{});
for j=1:numel(ix)
    k=ix(j);
    result=struct('sample_index',k,'time_s',double(D.t(k)),...
        'loadmass_kg',NaN,'input_max_error',Inf,...
        'uav_one_step_max_error_m',Inf,'payload_one_step_max_error_m',Inf,...
        'native_elapsed_ms',NaN,'rhs_evaluations',NaN,'rejects',NaN,...
        'status','UNKNOWN','reason','');
    try
        E=D.agent(1).estimator.result{k}.state;
        ref=D.agent(1).reference.result{k}.state;
        ml=double(E.mL);
        assert(isscalar(ml)&&isfinite(ml)&&ml>0,'Invalid historical EKF mass');
        result.loadmass_kg=ml;
        keys={'p','q','v','w','pL','vL','pT','wL'};
        x=zeros(24,1);packet=struct();
        for a=1:8
            v=double(E.(keys{a}));
            assert(isequal(size(v),[3 1])&&all(isfinite(v)),...
                'Historical EKF state has invalid field');
            packet.(keys{a})=v;
            x((3*a-2):(3*a))=v;
        end
        refvec=double(ref.xd(:));
        assert(numel(refvec)>=4&&numel(refvec)<=28&&all(isfinite(refvec)),...
            'Historical reference jets are invalid');
        ref28=zeros(28,1);ref28(1:numel(refvec))=refvec;
        pp=pp_original;pp(20)=ml;
        hp=double(P.get(names));hp(6)=ml;
        state.set_state(struct('p',packet.p,'q',packet.q,...
            'v',packet.v,'w',packet.w,'pL',packet.pL,'vL',packet.vL,...
            'pT',packet.pT,'wL',packet.wL,'mL',ml));
        refstate.set_state('xd',ref28);
        params_shadow=TRIGGER_SHADOW_PARAMETER(names,hp);
        shadow.parameter=params_shadow;
        % The original HLC has only shadow state/parameter containers.
        original=hlc.do();
        u_original=double(original.input(:));
        native_clock=tic;
        [q,l,u_native,counts]=TRIGGER_SYMBOLIC_NATIVE_CORE(x,...
            [0 dt],ref28,pp,hp(:),gain.F1(:),gain.F2(:),...
            gain.F3(:),gain.F4(:),dt,tol);
        result.native_elapsed_ms=1000*toc(native_clock);
        result.rhs_evaluations=counts(1);
        result.rejects=counts(2);
        result.input_max_error=max(abs(u_native(:)-u_original));
        [~,X]=plant.solver(@(tt,yy)plant.method(yy,u_original,pp(:).'),...
            [0 dt],x);
        endstate=double(X(end,:).');
        result.uav_one_step_max_error_m=max(abs(q(:,2)-endstate(1:3)));
        result.payload_one_step_max_error_m=max(abs(l(:,2)-endstate(13:15)));
        result.status='NUMERIC_EVALUATED';
    catch ME
        result.status='UNKNOWN_AUDIT_ERROR';
        result.reason=[ME.identifier ' | ' ME.message];
    end
    F(end+1)=result; %#ok<AGROW>
end
% Independent full 4 s (or available external reference horizon) A/B
% using the ORIGINAL MATLAB HLC and ode15s outside Flight. This is an
% offline regression, NOT a reachability or estimator-error certificate.
T=D.t(1:logger.k);
selected=ix(isfinite(T(ix)) & T(ix)+4 <= max(T(frames)));
selected=selected(round(linspace(1,numel(selected),min(2,numel(selected)))));
selected=unique(selected);
assert(~isempty(selected), ...
  'CHECK_TRIGGER_NATIVE_MODEL_PARITY:FuturePhase', ...
  'No full 4 s window exists inside flight phase.');
full=struct('sample_index',{},'time_s',{},'max_uav_error_m',{},...
    'max_payload_error_m',{},'max_control_error',{},...
    'original_elapsed_ms',{},'native_elapsed_ms',{},...
    'completed_s',{},'status',{},'reason',{});
nominal=agent.reference.timevarying;
for j=1:numel(selected)
    k=selected(j);
    r=struct('sample_index',k,'time_s',double(D.t(k)),...
        'max_uav_error_m',Inf,'max_payload_error_m',Inf,...
        'max_control_error',Inf,'original_elapsed_ms',NaN,...
        'native_elapsed_ms',NaN,'completed_s',NaN,...
        'status','UNKNOWN','reason','');
    try
        E=D.agent(1).estimator.result{k}.state;
        ml=double(E.mL);
        now=struct();x=zeros(24,1);
        keys={'p','q','v','w','pL','vL','pT','wL'};
        for ii=1:8
            now.(keys{ii})=double(E.(keys{ii}));
            x((3*ii-2):(3*ii))=now.(keys{ii});
        end
        pp=pp_original;pp(20)=ml;
        hp=double(P.get(names));hp(6)=ml;
        g=0:dt:4;g=[g(g<4),4];
        L=double(P.get("cableL"));
        refat=@(absolute_time) local_flight_reference(nominal,absolute_time,L);
        request=struct('time_s',r.time_s,'sample_times_s',g,...
            'horizon_s',4,'sample_dt_s',dt,'ekf_now',now,...
            'payload_mass_kg',ml,'cable_length_m',L,'reference_at',refat,...
            'numeric_tolerances',tol);
        binding=struct('model_parameters',pp(:).',...
            'loadmass_index',20,'hlc_param',gain,...
            'hlc_parameter_names',names,'hlc_parameter_values',hp,...
            'plant_rhs',plant.method,'plant_solver',plant.solver);
        clk=tic;
        original=CHECK_TRIGGER_ORIGINAL_NUMERIC_ORACLE.predict(request,binding);
        r.original_elapsed_ms=1000*toc(clk);
        assert(original.prediction_complete,'Original baseline did not complete 4 s');
        refs=zeros(28,numel(g)-1);
        for ii=1:numel(g)-1
            v=double(refat(r.time_s+g(ii)));
            refs(1:numel(v),ii)=v;
        end
        clk=tic;
        [q,l,u]=TRIGGER_SYMBOLIC_NATIVE_CORE(x,g,refs,pp(:),...
            hp(:),gain.F1(:),gain.F2(:),gain.F3(:),gain.F4(:),dt,tol);
        r.native_elapsed_ms=1000*toc(clk);
        r.max_uav_error_m=max(abs(q(:)-original.uav_m(:)));
        r.max_payload_error_m=max(abs(l(:)-original.payload_m(:)));
        r.max_control_error=max(abs(u(:)-original.input_thrust_torque(:)));
        r.completed_s=g(end);
        r.status='FULL_HORIZON_NUMERIC_COMPARED';
    catch ME
        r.status='UNKNOWN_FULL_HORIZON_AUDIT_ERROR';
        r.reason=[ME.identifier ' | ' ME.message];
    end
    full(end+1)=r; %#ok<AGROW>
end

ctrl=[F.input_max_error];du=[F.uav_one_step_max_error_m];dl=[F.payload_one_step_max_error_m];
% Diagnostic numeric equivalence acceptance thresholds, not geometry margins.
control_pass=all(ctrl<=1e-9);
integration_pass=all(du<=1e-4 & dl<=1e-4);
complete=all(strcmp({F.status},'NUMERIC_EVALUATED'));
full_completed=~isempty(full) && all(strcmp({full.status},'FULL_HORIZON_NUMERIC_COMPARED'));
full_pass=full_completed && all([full.max_uav_error_m]<=1e-3) && ...
    all([full.max_payload_error_m]<=1e-3) && ...
    all([full.max_control_error]<=2e-2);
A=struct('status','NATIVE_PARITY_FAIL_CLOSED',...
    'parity_pass',logical(control_pass&&integration_pass&&complete&&full_pass),...
    'source_match',logical(source.source_match),...
    'source_sha256',{source.source_sha256},...
    'native_mex_sha256',source.native_mex_sha256,...
    'tested_records',numel(ix),'records',F,'full_horizon',full,...
    'full_horizon_pass',full_pass,'full_horizon_records',numel(full),...
    'max_control_error',max(ctrl),...
    'max_uav_one_step_error_m',max(du),...
    'max_payload_one_step_error_m',max(dl),...
    'native_one_step_median_ms',median([F.native_elapsed_ms],'omitnan'),...
    'native_rhs_total',sum([F.rhs_evaluations],'omitnan'),...
    'numerical_abs_tol',tol(1),'numerical_rel_tol',tol(2),...
    'continuous_time_safe',false,...
    'unmodified_original_hlc',true,...
    'comment','One-step and complete 4s nominal HLC+ODE parity; continuous reachability, HLC saturation, and safety remain NOT certified');
if A.parity_pass
    A.status='PARITY_SAMPLED_PASS__NOMINAL_ONLY__NO_SAFETY_PROOF';
end
% Audit marker is required by Flight-side native predictor; always save
% failures too, which keep the runtime gate CLOSED.
save(fullfile(folder,'TRIGGER_NATIVE_PARITY_RESULT.mat'),'A','-v7');
fid=fopen(fullfile(folder,'TRIGGER_NATIVE_PARITY_RESULT.txt'),'w');
if fid~=-1
  clean=onCleanup(@()fclose(fid));
  fprintf(fid,'status=%s\nsource_match=%d\nparity_pass=%d\nsamples=%d\n',...
      A.status,A.source_match,A.parity_pass,A.tested_records);
  fprintf(fid,'max_control_error=%.17g\nmax_uav_ode_error_m=%.17g\n',...
      A.max_control_error,A.max_uav_one_step_error_m);
  fprintf(fid,'max_payload_ode_error_m=%.17g\nmedian_native_one_step_ms=%.17g\n',...
      A.max_payload_one_step_error_m,A.native_one_step_median_ms);
  for j=1:numel(full)
      r=full(j);
      fprintf(fid,'FULL_HORIZON t=%.6f dUAV=%.8g dPAYLOAD=%.8g dINPUT=%.8g original_ms=%.3f native_ms=%.3f status=%s reason=%s\n',...
          r.time_s,r.max_uav_error_m,r.max_payload_error_m,r.max_control_error,...
          r.original_elapsed_ms,r.native_elapsed_ms,r.status,r.reason);
  end
  for j=1:numel(F)
      r=F(j);
      fprintf(fid,'sample %d time=%.6f mL=%.8g du=%.8g dUAV=%.8g dPAYLOAD=%.8g ms=%.5f status=%s reason=%s\n',...
          r.sample_index,r.time_s,r.loadmass_kg,r.input_max_error,...
          r.uav_one_step_max_error_m,r.payload_one_step_max_error_m,...
          r.native_elapsed_ms,r.status,r.reason);
  end
  clear clean
end
fprintf('[NATIVE PARITY] %s | tests=%d | max input error=%.3g | max position error=%.3g m\n',...
    A.status,A.tested_records,A.max_control_error,...
    max(A.max_uav_one_step_error_m,A.max_payload_one_step_error_m));
disp(rmfield(A,{'records','source_sha256'}));
end

function ref=local_flight_reference(nominal,absolute_time,L)
ref=double(nominal.func(double(absolute_time)-double(nominal.t)));
ref=ref(:);
assert(numel(ref)>=4&&numel(ref)<=28&&all(isfinite(ref)),...
    'CHECK_TRIGGER_NATIVE_MODEL_PARITY:FutureReference','Original future reference invalid.');
ref(3)=ref(3)-L;
end
