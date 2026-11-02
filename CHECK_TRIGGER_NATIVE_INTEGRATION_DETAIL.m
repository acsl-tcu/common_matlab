function D=CHECK_TRIGGER_NATIVE_INTEGRATION_DETAIL(logger,agent)
% Read-only offline analysis of an existing native-parity FAIL.
% Uses already recorded LOGGER states; no run, preflight, sensor scan,
% warmup, plant mutation, controller mutation or runtime activation.
% Inspects full-horizon reasons (previous audit hides them in [1x2 struct])
% and separates ode15s default accuracy from native integration error.
% Invocation after existing Flight:
%   D=CHECK_TRIGGER_NATIVE_INTEGRATION_DETAIL(logger,agent);
% or, to print saved parity reasons only:
%   D=CHECK_TRIGGER_NATIVE_INTEGRATION_DETAIL();
if nargin<1, logger=[]; end
if nargin<2, agent=[]; end
path=which('CHECK_TRIGGER_NATIVE_MODEL_PARITY');
assert(~isempty(path),'CHECK_TRIGGER_NATIVE_INTEGRATION_DETAIL:MissingSource', ...
    'Original CHECK_TRIGGER_NATIVE_MODEL_PARITY.m must be on path.');
file=fullfile(fileparts(path),'TRIGGER_NATIVE_PARITY_RESULT.mat');
assert(isfile(file),'CHECK_TRIGGER_NATIVE_INTEGRATION_DETAIL:NoAudit', ...
    'Existing parity MAT is missing: %s',file);
saved=load(file,'A');
assert(isfield(saved,'A')&&isscalar(saved.A), ...
    'CHECK_TRIGGER_NATIVE_INTEGRATION_DETAIL:InvalidAudit', ...
    'Saved parity MAT is not valid.');
A=saved.A;
D=struct('status','OFFLINE_DETAILS__NO_NEW_FLIGHT', ...
    'original_parity_status',local_get(A,'status','UNKNOWN'), ...
    'parity_pass',local_get(A,'parity_pass',false), ...
    'full_horizon',local_get(A,'full_horizon',[]), ...
    'one_step',local_get(A,'records',[]), ...
    'accuracy_comparison',[], 'timing',struct(), ...
    'requires_new_flight',false, ...
    'continuous_time_certified',false);
if ~isempty(logger)&&~isempty(agent)
    assert(isobject(logger)&&isobject(agent), ...
        'CHECK_TRIGGER_NATIVE_INTEGRATION_DETAIL:Inputs','Use real logged agent and logger.');
    assert(isfield(A,'source_match')&&A.source_match, ...
        'CHECK_TRIGGER_NATIVE_INTEGRATION_DETAIL:SourceMismatch', ...
        'Source hash is not verified; numerical details are not meaningful.');
    if isempty(which('TRIGGER_SYMBOLIC_NATIVE_CORE'))
        error('CHECK_TRIGGER_NATIVE_INTEGRATION_DETAIL:NoMEX', ...
            'Compile the existing native MEX OFFLINE using the parity audit.');
    end
    assert(isfield(A,'records')&&~isempty(A.records), ...
        'CHECK_TRIGGER_NATIVE_INTEGRATION_DETAIL:NoRecords', ...
        'Existing parity MAT has no sampled records.');
    % Decompose recorded Flight-cycle latency without rerunning the Sim.
    n=double(logger.k);
    t=double(logger.Data.t(1:n));t=t(:);
    whole=nan(n,1);tr=nan(n,1);rp=nan(n,1);
    if isfield(logger.Data,'system_cycle_time_ms')
        z=double(logger.Data.system_cycle_time_ms(:));
        whole(1:min(n,numel(z)))=z(1:min(n,numel(z)));
    end
    if isfield(logger.Data,'agent') && isfield(logger.Data.agent(1),'reference')
        refentries=logger.Data.agent(1).reference.result;
        for idx=1:min(n,numel(refentries))
            if iscell(refentries),v=refentries{idx};else,v=refentries(idx);end
            if isstruct(v)&&isscalar(v)
                tr(idx)=local_scalar(v,'trigger_time_ms');
                rp(idx)=local_scalar(v,'replanning_time_ms');
            end
        end
    end
    other=whole-rp;
    other(~isfinite(other) | other<0)=NaN;
    D.timing=struct('cycle',local_stats(whole),'replanner',local_stats(rp), ...
        'trigger',local_stats(tr),'cycle_excluding_replanner',local_stats(other), ...
        'windows',[]);
    bounds=[4.95 5.20;9.80 10.10;33 42];
    windows=repmat(struct('from_s',NaN,'to_s',NaN,'cycle',struct(), ...
        'replanner',struct(),'trigger',struct(), ...
        'cycle_excluding_replanner',struct()),size(bounds,1),1);
    for ww=1:size(bounds,1)
        kkeep=isfinite(t)&t>=bounds(ww,1)&t<=bounds(ww,2);
        windows(ww).from_s=bounds(ww,1);
        windows(ww).to_s=bounds(ww,2);
        windows(ww).cycle=local_stats(whole(kkeep));
        windows(ww).replanner=local_stats(rp(kkeep));
        windows(ww).trigger=local_stats(tr(kkeep));
        windows(ww).cycle_excluding_replanner=local_stats(other(kkeep));
    end
    D.timing.windows=windows;
    F=A.records;
    err=[F.uav_one_step_max_error_m];
    eligible=find(isfinite(err)&[F.sample_index]>0);
    if ~isempty(eligible)
        [~,imax]=max(err(eligible));
        selected=unique([eligible(1),eligible(imax),eligible(end)]);
        names=["mass","jx","jy","jz","gravity","loadmass","cableL"];
        gain=agent.controller.hlc_suspended.param;
        dt=double(gain.dt);
        tol=[double(A.numerical_abs_tol);double(A.numerical_rel_tol)];
        oracle=odeset('AbsTol',tol(1),'RelTol',tol(2));
        pp0=double(agent.plant.param(:));
        assert(numel(pp0)==21,'CHECK_TRIGGER_NATIVE_INTEGRATION_DETAIL:Parameters', ...
            'Active plant parameter layout is not 21 elements.');
        X=struct('sample_index',{},'time_s',{},'default_ode_vs_native_m',{}, ...
            'tight_ode_vs_native_m',{},'native_refinement_delta_m',{}, ...
            'default_ode_vs_tight_ode_m',{},'native_ms',{},'reason',{});
        keys={'p','q','v','w','pL','vL','pT','wL'};
        for j=selected
            f=F(j);
            r=struct('sample_index',f.sample_index,'time_s',f.time_s, ...
                'default_ode_vs_native_m',NaN, 'tight_ode_vs_native_m',NaN, ...
                'native_refinement_delta_m',NaN,'default_ode_vs_tight_ode_m',NaN, ...
                'native_ms',NaN,'reason','');
            try
                k=double(f.sample_index);
                E=logger.Data.agent(1).estimator.result{k}.state;
                ref=double(logger.Data.agent(1).reference.result{k}.state.xd(:));
                assert(numel(ref)>=4&&numel(ref)<=28 && ...
                    all(isfinite(ref)), 'Recorded reference invalid');
                ref28=zeros(28,1);ref28(1:numel(ref))=ref;
                x=zeros(24,1);
                for m=1:8
                    a=double(E.(keys{m}));
                    assert(isequal(size(a),[3 1]) && all(isfinite(a)), ...
                        'Recorded EKF %s invalid',keys{m});
                    x((3*m-2):(3*m))=a;
                end
                ml=double(E.mL);
                assert(isscalar(ml)&&isfinite(ml)&&ml>0,'Invalid recorded EKF mass');
                pp=pp0;pp(20)=ml;
                hp=double(agent.parameter.get(names));hp=hp(:);hp(6)=ml;
                clock=tic;
                [q,l,u]=TRIGGER_SYMBOLIC_NATIVE_CORE(x,[0 dt],ref28,pp,hp, ...
                    gain.F1(:),gain.F2(:),gain.F3(:),gain.F4(:),dt,tol);
                r.native_ms=1000*toc(clock);
                [q2,l2,~]=TRIGGER_SYMBOLIC_NATIVE_CORE(x,[0 dt],ref28,pp,hp, ...
                    gain.F1(:),gain.F2(:),gain.F3(:),gain.F4(:),dt,tol/10);
                fun=@(t,y)agent.plant.method(y,u(:,1),pp(:).');
                [~,yd]=agent.plant.solver(fun,[0 dt],x);
                [~,yt]=agent.plant.solver(fun,[0 dt],x,oracle);
                n=[q(:,end);l(:,end)];
                nf=[q2(:,end);l2(:,end)];
                def=[yd(end,1:3)';yd(end,13:15)'];
                strict=[yt(end,1:3)';yt(end,13:15)'];
                r.default_ode_vs_native_m=max(abs(def-n));
                r.tight_ode_vs_native_m=max(abs(strict-n));
                r.native_refinement_delta_m=max(abs(n-nf));
                r.default_ode_vs_tight_ode_m=max(abs(def-strict));
            catch ME
                r.reason=[ME.identifier ' | ' ME.message];
            end
            X(end+1)=r; %#ok<AGROW>
        end
        D.accuracy_comparison=X;
    end
end
folder=fileparts(mfilename('fullpath'));
save(fullfile(folder,'TRIGGER_NATIVE_INTEGRATION_DETAIL.mat'),'D','-v7');
filename=fullfile(folder,'TRIGGER_NATIVE_INTEGRATION_DETAIL.txt');
fid=fopen(filename,'wt');
assert(fid~=-1,'CHECK_TRIGGER_NATIVE_INTEGRATION_DETAIL:Write', ...
    'Cannot write report: %s',filename);
cleanup=onCleanup(@()fclose(fid));
fprintf(fid,'original_status=%s\nparity_pass=%d\n',D.original_parity_status,D.parity_pass);
fprintf(fid,'no_runtime_changes=1\nno_new_flight=1\ncontinuous_time_certified=0\n');
for i=1:numel(D.full_horizon)
    r=D.full_horizon(i);
    fprintf(fid,'FULL_HORIZON_%d time_s=%.8g status=%s uav_error_m=%.9g payload_error_m=%.9g control_error=%.9g original_ms=%.8g native_ms=%.8g reason=%s\n', ...
        i,local_get(r,'time_s',NaN),local_get(r,'status','UNKNOWN'), ...
        local_get(r,'max_uav_error_m',NaN), ...
        local_get(r,'max_payload_error_m',NaN), ...
        local_get(r,'max_control_error',NaN), ...
        local_get(r,'original_elapsed_ms',NaN), ...
        local_get(r,'native_elapsed_ms',NaN),local_get(r,'reason',''));
end
for i=1:numel(D.accuracy_comparison)
    r=D.accuracy_comparison(i);
    fprintf(fid,['ONE_STEP_%d t=%.8g default_vs_native_m=%.9g ' ...
        'strict_vs_native_m=%.9g native_refinement_m=%.9g ' ...
        'default_vs_strict_ode_m=%.9g native_ms=%.8g reason=%s\n'], ...
        i,r.time_s,r.default_ode_vs_native_m, ...
        r.tight_ode_vs_native_m,r.native_refinement_delta_m, ...
        r.default_ode_vs_tight_ode_m,r.native_ms,r.reason);
end
if ~isempty(fieldnames(D.timing))
    for label={'cycle','replanner','trigger','cycle_excluding_replanner'}
        name=label{1};v=D.timing.(name);
        fprintf(fid,'TIMING_%s N=%d median_ms=%.8g P95_ms=%.8g max_ms=%.8g\n', ...
            name,v.N,v.median_ms,v.P95_ms,v.max_ms);
    end
    for ww=1:numel(D.timing.windows)
        v=D.timing.windows(ww);
        fprintf(fid,['TIMING_WINDOW_%.2f_%.2f cycle_max_ms=%.8g ' ...
            'replanner_max_ms=%.8g trigger_max_ms=%.8g ' ...
            'cycle_minus_replanner_max_ms=%.8g\n'], ...
            v.from_s,v.to_s,v.cycle.max_ms,v.replanner.max_ms, ...
            v.trigger.max_ms,v.cycle_excluding_replanner.max_ms);
    end
end
clear cleanup
fprintf('[NATIVE INTEGRATION DETAIL] %s | full-horizon records=%d | one-step diagnostics=%d\n', ...
    D.original_parity_status,numel(D.full_horizon),numel(D.accuracy_comparison));
fprintf('[NATIVE INTEGRATION DETAIL] Saved MAT/TXT, no Flight rerun.\n');
end
function v=local_get(s,key,other)
if isstruct(s)&&isfield(s,key),v=s.(key);else,v=other;end
end

function v=local_scalar(s,key)
v=NaN;
if isfield(s,key)
    x=s.(key);
    if isnumeric(x)&&isscalar(x)&&isfinite(x),v=double(x);end
end
end
function R=local_stats(x)
x=double(x(:));x=x(isfinite(x)&x>=0);x=sort(x);
R=struct('N',numel(x),'median_ms',NaN,'P95_ms',NaN,'max_ms',NaN);
if ~isempty(x)
    R.median_ms=median(x);
    R.P95_ms=x(max(1,ceil(.95*numel(x))));
    R.max_ms=x(end);
end
end
