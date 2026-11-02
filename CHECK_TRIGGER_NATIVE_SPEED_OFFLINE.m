function B = CHECK_TRIGGER_NATIVE_SPEED_OFFLINE(logger,agent,horizon_s)
% Read-only, no warmup, no Flight rerun. Measures the ORIGINAL cached native
% numeric candidate on recorded real EKF states, over the ENTIRE horizon.
% This is a performance test, not a model-parity / safety certificate.
% For runtime constraints the 4 s default is user-defined, not a cutoff:
%    B = CHECK_TRIGGER_NATIVE_SPEED_OFFLINE(logger,agent);
%    B = CHECK_TRIGGER_NATIVE_SPEED_OFFLINE(logger,agent,6);
% Source/HLC/plant/runner/EKF/sensor and LOGGER are never changed.
if nargin<3, horizon_s=4;end
B=struct('status','UNKNOWN_NOT_RUN', 'horizon_s',horizon_s, ...
    'model_source_match',false,'sample_count',0,'success_count',0, ...
    'samples',[],'native_median_ms',NaN,'native_P95_ms',NaN, ...
    'native_max_ms',NaN,'total_median_ms',NaN,'total_P95_ms',NaN, ...
    'total_max_ms',NaN,'under_1ms_count',0,'under_1ms_fraction',NaN, ...
    'no_warmup',true,'runner_modified',false, ...
    'continuous_time_certified',false,'parity_accepted',false,'error','');
try
    assert(isscalar(horizon_s)&&isnumeric(horizon_s)&&isreal(horizon_s)&& ...
        isfinite(horizon_s)&&horizon_s>0,'Invalid requested horizon.');
    assert(isobject(logger)&&isobject(agent), ...
        'CHECK_TRIGGER_NATIVE_SPEED_OFFLINE:Input','Use actual LOGGER and agent.');
    source=TRIGGER_NATIVE_MODEL_SOURCE_CHECK(false);
    B.model_source_match=logical(source.source_match);
    assert(source.source_match, ...
        'CHECK_TRIGGER_NATIVE_SPEED_OFFLINE:SourceMismatch', ...
        'Active MATLAB HLC/plant do not match the pinned native derivation.');
    assert(~isempty(which('TRIGGER_SYMBOLIC_NATIVE_CORE')), ...
        'CHECK_TRIGGER_NATIVE_SPEED_OFFLINE:MissingMEX', ...
        'The previously built MEX must be available. No compile during benchmark.');
    P=agent.parameter;
    names=["mass","jx","jy","jz","gravity","loadmass","cableL"];
    hp0=double(P.get(names));hp0=hp0(:);
    pp0=double(agent.plant.param(:));
    assert(numel(pp0)==21&&numel(hp0)==7, ...
        'CHECK_TRIGGER_NATIVE_SPEED_OFFLINE:ModelParameters', ...
        'External model parameters differ from the pinned 21/7 layout.');
    gain=agent.controller.hlc_suspended.param;
    dt=double(gain.dt);
    assert(isscalar(dt)&&isfinite(dt)&&dt>0, ...
        'CHECK_TRIGGER_NATIVE_SPEED_OFFLINE:Step','Invalid HLC dt.');
    assert(strcmp(func2str(agent.plant.method),'with_load_model_euler_for_HL'), ...
        'CHECK_TRIGGER_NATIVE_SPEED_OFFLINE:Plant','Unexpected plant RHS.');
    % Diagnostic numeric solver tolerances are those used by the native
    % candidate and prior parity study; never replace live model parameters.
    tol=[1e-10;1e-8];
    f1=double(gain.F1(:));f2=double(gain.F2(:));
    f3=double(gain.F3(:));f4=double(gain.F4(:));
    D=logger.Data;
    frames=find(char(D.phase(1:logger.k))=='f');
    assert(~isempty(frames),'CHECK_TRIGGER_NATIVE_SPEED_OFFLINE:NoFlight', ...
        'No recorded Flight-phase states.');
    flight_end=double(D.t(frames(end)));
    eligible=frames(double(D.t(frames))+horizon_s<=flight_end);
    assert(~isempty(eligible), ...
        'CHECK_TRIGGER_NATIVE_SPEED_OFFLINE:NoHorizon', ...
        'Recorded Flight does not contain a complete requested horizon.');
    chosen=unique(eligible(round(linspace(1,numel(eligible),min(12,numel(eligible))))));
    B.sample_count=numel(chosen);
    nominal=agent.reference.timevarying;
    assert(~isempty(nominal.t) && isa(nominal.func,'function_handle'), ...
        'CHECK_TRIGGER_NATIVE_SPEED_OFFLINE:Reference', ...
        'Actual Flight reference clock and function must exist.');
    L=double(P.get("cableL"));
    assert(isscalar(L)&&isfinite(L)&&L>0, ...
        'CHECK_TRIGGER_NATIVE_SPEED_OFFLINE:CableLength', ...
        'Invalid external cable length.');
    grid=0:dt:horizon_s;
    if isempty(grid)||grid(end)<horizon_s,grid=[grid,horizon_s];end
    if grid(end)>horizon_s,grid(end)=horizon_s;end
    num_intervals=numel(grid)-1;
    template=struct('time_s',NaN,'sample_index',NaN,'native_ms',NaN, ...
        'total_ms',NaN,'reference_and_pack_ms',NaN,'rhs_evaluations',NaN, ...
        'integrator_rejections',NaN,'status','UNKNOWN', 'reason','');
    tests=repmat(template,numel(chosen),1);
    names24={'p','q','v','w','pL','vL','pT','wL'};
    for j=1:numel(chosen)
        k=chosen(j);
        result=template;
        result.time_s=double(D.t(k));result.sample_index=k;
        outer=tic;
        try
            e=D.agent(1).estimator.result{k}.state;
            ml=double(e.mL);
            assert(isscalar(ml)&&isfinite(ml)&&ml>0,'Nonpositive EKF mass');
            x=zeros(24,1);
            for i=1:8
                v=double(e.(names24{i}));
                assert(isequal(size(v),[3 1])&&all(isfinite(v)), ...
                    'Invalid EKF state %s',names24{i});
                x((3*i-2):(3*i))=v;
            end
            pp=pp0;pp(20)=ml;hp=hp0;hp(6)=ml;
            refs=zeros(28,num_intervals);
            for i=1:num_intervals
                ref=double(nominal.func(result.time_s+grid(i)-double(nominal.t)));
                ref=ref(:);
                assert(numel(ref)>=4&&numel(ref)<=28&&all(isfinite(ref)), ...
                    'Actual reference jets invalid');
                ref(3)=ref(3)-L;
                refs(1:numel(ref),i)=ref;
            end
            result.reference_and_pack_ms=1000*toc(outer);
            core=tic;
            [q,l,u,counts]=TRIGGER_SYMBOLIC_NATIVE_CORE( ...
                x,grid,refs,pp,hp,f1,f2,f3,f4,dt,tol);
            result.native_ms=1000*toc(core);
            assert(isequal(size(q),[3 numel(grid)])&& ...
                isequal(size(l),[3 numel(grid)])&& ...
                isequal(size(u),[4 num_intervals])&& ...
                all(isfinite([q(:);l(:);u(:)])), ...
                'MEX did not complete the full requested horizon');
            result.rhs_evaluations=counts(1);
            result.integrator_rejections=counts(2);
            result.total_ms=1000*toc(outer);
            result.status='COMPLETE_NOMINAL_ONLY';
        catch ME
            result.status='UNKNOWN_NUMERIC_OR_INPUT_FAILURE';
            result.reason=[ME.identifier ' | ' ME.message];
        end
        tests(j)=result;
    end
    B.samples=tests;
    ok=strcmp({tests.status},'COMPLETE_NOMINAL_ONLY');
    B.success_count=sum(ok);
    if all(ok)
        B.status='COMPLETE_NATIVE_TIMING_ONLY__NOT_A_PARITY_OR_SAFETY_PASS';
    else
        B.status='INCOMPLETE_NATIVE_TIMING__NOT_A_PARITY_OR_SAFETY_PASS';
    end
    if B.success_count>0
        t0=[tests(ok).native_ms];t1=[tests(ok).total_ms];
        B.native_median_ms=median(t0); B.native_P95_ms=local_p95(t0);
        B.native_max_ms=max(t0); B.total_median_ms=median(t1);
        B.total_P95_ms=local_p95(t1);B.total_max_ms=max(t1);
        B.under_1ms_count=sum(t0<=1.0);
        B.under_1ms_fraction=B.under_1ms_count/B.success_count;
    end
catch ME
    B.status='UNKNOWN_OFFLINE_BENCHMARK_SETUP';
    B.error=[ME.identifier ' | ' ME.message];
end
folder=fileparts(mfilename('fullpath'));
save(fullfile(folder,'TRIGGER_NATIVE_SPEED_OFFLINE.mat'),'B','-v7');
fid=fopen(fullfile(folder,'TRIGGER_NATIVE_SPEED_OFFLINE.txt'),'wt');
if fid~=-1
    clean=onCleanup(@()fclose(fid));
    fprintf(fid,'status=%s\nsource_match=%d\nhorizon_s=%.12g\n', ...
        B.status,B.model_source_match,horizon_s);
    fprintf(fid,'complete=%d/%d\nnative_median_ms=%.9g\nnative_P95_ms=%.9g\n', ...
        B.success_count,B.sample_count,B.native_median_ms,B.native_P95_ms);
    fprintf(fid,'native_max_ms=%.9g\nend_to_end_median_ms=%.9g\n', ...
        B.native_max_ms,B.total_median_ms);
    fprintf(fid,'end_to_end_P95_ms=%.9g\nend_to_end_max_ms=%.9g\n', ...
        B.total_P95_ms,B.total_max_ms);
    fprintf(fid,'complete_at_or_under_1ms=%d/%d\n',B.under_1ms_count,B.success_count);
    fprintf(fid,'no_warmup=1\nno_runtime_changes=1\ncontinuous_time_certified=0\n');
    if ~isempty(B.error),fprintf(fid,'error=%s\n',B.error);end
    for i=1:numel(B.samples)
        r=B.samples(i);
        fprintf(fid,['sample=%d time_s=%.6f native_ms=%.6f ' ...
            'end_to_end_ms=%.6f rhs_evals=%g rejects=%g status=%s reason=%s\n'], ...
            r.sample_index,r.time_s,r.native_ms,r.total_ms, ...
            r.rhs_evaluations,r.integrator_rejections,r.status,r.reason);
    end
    clear clean
end
fprintf('[NATIVE SPEED] %s | complete=%d/%d | kernel median=%.4f ms | full call median=%.4f ms\n', ...
    B.status,B.success_count,B.sample_count,B.native_median_ms,B.total_median_ms);
end
function p=local_p95(a)
a=sort(a(isfinite(a)));p=NaN;
if ~isempty(a),p=a(max(1,ceil(.95*numel(a))));end
end
