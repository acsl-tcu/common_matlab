function C=CHECK_TRIGGER_REFERENCE_CACHE(agent)
% Read-only postflight equality test of the current reference cache.
% No Flight, no preflight, no HLC/plant, no sensor or LOGGER mutation.
% Validates EVERY reused sample against a FRESH evaluation of the active
% original TIME_VARYING_REFERENCE function (not a sampled subset).
C=struct('status','UNKNOWN','exact_match',false, ...
    'active_reference_cacheable',false,'first_call_ms',NaN, ...
    'second_call_ms',NaN,'fresh_reference_ms',NaN, ...
    'first_reused',0,'second_reused',0,'fresh_evaluations',NaN, ...
    'max_abs_error',Inf,'error','');
try
    assert(nargin==1&&isobject(agent)&&isscalar(agent), ...
        'CHECK_TRIGGER_REFERENCE_CACHE:Agent','Actual postflight agent required.');
    nominal=agent.reference.timevarying;
    assert(isa(nominal.func,'function_handle')&&~isempty(nominal.t), ...
        'CHECK_TRIGGER_REFERENCE_CACHE:Source','Original Flight reference unavailable.');
    dt=double(agent.controller.hlc_suspended.param.dt);
    L=double(agent.parameter.get("cableL"));
    assert(isscalar(dt)&&isfinite(dt)&&dt>0 && ...
        isscalar(L)&&isfinite(L)&&L>0, ...
        'CHECK_TRIGGER_REFERENCE_CACHE:Parameters','Live Flight parameters invalid.');
    H=4; % Diagnostic reference horizon only, not an operational change.
    t0=double(nominal.t);
    n=floor(H/dt);
    offsets=(0:n-1)*dt;
    if isempty(offsets),offsets=0;end
    first=t0+offsets;
    second=t0+dt+offsets;
    f=@(tt)local_reference(nominal,tt,L);
    cache=TRIGGER_REFERENCE_WINDOW_CACHE();
    tick=tic;
    [~,a]=cache.evaluate(nominal,first,L,f);
    C.first_call_ms=1000*toc(tick);
    tick=tic;
    [second_refs,b]=cache.evaluate(nominal,second,L,f);
    C.second_call_ms=1000*toc(tick);
    tick=tic;
    direct=zeros(28,numel(second));
    for i=1:numel(second)
        v=f(second(i));direct(1:numel(v),i)=v;
    end
    C.fresh_reference_ms=1000*toc(tick);
    C.max_abs_error=max(abs(second_refs(:)-direct(:)));
    C.exact_match=isequal(second_refs,direct);
    C.first_reused=a.exact_reused;
    C.second_reused=b.exact_reused;
    S=cache.get_status();
    C.fresh_evaluations=S.fresh_evaluations;
    C.active_reference_cacheable=S.active_source_cacheable;
    if C.exact_match
        C.status='PASS_EXACT_FUTURE_REFERENCE__NO_MODEL_OR_SAFETY_PROOF';
    else
        C.status='FAIL_FUTURE_REFERENCE_DIFFERENCE';
    end
catch ME
    C.status='FAIL_CLOSED_REFERENCE_CACHE_AUDIT';
    C.error=[ME.identifier ' | ' ME.message];
end
out=fileparts(mfilename('fullpath'));
save(fullfile(out,'TRIGGER_REFERENCE_CACHE_RESULT.mat'),'C','-v7');
fid=fopen(fullfile(out,'TRIGGER_REFERENCE_CACHE_RESULT.txt'),'wt');
if fid~=-1
    cleanup=onCleanup(@()fclose(fid));
    fprintf(fid,'status=%s\nexact_match=%d\ncacheable=%d\n', ...
        C.status,C.exact_match,C.active_reference_cacheable);
    fprintf(fid,['first_call_ms=%.8g\nsecond_call_ms=%.8g\n' ...
        'fresh_reference_ms=%.8g\nfirst_reused=%d\nsecond_reused=%d\n' ...
        'max_abs_error=%.17g\nerror=%s\n'], ...
        C.first_call_ms,C.second_call_ms,C.fresh_reference_ms, ...
        C.first_reused,C.second_reused,C.max_abs_error,C.error);
    clear cleanup
end
fprintf('[TRIGGER REF CACHE] %s | exact reused=%d | second=%.4f ms vs direct=%.4f ms\n', ...
    C.status,C.second_reused,C.second_call_ms,C.fresh_reference_ms);
end
function v=local_reference(nominal,t,L)
v=double(nominal.func(double(t)-double(nominal.t)));
assert(iscolumn(v)&&isreal(v)&&numel(v)>=4&&numel(v)<=28&& ...
    all(isfinite(v)), ...
    'CHECK_TRIGGER_REFERENCE_CACHE:OriginalReference','Invalid native Flight reference.');
v(3)=v(3)-L;
end
