function A=CHECK_TRIGGER_FIRST_EIGHT_TICKS(replanner_instance)
% Disposable common_matlab/ post-flight diagnostic. Real stored traces only.
% No Flight code executes; no future/unsensed data is queried.
A=struct('status','UNKNOWN_EIGHT_TICK_TRACE', ...
    'records',0,'times_s',zeros(1,0), ...
    'known_frame_ms',zeros(1,0), ...
    'reference_now_ms',zeros(1,0), ...
    'authority_current_ms',zeros(1,0), ...
    'reference_window_ms',zeros(1,0), ...
    'predictor_wrapped_ms',zeros(1,0), ...
    'mex_ms',zeros(1,0), ...
    'geometry_ms',zeros(1,0), ...
    'core_wall_ms',zeros(1,0), ...
    'reason','');
try
    S=replanner_instance.get_status();
    assert(isstruct(S)&&isfield(S,'first_flight_traces') && ...
        isstruct(S.first_flight_traces), ...
        'CHECK_TRIGGER_FIRST_EIGHT_TICKS:Trace', ...
        'First-flight trace struct is missing. Enable flag before Flight.');
    F=S.first_flight_traces;
    n=numel(F);
    assert(n>=8, 'CHECK_TRIGGER_FIRST_EIGHT_TICKS:Count', ...
        'Eight genuine Flight traces not captured; enable the flag before Flight.');
    F=F(1:8);n=8;
    A.records=n;
    A.times_s=[F.time_s];
    A.known_frame_ms=[F.known_frame_ms];
    A.reference_now_ms=[F.reference_now_ms];
    A.authority_current_ms=[F.authority_current_ms];
    A.core_wall_ms=[F.core_wall_ms];
    A.reference_window_ms=nan(1,n);
    A.predictor_wrapped_ms=nan(1,n);
    A.mex_ms=nan(1,n);
    A.geometry_ms=nan(1,n);
    for k=1:n
        C=F(k).core_trace;
        if isstruct(C)
            A.reference_window_ms(k)=pick(C,'reference_window_ms');
            A.predictor_wrapped_ms(k)=pick(C,'predictor_wrapped_wall_ms');
            A.mex_ms(k)=pick(C,'mex_native_call_ms');
            A.geometry_ms(k)=pick(C,'geometry_wall_ms');
        end
    end
    A.status='REAL_FIRST_EIGHT_FLIGHT_TICKS_CAPTURED__TIMING_NOT_SAFETY';
catch ME
    A.reason=[ME.identifier ' | ' ME.message];
end
save('TRIGGER_FIRST_EIGHT_TICKS_RESULT.mat','A');
fid=fopen('TRIGGER_FIRST_EIGHT_TICKS_RESULT.txt','w');
if fid>=0
    fprintf(fid,'status=%s\nrecords=%d\nreason=%s\n', ...
        A.status,A.records,A.reason);
    fprintf(fid,'time_s known_ms reference_now_ms authority_current_ms ref_window_ms predictor_ms mex_ms geometry_ms core_wall_ms\n');
    for k=1:A.records
        fprintf(fid,'%.9f %.6f %.6f %.6f %.6f %.6f %.6f %.6f %.6f\n', ...
            A.times_s(k),A.known_frame_ms(k),A.reference_now_ms(k), ...
            A.authority_current_ms(k),A.reference_window_ms(k), ...
            A.predictor_wrapped_ms(k),A.mex_ms(k), ...
            A.geometry_ms(k),A.core_wall_ms(k));
    end
    fclose(fid);
end
end

function v=pick(S,name)
v=NaN;
if isfield(S,name)
    x=S.(name);
    if isnumeric(x)&&isscalar(x)&&isreal(x)
        v=double(x);
    end
end
end
