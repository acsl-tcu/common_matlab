function R=CHECK_TRIGGER_PURE_REFERENCE_FLIGHT(replanner_instance)
% Flight post-analysis only: never evaluates the flight reference or EKF.
assert(isobject(replanner_instance)&&ismethod(replanner_instance,'get_status'), ...
    'CHECK_TRIGGER_PURE_REFERENCE_FLIGHT:Input', ...
    'Pass the actual replanner_instance from the completed Flight.');
Q=replanner_instance.get_status();
assert(isfield(Q,'trigger_reference_cache'), ...
    'CHECK_TRIGGER_PURE_REFERENCE_FLIGHT:Version', ...
    'Trigger reference cache status is unavailable.');
C=Q.trigger_reference_cache;
R=struct();
R.status='REFERENCE_CACHE_READONLY_FLIGHT_AUDIT__NO_SAFETY_CLAIM';
R.cache=C;
R.exact_reuses=C.exact_reuses;
R.fresh_evaluations=C.fresh_evaluations;
R.cacheable=C.active_source_cacheable;
R.cache_rejection_is_not_model_failure=~R.cacheable;
R.complete_prediction_count=NaN;
R.first_flight_trigger_ms=NaN;
if isfield(Q,'first_flight_traces') && ~isempty(Q.first_flight_traces)
    F=Q.first_flight_traces(1);
    if isfield(F,'pre_store_total_ms')
        R.first_flight_trigger_ms=F.pre_store_total_ms;
    end
end
R.safety_certified=false;
folder=fileparts(mfilename('fullpath'));
save(fullfile(folder,'TRIGGER_PURE_REFERENCE_FLIGHT.mat'),'R');
fid=fopen(fullfile(folder,'TRIGGER_PURE_REFERENCE_FLIGHT.txt'),'wt');
if fid>=0
    c=onCleanup(@()fclose(fid));
    fprintf(fid,'%s\ncacheable=%d exact_reuses=%d fresh_evaluations=%d first_pre_store_ms=%.6f\n',...
        R.status,R.cacheable,R.exact_reuses,R.fresh_evaluations,R.first_flight_trigger_ms);
end
fprintf('[REFERENCE CACHE] cacheable=%d exact_reuses=%d fresh=%d\n', ...
    R.cacheable,R.exact_reuses,R.fresh_evaluations);
end
