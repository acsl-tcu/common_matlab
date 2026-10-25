function [P,D]=AF7R58R104_PHYSICAL_PREDICTOR(replanner,F)
% Invoke PRE-EXISTING source-bound HLC+plant preview OUTSIDE Flight do().
% Do not create a new approximate predictor, alter HLC/plant, or claim a
% sampled rollout is an interval-reachability proof.
P=[];
D=struct('status','SOURCE_BOUND_HLC_PLANT_HELPER_MISSING', ...
    'invoked',false,'preview_structurally_valid',false, ...
    'closed_loop_interval_proved',false,'flight_loop_safe',false, ...
    'error_identifier','','error_message','');
assert(isa(replanner,'REPLANNING_SYSTEM')&&isscalar(replanner)&& ...
    isstruct(F)&&isscalar(F)&&isfield(F,'time_s'), ...
    'R104:PredictorBinding','Current replanner and event frame required.');
stack=dbstack;
if any(strcmp({stack.name},'REPLANNING_SYSTEM.do'))
    D.status='FORBIDDEN_INSIDE_25MS_FLIGHT_DO';return;
end
if exist('AF7R58R71_EXACT_HLC_PLANT_PREVIEW','file')~=2
    return;
end
if ~ismethod(replanner,'r71_predict_live_hlc_future')
    D.status='NATIVE_HLC_PREVIEW_METHOD_MISSING';return;
end
D.status='LIVE_HLC_PREVIEW_CALLED__SCHEMA_PENDING';
try
    D.invoked=true;
    out=replanner.r71_predict_live_hlc_future(double(F.time_s));
    if ~isstruct(out)||~isscalar(out)
        D.status='LIVE_HLC_PREVIEW_INVALID_OUTPUT';return;
    end
    req={'time_s','reference_version','pL','pQ','source'};
    if ~all(isfield(out,req))
        D.status='LIVE_HLC_PREVIEW_SCHEMA_NOT_COMPATIBLE';return;
    end
    if ~strcmp(char(out.source),'LIVE_EKF_HLC_PLANT_ROLLOUT')
        D.status='LIVE_HLC_PREVIEW_SOURCE_NOT_AUTHENTICATED';return;
    end
    if ~isequal(double(out.time_s),double(F.time_s))|| ...
        ~isequal(double(out.reference_version),double(F.reference_version))
        D.status='LIVE_HLC_PREVIEW_STALE';return;
    end
    N=round(double(F.horizon_s)/double(F.dt_s))+1;
    if ~isequal(size(out.pL),[3 N])||~isequal(size(out.pQ),[3 N])|| ...
        ~all(isfinite([out.pL(:);out.pQ(:)]))
        D.status='LIVE_HLC_PREVIEW_BAD_TIME_GRID';return;
    end
    P=out;
    D.preview_structurally_valid=true;
    D.status='LIVE_HLC_PREVIEW_AVAILABLE__NUMERIC_ONLY';
catch ME
    D.status='LIVE_HLC_PREDICTOR_EXCEPTION';
    D.error_identifier=char(ME.identifier);
    D.error_message=char(ME.message);
end
end
