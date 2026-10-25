function C=AF7R58R106_CAPTURE_STATUS(agent)
% Read last Flight's R88 exact EKF capture. No class mutations or solver.
O=local_find(agent,5);
assert(~isempty(O),'R106:NativeReplannerRequired', ...
    'Provide current REPLANNING_SYSTEM instance or agent.');
S=O.r88_snapshots;
C=struct('status','R106_NO_SNAPSHOTS_IN_CURRENT_FLIGHT', ...
    'count',numel(S),'authenticated_count',0,'raw_complete_count',0, ...
    'event_times_s',zeros(1,0),'reference_versions',zeros(1,0), ...
    'known_counts',zeros(1,0),'ekf_status',{{}}, ...
    'ekf_q_norm_error',zeros(1,0), ...
    'ekf_cable_relation_error_m',zeros(1,0), ...
    'current_geometry_residual_m',zeros(1,0), ...
    'current_pT_norm_error',zeros(1,0), ...
    'recommended_time_s',NaN,'recommended_version',NaN, ...
    'production_commit_authorized',false);
if isempty(S),return;end
n=numel(S);C.event_times_s=nan(1,n);
C.reference_versions=nan(1,n);C.known_counts=zeros(1,n);
C.ekf_status=cell(1,n);
C.ekf_q_norm_error=nan(1,n);
C.ekf_cable_relation_error_m=nan(1,n);
C.current_geometry_residual_m=nan(1,n);
C.current_pT_norm_error=nan(1,n);
for i=1:n
    z=S(i);
    C.event_times_s(i)=double(z.time_s);
    C.reference_versions(i)=double(z.reference_version);
    C.known_counts(i)=nnz(z.known_mask);
    if isfield(z,'ekf_event') && isstruct(z.ekf_event)
        if isfield(z.ekf_event,'q_norm_error')
            C.ekf_q_norm_error(i)=double(z.ekf_event.q_norm_error);
        end
        if isfield(z.ekf_event,'cable_relation_error_m')
            C.ekf_cable_relation_error_m(i)= ...
                double(z.ekf_event.cable_relation_error_m);
        end
    end
    [~,d]=AF7R58R106_SOURCE_EVENT(O,z.time_s,z.reference_version);
    C.ekf_status{i}=d.status;
    if isfield(d,'raw_state_present')&&d.raw_state_present
        C.raw_complete_count=C.raw_complete_count+1;
    end
    if isfield(d,'geometry_residual_m')
        C.current_geometry_residual_m(i)=d.geometry_residual_m;
    end
    if isfield(d,'pT_norm_error')
        C.current_pT_norm_error(i)=d.pT_norm_error;
    end
    if d.authenticated
        C.authenticated_count=C.authenticated_count+1;
        C.recommended_time_s=z.time_s;
        C.recommended_version=z.reference_version;
    end
end
if C.authenticated_count>0
    C.status='R106_SAME_TICK_EKF_AVAILABLE_FOR_OFFLINE_REPLAY';
else
    if C.raw_complete_count>0
        C.status='R106_RAW_EKF_CAPTURED__GEOMETRY_OR_PACKET_GATE_OPEN';
    else
        C.status='R106_R88_AVAILABLE_BUT_RAW_EKF_MISSING';
    end
end
end
function O=local_find(x,depth)
O=[];
if isa(x,'REPLANNING_SYSTEM')&&isscalar(x),O=x;return;end
if depth<1||isempty(x)||numel(x)~=1,return;end
if isobject(x)
    try,n=properties(x);catch,return;end
elseif isstruct(x)
    n=fieldnames(x);
else
    return;
end
skip={'self','result','parameter','plant','sensor','estimator', ...
    'controller','logger','time','state','parent'};
for j=1:numel(n)
    if any(strcmpi(n{j},skip)),continue;end
    try,O=local_find(x.(n{j}),depth-1);catch,O=[];end
    if ~isempty(O),return;end
end
end
