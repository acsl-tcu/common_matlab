function OUT = AF7R58R62_EKF_NOW_GATE(IN)
% R62: one-instant sensor-known whole-body conservative collision gate.
% Same production g26_future_trigger_kernel_mex EXACT-SUPPORT algorithm,
% queried with N=1 (time zero ONLY). No future extrapolation, no tracking tube.
% All vehicle radii and obstacle records must be externally supplied.
% An 'unsafe' result can mean that a sufficient support separator was not
% found; it is NOT a proof of physical contact.
% Does NOT replan, modify HLC, modify source, or issue a safety certificate.
need={'pQ_EKF','pL_EKF','q_EKF','cable_length_m', ...
      'centers','rotations','axes','ids','known_mask','radii_m'};
for i=1:numel(need)
    assert(isstruct(IN)&&isfield(IN,need{i}), ...
        'R62:MissingInput','Missing external field %s',need{i});
end
Q=double(IN.pQ_EKF(:));P=double(IN.pL_EKF(:));q=double(IN.q_EKF(:));
L=double(IN.cable_length_m);
C=double(IN.centers);R=double(IN.rotations);A=double(IN.axes);
ids=double(IN.ids(:).');mask=double(IN.known_mask(:).');
r=double(IN.radii_m(:));
assert(numel(Q)==3 && numel(P)==3&&numel(q)==3&& ...
    all(isfinite([Q;P;q]))&&isscalar(L)&&isfinite(L)&&L>0 && ...
    numel(r)==3&&all(isfinite(r))&&all(r>=0),'R62:State','Invalid state or external radii');
N=numel(ids);
assert(isequal(size(C),[3 N])&&isequal(size(R),[9 N])&& ...
    isequal(size(A),[3 N])&&numel(mask)==N&& ...
    all(isfinite([C(:);R(:);A(:);ids(:);mask(:)])) && ...
    all(A(:)>0)&&all(mask==0|mask==1), ...
    'R62:Packet','Invalid sensor-known obstacle packet');
OUT=struct('status','NOT_RUN','unsafe_or_unproved',false, ...
    'known_obstacle_count',nnz(mask), ...
    'offending_obstacle_id',NaN,'offending_component_code',NaN, ...
    'estimated_cable_residual_m',norm(P-Q-L*q), ...
    'estimated_direction_norm_error',abs(norm(q)-1), ...
    'ekf_cable_constraint_certified',false, ...
    'time_to_collision_s',NaN,'prediction_horizon_s',0, ...
    'real_hlc_future_computed',false,'tube_installed',false, ...
    'committable',false);
if N==0 || ~any(mask)
    OUT.status='NO_SENSOR_KNOWN_OBSTACLES__NOT_GLOBAL_CLEARANCE';
    return;
end
assert(exist('g26_future_trigger_kernel_mex','file')==3, ...
    'R62:MEX','Existing production g26_future_trigger_kernel_mex is required; do not approximate it');
% Reconstruct displacement from BOTH EKF endpoints. This preserves the vehicle
% sensor center and estimated payload. Disagreement with EKF q is reported,
% never hidden by arbitrary tolerances; physical-cable consistency is unproved.
qend=(P-Q)/L;
[hit,idx,oid,comp]=g26_future_trigger_kernel_mex( ...
    0,1,L,P,qend,C,R,A,r,ids,mask);
assert(~hit || isequal(double(idx),1),'R62:Current','Instantaneous kernel index invalid');
OUT.unsafe_or_unproved=logical(hit);
if hit
    OUT.status='CURRENT_KNOWN_OBSTACLE_UNSAFE_OR_UNSEPARATED';
    OUT.offending_obstacle_id=double(oid);
    OUT.offending_component_code=double(comp);
    OUT.time_to_collision_s=0;
else
    OUT.status='CURRENT_KNOWN_OBSTACLES_SEPARATED__FUTURE_UNKNOWN';
end
end
