function A=AF7R58R113_INPUT_GATES(route,live,opts)
% R113 production-boundary nominal-input audit using EXISTING R90.
% This is a REFERENCE computation. It is NOT a proof of actual HLC torque,
% observer error, actuator reachable set, or a certified production commit.
% Required live authority is explicit; never invent mQ, mL, thrust limits,
% torque limits or cable rated strength. Work budget is not a time cap.
assert(isstruct(route)&&isscalar(route)&&isfield(route,'segments')&& ...
 ~isempty(route.segments)&&isstruct(live)&&isscalar(live)&& ...
 isstruct(opts)&&isscalar(opts), ...
 'R113 input: need route, runtime authority and work budget.');
required={'gravity_m_s2','cable_length_m','vehicle_mass_kg', ...
 'latest_EKF_mL_kg','thrust_bounds_N','torque_bounds_Nm'};
missing=required(~isfield(live,required));
A=struct('status','MISSING_RUNTIME_INPUT_AUTHORITY__NOT_COMMITTABLE', ...
 'missing_authorities',{missing},'segments',struct([]), ...
 'nominal_thrust_tension_strength_pass',false, ...
 'nominal_thrust_certified',false,'taut_tension_certified',false, ...
 'cable_strength_certified',false,'nominal_torque_certified',false, ...
 'real_hlc_closed_loop_certified',false, ...
 'production_commit_authorized',false, ...
 'nominal_thrust_upper_N',NaN,'nominal_tension_upper_N',NaN, ...
 'nominal_tension_lower_N',NaN,'runtime_mass_snapshot_kg',NaN, ...
 'cable_rating_authority_present',false);
if ~isempty(missing),return;end
% Validate authority with R90's established checks. R90 never proves torque.
P=struct('gravity_m_s2',live.gravity_m_s2, ...
 'cable_length_m',live.cable_length_m, ...
 'vehicle_mass_kg',live.vehicle_mass_kg, ...
 'latest_EKF_mL_kg',live.latest_EKF_mL_kg, ...
 'thrust_bounds_N',live.thrust_bounds_N, ...
 'torque_bounds_Nm',live.torque_bounds_Nm);
if isfield(live,'tension_max_N')
 P.tension_max_N=live.tension_max_N;
 A.cable_rating_authority_present=true;
end
A.runtime_mass_snapshot_kg=double(P.latest_EKF_mL_kg);
if isfield(opts,'input_max_nodes_per_check')
 nodes=opts.input_max_nodes_per_check;
else
 assert(isfield(opts,'max_nodes_per_check'), ...
  'R113 input: input work budget not supplied.');
 nodes=opts.max_nodes_per_check;
end
N=numel(route.segments);
records=repmat(struct('status','NOT_RUN','R90',struct([])),1,N);
allThrust=true;allTaut=true;allRating=true;
maxF=-Inf;maxT=-Inf;minT=Inf;
for s=1:N
 C=route.segments(s);
 assert(all(isfield(C,{'t0','t1','payload_cp'})), ...
  'R113 input: malformed C7 segment.');
 B=struct('t0',C.t0,'t1',C.t1, ...
     'duration_s',C.t1-C.t0,'payload_cp',C.payload_cp);
 R=AF7R58R90_INPUT_ENVELOPE(B,P,nodes);
 records(s).status=char(R.status);records(s).R90=R;
 allThrust=allThrust && logical(R.nominal_thrust_certified);
 allTaut=allTaut && logical(R.taut_tension_positive_certified);
 allRating=allRating && logical(R.cable_strength_certified);
 if isfinite(R.nominal_thrust_upper_N)
  maxF=max(maxF,R.nominal_thrust_upper_N);
 end
 if isfinite(R.tension_upper_N)
  maxT=max(maxT,R.tension_upper_N);
 end
 if isfinite(R.tension_lower_N)
  minT=min(minT,R.tension_lower_N);
 end
end
A.segments=records;
A.nominal_thrust_certified=allThrust;
A.taut_tension_certified=allTaut;
A.cable_strength_certified=allRating;
if isfinite(maxF),A.nominal_thrust_upper_N=maxF;end
if isfinite(maxT),A.nominal_tension_upper_N=maxT;end
if isfinite(minT),A.nominal_tension_lower_N=minT;end
A.nominal_thrust_tension_strength_pass=allThrust&&allTaut&&allRating;
if ~allThrust||~allTaut
 A.status='R90_NOMINAL_THRUST_OR_TAUT_TENSION_NOT_CERTIFIED';
elseif ~A.cable_rating_authority_present
 A.status='R90_NOMINAL_THRUST_TAUT_PASS__CABLE_RATING_MISSING';
elseif ~allRating
 A.status='R90_CABLE_STRENGTH_NOT_CERTIFIED';
else
 A.status='R90_NOMINAL_THRUST_TAUT_STRENGTH_PASS__HLC_TORQUE_TUBE_OPEN';
end
% Deliberate hard boundary: torque_bounds is a clamp source, not proof of
% unsaturated commanded torque for this route. Never promote R90 to commit.
end
