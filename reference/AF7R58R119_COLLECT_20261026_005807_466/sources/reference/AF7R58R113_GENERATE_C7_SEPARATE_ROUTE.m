function O=AF7R58R113_GENERATE_C7_SEPARATE_ROUTE(spec,obs,live,opts)
% R113: GENERATE a real multisegment C7 load reference, then verify its
% independently inflated UAV / full cable / payload continuous sweep.
% Reuses R92 coupled minimum-eighth-derivative Bezier generator, the
% source's R99 multi-size knot corridors, R91 finite-interval kinematics,
% R80 tangent yaw and R80 independent geometry. No R112 big ball used.
% The geometric waypoints and times are planned/selected by the caller;
% no fixed maneuver duration, nominal-clock catch-up or path-specific data.
% Uncertified candidates NEVER become native production packets.
assert(isstruct(spec)&&isscalar(spec)&&isstruct(live)&&isscalar(live)&& ...
  isstruct(opts)&&isscalar(opts)&&isstruct(obs),'R113 failure - Inputs');
keys={'times_s','waypoints','start_jets','end_jets','phase_delay_s'};
for k=1:numel(keys)
 assert(isfield(spec,keys{k}),'R113:ExplicitC7GenerationInput', ...
  'Missing %s',keys{k});
end
req={'gravity_m_s2','cable_length_m','rQ_m','rL_m','rC_m', ...
 'd_margin_m','finite_sensing_vQ_limit_m_s', ...
 'robust_tracking_margin_enabled'};
for k=1:numel(req)
 assert(isfield(live,req{k}),'R113:ExternalParameterAuthority', ...
  'Missing %s',req{k});
end
assert(isfield(opts,'max_nodes_per_check')&& ...
  opts.max_nodes_per_check>=1,'R113 failure - ExplicitNumericalWorkBudget');
O=struct('status','INCONCLUSIVE_NOT_GENERATED', ...
 'route',struct([]),'multisize_knot',struct([]), ...
 'swept_corridor',struct([]),'checks',struct([]), ...
 'reference_geometry_speed_heading_pass',false, ...
 'full_execution_certified',false,'production_commit_authorized',false, ...
 'uses_r112_whole_body_ball',false, ...
 'nominal_input',struct([]),'nominal_thrust_tension_strength_pass',false);
R=AF7R58R92_C7_MINIMUM8_ROUTE(spec.times_s,spec.waypoints, ...
 spec.start_jets,spec.end_jets,spec.phase_delay_s,live.cable_length_m);
O.route=R;
if ~isfield(R,'reference_candidate_found')||~R.reference_candidate_found
 O.status=['INCONCLUSIVE_C7_GENERATION__' char(R.status)];return;
end
% Numerically match position and 0..7 shared endpoint jets across all cells.
scale=max([1;abs(R.knot_jets(:));abs(R.waypoints(:))]);
Tmin=min(diff(double(spec.times_s)));
% Differentiating degree-15 Bezier seven times amplifies floating error.
% This is a numerical guard, NOT an IEEE proof of exact equality.
C7tol=8192*eps(scale)*max(1,(15/Tmin)^7);
if ~isfinite(R.max_jet_residual)||R.max_jet_residual>C7tol
 O.status='INCONCLUSIVE_C7_JET_RESIDUAL';return;
end
% R99 is an optional NON-COMPLETE knot corridor seed test. Its failure does
% not disprove a candidate independently certified by full-time R113/R80.
O.multisize_knot=AF7R58R99_MULTI_SIZE_KNOT_CORRIDOR(R,obs,live);
O.swept_corridor=AF7R58R113_SEPARATE_SWEPT_CORRIDORS(R,obs,live,opts);
if ~O.swept_corridor.reference_geometry_certified
 O.status=['INCONCLUSIVE_THREE_BODY_SWEEP__' O.swept_corridor.status];return;
end
rInfl=live.d_margin_m;
if live.robust_tracking_margin_enabled
 rInfl=rInfl+live.tracking_tube_m;
end
vlim=double(live.finite_sensing_vQ_limit_m_s);
assert(isfinite(vlim)&&vlim>0,'R113 failure - LiveSpeedLimit');
if isfield(live,'operational_uav_speed_limit_m_s')
 vop=live.operational_uav_speed_limit_m_s;
 assert(isnumeric(vop)&&isscalar(vop)&&isfinite(vop)&&vop>0, ...
  'R113 failure - OperationalSpeedAuthority');
 vlim=min(vlim,vop);
end
S=numel(R.segments);
checks=repmat(struct('kinematics',struct([]),'yaw',struct([]), ...
 'geometry_independent',struct([]),'pass',false,'reason','NOT_RUN'),1,S);
allpass=true;
for s=1:S
 C=R.segments(s);
 K=AF7R58R91_STABLE_KINEMATIC_CERT(C,live.gravity_m_s2, ...
  live.cable_length_m,vlim,opts.max_nodes_per_check);
 H=AF7R58R80_HEADING_C6_CERT(C,opts.max_nodes_per_check);
 G=AF7R58R80_CONTINUOUS_SEPARATE_BODY_CERT(C,obs, ...
  live.gravity_m_s2,live.cable_length_m,live.rQ_m+rInfl, ...
  live.rC_m+rInfl,live.rL_m+rInfl,opts.max_nodes_per_check);
 checks(s).kinematics=K;checks(s).yaw=H;
 checks(s).geometry_independent=G;
 checks(s).pass=K.certified&&H.certified&&G.certified;
 if ~checks(s).pass
  checks(s).reason=[char(K.status) ' | ' char(H.status) ' | ' char(G.status)];
  allpass=false;
 else
  checks(s).reason='REFERENCE_GEOMETRY_SPEED_YAW_NUMERIC_CHECKS_PASS';
 end
end
O.checks=checks;
O.reference_geometry_speed_heading_pass=allpass;
if allpass
 O.nominal_input=AF7R58R113_INPUT_GATES(R,live,opts);
 O.nominal_thrust_tension_strength_pass= ...
    O.nominal_input.nominal_thrust_tension_strength_pass;
 if O.nominal_thrust_tension_strength_pass
  O.status='C7_GEOMETRY_SPEED_YAW_AND_NOMINAL_INPUT_PASS__HLC_TORQUE_TUBE_OPEN';
 else
  O.status=['C7_REFERENCE_PASS__INPUT_OPEN__' char(O.nominal_input.status)];
 end
else
 O.status='GENERATED_BUT_INDEPENDENT_REFERENCE_GATES_INCONCLUSIVE';
end
% Real HLC torque, actual tracking tube, cable rated strength and entire
% future/tail are NOT inferred from reference checks. Atomic commit remains
% exclusively the original production Stage2/Stage3 responsibility.
end
