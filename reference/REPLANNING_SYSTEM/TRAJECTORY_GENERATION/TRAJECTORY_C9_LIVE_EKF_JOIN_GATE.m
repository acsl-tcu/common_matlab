function R=TRAJECTORY_C9_LIVE_EKF_JOIN_GATE(piece,binding,agent,time,tol)
%TRAJECTORY_C9_LIVE_EKF_JOIN_GATE  Necessary live-state start test for C9.
% Read only the CURRENT Flight estimator and live physical authority. The
% existing C9 0..9 endpoint jets imply UAV 0..7 jets under the ideal taut
% model, but the EKF cannot supply the full high-order jets: matching
% position/velocity/direction is NECESSARY, NEVER sufficient for C7 join,
% tracking, tension under real dynamics, or HLC/torque certification.
% No cached mL, private limit, nominal-time rejoin, or HLC COMMIT.
start=tic;
R=struct('status','UNKNOWN_NOT_EVALUATED','necessary_start_pass',false, ...
    'safe_to_commit',false,'full_C7_join_proven',false, ...
    'real_HLC_tracking_certified',false,'authority_snapshot',struct(), ...
    'time_s',NaN,'uav_position_error_m',NaN, ...
    'uav_velocity_error_mps',NaN, ...
    'payload_position_error_m',NaN,'payload_velocity_error_mps',NaN, ...
    'cable_direction_error',NaN,'cable_length_error_m',NaN, ...
    'start_ideal_taut_tension_N',NaN,'start_ideal_thrust_N',NaN, ...
    'elapsed_ms',NaN, ...
    'scope','REAL_EKF_ZERO_FIRST_JETS_NECESSITY_ONLY__HLC_PENDING');
keys={'uav_position_m','uav_velocity_mps', ...
    'payload_position_m','payload_velocity_mps', ...
    'cable_direction_unitless','cable_length_m'};
assert(isstruct(tol)&&isscalar(tol)&&all(isfield(tol,keys)), ...
    'C9_LIVE_JOIN:Tolerances','Require all externally-owned EKF comparison bounds.');
for k=1:numel(keys)
    v=tol.(keys{k});
    assert(isnumeric(v)&&isreal(v)&&isscalar(v)&&isfinite(v)&&v>=0, ...
        'C9_LIVE_JOIN:Tolerances','Invalid caller-owned bound.');
end
S=TRAJECTORY_C9_HLC_FORCE_AUTHORITY.current(binding,agent,time);
R.authority_snapshot=S;
if ~S.valid
    R.status=S.status;
    R.elapsed_ms=toc(start)*1000;return
end
R.time_s=double(time.t);
if ~(isstruct(piece)&&isscalar(piece)&&isfield(piece,'t0_s')&& ...
        isequal(double(piece.t0_s),double(time.t)))
    R.status='UNKNOWN_PIECE_NOT_AT_CURRENT_EKF_TIME';
    R.elapsed_ms=toc(start)*1000;return
end
state=agent.estimator.result.state;
fields={'p','v','pL','vL','pT'};
X=struct();
for k=1:numel(fields)
    f=fields{k};
    if isstruct(state)
        present=isfield(state,f);
    else
        present=isobject(state)&&isprop(state,f);
    end
    if ~present
        R.status=['UNKNOWN_MISSING_REAL_EKF_' upper(f)];
        R.elapsed_ms=toc(start)*1000;return
    end
    x=state.(f);
    if ~(isnumeric(x)&&isreal(x)&&isequal(size(x),[3 1])&& ...
            all(isfinite(x)))
        R.status=['UNKNOWN_INVALID_REAL_EKF_' upper(f)];
        R.elapsed_ms=toc(start)*1000;return
    end
    X.(f)=double(x);
end
try
    E=TRAJECTORY_TAUT_FLATNESS.evaluate(piece,time.t,S.authority);
catch ME
    R.status=['UNKNOWN_C9_FLATNESS_' ME.identifier];
    R.elapsed_ms=toc(start)*1000;return
end
if ~E.valid
    R.status='UNKNOWN_C9_START_SINGULAR';
    R.elapsed_ms=toc(start)*1000;return
end
R.uav_position_error_m=norm(E.uav_jets(:,1)-X.p);
R.uav_velocity_error_mps=norm(E.uav_jets(:,2)-X.v);
R.payload_position_error_m=norm(E.load_jets(:,1)-X.pL);
R.payload_velocity_error_mps=norm(E.load_jets(:,2)-X.vL);
R.cable_direction_error=norm(E.cable_direction_jets(:,1)-X.pT);
R.cable_length_error_m=abs(norm(X.pL-X.p)-S.authority.cable_length_m);
R.start_ideal_taut_tension_N=E.tension_N;
R.start_ideal_thrust_N=E.thrust_N;
if ~(E.sampled_tension_above_minimum&&E.sampled_thrust_within_limits)
    R.status='UNKNOWN_C9_START_IDEAL_FORCE_OUT_OF_LIMITS';
elseif R.uav_position_error_m>tol.uav_position_m || ...
        R.uav_velocity_error_mps>tol.uav_velocity_mps || ...
        R.payload_position_error_m>tol.payload_position_m || ...
        R.payload_velocity_error_mps>tol.payload_velocity_mps || ...
        R.cable_direction_error>tol.cable_direction_unitless || ...
        R.cable_length_error_m>tol.cable_length_m
    R.status='REJECT_C9_START_NOT_CURRENT_EKF_STATE';
else
    R.necessary_start_pass=true;
    R.status='PASS_LIVE_EKF_C9_START_NECESSARY_ONLY__C7_HLC_PENDING';
end
R.elapsed_ms=toc(start)*1000;
end
