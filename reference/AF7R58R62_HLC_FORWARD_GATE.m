function OUT = AF7R58R62_HLC_FORWARD_GATE(IN)
% R62: isolated rollout driver for the USER'S EXISTING HLC and plant.
% No substitute controller or guessed dynamics. Required callbacks must run
% on a *separate model state*, without altering the live flight agent.
% Calls: stateNext=isolated_hlc_plant_step_fn(state,time,dt)
%        geometry=geometry_fn(state) with pQ,pL,q fields.
% Continuous-time speed bounds are optional; without a CERTIFIED provider
% the interval gate is UNKNOWN, never accepted. No tracking-error tube.
required={'state0','isolated_hlc_plant_step_fn','geometry_fn', ...
          't0_s','dt_s','horizon_s','packet', ...
          'state_consistency_tolerance_m','isolated_state_authorized'};
for i=1:numel(required)
    assert(isstruct(IN)&&isfield(IN,required{i}), ...
        'R62:Missing','Missing external input: %s',required{i});
end
assert(isa(IN.isolated_hlc_plant_step_fn,'function_handle')&& ...
       isa(IN.geometry_fn,'function_handle')&& ...
       islogical(IN.isolated_state_authorized)&& ...
       isscalar(IN.isolated_state_authorized)&&IN.isolated_state_authorized, ...
       'R62:Isolation','Provide verified cloned-state HLC+plant callbacks');
t0=double(IN.t0_s);dt=double(IN.dt_s);H=double(IN.horizon_s);
assert(all(isfinite([t0 dt H]))&&dt>0&&H>0,'R62:Timing','Bad external timing');
N=round(H/dt)+1;
assert(abs((N-1)*dt-H)<=64*eps(max(1,H)), ...
    'R62:Grid','External horizon must align to control grid');
S=IN.state0;tt=t0+(0:N-1)*dt;
Q=zeros(3,N);P=zeros(3,N);q=zeros(3,N);
OUT=struct('status','UNKNOWN_UNCERTIFIED_FUTURE','samples',N, ...
    'first_unsafe_sample_s',NaN,'sampled_unsafe',false, ...
    'wholebody_interval',struct(),'committable',false, ...
    'tube_installed',false,'hlc_source_hash_verified',false);
for k=1:N
    G=IN.geometry_fn(S);
    assert(isstruct(G)&&all(isfield(G,{'pQ','pL','q'})), ...
        'R62:Geometry','Invalid external HLC/plant geometry');
    Q(:,k)=double(G.pQ(:));P(:,k)=double(G.pL(:));q(:,k)=double(G.q(:));
    packet=IN.packet;
    packet.pQ_EKF=Q(:,k);packet.pL_EKF=P(:,k);packet.q_EKF=q(:,k);
    packet.state_consistency_tolerance_m=IN.state_consistency_tolerance_m;
    A=AF7R58R62_EKF_NOW_GATE(packet);
    if A.unsafe_or_unproved && ~OUT.sampled_unsafe
        OUT.sampled_unsafe=true;
        OUT.first_unsafe_sample_s=tt(k)-t0;
        OUT.first_unsafe_status=A.status;
        OUT.first_unsafe_obstacle_id=A.offending_obstacle_id;
        OUT.first_unsafe_component=A.offending_component_code;
    end
    if k<N
        S=IN.isolated_hlc_plant_step_fn(S,tt(k),dt);
    end
end
OUT.time_s=tt;OUT.pQ=Q;OUT.pL=P;OUT.q=q;
if OUT.sampled_unsafe
    OUT.status='SAMPLED_MODEL_UNSAFE_OR_UNSEPARATED__NOT_PHYSICAL_CONTACT_PROOF';
else
    OUT.status='SAMPLED_MODEL_CLEAR_ONLY__NOT_CONTINUOUS_TIME_PROOF';
end
if isfield(IN,'certified_speed_bounds_fn') && ...
        isa(IN.certified_speed_bounds_fn,'function_handle')
    B=IN.certified_speed_bounds_fn(IN,tt,Q,P,q);
    assert(isstruct(B) && all(isfield(B,{'certified','speed_ub_Q','speed_ub_L'})), ...
        'R62:Bounds','Invalid external certified bound provider');
    known=IN.packet.known_mask(:)~=0;
    obs=repmat(struct('center',[],'rotation',[],'axes',[]),1,nnz(known));
    inds=find(known);
    for j=1:numel(inds)
        i=inds(j);
        obs(j).center=IN.packet.centers(:,i);
        obs(j).rotation=reshape(IN.packet.rotations(:,i),3,3);
        obs(j).axes=IN.packet.axes(:,i);
    end
    V=struct('time_s',tt,'pQ',Q,'pL',P,'q',q, ...
        'cable_length',IN.packet.cable_length_m, ...
        'radii',[IN.packet.radii_m(2);IN.packet.radii_m(1);IN.packet.radii_m(3)], ...
        'obstacles',obs,'speed_ub_Q',B.speed_ub_Q, ...
        'speed_ub_L',B.speed_ub_L, ...
        'speed_bounds_certified',logical(B.certified), ...
        'prediction_horizon_s',H, ...
        'state_consistency_tolerance_m',IN.state_consistency_tolerance_m);
    OUT.wholebody_interval=AF7R58R61_INTERVAL_WHOLEBODY_GATE(V);
    if OUT.wholebody_interval.conditional_continuous_model_clearance
        OUT.status='CONDITIONAL_CONTINUOUS_MODEL_CLEARANCE__EXECUTION_UNCERTIFIED';
    end
end
% The physical HLC+plant and formal interval bounds are supplied externally,
% so this function never authorizes a trajectory commit or real-aircraft safety.
end
