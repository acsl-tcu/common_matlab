function [A,b,D]=TRAJECTORY_C9_13D_CERTIFICATE_LEAF_CUTS( ...
        piece,S,shapes,prior_proof,payload_radius,clearance,physical_leaf_time,payload_witness_gate)
%TRAJECTORY_C9_13D_CERTIFICATE_LEAF_CUTS Certificate-guided LOCAL-QP cuts.
% At a failed full-time proof leaf I, the restricted payload Bernstein
% controls are H=P*C_I. For a normal n derived from CURRENT shape/curve:
%    n'*H_j >= support_shape(n)+rPayload+clearance, j=1..24
% is sufficient for the PAYLOAD over I at FIXED T and shape.
% Here ONLY ONE violated, movable control point per unresolved shape is
% linearized in 13 variables: 12 free controls + physical duration.
% The next candidate must still pass ALL full-time geometry/tension/thrust,
% complete cable/UAV, nominal torque and actual HLC proof gates. This is
% not a true globally feasible constraint or all-time certification.
% No fixed avoidance directions or obstacle-specific position branches.
% Backwards-compatible explicit diagnostic: fixed normalized leaf u is the
% earlier search tangent. For variable physical T, the *fixed physical
% time interval* returned by the prior full-time oracle has moving phases:
% u_a(T)=(t_a-t0)/T, u_b(T)=(t_b-t0)/T. The correct local search
% derivative includes both endpoint-phase terms. New QP opts in, while
% old direct call (6 arguments) preserves the original analytic test.
if nargin<7,physical_leaf_time=false;end
assert((islogical(physical_leaf_time)||isnumeric(physical_leaf_time))&& ...
    isscalar(physical_leaf_time)&&isreal(physical_leaf_time)&& ...
    isfinite(double(physical_leaf_time))&&any(double(physical_leaf_time)==[0 1]), ...
    'C9_LEAF_CUT:PhysicalLeafMode','Expected finite boolean mode.');
physical_leaf_time=logical(physical_leaf_time);
if nargin<8,payload_witness_gate=false;end
assert((islogical(payload_witness_gate)||isnumeric(payload_witness_gate))&& ...
    isscalar(payload_witness_gate)&&isreal(payload_witness_gate)&& ...
    isfinite(double(payload_witness_gate))&&any(double(payload_witness_gate)==[0 1]), ...
    'C9_LEAF_CUT:PayloadWitnessMode','Expected finite boolean mode.');
payload_witness_gate=logical(payload_witness_gate);
A=zeros(13,0);b=zeros(0,1);
D=struct('status','NO_FAILED_LEAF_CUT','plane_count',0, ...
    'support_calls',0,'skipped_no_movable_face',0, ...
    'maximum_payload_hull_deficit_m',0,'ids',zeros(1,0), ...
    'selected_control_indices',zeros(1,0), ...
    'payload_witness_gate_active',payload_witness_gate, ...
    'leaf_interval_s',[NaN NaN],'physical_leaf_time_tangent',physical_leaf_time, ...
    'safe_to_commit',false);
if ~isstruct(prior_proof)||~isscalar(prior_proof)|| ...
        ~all(isfield(prior_proof,{'unresolved_time_interval_s', ...
                                 'unresolved_obstacle_ids'}))
    D.status='NO_PROOF_WITNESS';return
end
if payload_witness_gate
    % The failed full-time oracle reports which of UAV/cable/payload
    % actually remained UNSEPARATED. This helper only produces PAYLOAD
    % Bernstein-hull cuts. Applying them to a cable-only failure can force
    % unrelated maneuvers and extra proof calls without mathematical gain.
    if ~isstruct(prior_proof)|| ...
            ~isfield(prior_proof,'unresolved_body_indices')
        D.status='NO_PAYLOAD_BODY_WITNESS';return
    end
    bad=double(prior_proof.unresolved_body_indices(:));
    if ~any(bad==3)
        D.status='SKIPPED_PAYLOAD_CUT_FOR_UAV_CABLE_ONLY_FAILURE';return
    end
end
if ~isfield(S,'dcontrol_points_dT_mps')|| ...
        ~isequal(size(S.dcontrol_points_dT_mps),[3 24]) || ...
        ~isequal(S.piece.control_points_m,piece.control_points_m)|| ...
        ~isequal(S.piece.duration_s,piece.duration_s)
    D.status='UNKNOWN_INVALID_SAME_CANDIDATE_DURATION_JACOBIAN';return
end
assert(isnumeric(payload_radius)&&isscalar(payload_radius)&& ...
    isfinite(payload_radius)&&payload_radius>=0&& ...
    isnumeric(clearance)&&isscalar(clearance)&& ...
    isfinite(clearance)&&clearance>=0, ...
    'C9_LEAF_CUT:Inflation','Finite validated geometry radius/clearance.');
ids=double(prior_proof.unresolved_obstacle_ids(:)).';
% This function is called with the canonical PREPARED shape snapshot from
% the independent all-time oracle (same snapshot as the local QP).  Raw
% OBSTACLE_INFORMATION.define entries have position/geometry, not center.
% Never silently reprepare here: it would introduce expensive per-QP
% validation and might accept a different obstacle snapshot.  Fail closed.
if ~isempty(shapes) && (~isstruct(shapes) || ...
        ~all(isfield(shapes,{'id','center','kind','dim','R','local_vertices'})))
    D.status='UNKNOWN_UNPREPARED_OBSTACLE_SHAPES';return
end
if isempty(ids)||isempty(shapes)
    D.status='NO_UNRESOLVED_OBSTACLES';return
end
t=double(prior_proof.unresolved_time_interval_s(:));
t0=double(piece.t0_s);T=double(piece.duration_s);
if numel(t)~=2||any(~isfinite(t))||t(2)<=t(1)|| ...
        t(1)<t0||t(2)>t0+T
    D.status='UNKNOWN_INVALID_FAILED_LEAF_INTERVAL';return
end
D.leaf_interval_s=t.';
u0=(t(1)-t0)/T;u1=(t(2)-t0)/T;
if ~(u0>=0&&u0<u1&&u1<=1)
    D.status='UNKNOWN_INVALID_NORMALIZED_LEAF';return
end
[H,C]=TRAJECTORY_C9_BERNSTEIN_SUBINTERVAL_CONTROLS( ...
    double(piece.control_points_m),u0,u1);
if ~isequal(size(C),[24 24])||any(~isfinite(C(:)))|| ...
        any(~isfinite(H(:)))
    D.status='UNKNOWN_INVALID_BERNSTEIN_SUBINTERVAL';return
end
scale=max([1;abs(C(:))]);
if max(abs(sum(C,1)-1))>128*eps(scale)
    D.status='UNKNOWN_SUBINTERVAL_BASIS_PARTITION';return
end
Dt=double(S.dcontrol_points_dT_mps)*C;
if physical_leaf_time
    % Bezier blossom for the restricted controls H_j (j=0..23):
    % dH_j/du0=(23-j)*(H_{j+1}-H_j)/(u1-u0),
    % dH_j/du1=j*(H_j-H_{j-1})/(u1-u0).
    % At FIXED absolute [t_a,t_b], du_i/dT=-u_i/T. This term was absent
    % in the previous fixed-phase tangent. The formula is exact in real
    % arithmetic and does not touch the independent certificate.
    du0=-u0/T;du1=-u1/T;
    span=u1-u0;
    for j=0:23
        if j<23
            Dt(:,j+1)=Dt(:,j+1)+ ...
                ((23-j)*du0/span)*(H(:,j+2)-H(:,j+1));
        end
        if j>0
            Dt(:,j+1)=Dt(:,j+1)+ ...
                (j*du1/span)*(H(:,j+1)-H(:,j));
        end
    end
end
if ~all(isfinite(Dt(:)))
    D.status='UNKNOWN_NONFINITE_FAILED_LEAF_DURATION_DERIVATIVE';return
end
for si=1:numel(shapes)
    shape=shapes(si);
    if ~ismember(double(shape.id),ids),continue;end
    v=mean(H,2)-double(shape.center(:));
    if norm(v)==0,v=H(:,end)-H(:,1);end
    if ~(all(isfinite(v))&&norm(v)>0)
        D.skipped_no_movable_face=D.skipped_no_movable_face+1;
        continue
    end
    n=v/norm(v);
    h=TRAJECTORY_OBSTACLE_SUPPORT.value(shape,n);
    D.support_calls=D.support_calls+1;
    if ~isfinite(h),D.status='UNKNOWN_NONFINITE_SHAPE_SUPPORT';return;end
    deficit=h+payload_radius+clearance-(n.'*H);
    D.maximum_payload_hull_deficit_m=max( ...
        D.maximum_payload_hull_deficit_m,max([0,deficit]));
    [~,order]=sort(deficit,'descend');
    for k=order
        if deficit(k)<=0,break;end
        grad=zeros(13,1);
        for j=1:4
            grad(3*j-2:3*j)=n*C(10+j,k);
        end
        grad(13)=n.'*Dt(:,k);
        if all(isfinite(grad))&&any(grad~=0)
            A(:,end+1)=grad; %#ok<AGROW>
            b(end+1,1)=deficit(k); %#ok<AGROW>
            D.ids(end+1)=double(shape.id); %#ok<AGROW>
            D.selected_control_indices(end+1)=k; %#ok<AGROW>
            D.plane_count=D.plane_count+1;
            break
        end
        D.skipped_no_movable_face=D.skipped_no_movable_face+1;
    end
end
if D.plane_count>0
    D.status='LOCAL_13D_FAILED_LEAF_HULL_CUTS__FULL_ORACLE_STILL_REQUIRED';
elseif strcmp(D.status,'NO_FAILED_LEAF_CUT')
    D.status='NO_MOVABLE_PAYLOAD_HULL_DEFICIT';
end
end
