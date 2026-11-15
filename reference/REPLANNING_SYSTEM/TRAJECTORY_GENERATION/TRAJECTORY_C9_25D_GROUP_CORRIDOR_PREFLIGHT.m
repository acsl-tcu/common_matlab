function R=TRAJECTORY_C9_25D_GROUP_CORRIDOR_PREFLIGHT()
%TRAJECTORY_C9_25D_GROUP_CORRIDOR_PREFLIGHT Static software-path JIT warmup.
% Synthetic spheres/ellipsoids only, independent of current sensor/Flight.
% Does not look up or reserve obstacle slots, Flight physical parameters,
% geometry, EKF or controller state. No pre-Flight path is planned.
clock=tic;
R=struct('ready',false,'status','UNKNOWN_CORRIDOR_PREFLIGHT', ...
 'group_count',0,'support_evaluations',0,'elapsed_ms',NaN, ...
 'used_real_obstacle_data',false,'safe_to_commit',false, ...
 'stage','NOT_STARTED','error_identifier','','error_message','');
try
 R.stage='BUILD_SYNTHETIC_FIXTURE';
 O(1)=OBSTACLE_INFORMATION.define(9301,'sphere', ...
              [0.25 0 3.15],0.65,'euler_deg',[13 21 8]);
 O(2)=OBSTACLE_INFORMATION.define(9302,'ellipsoid', ...
              [-0.27 0 3.2],[.7 .4 .9],'euler_deg',[32 11 71]);
 shapes=TRAJECTORY_OBSTACLE_SUPPORT.prepare(O);
 S=struct('phase',[.25 .5 .75], ...
   'payload_m',repmat([0;0;3],1,3), ...
   'uav_m',repmat([0;0;5],1,3), ...
   'cable_q',repmat([0;0;-1],1,3), ...
   'payload_velocity_mps',repmat([0;1;0],1,3));
 % Synthetic geometry/physics MUST be defined before q-Jacobian uses L.
 L=2;
 radii=[.3 .01 .2];margin=.001;
 assert(norm(S.uav_m(:,1)-S.payload_m(:,1))==L && ...
        norm(S.cable_q(:,1)-(S.payload_m(:,1)-S.uav_m(:,1))/L)==0, ...
        'C9_V9:FixturePhysics', ...
        'Synthetic fixture must have consistent L and cable direction.');
 J=zeros(3,25,3);J(1,1,:)=1;J(2,2,:)=1;J(3,3,:)=1;
 S.payload_J_fixed_phase=J;
S.uav_J_fixed_phase=J;
% q=(pL-pUAV)/L. The synthetic fixture keeps the cable direction
% constant under all 25 decision variables; its exact derivative is zero.
% Batch local planes require this field, exactly as the real C9 tangent
% generator TRAJECTORY_C9_25D_FULLBODY_TANGENT provides it.
S.cable_q_J_fixed_phase=(S.payload_J_fixed_phase- ...
                            S.uav_J_fixed_phase)/L;
assert(all(S.cable_q_J_fixed_phase(:)==0), ...
   'C9_V9:FixtureCableDerivative','Synthetic cable derivative must be zero.');
 R.stage='BATCH_SUPPORT_PLANES';
 G=TRAJECTORY_C9_25D_BATCH_LOCAL_PLANES(S,shapes,radii,L,margin);
 if ~G.valid
     R.status=G.status;
     if isfield(G,'error_message'),R.error_message=G.error_message;end
     R.elapsed_ms=1000*toc(clock);return
 end
 R.stage='OBSERVED_NEIGHBOR_GRAPH';
 A=TRAJECTORY_C9_25D_OBSERVED_NEIGHBOR_GRAPH(shapes,radii,margin);
 if ~A.valid,R.status=A.status;R.elapsed_ms=1000*toc(clock);return;end
 R.stage='COMMON_CORRIDOR_CUTS';
 C=TRAJECTORY_C9_25D_GROUP_CORRIDOR_CUTS(S,shapes,G,radii,L, ...
          margin,S.phase,[0;0;-9.80665], ...
          struct('phases',[],'ids',[],'body_mask',false(0,3)),A.adjacency);
 if ~C.valid||C.group_count<1
     R.status=['UNKNOWN_CORRIDOR_PREFLIGHT_GROUP_' C.status];
     R.elapsed_ms=1000*toc(clock);return
 end
 % Regression for the actual V9 scene-26 error: the QP may have no
 % extra witness bank at this outer iteration and passes [].  It must
 % have the identical group cut result as a canonical 0x3 empty mask.
 R.stage='EMPTY_WITNESS_BANK_CONTRACT';
 Cempty=TRAJECTORY_C9_25D_GROUP_CORRIDOR_CUTS( ...
          S,shapes,G,radii,L,margin,S.phase,[0;0;-9.80665], ...
          struct('phases',[],'ids',[],'body_mask',[]),A.adjacency);
 if ~(Cempty.valid&&isequal(Cempty.A,C.A)&& ...
      isequal(Cempty.b,C.b)&& ...
      isequal(Cempty.suppress,C.suppress))
     R.status=['UNKNOWN_CORRIDOR_PREFLIGHT_EMPTY_BANK_' Cempty.status];
     R.elapsed_ms=1000*toc(clock);return
 end
 R.stage='READY';
 R.ready=true;
 R.group_count=C.group_count;
 R.support_evaluations=C.support_evaluations;
 R.status='READY_GROUP_CORRIDOR_JIT_WARMED__NO_FLIGHT_PROOF';
catch ME
 R.status=['UNKNOWN_CORRIDOR_PREFLIGHT_EXCEPTION_' ME.identifier];
 R.error_identifier=ME.identifier;
 R.error_message=ME.message;
end
R.elapsed_ms=1000*toc(clock);
end
