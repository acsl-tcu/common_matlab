function R=CHECK_OBSERVED_OBSTACLE_MEMORY()
% Synthetic logic test only: does NOT validate the original flight sensor.
% Use source from the same Git tree; no model or sensor is changed.
root=fileparts(mfilename('fullpath'));
addpath(genpath(fullfile(root,'reference','REPLANNING_SYSTEM')),'-begin');
empty=struct('id',{},'geometry',{});
M=OBSERVED_OBSTACLE_MEMORY(empty);
p=[0;0;0];
a=struct('id',17,'geometry',struct('type','sphere','dimensions',1));
b=struct('id',28,'geometry',struct('type','sphere','dimensions',1));
fr=@(t,o,ids) struct('time_s',t,'uav_position_ekf_m',p, ...
  'detected_count',numel(ids),'detected_ids',ids, ...
  'detected_obstacles',o);
A=M.update(fr(1,a,17));
D=M.update(fr(1,a,17)); % same physical t=0-like arming/Flight duplicate
assert(isempty(D.newly_acquired_ids));
B=M.update(fr(2,empty,[]));
known=M.known_frame(fr(2,empty,[]));
C=M.update(fr(3,b,28));
st=M.get_status();
assert(isequal(A.known_obstacle_ids,17));
assert(isequal(B.held_outside_current_detection_ids,17));
assert(isequal(known.detected_ids,17)&&isempty(known.currently_detected_ids));
assert(isequal(C.known_obstacle_ids,[17 28]));
assert(isequal(st.retained_only_ids,17));
assert(st.memory_version==uint64(2)&&st.observation_count==3);
blocked=false;
try,M.release(17);catch E,blocked=strcmp(E.identifier, ...
 'OBSERVED_OBSTACLE_MEMORY:ReleaseNotCertified');end
assert(blocked,'Unsafe automatic release was possible.');
R=struct('status','PASS_SYNTHETIC_SENSOR_HISTORY__NOT_FLIGHT_PROOF', ...
 'known_ids',st.known_ids,'retained_ids',st.retained_only_ids, ...
 'memory_version',st.memory_version,'release_prohibited',blocked);
disp(R);
end
