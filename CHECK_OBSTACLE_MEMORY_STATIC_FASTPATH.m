function R=CHECK_OBSTACLE_MEMORY_STATIC_FASTPATH()
% Synthetic-only equivalence test; NEVER reads sensor or world source.
% Checks static reuse, moving updates, retention, duplicate suppression.
root=fileparts(mfilename('fullpath'));
addpath(genpath(fullfile(root,'reference','REPLANNING_SYSTEM')),'-begin');
empty=struct('id',{},'motion',{},'geometry',{},'position',{});
M=OBSERVED_OBSTACLE_MEMORY(empty);
p=[0;0;0];
s=struct('id',11,'motion','static', ...
    'geometry',struct('type','box','dimensions',[1 2 3]), ...
    'position',[1;2;3]);
m=struct('id',28,'motion','moving', ...
    'geometry',struct('type','sphere','dimensions',1), ...
    'position',[2;1;0]);
F=@(t,obs,ids)struct('time_s',t,'uav_position_ekf_m',p, ...
    'detected_count',numel(ids),'detected_ids',ids, ...
    'detected_obstacles',obs);
A=M.update(F(0,s,11));
assert(isequal(A.newly_acquired_ids,11));
B=M.update(F(.025,s,11));
assert(isempty(B.newly_acquired_ids));
N=M.update(F(.05,empty,[]));
assert(isequal(N.known_obstacle_ids,11));
O=M.update(F(.075,m,28));
assert(isequal(O.known_obstacle_ids,[11 28]));
m2=m;m2.position=[3;1;0];
P=M.update(F(.1,m2,28));
assert(isempty(P.newly_acquired_ids));
K=M.known_frame(F(.1,m2,28));
assert(isequal(K.detected_ids,[11 28]));
assert(isequal(K.detected_obstacles(1).position,s.position));
assert(isequal(K.detected_obstacles(2).position,m2.position));
assert(isequal(K.retained_only_ids,11));
st=M.get_status();
assert(st.memory_version==uint64(2));
blocked=false;
try
    M.release(11);
catch ME
    blocked=strcmp(ME.identifier, ...
        'OBSERVED_OBSTACLE_MEMORY:ReleaseNotCertified');
end
assert(blocked);
R=struct('status','PASS_SYNTHETIC_STATIC_AND_MOVING_MEMORY__NO_FLIGHT', ...
    'known_ids',K.detected_ids,'static_retained',true, ...
    'moving_shape_refreshed',true,'automatic_release_blocked',blocked, ...
    'real_sensor_tested',false,'continuous_time_safe',false);
output_dir=fileparts(mfilename('fullpath'));
save(fullfile(output_dir,'OBSTACLE_MEMORY_FASTPATH_TEST.mat'),'R');
fid=fopen(fullfile(output_dir,'OBSTACLE_MEMORY_FASTPATH_TEST.txt'),'wt');
if fid>=0
    cleanup=onCleanup(@()fclose(fid)); %#ok<NASGU>
    fprintf(fid,'status=%s\nknown_ids=%s\n',R.status,mat2str(R.known_ids));
end
disp(R)
end
