function R=CHECK_TRIGGER_MEMORY_VERSION_REUSE(logger)
% CHECK_TRIGGER_MEMORY_VERSION_REUSE
% Synthetic memory invariant and optional READ-ONLY Flight reuse report.
% No sensor scan, world data access, model execution or Flight modification.
root=fileparts(mfilename('fullpath'));
addpath(genpath(fullfile(root,'reference','REPLANNING_SYSTEM')),'-begin');
empty=struct('id',{},'motion',{},'geometry',{},'position',{});
M=OBSERVED_OBSTACLE_MEMORY(empty);
p=[0;0;0];
F=@(t,obs,ids,dist)struct('time_s',t, ...
    'uav_position_ekf_m',p,'detected_ids',ids, ...
    'detected_count',numel(ids),'detected_obstacles',obs, ...
    'detected_distances_m',dist);
a=struct('id',11,'motion','static', ...
    'geometry',struct('type','box','dimensions',[1;2;3]), ...
    'position',[1;2;3]);
b=a;b.id=28;b.position=[3;4;5];
c=a;c.id=33;c.motion='moving';c.position=[6;7;8];
M.update(F(0,empty,[],[]));
A=M.update(F(0.025,[a b],[11 28],[1.1 2.2]));
assert(isequal(A.newly_acquired_ids,[11 28]));
S=M.update(F(0.05,[a b],[11 28],[1.2 2.3]));
assert(isequal(S.known_obstacle_ids,[11 28]));
assert(isempty(S.newly_acquired_ids));
assert(S.memory_version==uint64(2) && S.geometry_version==uint64(0));
Q=M.known_frame(F(0.05,[a b],[11 28],[1.2 2.3]));
assert(isequal(Q.detected_distances_m,[1.2 2.3]));
assert(Q.memory_version==S.memory_version);
H=M.update(F(0.075,a,11,1.5));
assert(isequal(H.held_outside_current_detection_ids,28));
Q=M.known_frame(F(0.075,a,11,1.5));
assert(isequal(Q.detected_ids,[11 28]));
assert(isequaln(Q.detected_distances_m,[1.5 NaN]));
J=M.update(F(0.1,[a c],[11 33],[1.7 3.1]));
assert(isequal(J.newly_acquired_ids,33));
assert(isequal(J.known_obstacle_ids,[11 28 33]));
c.position=[6.1;7;8];
K=M.update(F(0.125,[a c],[11 33],[1.8 3.0]));
assert(K.memory_version==J.memory_version);
assert(K.geometry_version>J.geometry_version);
W=M.known_frame(F(0.125,[a c],[11 33],[1.8 3.0]));
assert(isequal(W.detected_obstacles(3).position,c.position));
assert(isequaln(W.detected_distances_m,[1.8 NaN 3.0]));
M.update(F(0.15,empty,[],[]));
N=M.get_status();
assert(isequal(N.known_ids,[11 28 33]));
assert(isequal(N.retained_only_ids,[11 28 33]));
assert(~N.automatic_release_enabled);
R=struct('status','PASS_SYNTHETIC_MEMORY_ID_AND_REVISION_ONLY', ...
    'known_ids',N.known_ids, ...
    'memory_version',N.memory_version, ...
    'geometry_version',N.geometry_version, ...
    'real_flight_checked',false,'prediction_count',0, ...
    'sensor_prepared_count',0,'version_fast_reuse_count',0, ...
    'geometry_median_ms',NaN,'trigger_median_ms',NaN, ...
    'continuous_time_safe',false);
if nargin>=1 && ~isempty(logger)
    assert(isobject(logger)&&isprop(logger,'Data')&&isprop(logger,'k'), ...
        'CHECK_TRIGGER_MEMORY_VERSION_REUSE:Logger', ...
        'Pass an existing Sim LOGGER object.');
    D=logger.Data;
    assert(isfield(D,'agent')&&~isempty(D.agent) && ...
        isfield(D.agent(1),'reference')&& ...
        isfield(D.agent(1).reference,'result'), ...
        'CHECK_TRIGGER_MEMORY_VERSION_REUSE:Data', ...
        'Original Sim LOGGER reference.result is unavailable.');
    C=D.agent(1).reference.result;
    if ~iscell(C), C=num2cell(C); end
    n=min(numel(C),double(logger.k));
    good=0;from_sensor=0;fast=0;
    geo=nan(n,1);tr=nan(n,1);
    for j=1:n
        q=C{j};
        if ~isstruct(q)||~isscalar(q)|| ...
                ~isfield(q,'future_collision_trigger'),continue;end
        a=q.future_collision_trigger;
        if ~isstruct(a)||~isscalar(a),continue;end
        if isfield(a,'prediction_complete')&&isequal(a.prediction_complete,true)
            good=good+1;
            if isfield(a,'geometry_prepared_from_observed_sensor') && ...
                    isequal(a.geometry_prepared_from_observed_sensor,true)
                from_sensor=from_sensor+1;
            end
            if isfield(a,'geometry_fast_verified_reuse') && ...
                    isequal(a.geometry_fast_verified_reuse,true)
                fast=fast+1;
            end
        end
        if isfield(a,'geometry_wall_ms')&&isnumeric(a.geometry_wall_ms) && ...
                isscalar(a.geometry_wall_ms)
            geo(j)=a.geometry_wall_ms;
        end
        if isfield(q,'trigger_time_ms')&&isnumeric(q.trigger_time_ms) && ...
                isscalar(q.trigger_time_ms)
            tr(j)=q.trigger_time_ms;
        end
    end
    R.real_flight_checked=true; R.prediction_count=good;
    R.sensor_prepared_count=from_sensor;
    R.version_fast_reuse_count=fast;
    R.geometry_median_ms=median(geo(isfinite(geo)));
    R.trigger_median_ms=median(tr(isfinite(tr)));
    R.status='PASS_SYNTHETIC__FLIGHT_REUSE_TIMING_READ_ONLY_NOT_SAFETY';
end
save(fullfile(root,'TRIGGER_MEMORY_VERSION_REUSE.mat'),'R','-v7');
fid=fopen(fullfile(root,'TRIGGER_MEMORY_VERSION_REUSE.txt'),'wt');
if fid>=0
    cleanup=onCleanup(@()fclose(fid)); %#ok<NASGU>
    fprintf(fid,'status=%s\n',R.status);
    fprintf(fid,'known_ids=%s memory_version=%d geometry_version=%d\n', ...
        mat2str(R.known_ids),R.memory_version,R.geometry_version);
    fprintf(fid,'flight=%d prediction_count=%d prepared_count=%d fast_reuse_count=%d\n', ...
        R.real_flight_checked,R.prediction_count, ...
        R.sensor_prepared_count,R.version_fast_reuse_count);
    fprintf(fid,'geometry_median_ms=%.6f trigger_median_ms=%.6f\n', ...
        R.geometry_median_ms,R.trigger_median_ms);
    fprintf(fid,'continuous_time_safe=0\n');
end
fprintf('[MEMORY/GEOMETRY] %s | predicted=%d | fast reuse=%d\n', ...
    R.status,R.prediction_count,R.version_fast_reuse_count);
end
