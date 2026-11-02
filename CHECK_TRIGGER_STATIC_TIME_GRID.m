function T=CHECK_TRIGGER_STATIC_TIME_GRID()
% Real, Flight-independent dt/horizon preparation test only.
% No estimator, sensor, controller, plant, obstacle or reference is accessed.
q=FUTURE_COLLISION_TRIGGER(struct('horizon_s',4));
q.prepare_static_timing(0.025,4);
a=q.static_preparation_status();
assert(a.grid_points==161 && a.grid_build_count==1, ...
    'CHECK_TRIGGER_STATIC_TIME_GRID:Initial','4s/0.025s grid did not match.');
q.prepare_static_timing(0.025,4);
b=q.static_preparation_status();
assert(b.grid_build_count==1 && b.grid_points==a.grid_points, ...
    'CHECK_TRIGGER_STATIC_TIME_GRID:Reuse','Same configuration was rebuilt.');
q.prepare_static_timing(0.03,4);
c=q.static_preparation_status();
assert(c.grid_build_count==2 && c.grid_points==135, ...
    'CHECK_TRIGGER_STATIC_TIME_GRID:DT','Nonintegral horizon/dt grid is wrong.');
q.set_horizon(3.987);
q.prepare_static_timing(0.03,3.987);
d=q.static_preparation_status();
assert(d.grid_build_count==3 && d.grid_points==134, ...
    'CHECK_TRIGGER_STATIC_TIME_GRID:H','Externally changed horizon did not invalidate.');
T=struct('status','PASS_STATIC_GRID_CONFIGURATION_ONLY__NO_FLIGHT_PROOF', ...
    'tested_configurations',3,'same_config_reused',true, ...
    'external_dt_and_horizon_invalidation',true, ...
    'flight_tested',false,'continuous_safety_certified',false);
folder=fileparts(mfilename('fullpath'));
save(fullfile(folder,'TRIGGER_STATIC_GRID_TEST.mat'),'T');
fid=fopen(fullfile(folder,'TRIGGER_STATIC_GRID_TEST.txt'),'wt');
if fid>=0
    guard=onCleanup(@()fclose(fid)); %#ok<NASGU>
    fprintf(fid,'status=%s\n',T.status);
    fprintf(fid,'grid4s_25ms=%d grid4s_30ms=%d grid3p987s_30ms=%d\n', ...
        a.grid_points,c.grid_points,d.grid_points);
end
end
