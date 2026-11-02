function A=CHECK_TRIGGER_OBSERVED_PREPARED_BINDING()
% OFFLINE synthetic regression. Does NOT read the real Flight, or alter
% its sensor, EKF, HLC, reference, obstacle memory or LOGGER.
folder=fileparts(mfilename('fullpath'));
try
    obs=[OBSTACLE_INFORMATION.define(41,'sphere',[0 0 0],1) ...
         OBSTACLE_INFORMATION.define(73,'box',[10 0 0],[2 2 2])];
    S=SIMULATED_OBSTACLE_SENSOR(obs,1,false);
    M=OBSERVED_OBSTACLE_MEMORY(obs([]));
    before=S.observe(0,[-100;0;0]);
    assert(isempty(before.detected_ids));
    refused=false;
    try
        S.observed_prepared_only(41); % no observation, MUST NOT leak
    catch ME
        refused=strcmp(ME.identifier,'SIMULATED_OBSTACLE_SENSOR:UnseenGeometry');
    end
    assert(refused,'Undetected shape was exposed.');
    first=S.observe(0.1,[0;0;0]);
    assert(isequal(first.detected_ids,41));
    P1=S.observed_prepared_only(41);
    assert(numel(P1)==1 && P1.id==41);
    M.update(first);
    second=S.observe(0.2,[10;0;0]);
    assert(isequal(second.detected_ids,73));
    P2=S.observed_prepared_only(73);
    assert(numel(P2)==1 && P2.id==73);
    M.update(second);
    F=M.known_frame(second);
    assert(isequal(F.detected_ids,[41 73]));
    memory_status=M.get_status();
    assert(isequal(memory_status.retained_only_ids,41));
    A=struct('status','PASS_ONLY_ACTUALLY_DETECTED_PREPARED_SHAPES', ...
       'unseen_shape_blocked',refused,'retained_ids',[41 73], ...
       'known_after_first',41,'known_after_second',[41 73], ...
       'real_flight_tested',false,'continuous_time_certified',false);
catch ME
    A=struct('status','FAIL_SENSOR_PREPARED_PROVENANCE', ...
       'reason',[ME.identifier ' | ' ME.message], ...
       'real_flight_tested',false,'continuous_time_certified',false);
end
save(fullfile(folder,'TRIGGER_PREPARED_BINDING_RESULT.mat'),'A','-v7');
fid=fopen(fullfile(folder,'TRIGGER_PREPARED_BINDING_RESULT.txt'),'w');
if fid>=0
    c=onCleanup(@()fclose(fid));
    fprintf(fid,'status=%s\n',A.status);
    if isfield(A,'reason'), fprintf(fid,'reason=%s\n',A.reason);end
    fprintf(fid,'real_flight_tested=0\ncontinuous_time_certified=0\n');
end
end
