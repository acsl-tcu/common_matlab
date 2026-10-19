function AF7R58ZZY_R5_TWO_TARGET_FINAL_SPLIT
% AF7R58ZZY-R5 | TWO-TARGET FINAL SPLIT
% -------------------------------------------------------------------------
% Diagnostic-only one-run audit for the exact R4 authority source.
% Production REPLANNING_SYSTEM.m is NEVER modified.
%
% Required beside/on MATLAB path:
%   REPLANNING_SYSTEM.m
%   REPLANNING_SYSTEM_R5_DIAG.m   (provided with this file)
%   run_flight_headless.m         (same runner used by R4)
%
% Captured windows only:
%   Flight entry : 4.95--5.20 s
%   Replan       : 9.80--10.10 s
%
% Main questions closed by this run:
%   1) Flight-entry spike = observer vs live HLC binding vs dependency watcher?
%   2) Replan spike = sensor/reference/trigger vs staged pipeline?
%   3) Stage 2 = reference certificate vs resource/batch preparation vs
%      post-commit full repaired-reference cache construction?
%
% Timing is empirical diagnostic evidence, NOT a formal future WCET theorem.

clc;
fprintf('\n%s\n','====================================================================================================');
fprintf(' AF7R58ZZY-R5 | TWO-TARGET FINAL SPLIT\n');
fprintf('%s\n','====================================================================================================');
fprintf('[AUTHORITY] productionModified=0 | diagnosticClone=1 | oneFlightRun=1 | empiricalTimingOnly=1\n');
fprintf('[WINDOWS] FlightEntry=4.95--5.20 s | Replan=9.80--10.10 s | unrelatedSpikesIgnored=1\n');

EXPECTED_SOURCE_SHA = '310cfdd01f9554a80b71e8d90464816a721859709ad7c4b9137cdc0f3a6eaf49';
EXPECTED_RUNNER_SHA = '91fa19d72b13e6909d8920659422e3d0cfe169dc9c106b8c78f6d984c60cf507';

columns = { ...
    't_s', ...                         %  1
    'do_total_ms', ...                 %  2
    'loadmass_ms', ...                 %  3
    'observer_ms', ...                 %  4
    'live_binding_ms', ...             %  5
    'dependency_ms', ...               %  6
    'nominal_xd_ms', ...               %  7
    'cert_guard_ms', ...               %  8
    'active_ref_ms', ...               %  9
    'geometry_ms', ...                  % 10
    'reference_packet_ms', ...         % 11
    'raw_sensor_ms', ...                % 12
    'sensor_memory_ms', ...             % 13
    'trigger_dispatch_ms', ...         % 14
    'mex_kernel_ms', ...                % 15
    'trigger_decode_ms', ...            % 16
    'request_block_ms', ...             % 17
    'stage1_total_ms', ...              % 18
    'stage1_future_ref_ms', ...         % 19
    'stage1_context_ms', ...            % 20
    'stage1_proposal_ms', ...           % 21
    'stage1_other_ms', ...              % 22
    'advance_pipeline_ms', ...          % 23
    'pipeline_freshness_ms', ...        % 24
    'stage2_refcert_ms', ...            % 25
    'stage2_prepare_total_ms', ...       % 26
    'stage2_pregates_ms', ...           % 27
    'stage2_batch_model_ms', ...        % 28
    'stage2_sweep_ms', ...              % 29
    'stage2_packet_ms', ...             % 30
    'stage2_sensor_cert_ms', ...        % 31
    'stage2_handles_meta_ms', ...       % 32
    'stage2_postcache_batch_ms', ...    % 33
    'stage2_postcache_derive_ms', ...   % 34
    'stage2_bundle_store_ms', ...       % 35
    'stage2_total_ms', ...              % 36
    'stage3_total_ms', ...              % 37
    'stage3_precheck_ms', ...           % 38
    'stage3_snapshot_ms', ...           % 39
    'stage3_swap_guard_ms', ...         % 40
    'stage3_commit_tail_ms', ...        % 41
    'output_state_ms', ...              % 42
    'reserved', ...                     % 43
    'request_delta', ...                % 44
    'commit_delta', ...                 % 45
    'need_replan', ...                  % 46
    'known_obstacle_count', ...         % 47
    'reference_version'};               % 48

srcPath=which('REPLANNING_SYSTEM');
diagPath=which('REPLANNING_SYSTEM_R5_DIAG');
runnerPath=which('run_flight_headless');
assert(~isempty(srcPath)&&isfile(srcPath),'[AF7R58ZZY-R5] REPLANNING_SYSTEM.m not found on path.');
assert(~isempty(diagPath)&&isfile(diagPath),'[AF7R58ZZY-R5] REPLANNING_SYSTEM_R5_DIAG.m not found on path.');
assert(~isempty(runnerPath)&&isfile(runnerPath),'[AF7R58ZZY-R5] run_flight_headless.m not found on path.');

sourceSHA=sha256_file_r5(srcPath);
runnerSHA=sha256_file_r5(runnerPath);
diagSHA=sha256_file_r5(diagPath);

fprintf('\n[A. EXACT BINDING]\n');
fprintf(' production source = %s\n',srcPath);
fprintf(' source SHA        = %s\n',sourceSHA);
fprintf(' R4 source exact   = %d\n',strcmpi(sourceSHA,EXPECTED_SOURCE_SHA));
fprintf(' runner             = %s\n',runnerPath);
fprintf(' runner SHA         = %s\n',runnerSHA);
fprintf(' R4 runner exact    = %d\n',strcmpi(runnerSHA,EXPECTED_RUNNER_SHA));
fprintf(' diagnostic class   = %s\n',diagPath);
fprintf(' diagnostic SHA     = %s\n',diagSHA);

assert(strcmpi(sourceSHA,EXPECTED_SOURCE_SHA), ...
    '[AF7R58ZZY-R5] production source SHA differs from R4 authority.');
assert(strcmpi(runnerSHA,EXPECTED_RUNNER_SHA), ...
    '[AF7R58ZZY-R5] run_flight_headless SHA differs from R4 runner.');

Dtxt=fileread(diagPath);
assert(contains(Dtxt,['Authority source SHA-256: ' EXPECTED_SOURCE_SHA]), ...
    '[AF7R58ZZY-R5] diagnostic class is not bound to the exact R4 source.');
assert(contains(Dtxt,'classdef REPLANNING_SYSTEM_R5_DIAG < handle'), ...
    '[AF7R58ZZY-R5] diagnostic class name mismatch.');

root=fileparts(srcPath);
outDir=tempname(root);
mkdir(outDir);
reportPath=fullfile(outDir,'AF7R58ZZY_R5_REPORT.txt');
outPath=fullfile(outDir,'AF7R58ZZY_R5_OUT.mat');
rawPath=fullfile(outDir,'AF7R58ZZY_R5_RAW_CAPTURE.mat');
tempRunnerPath=fullfile(root,'AF7R58ZZY_R5_TEMP_RUNNER.m');

% Build a temporary runner clone. Only the replanner class constructor name
% changes; the R4 scenario/ordering remains byte-for-byte otherwise.
runnerText=fileread(runnerPath);
oldCtor='replanner_instance = REPLANNING_SYSTEM(agent,nominal_ref,replan_opts);';
newCtor='replanner_instance = REPLANNING_SYSTEM_R5_DIAG(agent,nominal_ref,replan_opts);';
assert(count_token_r5(runnerText,oldCtor)==1, ...
    '[AF7R58ZZY-R5] runner constructor anchor count is not one.');
runnerDiag=strrep(runnerText,oldCtor,newCtor);
write_utf8_r5(tempRunnerPath,runnerDiag);
cleanupRunner=onCleanup(@()delete_if_exists_r5(tempRunnerPath)); %#ok<NASGU>

RAW=struct();
OUT=struct();
runError=struct('identifier','','message','');
ring=nan(0,48);
runtime_profile=struct();
staged_status=struct();

fprintf('\n[B. RUN]\n');
fprintf(' production source writes = 0\n');
fprintf(' temporary runner         = %s\n',tempRunnerPath);
fprintf(' starting exact R4 flight scenario with diagnostic clone...\n');

try
    % Runner starts with clearvars; execute it in BASE so this audit function's
    % cleanup/output state cannot be cleared by the runner.
    evalin('base','clear REPLANNING_SYSTEM_R5_DIAG');
    rehash;
    cmd=sprintf('run(''%s'')',strrep(tempRunnerPath,'''',''''''));
    evalin('base',cmd);

    obj=evalin('base','replanner_instance');
    nr=double(obj.af7r58zzy_r5_count);
    assert(nr>=1 && nr<=double(obj.af7r58zzy_r5_capacity), ...
        '[AF7R58ZZY-R5] invalid diagnostic ring count.');
    ring=double(obj.af7r58zzy_r5_ring(1:nr,:));
    staged_status=obj.get_staged_pipeline_status();

    if evalin('base','exist(''runtime_profile'',''var'')==1')
        runtime_profile=evalin('base','runtime_profile');
    end
catch ME
    runError.identifier=ME.identifier;
    runError.message=ME.message;
    fprintf(2,'[AF7R58ZZY-R5 RUN ERROR] %s | %s\n',ME.identifier,ME.message);
    try
        if evalin('base','exist(''replanner_instance'',''var'')==1')
            obj=evalin('base','replanner_instance');
            nr=double(obj.af7r58zzy_r5_count);
            if nr>=1
                ring=double(obj.af7r58zzy_r5_ring(1:nr,:));
                staged_status=obj.get_staged_pipeline_status();
            end
        end
        if evalin('base','exist(''runtime_profile'',''var'')==1')
            runtime_profile=evalin('base','runtime_profile');
        end
    catch
    end
end

% External reference time from the original R4 runner, when available.
extRefMs=nan(size(ring,1),1);
if isstruct(runtime_profile) && isfield(runtime_profile,'t') && ...
        isfield(runtime_profile,'reference_s')
    tr=double(runtime_profile.t(:));
    rr=1e3*double(runtime_profile.reference_s(:));
    for i=1:size(ring,1)
        [dmin,j]=min(abs(tr-ring(i,1)));
        if ~isempty(j) && isfinite(dmin) && dmin<=1e-8
            extRefMs(i)=rr(j);
        end
    end
end

% Sort by time for deterministic output.
if ~isempty(ring)
    [~,ord]=sort(ring(:,1));
    ring=ring(ord,:);
    extRefMs=extRefMs(ord);
end

RAW.schema='AF7R58ZZY_R5_RAW_V1';
RAW.source_sha=sourceSHA;
RAW.runner_sha=runnerSHA;
RAW.diagnostic_sha=diagSHA;
RAW.columns=columns;
RAW.ring=ring;
RAW.external_reference_ms=extRefMs;
RAW.runtime_profile=runtime_profile;
RAW.staged_pipeline_status=staged_status;
RAW.run_error=runError;
save(rawPath,'RAW','-v7.3');

% ----------------------------- Analysis ----------------------------------
flightMask=~isempty(ring) & ring(:,1)>=4.95-1e-9 & ring(:,1)<=5.20+1e-9;
replanMask=~isempty(ring) & ring(:,1)>=9.80-1e-9 & ring(:,1)<=10.10+1e-9;
flightRows=ring(flightMask,:);
flightExt=extRefMs(flightMask);
replanRows=ring(replanMask,:);
replanExt=extRefMs(replanMask);

flightStageCols=[4 5 6 8 9 10 11 12 13 14 17 23 42];
flightStageNames={ ...
    'OBSERVER','LIVE_BINDING','DEPENDENCY','CERT_GUARD','ACTIVE_REF', ...
    'GEOMETRY','REFERENCE_PACKET','RAW_SENSOR','SENSOR_MEMORY', ...
    'TRIGGER_DISPATCH','REQUEST_BLOCK','PIPELINE_ADVANCE','OUTPUT_STATE'};
[flightDomName,flightDomMs,flightDomT]=dominant_stage_r5(flightRows,flightStageCols,flightStageNames);

stage1Rows=replanRows(isfinite(replanRows(:,18)),:);
stage2Rows=replanRows(isfinite(replanRows(:,36)),:);
stage3Rows=replanRows(isfinite(replanRows(:,37)),:);

stage2SubCols=[25 27 28 29 30 31 32 33 34 35];
stage2SubNames={ ...
    'REFERENCE_CERT','PREGATES','RESOURCE_BATCH_MODEL','SWEEP', ...
    'OBSTACLE_PACKET','SENSOR_CERT','HANDLES_META', ...
    'POSTCACHE_BATCH','POSTCACHE_DERIVE','BUNDLE_STORE'};
[stage2DomName,stage2DomMs,stage2DomT]=dominant_stage_r5(stage2Rows,stage2SubCols,stage2SubNames);

flightAction=next_action_flight_r5(flightDomName);
stage2Action=next_action_stage2_r5(stage2DomName);

OUT.schema='AF7R58ZZY_R5_OUT_V1';
OUT.status='TWO_TARGET_FINAL_SPLIT_R5_COMPLETE';
if ~isempty(runError.identifier)
    OUT.status='TWO_TARGET_FINAL_SPLIT_R5_RUN_ERROR';
end
OUT.production_modified=false;
OUT.source_sha=sourceSHA;
OUT.runner_sha=runnerSHA;
OUT.diagnostic_sha=diagSHA;
OUT.raw_capture_path=rawPath;
OUT.report_path=reportPath;
OUT.flight_entry=struct('dominant',flightDomName,'dominant_ms',flightDomMs, ...
    'dominant_t',flightDomT,'next_action',flightAction);
OUT.stage2=struct('dominant',stage2DomName,'dominant_ms',stage2DomMs, ...
    'dominant_t',stage2DomT,'next_action',stage2Action);
OUT.stage1_rows=stage1Rows;
OUT.stage2_rows=stage2Rows;
OUT.stage3_rows=stage3Rows;
OUT.staged_pipeline_status=staged_status;
OUT.run_error=runError;
save(outPath,'OUT','-v7.3');

% ------------------------------ Report -----------------------------------
fid=fopen(reportPath,'w');
assert(fid>=0,'[AF7R58ZZY-R5] failed to open report file.');
reportCleanup=onCleanup(@()safe_fclose_r5(fid)); %#ok<NASGU>

fprintf(fid,'status=%s\n',OUT.status);
fprintf(fid,'productionModified=0\n');
fprintf(fid,'sourceSHA=%s\n',sourceSHA);
fprintf(fid,'runnerSHA=%s\n',runnerSHA);
fprintf(fid,'diagnosticSHA=%s\n',diagSHA);
fprintf(fid,'raw=%s\n',rawPath);
fprintf(fid,'out=%s\n\n',outPath);

fprintf(fid,'[FLIGHT_ENTRY_FINAL_SPLIT]\n');
fprintf(fid,'dominant=%s\n',flightDomName);
fprintf(fid,'dominantMs=%.9g\n',flightDomMs);
fprintf(fid,'dominantT=%.12g\n',flightDomT);
fprintf(fid,'nextAction=%s\n',flightAction);
fprintf(fid,['columns=t extRef do observer liveBind dependency certGuard activeRef geometry ' ...
    'refPacket rawSensor sensorMemory trigger request advance output req commit need known ver\n']);
for i=1:size(flightRows,1)
    r=flightRows(i,:);
    fprintf(fid,['row= %.12g %.9g %.9g %.9g %.9g %.9g %.9g %.9g %.9g %.9g ' ...
        '%.9g %.9g %.9g %.9g %.9g %.9g %.9g %.0f %.0f %.0f %.0f %.0f\n'], ...
        r(1),flightExt(i),r(2),r(4),r(5),r(6),r(8),r(9),r(10),r(11), ...
        r(12),r(13),r(14),r(17),r(23),r(42),r(3),r(44),r(45),r(46),r(47),r(48));
end

fprintf(fid,'\n[REPLAN_PIPELINE_FINAL_SPLIT]\n');
fprintf(fid,'stage2Dominant=%s\n',stage2DomName);
fprintf(fid,'stage2DominantMs=%.9g\n',stage2DomMs);
fprintf(fid,'stage2DominantT=%.12g\n',stage2DomT);
fprintf(fid,'nextAction=%s\n',stage2Action);
fprintf(fid,['columns=t extRef do refPacket rawSensor sensorMemory trigger mex decode request ' ...
    'stage1 stage2 stage3 advance req commit need known ver\n']);
for i=1:size(replanRows,1)
    r=replanRows(i,:);
    fprintf(fid,['row= %.12g %.9g %.9g %.9g %.9g %.9g %.9g %.9g %.9g %.9g ' ...
        '%.9g %.9g %.9g %.9g %.0f %.0f %.0f %.0f %.0f\n'], ...
        r(1),replanExt(i),r(2),r(11),r(12),r(13),r(14),r(15),r(16),r(17), ...
        r(18),r(36),r(37),r(23),r(44),r(45),r(46),r(47),r(48));
end

fprintf(fid,'\n[STAGE1_DETAIL]\n');
fprintf(fid,'columns=t total futureRef context proposal other\n');
for i=1:size(stage1Rows,1)
    r=stage1Rows(i,:);
    fprintf(fid,'row= %.12g %.9g %.9g %.9g %.9g %.9g\n', ...
        r(1),r(18),r(19),r(20),r(21),r(22));
end

fprintf(fid,'\n[STAGE2_DETAIL]\n');
fprintf(fid,['columns=t total refCert prepareTotal pregates batchModel sweep packet sensorCert ' ...
    'handlesMeta postcacheBatch postcacheDerive bundleStore freshness\n']);
for i=1:size(stage2Rows,1)
    r=stage2Rows(i,:);
    fprintf(fid,['row= %.12g %.9g %.9g %.9g %.9g %.9g %.9g %.9g %.9g %.9g ' ...
        '%.9g %.9g %.9g %.9g\n'], ...
        r(1),r(36),r(25),r(26),r(27),r(28),r(29),r(30),r(31),r(32), ...
        r(33),r(34),r(35),r(24));
end

fprintf(fid,'\n[STAGE3_DETAIL]\n');
fprintf(fid,'columns=t total precheck snapshot swapGuard commitTail\n');
for i=1:size(stage3Rows,1)
    r=stage3Rows(i,:);
    fprintf(fid,'row= %.12g %.9g %.9g %.9g %.9g %.9g\n', ...
        r(1),r(37),r(38),r(39),r(40),r(41));
end

fprintf(fid,'\n[INTERPRETATION_CONTRACT]\n');
fprintf(fid,'timingIsEmpirical=1\n');
fprintf(fid,'formalFutureWCETClaim=0\n');
fprintf(fid,'trajectoryMathChanged=0\n');
fprintf(fid,'certificateAcceptanceChanged=0\n');
fprintf(fid,'sensorGeometryChanged=0\n');
fprintf(fid,'commitPolicyChanged=0\n');
fprintf(fid,'productionSourceModified=0\n');
fprintf(fid,'diagnosticTicTocOverheadPresent=1\n');
if ~isempty(runError.identifier)
    fprintf(fid,'errorIdentifier=%s\n',runError.identifier);
    fprintf(fid,'errorMessage=%s\n',runError.message);
end

safe_fclose_r5(fid);
clear reportCleanup;

fprintf('\n[C. RESULT]\n');
fprintf(' Flight-entry dominant = %s | %.6f ms @ %.3f s\n', ...
    flightDomName,flightDomMs,flightDomT);
fprintf(' Stage-2 dominant       = %s | %.6f ms @ %.3f s\n', ...
    stage2DomName,stage2DomMs,stage2DomT);
fprintf(' Flight next action     = %s\n',flightAction);
fprintf(' Stage-2 next action    = %s\n',stage2Action);
fprintf('\nRETURN THESE THREE FILES:\n');
fprintf('  %s\n',reportPath);
fprintf('  %s\n',outPath);
fprintf('  %s\n',rawPath);
fprintf('status = %s\n',OUT.status);

end


function [name,val,t] = dominant_stage_r5(rows,cols,names)
name='NONE'; val=NaN; t=NaN;
if isempty(rows), return; end
best=-Inf;
for i=1:size(rows,1)
    for j=1:numel(cols)
        x=rows(i,cols(j));
        if isfinite(x) && x>best
            best=x; name=names{j}; t=rows(i,1);
        end
    end
end
if isfinite(best), val=best; end
end


function action = next_action_flight_r5(name)
switch upper(char(name))
    case 'OBSERVER'
        action=['OBSERVER_REAL_EVENT_PATH: split measurement/build, one-step support, ' ...
            'two-event contraction and bridge allocation; preserve observer theorem.'];
    case 'LIVE_BINDING'
        action=['LIVE_HLC_BINDING: remove recurring matrix/source work only after an ' ...
            'equivalent mutation/freshness authority is installed; keep fail-closed equality.'];
    case 'DEPENDENCY'
        action=['DEPENDENCY_WATCHER: split takeoff-origin, reference handle, mQ/mL and inertia reads; ' ...
            'cache only proven-static dependencies and retain live invalidation.'];
    otherwise
        action=['FOLLOW_DOMINANT_BUCKET: optimize only the measured bucket; do not weaken ' ...
            'continuous certificate, finite sensing or stale rejection.'];
end
end


function action = next_action_stage2_r5(name)
switch upper(char(name))
    case {'POSTCACHE_BATCH','POSTCACHE_DERIVE'}
        action=['POSTCOMMIT_CACHE: full N-point repaired packet preparation is the target. ' ...
            'Redistribute/accelerate exact cache construction without changing the candidate, ' ...
            'certificate, activation time, or stale/atomic gates.'];
    case 'RESOURCE_BATCH_MODEL'
        action=['RESOURCE_BATCH_MODEL: optimize/carry forward canonical batch-model construction ' ...
            'with exact equivalence; do not replace continuous certificate by sampling.'];
    case 'REFERENCE_CERT'
        action=['REFERENCE_CERTIFICATE: optimize the exact continuous certificate implementation ' ...
            'only; certificate semantics and sampled-acceptance prohibition remain unchanged.'];
    case {'OBSTACLE_PACKET','SENSOR_CERT','SWEEP'}
        action=['TARGET_PACKET_CERT: optimize version-bound target packet/certificate construction ' ...
            'while retaining exact target-version binding and fail-closed freshness.'];
    otherwise
        action=['FOLLOW_STAGE2_DOMINANT: change only implementation placement/caching after the ' ...
            'measured dominant is confirmed; safety mathematics remains unchanged.'];
end
end


function n = count_token_r5(txt,token)
if isempty(token), n=0; return; end
n=numel(strfind(txt,token)); %#ok<STREMP>
end


function h = sha256_file_r5(path)
fid=fopen(path,'rb');
assert(fid>=0,'[AF7R58ZZY-R5] cannot open file for SHA: %s',path);
c=onCleanup(@()fclose(fid)); %#ok<NASGU>
b=fread(fid,Inf,'*uint8');
md=javaMethod('getInstance','java.security.MessageDigest','SHA-256');
md.update(typecast(b,'int8'));
d=md.digest();
u=mod(double(d),256);
h=lower(reshape(dec2hex(u,2).',1,[]));
end


function write_utf8_r5(path,txt)
fid=fopen(path,'w','n','UTF-8');
assert(fid>=0,'[AF7R58ZZY-R5] cannot create temporary runner.');
c=onCleanup(@()fclose(fid)); %#ok<NASGU>
fprintf(fid,'%s',txt);
end


function delete_if_exists_r5(path)
try
    if isfile(path), delete(path); end
catch
end
end


function safe_fclose_r5(fid)
try
    if fid>0, fclose(fid); end
catch
end
end
