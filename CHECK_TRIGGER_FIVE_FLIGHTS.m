function S = CHECK_TRIGGER_FIVE_FLIGHTS()
% CHECK_TRIGGER_FIVE_FLIGHTS  Run 5 UNMODIFIED production simulation flights
% and audit live collision-trigger latency, event semantics and spikes.
%
% Usage in common_matlab/ after manually extracting the ZIP:
%   clear classes; rehash
%   S = CHECK_TRIGGER_FIVE_FLIGHTS(); disp(S.status)
%
% This file only runs the existing run_flight_headless script and inspects
% its Sim LOGGER AFTER each flight. It does not change runner, HLC, EKF,
% plant, sensor, MEX, reference or trajectory. MATLAB is required to run it.
% All results and one shareable ZIP are placed at common_matlab/ (NOT pwd).
%
% PASS is a software/Flight audit; it is NOT a proof of real-plant ODE
% continuous-time clearance, prediction error bounds or hard WCET.
script_dir = fileparts(mfilename('fullpath'));
root = locate_common_matlab_root(script_dir);
base = fullfile(root,'reference','REPLANNING_SYSTEM');
% Resolve the repository root by walking upward from THIS script.
% The prior script erroneously appended reference/REPLANNING_SYSTEM twice
% when the audit .m files were placed under the production directory.
% Both auditor files should be unpacked to common_matlab/; tolerate both
% beside this script under reference/REPLANNING_SYSTEM if already present.
if isfile(fullfile(script_dir,'CHECK_TRIGGER_FLIGHT_FINAL.m'))
    audit_dir = script_dir;
elseif isfile(fullfile(root,'CHECK_TRIGGER_FLIGHT_FINAL.m'))
    audit_dir = root;
else
    error('CHECK_TRIGGER_FIVE_FLIGHTS:MissingAudit', ...
        'CHECK_TRIGGER_FLIGHT_FINAL.m missing. Unpack both CHECK_*.m files at common_matlab/.');
end
old_cwd = pwd;
restore_cwd = onCleanup(@()cd(old_cwd)); %#ok<NASGU>
cd(root);
addpath(fullfile(base,'FUTURE_COLLISION_TRIGGER'),'-begin');
addpath(base,'-begin');
addpath(root,'-begin');
addpath(audit_dir,'-begin');
% Make the tested postflight audit deterministic; do not silently invoke a
% stale CHECK_TRIGGER_FLIGHT_FINAL from a different folder on MATLAB path.
auditor = which('CHECK_TRIGGER_FLIGHT_FINAL');
assert(strcmpi(strrep(auditor,'\','/'), ...
    strrep(fullfile(audit_dir,'CHECK_TRIGGER_FLIGHT_FINAL.m'),'\','/')), ...
    'CHECK_TRIGGER_FIVE_FLIGHTS:StaleAuditor', ...
    'The active CHECK_TRIGGER_FLIGHT_FINAL.m is not from the corrected ZIP.');
assert(isfile(fullfile(root,'run_flight_headless.m')) || ...
    ~isempty(which('run_flight_headless')), ...
    'CHECK_TRIGGER_FIVE_FLIGHTS:MissingRunner', ...
    'The original run_flight_headless must be available on MATLAB path.');
% Refuse to benchmark a stale AABB-based installation. This checks the
% exact production file, not merely whatever happens to be on MATLAB path.
directional_file = fullfile(base,'FUTURE_COLLISION_TRIGGER', ...
    'TRIGGER_NOMINAL_SWEPT_INTERVALS.m');
assert(isfile(directional_file) && ...
    contains(fileread(directional_file),'DIRECTIONAL_CONVEX_HULL_NOMINAL_SWEEP') && ...
    ~contains(fileread(directional_file),'box_lo='), ...
    'CHECK_TRIGGER_FIVE_FLIGHTS:StaleDirectional', ...
    'Install the latest directional collision patch before this benchmark.');
assert(~isempty(which('TRIGGER_MERGE_EARLIEST_NOMINAL_RISK')), ...
    'CHECK_TRIGGER_FIVE_FLIGHTS:StaleTrigger', ...
    'Final nominal swept trigger not found. Extract the last production ZIP.');
% Guard against unintentionally auditing an old trigger class or source.
core = which('FUTURE_COLLISION_TRIGGER');
assert(~isempty(core) && contains(fileread(core), ...
    'TRIGGER_MERGE_EARLIEST_NOMINAL_RISK('), ...
    'CHECK_TRIGGER_FIVE_FLIGHTS:StaleTrigger', ...
    'The current production trigger must include the nominal sweep merge.');
expected = fullfile(root,'reference','REPLANNING_SYSTEM', ...
    'FUTURE_COLLISION_TRIGGER','FUTURE_COLLISION_TRIGGER.m');
assert(strcmpi(strrep(core,'\','/'), ...
               strrep(expected,'\','/')), ...
    'CHECK_TRIGGER_FIVE_FLIGHTS:WrongSource', ...
    'MATLAB resolves FUTURE_COLLISION_TRIGGER from a different location.');

% The share ZIP intentionally stores both audit scripts at common_matlab/.
% If an earlier ZIP was accidentally extracted into a production subfolder,
% copy ONLY these temporary auditors up to the repository root. No changes
% to REPLANNING_SYSTEM, HLC, plant, sensor, runner or native MEX.
if ~strcmpi(script_dir,root)
    audit_names = {'CHECK_TRIGGER_FIVE_FLIGHTS.m', ...
                   'CHECK_TRIGGER_FLIGHT_FINAL.m'};
    for ii=1:numel(audit_names)
        if ii==1
            src = fullfile(script_dir,audit_names{ii});
        else
            src = fullfile(audit_dir,audit_names{ii});
        end
        dst = fullfile(root,audit_names{ii});
        if ~strcmpi(src,dst)
            [ok,msg] = copyfile(src,dst,'f');
            assert(ok,'CHECK_TRIGGER_FIVE_FLIGHTS:AuditRelocate', '%s',msg);
        end
    end
end
% Keep the original runner's code unchanged, audit outside Flight loop.
fprintf('[FIVE-FLIGHT] auditing root: %s\n',root);

N = 5;
per_run = cell(N,1);
error_messages = cell(N,1);
spikes = cell(N,1);
source_file_list = {
    'reference/REPLANNING_SYSTEM/REPLANNING_SYSTEM.m', ...
    'reference/REPLANNING_SYSTEM/FUTURE_COLLISION_TRIGGER/FUTURE_COLLISION_TRIGGER.m', ...
    'reference/REPLANNING_SYSTEM/FUTURE_COLLISION_TRIGGER/TRIGGER_NOMINAL_SWEPT_INTERVALS.m', ...
    'reference/REPLANNING_SYSTEM/FUTURE_COLLISION_TRIGGER/TRIGGER_MERGE_EARLIEST_NOMINAL_RISK.m', ...
    'reference/REPLANNING_SYSTEM/FUTURE_COLLISION_TRIGGER/TRIGGER_EARLIEST_SAMPLED_CONTACT.m', ...
    'reference/REPLANNING_SYSTEM/FUTURE_COLLISION_TRIGGER/TRIGGER_REFERENCE_WINDOW_CACHE.m'};

% One set of original source snapshots; checking these names is not a source
% change, and does not add any work to the Flight loop.
for j = 1:numel(source_file_list)
    assert(isfile(fullfile(root,source_file_list{j})), ...
        'CHECK_TRIGGER_FIVE_FLIGHTS:MissingProductionSource', ...
        'Missing source: %s',source_file_list{j});
end
fprintf('\n[FIVE-FLIGHT] Original runner x 5; logs audited AFTER each flight.\n');
for k = 1:N
    fprintf('\n[FIVE-FLIGHT] Running %d/%d ...\n',k,N);
    try
        cd(root);
        % Prevent a failed flight from accidentally reusing the last logger.
        evalin('base','clear logger');
        evalin('base','run_flight_headless;');
        assert(evalin('base','exist(''logger'',''var'')')==1, ...
            'CHECK_TRIGGER_FIVE_FLIGHTS:MissingLogger', ...
            'Original runner did not leave the Sim LOGGER in base workspace.');
        this_logger = evalin('base','logger');
        A = CHECK_TRIGGER_FLIGHT_FINAL(this_logger,true);
        clear this_logger;
        per_run{k} = A;
        latest_mat = fullfile(root,'TRIGGER_FLIGHT_FINAL_RESULT.mat');
        latest_txt = fullfile(root,'TRIGGER_FLIGHT_FINAL_RESULT.txt');
        assert(isfile(latest_mat) && isfile(latest_txt), ...
            'CHECK_TRIGGER_FIVE_FLIGHTS:MissingAuditData', ...
            'The postflight auditor did not save the expected results.');
        dest_mat = fullfile(root,sprintf('TRIGGER_FLIGHT_RUN_%02d.mat',k));
        dest_txt = fullfile(root,sprintf('TRIGGER_FLIGHT_RUN_%02d.txt',k));
        [ok,why] = copyfile(latest_mat,dest_mat,'f');
        assert(ok,'CHECK_TRIGGER_FIVE_FLIGHTS:Copy', '%s',why);
        [ok,why] = copyfile(latest_txt,dest_txt,'f');
        assert(ok,'CHECK_TRIGGER_FIVE_FLIGHTS:Copy', '%s',why);
        V = load(latest_mat,'T','wall_ms','full_ms','available', ...
                     'early','complete','contact','possible','status', ...
                     'phase_names','phase_ms','prediction_horizon_s', ...
                     'sampling_step_s','n_observed');
        spikes{k} = identify_spikes(V,10);
        fprintf(['[FIVE-FLIGHT] run=%d %s | prediction=%d | ' ...
            'trigger median/P95/max=%.3f/%.3f/%.3f ms | ' ...
            '>4ms=%d | replanner max=%.3f ms\n'], ...
            k,A.status,A.predictions,A.trigger_median_ms,A.trigger_p95_ms, ...
            A.trigger_max_ms,A.trigger_above_4ms_count,A.replanning_max_ms);
    catch ME
        error_messages{k} = sprintf('%s: %s',ME.identifier,ME.message);
        per_run{k} = struct('status','FAIL_FLIGHT_OR_AUDIT', ...
            'failures',1,'messages',{{error_messages{k}}});
        fprintf(2,'[FIVE-FLIGHT] run=%d FAIL: %s\n',k,error_messages{k});
        % Continue the remaining scheduled runs; failed ones can never be
        % counted as a PASS. Do not silently use an earlier log.
    end
end

all_pass = all(cellfun(@(r)isstruct(r) && isfield(r,'status') && ...
    strcmp(r.status,'PASS_REAL_FLIGHT_NOMINAL_SAMPLED_AND_SWEPT_TRIGGER'),per_run));
max_trigger = nan(N,1); p95_trigger = nan(N,1);
max_replanner = nan(N,1); over4_trigger = nan(N,1);
over4_replanner = nan(N,1); early_stops = nan(N,1);
for k = 1:N
    a = per_run{k};
    if isfield(a,'trigger_max_ms')
        max_trigger(k) = a.trigger_max_ms;
        p95_trigger(k) = a.trigger_p95_ms;
        max_replanner(k) = a.replanning_max_ms;
        over4_trigger(k) = a.trigger_above_4ms_count;
        over4_replanner(k) = a.replanning_above_4ms_count;
        early_stops(k) = a.confirmed_early_stops;
    end
end
under4_observed = all_pass && all(max_trigger<=4) && all(max_replanner<=4);
if all_pass
    st = 'PASS_FIVE_REAL_FLIGHT_NOMINAL_TRIGGER_AUDITS';
else
    st = 'FAIL_OR_INCOMPLETE_FIVE_REAL_FLIGHT_AUDITS';
end
S = struct('status',st,'runs',{per_run},'run_errors',{error_messages}, ...
    'spikes',{spikes},'audited_runs',N, ...
    'functional_flight_audits_passed',all_pass, ...
    'max_trigger_ms',max_trigger,'p95_trigger_ms',p95_trigger, ...
    'max_replanner_ms',max_replanner, ...
    'trigger_over_4ms_count',over4_trigger, ...
    'replanner_over_4ms_count',over4_replanner, ...
    'confirmed_early_stops',early_stops, ...
    'observed_all_runs_below_4ms',under4_observed, ...
    'wcet_proven',false, ...
    'true_plant_continuous_time_safety_proven',false);
result_mat = fullfile(root,'TRIGGER_FIVE_FLIGHTS_RESULT.mat');
result_txt = fullfile(root,'TRIGGER_FIVE_FLIGHTS_RESULT.txt');
save(result_mat,'S','-v7');
write_report(result_txt,S);

% Single shareable archive in common_matlab/ containing audit scripts,
% results for every completed run and exactly the installed source files.
files = {'CHECK_TRIGGER_FIVE_FLIGHTS.m','CHECK_TRIGGER_FLIGHT_FINAL.m', ...
         'TRIGGER_FIVE_FLIGHTS_RESULT.mat','TRIGGER_FIVE_FLIGHTS_RESULT.txt'};
for k = 1:N
    for ext = {'.mat','.txt'}
        name=sprintf('TRIGGER_FLIGHT_RUN_%02d%s',k,ext{1});
        if isfile(fullfile(root,name)),files{end+1}=name;end %#ok<AGROW>
    end
end
files = [files source_file_list];
archive = fullfile(root,'TRIGGER_FIVE_FLIGHTS_FOR_CHATGPT.zip');
if isfile(archive),delete(archive);end
zip(archive,files,root);
S.share_zip = archive;
fprintf('\n[FIVE-FLIGHT] %s\n',S.status);
fprintf('[FIVE-FLIGHT] 4ms in ALL observed runs = %d (not a WCET proof).\n', ...
    S.observed_all_runs_below_4ms);
fprintf('[FIVE-FLIGHT] Share this ONE file: %s\n',archive);
end

function P = identify_spikes(V,n_top)
% Analysis is postflight only. Times are the logged Flight clock, not wall.
T = double(V.T(:));
x = double(V.wall_ms(:));
y = double(V.full_ms(:));
valid = find(isfinite(x) & isfinite(T));
[~,idx] = sort(x(valid),'descend');
selected = valid(idx(1:min(n_top,numel(idx))));
P = struct('top_frame_index',selected,'top_time_s',T(selected), ...
    'top_trigger_ms',x(selected),'top_replanning_ms',y(selected), ...
    'phase_names',{V.phase_names}, ...
    'top_phase_ms',V.phase_ms(selected,:), ...
    'horizon_s_at_top',V.prediction_horizon_s(selected), ...
    'sampling_step_s_at_top',V.sampling_step_s(selected), ...
    'observed_obstacle_count_at_top',V.n_observed(selected), ...
    'first_5s_over4ms',sum(x(isfinite(T)&T<=5)>4), ...
    'first_5s_max_ms',safe_max(x(isfinite(T)&T<=5)), ...
    'later_max_ms',safe_max(x(isfinite(T)&T>5)), ...
    'trigger_over_10ms_count',sum(x(valid)>10), ...
    'replanning_over_10ms_count',sum(y(isfinite(y))>10), ...
    'status_at_top',{V.status(selected)});
end

function m = safe_max(a)
a=a(isfinite(a));
if isempty(a),m=NaN;else,m=max(a);end
end

function write_report(filename,S)
fid = fopen(filename,'w');
assert(fid>0,'CHECK_TRIGGER_FIVE_FLIGHTS:Output', ...
    'Cannot save summary in common_matlab/.');
g = onCleanup(@()fclose(fid)); %#ok<NASGU>
fprintf(fid,'%s\n',S.status);
fprintf(fid,'audit_scope=5 real Flight runs; nominal sampled + piecewise-linear sweep\n');
fprintf(fid,'physical_continuous_time_proof=0 WCET_proof=0\n');
fprintf(fid,'observed_all_runs_below_4ms=%d\n',S.observed_all_runs_below_4ms);
fprintf(fid,['run,status,frames,predictions,events,early_stops,' ...
    'trigger_median_ms,trigger_p95_ms,trigger_max_ms,trigger_over_4ms,' ...
    'replanner_median_ms,replanner_p95_ms,replanner_max_ms,' ...
    'replanner_over_4ms,issues\n']);
for k = 1:S.audited_runs
    a = S.runs{k};
    if ~isfield(a,'trigger_max_ms')
        fprintf(fid,'%d,%s,,,,,,,,,,,,,%s\n',k,a.status,S.run_errors{k});
        continue
    end
    fprintf(fid,['%d,%s,%d,%d,%d,%d,%.6f,%.6f,%.6f,%d,' ...
       '%.6f,%.6f,%.6f,%d,%d\n'],k,a.status,a.flight_frames, ...
       a.native_predictions,size(a.new_event_table,1), ...
       a.confirmed_early_stops,a.trigger_median_ms,a.trigger_p95_ms, ...
       a.trigger_max_ms,a.trigger_above_4ms_count, ...
       a.replanning_median_ms,a.replanning_p95_ms,a.replanning_max_ms, ...
       a.replanning_above_4ms_count,a.failures);
end
for k = 1:S.audited_runs
    p = S.spikes{k};
    if isempty(p),continue;end
    fprintf(fid,['\n=== Run %d spike details (first_5s >4ms=%d; ' ...
       'first_5s_max=%.6f; later_max=%.6f; trigger >10ms=%d; ' ...
       'replanner >10ms=%d) ===\n'],k,p.first_5s_over4ms, ...
       p.first_5s_max_ms,p.later_max_ms,p.trigger_over_10ms_count, ...
       p.replanning_over_10ms_count);
    fprintf(fid,['rank,frame,flight_time_s,trigger_ms,replanner_ms,' ...
        'pred_wall_ms,ref_window_ms,hlc_ms,plant_ms,sweep_ms,geom_ms,' ...
        'pre_predictor_ms,post_geometry_ms,cache_ms,horizon_s,dt_s,' ...
        'known_obstacles,status\n']);
    for j = 1:numel(p.top_frame_index)
        z = p.top_phase_ms(j,:);
        fprintf(fid,['%d,%d,%.9g,%.6f,%.6f,' ...
            '%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,' ...
            '%.9g,%.9g,%.0f,%s\n'],j, ...
            p.top_frame_index(j),p.top_time_s(j),p.top_trigger_ms(j), ...
            p.top_replanning_ms(j),z(1),z(2),z(3),z(4),z(5),z(6), ...
            z(7),z(8),z(9),p.horizon_s_at_top(j), ...
            p.sampling_step_s_at_top(j), ...
            p.observed_obstacle_count_at_top(j),p.status_at_top{j});
    end
end
end

function root = locate_common_matlab_root(start_dir)
% Search UPWARDS for the repository layout, independent of cwd or where
% the .m audit was mistakenly extracted. Never create a nested reference/.
root = start_dir;
for depth = 0:8
    base = fullfile(root,'reference','REPLANNING_SYSTEM');
    if isfile(fullfile(base,'REPLANNING_SYSTEM.m')) && ...
            isfile(fullfile(base,'FUTURE_COLLISION_TRIGGER', ...
                                    'FUTURE_COLLISION_TRIGGER.m'))
        return
    end
    parent = fileparts(root);
    if strcmp(parent,root)
        break
    end
    root = parent;
end
error('CHECK_TRIGGER_FIVE_FLIGHTS:RootNotFound', ...
    'Cannot locate common_matlab root above: %s',start_dir);
end
