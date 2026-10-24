function R = AF7R58R69_EXPORT_LIVE_MODEL_SOURCES()
% Read-only source capture for binding the CURRENT EKF-initialized HLC/plant
% rollout.  Does not call the controller, plant, replanner, or write agent.
% Output files are placed beside this script.  No model parameter defaults.
root = fileparts(mfilename('fullpath'));
names = {'REPLANNING_SYSTEM','HLC_SUSPENDED_LOAD','HLC', ...
    'Controller_HL_Suspended_Load','Controller_HL', ...
    'Model_Quat13','Model_Quat17','Model_Quat14','Model_R18', ...
    'Model_Load','MODEL_CLASS','SimHL','run_flight_headless', ...
    'TIME_VARYING_REFERENCE','af7r34_make_core_packet'};
paths = cell(size(names)); status = cell(size(names));
for k = 1:numel(names)
    p = which(names{k});
    if isempty(p)
        status{k} = 'NOT_RESOLVED';
    elseif endsWith(lower(p), '.m') && isfile(p)
        paths{k} = p;
        status{k} = 'SOURCE_FOUND';
    else
        status{k} = ['NOT_READABLE_M_SOURCE: ' char(p)];
    end
end
R = struct();
R.model_source_names = names;
R.model_source_paths = paths;
R.source_status = status;
R.hlc_source_resolved = ~isempty(paths{2}) || ~isempty(paths{3});
R.plant_source_resolved = any(~cellfun(@isempty, paths(6:10)));
R.report_file = fullfile(root, 'AF7R58R69_SOURCE_INVENTORY.txt');
R.zip_file = fullfile(root, 'AF7R58R69_LIVE_MODEL_SOURCES.zip');
fid = fopen(R.report_file,'wt');
assert(fid >= 0, 'Unable to create source inventory in script directory');
cln = onCleanup(@() fclose(fid));
fprintf(fid, 'R69 current MATLAB execution sources (read-only)\n');
fprintf(fid, 'HLC resolved=%d  model source resolved=%d\n', ...
    R.hlc_source_resolved, R.plant_source_resolved);
for k=1:numel(names)
    fprintf(fid, '%s | %s | %s\n', names{k}, status{k}, paths{k});
end
% Use only classnames / file paths, never invoke or clone real controller.
R.agent_classes = {};
if evalin('base', 'exist(''agent'', ''var'')')
    try
        A = evalin('base','agent');
        props = {'plant','controller','estimator','reference','parameter'};
        for k = 1:numel(props)
            try
                c = class(A.(props{k}));
                R.agent_classes{end+1,1} = [props{k} ': ' c]; %#ok<AGROW>
                fprintf(fid, 'agent.%s class=%s\n',props{k},c);
            catch
                fprintf(fid,'agent.%s class=UNAVAILABLE\n',props{k});
            end
        end
    catch
        fprintf(fid,'agent binding not accessible\n');
    end
else
    fprintf(fid,'agent not in base workspace; run after actual SimHL\n');
end
% ZIP copies inside OS temp only. No new folders under reference directory.
tmp = tempname;
mkdir(tmp);
cleanTmp = onCleanup(@() rmdir(tmp,'s'));
added = {};
for k = 1:numel(names)
    if isempty(paths{k}), continue; end
    target = fullfile(tmp,[names{k} '.m']);
    if copyfile(paths{k}, target)
        added{end+1} = [names{k} '.m']; %#ok<AGROW>
    end
end
if isempty(added)
    fprintf(fid,'ZIP skipped: no readable MATLAB sources\n');
    R.zip_file = '';
else
    zip(R.zip_file, added, tmp);
    fprintf(fid,'ZIP created with %d unmodified source files\n', numel(added));
end
R.source_count = numel(added);
R.production_modified = false;
R.hlc_modified = false;
R.four_second_rollout_certified = false;
R.natural_c7_committed = false;
fprintf('R69 source count: %d\nTXT: %s\nZIP: %s\n', ...
    R.source_count,R.report_file,R.zip_file);
end
