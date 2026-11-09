function R = CHECK_TRAJECTORY_C9_FLIGHT_WITNESS_SAVED(log_mat_file)
%CHECK_TRAJECTORY_C9_FLIGHT_WITNESS_SAVED Offline saved Sim LOGGER analysis.
% Usage, from common_matlab:
%   R = CHECK_TRAJECTORY_C9_FLIGHT_WITNESS_SAVED();
%   R = CHECK_TRAJECTORY_C9_FLIGHT_WITNESS_SAVED(fullfile(...,'.mat'));
%
% Does NOT require a base-workspace variable named logger, a new Flight,
% or modifications to run_flight_headless / HLC / EKF / plant.
% Searches Data/Sim_data/ObstacleSensorFlight_Log*.mat, then requires
% EXACTLY the same Flight tick IDs, timestamps, recorded 28 references,
% four pre-clamp HLC inputs, and estimated payload masses as in the
% previous *real* recorded-HLC audit. A different Flight is never merged.
% Only logged DISCRETE-TIME tracking errors are assessed. No C9 candidate
% is selected, no all-time tracking tube or torque guarantee is claimed.
root = fileparts(mfilename('fullpath'));
assert(isfile(fullfile(root,'run_flight_headless.m')), ...
    'C9_SAVED_WITNESS:Root', ...
    'Place this file in common_matlab, beside run_flight_headless.m.');
addpath(root,'-begin');
addpath(fullfile(root,'reference','REPLANNING_SYSTEM', ...
    'TRAJECTORY_GENERATION'),'-begin');
previous_mat = fullfile(root,'TRAJECTORY_C9_REAL_EKF_HLC_TICK_RESULT.mat');
assert(isfile(previous_mat),'C9_SAVED_WITNESS:PreviousMAT', ...
    ['Previous HLC audit MAT is missing. Execute ', ...
     'CHECK_TRAJECTORY_C9_REAL_EKF_HLC_TICK on the original log first.']);
S = load(previous_mat,'R');
assert(isfield(S,'R') && isstruct(S.R) && ...
    isfield(S.R,'saved_flight_trace'), ...
    'C9_SAVED_WITNESS:PreviousTrace', ...
    'Previous HLC audit MAT does not contain the raw Flight summary.');
previous = S.R;
trace = previous.saved_flight_trace;
assert(isstruct(trace) && isscalar(trace) && ...
    all(isfield(trace,{'logger_index','time_s','actual_ref28', ...
                       'unclamped_input4','estimated_payload_mass_kg'})) && ...
    numel(trace.logger_index) > 0, ...
    'C9_SAVED_WITNESS:Trace', ...
    'Expected actual Flight trace is missing or empty.');

if nargin>=1 && ~isempty(log_mat_file)
    assert((ischar(log_mat_file)||isstring(log_mat_file)) && ...
        isscalar(string(log_mat_file)), ...
        'C9_SAVED_WITNESS:Path', 'log_mat_file must be a file path.');
    raw_file = char(log_mat_file);
    assert(isfile(raw_file),'C9_SAVED_WITNESS:MissingLog', ...
        'The specified raw Sim LOGGER MAT does not exist: %s',raw_file);
    files = dir(raw_file);
else
    files = dir(fullfile(root,'Data','Sim_data', ...
        'ObstacleSensorFlight_Log*.mat'));
    assert(~isempty(files),'C9_SAVED_WITNESS:MissingLog', ...
        ['No saved raw Sim LOGGER was found under Data/Sim_data. ', ...
         'A previous R summary ZIP does not contain payload tracking states. ', ...
         'Find an ObstacleSensorFlight_Log(...).mat file or run the ', ...
         'usual run_flight_headless once.']);
    [~,order] = sort([files.datenum],'descend');
    files = files(order);
end

matched_data = [];
matched_file = '';
rejections = cell(numel(files),1);
for i=1:numel(files)
    if isfield(files,'folder') && ~isempty(files(i).folder)
        path = fullfile(files(i).folder,files(i).name);
    else
        path = files(i).name;
    end
    try
        A = load(path,'log');
        if ~isfield(A,'log') || ~isstruct(A.log) || ...
                ~isfield(A.log,'Data')
            rejections{i}='No log.Data field';
            continue
        end
        [same,why] = c9_matches_real_recorded_trace(A.log.Data,trace);
        if ~same
            rejections{i}=why;
            continue
        end
        matched_data=A.log.Data;
        matched_file=path;
        break
    catch ME
        rejections{i}=[ME.identifier ': ' ME.message];
    end
end
if isempty(matched_file)
    disp('[C9 SAVED LOGGER] No raw Flight logfile matches the previous HLC audit.');
    for i=1:numel(files)
        fprintf('  %s: %s\n',files(i).name,rejections{i});
    end
    error('C9_SAVED_WITNESS:DifferentFlight', ...
        ['The selected/saved raw logger is not the SAME Flight as the ', ...
         'previous 28-reference/4-input trace. Do not combine different ', ...
         'Flights. Re-audit the selected Flight with ', ...
         'CHECK_TRAJECTORY_C9_REAL_EKF_HLC_TICK, then run this check.']);
end

% Existing check reads the original Data struct; no LOGGER object required.
R = CHECK_TRAJECTORY_C9_FLIGHT_WITNESS(previous,matched_data);
assert(R.raw_flight_logger_supplied && ...
    ~isempty(fieldnames(R.sampled_payload_tracking)), ...
    'C9_SAVED_WITNESS:NoTracking', ...
    'Saved raw data was supplied but produced no tracking witness.');
R.raw_flight_log_path = matched_file;
R.raw_flight_matched_archived_trace = true;
R.raw_flight_log_source = 'ORIGINAL_SAVED_SIM_LOGGER__OFFLINE';
R.hlc_tracking_continuous_time_certified = false;
R.safe_to_commit = false;

% Update the original result so the same single ZIP includes provenance.
base='TRAJECTORY_C9_FLIGHT_WITNESS_RESULT';
matfile=fullfile(root,[base '.mat']);
txtfile=fullfile(root,[base '.txt']);
zipfile=fullfile(root,[base '_FOR_CHATGPT.zip']);
save(matfile,'R');
fid=fopen(txtfile,'at');
assert(fid>0,'C9_SAVED_WITNESS:Write','Could not append raw log provenance.');
closer=onCleanup(@() fclose(fid));
fprintf(fid,'raw_flight_log_path=%s\n',matched_file);
fprintf(fid,'raw_flight_matched_archived_trace=1\n');
fprintf(fid,'raw_flight_log_source=ORIGINAL_SAVED_SIM_LOGGER__OFFLINE\n');
clear closer
if isfile(zipfile), delete(zipfile); end
zip(zipfile,{[base '.mat'],[base '.txt']},root);
zinfo=dir(zipfile);
assert(isscalar(zinfo) && zinfo.bytes>100, ...
    'C9_SAVED_WITNESS:ZIP','Output ZIP was not created.');
fprintf('[C9 SAVED LOGGER] matched raw Flight: %s\n',matched_file);
fprintf('[C9 SAVED LOGGER] %s\n',R.sampled_payload_tracking.status);
fprintf('[C9 SAVED LOGGER] result ZIP: %s\n',zipfile);
end

function [pass,reason] = c9_matches_real_recorded_trace(D,T)
pass=false;
reason='Malformed or unmatched saved Flight';
if ~isstruct(D) || ~isscalar(D) || ...
        ~all(isfield(D,{'t','phase','agent'})) || ...
        isempty(D.agent) || ~isfield(D.agent(1),'reference') || ...
        ~isfield(D.agent(1),'controller') || ...
        ~isfield(D.agent(1),'estimator')
    reason='Missing Sim LOGGER Data members';return
end
ref=D.agent(1).reference.result;
ctl=D.agent(1).controller.result;
est=D.agent(1).estimator.result;
if ~iscell(ref) || ~iscell(ctl) || ~iscell(est)
    reason='Reference/controller/estimator histories must be cells';return
end
n=numel(ctl);
if numel(ref)<n || numel(est)<n || numel(D.phase)<n || numel(D.t)<n
    reason='Incomplete Sim LOGGER';return
end
phase=D.phase(:);
if isnumeric(phase)
    ids=find(phase(1:n)==double('f'));
elseif ischar(phase)
    ids=find(phase(1:n)=='f');
elseif isstring(phase) || iscellstr(phase)
    ids=find(strcmp(string(phase(1:n)),"f"));
else
    reason='Unsupported phase encoding';return
end
want=double(T.logger_index(:));
if ~isequal(double(ids(:)),want)
    reason='Flight sample indices differ';return
end
m=numel(ids);
if ~isequal(size(T.actual_ref28),[28 m]) || ...
        ~isequal(size(T.unclamped_input4),[4 m]) || ...
        numel(T.estimated_payload_mass_kg)~=m
    reason='Archived reference, input or mass vector has wrong size';return
end
a=double(D.t(ids));b=double(T.time_s(:));
if numel(a)~=m || numel(b)~=m || ...
        any(~isfinite(a)) || any(~isfinite(b)) || ...
        any(abs(a-b)>1e-9.*max(1,abs(b)))
    reason='Flight timestamps differ';return
end
for j=1:m
    k=ids(j);
    try
        r=ref{k};c=ctl{k};e=est{k};
        if ~isstruct(r) || ~isfield(r,'state') || ...
                ~isstruct(c) || ~isfield(c,'tmp') || ...
                ~isstruct(e) || ~isfield(e,'state') || ...
                ~isstruct(e.state) || ~isfield(e.state,'mL')
            reason=sprintf('Missing reference/input/mass at Flight record %d',k);
            return
        end
        s=r.state;
        if isstruct(s) && isfield(s,'xd')
            xd=s.xd;
        elseif isobject(s) && isprop(s,'xd')
            xd=s.xd;
        elseif isobject(s) && ismethod(s,'get')
            xd=s.get();
        else
            reason=sprintf('Missing 28-reference at record %d',k);return
        end
        if ~c9_same_numeric(xd,T.actual_ref28(:,j),28) || ...
                ~c9_same_numeric(c.tmp,T.unclamped_input4(:,j),4) || ...
                ~c9_same_numeric(e.state.mL,T.estimated_payload_mass_kg(j),1)
            reason=sprintf('Reference/input/mass mismatch at Flight record %d',k);
            return
        end
    catch ME
        reason=sprintf('Cannot read Flight record %d: %s',k,ME.identifier);
        return
    end
end
pass=true;
reason='MATCHED_ALL_FLIGHT_RECORDS';
end

function yes=c9_same_numeric(actual,expected,count)
yes=isnumeric(actual)&&isnumeric(expected)&&isreal(actual)&& ...
    isreal(expected)&&numel(actual)==count&&numel(expected)==count;
if ~yes,return;end
x=double(actual(:));y=double(expected(:));
yes=all(isfinite(x)) && all(isfinite(y)) && ...
    all(abs(x-y)<=1e-10.*max(1,abs(y)));
end
