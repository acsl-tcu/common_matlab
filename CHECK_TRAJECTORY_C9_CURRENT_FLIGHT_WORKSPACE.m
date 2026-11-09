function R = CHECK_TRAJECTORY_C9_CURRENT_FLIGHT_WORKSPACE(logger_object)
%CHECK_TRAJECTORY_C9_CURRENT_FLIGHT_WORKSPACE
% Read the CURRENT Flight's Sim LOGGER in memory. DO NOT change the runner.
%
% Usage from the MATLAB command window, immediately after the existing run:
%   run_flight_headless
%   R = CHECK_TRAJECTORY_C9_CURRENT_FLIGHT_WORKSPACE();
%
% The existing run_flight_headless is a script. Its 'logger' variable remains
% in the base workspace after the run finishes, unless cleared externally.
% This function does NOT read any saved Flight MAT or run a simulation.
% The resulting MAT/TXT/ZIP are OUTPUT files, written only after Flight.
%
% If invoking from a function workspace, explicitly pass its LOGGER object:
%   R = CHECK_TRAJECTORY_C9_CURRENT_FLIGHT_WORKSPACE(logger);
%
% WARNING: sampled evidence is NOT an all-time HLC tracking or tube proof.

root = fileparts(mfilename('fullpath'));
assert(isfile(fullfile(root,'run_flight_headless.m')), ...
    'C9_CURRENT:Root', 'Place CHECK in the common_matlab root.');
module = fullfile(root,'reference','REPLANNING_SYSTEM','TRAJECTORY_GENERATION');
assert(isfolder(module),'C9_CURRENT:Module','TRAJECTORY_GENERATION not found.');
if exist('TRAJECTORY_C9_RECORDED_FLIGHT_HLC_AUDIT','file') ~= 2 || ...
        exist('TRAJECTORY_C9_RECORDED_TRACKING_WITNESS','file') ~= 2
    addpath(module,'-begin');
end
assert(exist('TRAJECTORY_C9_RECORDED_FLIGHT_HLC_AUDIT','file')==2 && ...
       exist('TRAJECTORY_C9_RECORDED_TRACKING_WITNESS','file')==2, ...
       'C9_CURRENT:MissingAudit', ...
       'The previously installed recorded-flight audit functions are required.');

if nargin < 1 || isempty(logger_object)
    if evalin('base','exist(''logger'',''var'')') ~= 1
        error('C9_CURRENT:NoInMemoryLogger', ...
            ['No logger exists in the MATLAB base workspace. Run the ' ...
             'UNCHANGED run_flight_headless script from the command window, ' ...
             'then call this CHECK without clearvars/clear classes.']);
    end
    logger_object = evalin('base','logger');
    source = 'CURRENT_BASE_WORKSPACE_logger_Data';
else
    source = 'CURRENT_CALLER_SUPPLIED_logger_Data';
end

if isobject(logger_object)
    assert(isprop(logger_object,'Data'),'C9_CURRENT:Data', ...
        'The input must be the active Sim LOGGER with its Data property.');
    D = logger_object.Data;
elseif isstruct(logger_object) && isscalar(logger_object) && ...
        isfield(logger_object,'Data')
    D = logger_object.Data;
else
    error('C9_CURRENT:Data','Expected the actual in-memory LOGGER object.');
end
assert(isstruct(D)&&isscalar(D)&& ...
    all(isfield(D,{'t','phase','agent'})), ...
    'C9_CURRENT:Data', 'Incomplete active LOGGER.Data.');

R = struct();
R.status = 'UNKNOWN_NOT_EVALUATED';
R.data_source = source;
R.read_saved_flight_mat = false;
R.modified_runner = false;
R.real_flight_data_evaluated = false;
R.tracking_all_time_certified = false;
R.HLC_three_axis_torque_all_time_certified = false;
R.safe_to_commit = false;
R.trace = struct();
R.tracking = struct();
R.trace_error = '';
R.tracking_error = '';
R.flight_records = 0;
R.checked_flight_records = 0;
R.preclamp_saturation_count = NaN;
R.max_observed_payload_position_error_m = NaN;
R.max_observed_payload_velocity_error_mps = NaN;
R.sampled_payload_error_tube_necessary_radius_m = NaN;
R.estimated_payload_mass_kg = [];
R.HLC_consumed_mass_per_tick_verified = false;
R.generated_C9_reference_actually_sent = false;

try
    R.trace = TRAJECTORY_C9_RECORDED_FLIGHT_HLC_AUDIT(D);
    R.flight_records = R.trace.flight_records;
    R.checked_flight_records = R.trace.checked_flight_records;
    R.preclamp_saturation_count = R.trace.preclamp_saturation_count;
    R.estimated_payload_mass_kg = R.trace.estimated_payload_mass_kg;
catch ME
    R.trace_error = sprintf('%s | %s',ME.identifier,ME.message);
end
try
    R.tracking = TRAJECTORY_C9_RECORDED_TRACKING_WITNESS(D);
    R.max_observed_payload_position_error_m = ...
        R.tracking.max_observed_payload_position_error_m;
    R.max_observed_payload_velocity_error_mps = ...
        R.tracking.max_observed_payload_velocity_error_mps;
    R.sampled_payload_error_tube_necessary_radius_m = ...
        R.tracking.sampled_payload_error_tube_necessary_radius_m;
catch ME
    R.tracking_error = sprintf('%s | %s',ME.identifier,ME.message);
end

if isempty(R.trace_error) && isempty(R.tracking_error) && ...
        R.flight_records > 0 && ...
        R.tracking.sample_count == R.flight_records && ...
        R.tracking.invalid_sample_count == 0
    R.real_flight_data_evaluated = true;
    if R.trace.failed_flight_records > 0 || ...
            R.preclamp_saturation_count > 0
        R.status = 'OBSERVED_FLIGHT_INPUT_OR_REFERENCE_FAILURE__NO_COMMIT';
    else
        R.status = 'CHECKED_ACTUAL_IN_MEMORY_FLIGHT_SAMPLES__NO_ALL_TIME_PROOF';
    end
else
    R.status = 'UNKNOWN_INCOMPLETE_ACTUAL_FLIGHT_TRACE__NO_COMMIT';
end

base = 'TRAJECTORY_C9_CURRENT_FLIGHT_WORKSPACE_RESULT';
mat_path = fullfile(root,[base '.mat']);
txt_path = fullfile(root,[base '.txt']);
zip_path = fullfile(root,[base '_FOR_CHATGPT.zip']);
save(mat_path,'R','-v7');
fid = fopen(txt_path,'w');
assert(fid>=0,'C9_CURRENT:Write','Cannot create audit text in common_matlab.');
cleanup = onCleanup(@() fclose(fid));
fprintf(fid,'status=%s\nsource=%s\n',R.status,R.data_source);
fprintf(fid,'saved_flight_mat_input=0\nrunner_modified=0\n');
fprintf(fid,'real_flight_evaluated=%d\n',R.real_flight_data_evaluated);
fprintf(fid,'flight_records=%d\nchecked_flight_records=%d\n', ...
    R.flight_records,R.checked_flight_records);
fprintf(fid,'preclamp_saturation_count=%.12g\n',R.preclamp_saturation_count);
fprintf(fid,'max_observed_payload_position_error_m=%.12g\n', ...
    R.max_observed_payload_position_error_m);
fprintf(fid,'max_observed_payload_velocity_error_mps=%.12g\n', ...
    R.max_observed_payload_velocity_error_mps);
fprintf(fid,'sampled_payload_error_tube_necessary_radius_m=%.12g\n', ...
    R.sampled_payload_error_tube_necessary_radius_m);
fprintf(fid,'trace_error=%s\ntracking_error=%s\n', ...
    R.trace_error,R.tracking_error);
fprintf(fid,'HLC_consumed_mass_per_tick_verified=0\n');
fprintf(fid,'generated_C9_reference_actually_sent=0\n');
fprintf(fid,'continuous_time_HLC_tracking_certified=0\n');
fprintf(fid,'full_time_input_torque_certified=0\n');
fprintf(fid,'safe_to_commit=0\n');
clear cleanup
if isfile(zip_path),delete(zip_path);end
zip(zip_path,{[base '.mat'],[base '.txt']},root);
assert(isfile(zip_path),'C9_CURRENT:ZIP','Result ZIP was not created.');
fileinfo = dir(zip_path);
assert(fileinfo.bytes>0,'C9_CURRENT:ZIP','Result ZIP was empty.');
R.export_zip_path = zip_path;
R.export_zip_bytes = fileinfo.bytes;
fprintf('[C9 CURRENT FLIGHT] %s | %d samples | %s\n', ...
    R.status,R.flight_records,zip_path);
end
