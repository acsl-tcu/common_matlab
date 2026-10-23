% AF7R58R47_AUTO_R3.m   SOURCE-BOUND, READ-ONLY, SINGLE-FILE DIAGNOSTIC
% Place this file in Downloads or reference (no ZIP selection required).
% Run from MATLAB Command Window:
%   run(fullfile(getenv('USERPROFILE'),'Downloads','AF7R58R47_AUTO_R3.m'))
% Alternatively: run('C:\path\to\AF7R58R47_AUTO_R3.m')
%
% This is a script so run(...) works even when path has not been added.
% Does not change the source, physical values, HLC, plant, MEX, commits or
% obstacle certificate. Missing evidence is always NOT_RUN, never PASS.
% C7, towing and timing diagnostics are evidence, NOT safety certification.
%
% The R45R1 schema is searched exactly; unrelated R44R2 live snapshot is
% NEVER misreported as a R45R1 terminal audit.

fprintf('\n[R47R3] Starting standalone read-only analysis...\n');
try
    OUT = r47_auto_three_tracks();
    assignin('base','AF7R58R47_R3_OUT',OUT);
    fprintf('[R47R3] Base variable: AF7R58R47_R3_OUT\n');
catch ME
    fprintf(2,'[R47R3] ERROR: %s\n',ME.message);
    fprintf(2,'%s\n',getReport(ME,'extended','hyperlinks','off'));
    rethrow(ME);
end

function OUT = r47_auto_three_tracks(varargin)
% R47R3 internal implementation for the direct-run entry script.
%
% Input sources are NEVER synthesized, guessed from other versions, or used
% to modify REPLANNING_SYSTEM/plant/controller. Any absent track stays OPEN.
% Zero-argument mode searches base-workspace structs and R45R1-named .mat,
% a strictly schema-mapped towing log, and the latest R23 report. Selected
% sources are printed and recorded; different-run files are NOT assumed to
% have matching vehicle/controller/obstacle conditions.

assert(mod(numel(varargin),2)==0, ...
    '[R47] Supply name/value pairs: terminal, towing, timing.');
opt=struct('terminal','','towing','','timing','');
for k=1:2:numel(varargin)
    name=lower(char(varargin{k}));
    assert(isfield(opt,name),'[R47] Unknown option: %s',name);
    assert(ischar(varargin{k+1}) || ...
        (isstring(varargin{k+1}) && isscalar(varargin{k+1})), ...
        '[R47] Explicit option %s must be a file path.',name);
    opt.(name)=char(varargin{k+1});
    assert(isfile(opt.(name)), ...
        '[R47] Explicit %s file does not exist: %s',name,opt.(name));
end

scriptdir=fileparts(mfilename('fullpath'));
profile=getenv('USERPROFILE');
roots={pwd,scriptdir};
if ~isempty(profile)
    roots{end+1}=fullfile(profile,'Documents','GitHub','common_matlab','reference'); %#ok<AGROW>
    roots{end+1}=fullfile(profile,'OneDrive','Documents','GitHub','common_matlab','reference'); %#ok<AGROW>
    roots{end+1}=fullfile(profile,'Downloads'); %#ok<AGROW>
end
paths=roots;
if ~strcmpi(pwd,scriptdir)
    [~,package_name]=fileparts(scriptdir);
    % Include parent ONLY for an extracted R47-named package directory;
    % never recursively crawl an unrelated GitHub/common_matlab ancestor.
    if startsWith(package_name,'AF7R58R47')
        parent=fileparts(scriptdir);
        if ~isempty(parent) && ~strcmpi(parent,scriptdir)
            paths{end+1}=parent; %#ok<AGROW>
        end
    end
end
paths=unique(paths,'stable');
paths=paths(cellfun(@isfolder,paths));
fprintf('[R47R3] Search directories:\n');
for kk=1:numel(paths)
    fprintf('  %s\n',paths{kk});
end
output_root=scriptdir;
for kk=1:numel(paths)
    if isfile(fullfile(paths{kk},'REPLANNING_SYSTEM.m'))
        output_root=paths{kk};break
    end
end
if ~isfolder(output_root)
    output_root=pwd;
end
OUT=struct();
OUT.schema='AF7R58R47_STANDALONE_AUTO_V3';
OUT.production_modified=false;
OUT.return=struct('status','NOT_RUN__R45R1_SCHEMA_SOURCE_NOT_FOUND');
OUT.towing=struct('status','NOT_RUN__MAPPED_LIVE_TOWING_LOG_NOT_FOUND');
OUT.timing=struct('status','NOT_RUN__R23_TIMING_REPORT_NOT_FOUND');
OUT.return_source='';OUT.towing_source='';OUT.timing_source='';
OUT.source_pairing_verified=false;
OUT.workspace_struct_inventory=base_struct_inventory();
OUT.production_reference_certified=false;
OUT.closed_loop_continuous_obstacle_certified=false;
OUT.control_period_25ms_certified=false;

fprintf('\n=============================================================\n');
fprintf(' AF7R58R47-R3 STANDALONE 3-TRACK READ-ONLY\n');
fprintf(' No production modification / no fabricated source data\n');
fprintf('=============================================================\n');

% Track 1: exact R45R1 TERMINAL_AUDIT_V1, never R44R2 snapshot alone.
terminal=[];
if ~isempty(opt.terminal)
    terminal=opt.terminal;
    OUT.return_source=['EXPLICIT_FILE: ' opt.terminal];
else
    [terminal,src]=find_workspace_source('terminal');
    if ~isempty(src)
        OUT.return_source=src;
    else
        candidates=collect_matching(paths,{'*R45R1*.mat','*R45*TERMINAL*.mat','*R45*OUT*.mat'});
        for k=1:numel(candidates)
            try
                S=load(candidates{k});
                if contains_terminal(S,0)
                    terminal=S;
                    OUT.return_source=['AUTO_EXACT_SCHEMA_FILE: ' candidates{k}];
                    break
                end
            catch ME
                fprintf('[R47] Terminal candidate unreadable: %s (%s)\n', ...
                    candidates{k},ME.identifier);
            end
        end
    end
end
if ~isempty(OUT.return_source)
    fprintf('\n[R47] RETURN source: %s\n',OUT.return_source);
    try
        OUT.return=r47_return_audit(terminal);
    catch ME
        OUT.return=failed('RETURN_AUDIT_ERROR',ME);
    end
else
    fprintf('\n[R47] RETURN OPEN: actual R45R1 terminal audit not found.\n');
end

% Track 2: only *exact* recorded fields, never inferred state mapping.
tow=[];
if ~isempty(opt.towing)
    tow=opt.towing;
    OUT.towing_source=['EXPLICIT_FILE: ' opt.towing];
else
    [tow,src]=find_workspace_source('towing');
    if ~isempty(src)
        OUT.towing_source=src;
    else
        candidates=collect_matching(paths,{'AF7R58R47_TOWING_LOG*.mat'});
        for k=1:numel(candidates)
            try
                S=load(candidates{k});
                [mapped,found]=extract_towing(S,0);
                if found
                    tow=mapped;
                    OUT.towing_source=['AUTO_EXACT_FIELD_FILE: ' candidates{k}];
                    break
                end
            catch ME
                fprintf('[R47] Towing candidate unreadable: %s (%s)\n', ...
                    candidates{k},ME.identifier);
            end
        end
    end
end
if ~isempty(OUT.towing_source)
    fprintf('\n[R47] TOWING source: %s\n',OUT.towing_source);
    try
        if ischar(tow)
            S=load(tow);
            [tow,found]=extract_towing(S,0);
            assert(found,'[R47] Explicit towing MAT has no 7-field mapped log.');
        end
        OUT.towing=r47_towing_log_audit(tow);
    catch ME
        OUT.towing=failed('TOWING_AUDIT_ERROR',ME);
    end
else
    fprintf('\n[R47] TOWING OPEN: exact recorded log fields not found.\n');
end

% Track 3: the full-flight summary in the R23 report, not a made-up trace.
report='';
if ~isempty(opt.timing)
    report=opt.timing;
    OUT.timing_source=['EXPLICIT_FILE: ' opt.timing];
else
    candidates=collect_matching(paths,{'AF7R58R23_SPIKE_REPORT*.txt'});
    % The most recently modified report is selected, NOT declared to be
    % paired to the R45R1 snapshot or to the optional towing log.
    for k=1:numel(candidates)
        try
            file=fileread(candidates{k});
            if contains(file,'AF7R58R23 | FULL ITERATION SPIKE ATTRIBUTION')
                report=candidates{k};
                OUT.timing_source=['AUTO_LATEST_MATCHING_R23_REPORT: ' report];
                break
            end
        catch
            % Continue looking; other tracks remain independent.
        end
    end
end
if ~isempty(report)
    fprintf('\n[R47] TIMING source: %s\n',OUT.timing_source);
    try
        OUT.timing=r47_timing_source_split(report);
    catch ME
        OUT.timing=failed('TIMING_AUDIT_ERROR',ME);
    end
else
    fprintf('\n[R47] TIMING OPEN: R23 spike report not found.\n');
end

OUT.finished_at=datestr(now,31);
name=['AF7R58R47_R3_RESULTS_' datestr(now,'yyyymmdd_HHMMSS')];
outdir=fullfile(output_root,name);
ix=1;
while exist(outdir,'dir')
    outdir=fullfile(output_root,[name '_' num2str(ix)]);
    ix=ix+1;
end
mkdir(outdir);
OUT.result_directory=outdir;
OUT.mat_file=fullfile(outdir,'AF7R58R47_OUT.mat');
OUT.report_file=fullfile(outdir,'AF7R58R47_REPORT.txt');
save(OUT.mat_file,'OUT');
fid=fopen(OUT.report_file,'w');
assert(fid>0,'[R47] Could not write result report.');
cleaner=onCleanup(@() fclose(fid));
fprintf(fid,'AF7R58R47 THREE TRACKS | STANDALONE READ-ONLY V3\n');
fprintf(fid,'finished_at = %s\n',OUT.finished_at);
fprintf(fid,'production_modified = 0\n');
fprintf(fid,'return_source = %s\n',OUT.return_source);
fprintf(fid,'return_status = %s\n',OUT.return.status);
fprintf(fid,'towing_source = %s\n',OUT.towing_source);
fprintf(fid,'towing_status = %s\n',OUT.towing.status);
fprintf(fid,'timing_source = %s\n',OUT.timing_source);
fprintf(fid,'timing_status = %s\n',OUT.timing.status);
fprintf(fid,'source_pairing_verified = 0\n');
fprintf(fid,'base_workspace_struct_variables = %s\n', ...
    strjoin(OUT.workspace_struct_inventory,', '));
fprintf(fid,'production_reference_certified = 0\n');
fprintf(fid,'closed_loop_continuous_obstacle_certified = 0\n');
fprintf(fid,'control_period_25ms_certified = 0\n');
if isfield(OUT.return,'live_terminal_left_limit_norm_m')
    fprintf(fid,'live_return_offset_norm_m = %.17g\n', ...
        OUT.return.live_terminal_left_limit_norm_m);
end
if isfield(OUT.towing,'max_cable_length_abs_residual_m')
    fprintf(fid,'towing_sampled_cable_length_residual_m = %.17g\n', ...
        OUT.towing.max_cable_length_abs_residual_m);
end
if isfield(OUT.timing,'max_spike_full_ms')
    fprintf(fid,'timing_max_spike_full_ms = %.17g\n', ...
        OUT.timing.max_spike_full_ms);
    fprintf(fid,'timing_dominant_spike_component = %s\n', ...
        OUT.timing.max_spike_dominant_component);
    fprintf(fid,'timing_dominant_spike_component_ms = %.9f\n', ...
        OUT.timing.max_spike_dominant_component_ms);
    fprintf(fid,'top_reported_spikes_over_25ms = %d\n', ...
        OUT.timing.top_row_max_budget_exceed_count);
end
if isfield(OUT.timing,'all_flight_summary_ms')
    S=OUT.timing.all_flight_summary_ms;
    fprintf(fid,'reference_p50_p95_max_ms = %.6f %.6f %.6f\n',S.reference_replanner);
    fprintf(fid,'controller_p50_p95_max_ms = %.6f %.6f %.6f\n',S.controller);
    fprintf(fid,'full_iteration_p50_p95_max_ms = %.6f %.6f %.6f\n',S.full_iteration);
end
if isfield(OUT.return,'live_rejoin_proved_by_source')
    fprintf(fid,'live_rejoin_proved_by_source = %d\n',OUT.return.live_rejoin_proved_by_source);
    fprintf(fid,'live_terminal_hold_after_flight = %d\n',OUT.return.hold_starts_after_flight_end);
end
clear cleaner;
fprintf('\n=============================================================\n');
fprintf('[R47] RETURN = %s\n',OUT.return.status);
fprintf('[R47] TOWING = %s\n',OUT.towing.status);
fprintf('[R47] TIMING = %s\n',OUT.timing.status);
fprintf('[R47] Result MAT = %s\n',OUT.mat_file);
fprintf('[R47] Report     = %s\n',OUT.report_file);
fprintf('[R47] No production commit or safety proof claim.\n');
fprintf('=============================================================\n');
end

function A=failed(label,ME)
A=struct('status',label,'identifier',ME.identifier,'message',ME.message);
fprintf('[R47] %s: %s\n',label,ME.message);
end

function [S,source]=find_workspace_source(kind)
S=[];source='';
try
    vars=evalin('base','whos');
catch
    return
end
for k=1:numel(vars)
    if ~strcmp(vars(k).class,'struct'),continue;end
    name=vars(k).name;
    try
        v=evalin('base',name);
        if strcmp(kind,'terminal') && contains_terminal(v,0)
            S=v; source=['BASE_WORKSPACE_EXACT_SCHEMA: ' name];return
        elseif strcmp(kind,'towing')
            [z,found]=extract_towing(v,0);
            if found
                S=z; source=['BASE_WORKSPACE_EXACT_FIELDS: ' name];return
            end
        end
    catch
        % A variable that cannot be inspected must not be treated as data.
    end
end
end

function tf=contains_terminal(S,depth)
tf=false;
if ~isstruct(S) || depth>16 || numel(S)>200,return;end
for i=1:numel(S)
    x=S(i);
    if isfield(x,'schema') && ...
            (ischar(x.schema)|| (isstring(x.schema)&&isscalar(x.schema))) && ...
            strcmp(char(x.schema),'AF7R58R45R1_TERMINAL_AUDIT_V1')
        tf=true;return
    end
    names=fieldnames(x);
    for j=1:numel(names)
        v=x.(names{j});
        if isstruct(v) && contains_terminal(v,depth+1)
            tf=true;return
        end
    end
end
end

function [T,found]=extract_towing(S,depth)
T=[];found=false;
if ~isstruct(S) || depth>10 || numel(S)>40,return;end
required={'t_s','pQ_m','pL_m','q_cable','L_m','mL_EKF_kg','mL_HLC_kg'};
for i=1:numel(S)
    x=S(i);
    if all(isfield(x,required))
        T=x;found=true;return
    end
    names=fieldnames(x);
    for j=1:numel(names)
        v=x.(names{j});
        if isstruct(v)
            [T,found]=extract_towing(v,depth+1);
            if found,return;end
        end
    end
end
end

function paths=collect_matching(folders,patterns)
% Local directory and subfolders only. Do not crawl machine-wide or web.
paths={};stamps=[];
for f=1:numel(folders)
    folder=folders{f};
    if ~isfolder(folder),continue;end
    for p=1:numel(patterns)
        D=dir(fullfile(folder,patterns{p}));
        % Downloads is a staging location; do not recursively crawl it.
        if ~strcmpi(folder,fullfile(getenv('USERPROFILE'),'Downloads'))
            D=[D;dir(fullfile(folder,'**',patterns{p}))]; %#ok<AGROW>
        end
        for j=1:numel(D)
            if D(j).isdir,continue;end
            file=fullfile(D(j).folder,D(j).name);
            if ~any(strcmpi(paths,file))
                paths{end+1}=file; %#ok<AGROW>
                stamps(end+1)=D(j).datenum; %#ok<AGROW>
            end
        end
    end
end
if ~isempty(stamps)
    [~,order]=sort(stamps,'descend');
    paths=paths(order);
end
end

function names=base_struct_inventory()
names={};
try
    v=evalin('base','whos');
    for k=1:numel(v)
        if strcmp(v(k).class,'struct')
            names{end+1}=v(k).name; %#ok<AGROW>
        end
    end
catch
    % Workspace inventory is diagnostic metadata, not a required source.
end
end
function A = r47_return_audit(source)
% AF7R58R47_RETURN_AUDIT  Read-only, source-bound terminal-continuation audit.
% source: exact terminal-behavior struct, enclosing result struct, or .mat path.
% Never edits REPLANNING_SYSTEM or a committed reference.
% A zero correction at a diagnostic candidate endpoint is not a proof of
% authoritative live rejoin or of continuous-time whole-body safety.

if ischar(source) || (isstring(source) && isscalar(source))
    file = char(source);
    assert(isfile(file), '[R47] Input .mat not found: %s', file);
    source = load(file);
end
assert(isstruct(source), '[R47] Source must be a struct or .mat path.');
[T, source_path] = find_terminal(source, 'source', 0);
assert(~isempty(T), ['[R47] Could not find the exact R45R1 ' ...
    'TERMINAL_AUDIT_V1 schema; no guessed interpretation.']);
required = {'commits','stored_end_s','combined_left_limit_m', ...
    'combined_left_limit_norm_m','terminal_continuous_rejoin_proved', ...
    'extension_alone_guarantees_rejoin','simulation_flight_end_s', ...
    'simulation_total_end_s','path_parameterization_kind', ...
    'live_terminal_hold_start_s','last_commit_terminal_offset_m', ...
    'live_rejoin_time_s'};
for i=1:numel(required)
    assert(isfield(T,required{i}), '[R47] Terminal field missing: %s',required{i});
end
live_offset = double(T.combined_left_limit_m(:));
assert(numel(live_offset)==3 && all(isfinite(live_offset)), ...
    '[R47] Invalid live terminal correction vector.');
A = struct();
A.schema = 'AF7R58R47_RETURN_SOURCE_AUDIT_V1';
A.source_path_in_struct = source_path;
A.stored_end_s = double(T.stored_end_s);
A.commits = double(T.commits);
A.live_terminal_left_limit_m = live_offset;
A.live_terminal_left_limit_norm_m = norm(live_offset);
A.stored_terminal_norm_m = double(T.combined_left_limit_norm_m);
A.norm_consistency_residual_m = abs(A.live_terminal_left_limit_norm_m - ...
    A.stored_terminal_norm_m);
A.live_terminal_hold_start_s = double(T.live_terminal_hold_start_s);
A.live_rejoin_time_s = double(T.live_rejoin_time_s);
A.flight_end_s = double(T.simulation_flight_end_s);
A.simulation_end_s = double(T.simulation_total_end_s);
A.path_parameterization_kind = char(T.path_parameterization_kind);
A.live_rejoin_proved_by_source = logical(T.terminal_continuous_rejoin_proved);
A.extension_guarantees_rejoin_by_source = logical(T.extension_alone_guarantees_rejoin);
A.hold_starts_after_flight_end = A.live_terminal_hold_start_s > A.flight_end_s;
A.live_nonzero_terminal_offset = any(live_offset~=0);
A.live_rejoin_time_finite = isfinite(A.live_rejoin_time_s);
A.candidate_shape_is_live_commit = false;
A.whole_body_obstacle_safety_certified_here = false;
A.closed_loop_towing_safety_certified_here = false;
A.production_modified = false;
A.status = 'CONTINUATION_AUTHORITY_UNPROVED';
if A.live_rejoin_proved_by_source && A.live_rejoin_time_finite && ...
        ~A.live_nonzero_terminal_offset
    A.status = 'LIVE_REJOIN_MARKED_IN_SOURCE__OTHER_CERTIFICATES_STILL_REQUIRED';
elseif contains(A.path_parameterization_kind,'OFFSET_HOLD')
    A.status = 'LIVE_OFFSET_HOLD__NOT_A_REJOIN';
end
fprintf('\n[R47/RETURN] path=%s\n', A.source_path_in_struct);
fprintf('[R47/RETURN] live offset norm=%.12g m | Tstored=%.9f | Tflight=%.9f\n', ...
    A.live_terminal_left_limit_norm_m, A.stored_end_s, A.flight_end_s);
fprintf('[R47/RETURN] continuation=%s | finite rejoin=%d | source proof=%d\n', ...
    A.path_parameterization_kind,A.live_rejoin_time_finite,A.live_rejoin_proved_by_source);
fprintf('[R47/RETURN] %s\n',A.status);
end

function [T, where] = find_terminal(S, path, depth)
T=[]; where='';
assert(depth<=24,'[R47] Terminal nesting exceeds diagnostic search bound.');
if ~isstruct(S), return; end
for j=1:numel(S)
    sj=S(j);
    if isfield(sj,'schema') && ...
            (ischar(sj.schema) || (isstring(sj.schema) && isscalar(sj.schema))) && ...
            strcmp(char(sj.schema),'AF7R58R45R1_TERMINAL_AUDIT_V1')
        T=sj; where=path; return;
    end
    fs=fieldnames(sj);
    for k=1:numel(fs)
        v=sj.(fs{k});
        if isstruct(v)
            [T,where]=find_terminal(v,[path '.' fs{k}],depth+1);
            if ~isempty(T), return; end
        end
    end
end
end


function A = r47_towing_log_audit(log_source)
% AF7R58R47_TOWING_LOG_AUDIT  Read-only sampled towing-log analysis.
% Required struct fields (columns are chronological, in world frame):
%   t_s (Nx1), pQ_m (Nx3), pL_m (Nx3), q_cable (Nx3), L_m (Nx1 or scalar)
%   mL_EKF_kg (Nx1), mL_HLC_kg (Nx1)
% The two mass channels are retained separately, never cross-written.
% Optional explicit logged fields: pQ_ref_m, pL_ref_m, qdot_logged_s,
%   measured_tension_N, thrust_command_N, torque_command_Nm.
% THIS IS NOT a plant propagation and cannot certify between-sample safety,
% ellipsoid clearance, force/tension feasibility, or HLC stability.

if ischar(log_source) || (isstring(log_source) && isscalar(log_source))
    file=char(log_source);
    assert(isfile(file),'[R47] Logged-data .mat not found: %s',file);
    log_source=load(file);
end
assert(isstruct(log_source) && isscalar(log_source), ...
    '[R47] Input must be an explicitly mapped scalar log struct.');
required={'t_s','pQ_m','pL_m','q_cable','L_m', ...
          'mL_EKF_kg','mL_HLC_kg'};
for i=1:numel(required)
    assert(isfield(log_source,required{i}), ...
        '[R47] Missing explicitly required live logged field: %s',required{i});
end
L=log_source;
t=double(L.t_s(:)); n=numel(t);
assert(n>=2 && all(isfinite(t)) && all(diff(t)>0), ...
    '[R47] Log timestamps must be finite, strictly increasing and have N>=2.');
pQ=vec3(L.pQ_m,n,'pQ_m');
pL=vec3(L.pL_m,n,'pL_m');
q=vec3(L.q_cable,n,'q_cable');
len=double(L.L_m(:));
if isscalar(len),len=repmat(len,n,1);end
assert(numel(len)==n && all(isfinite(len)) && all(len>0), ...
    '[R47] Cable lengths are not valid logged values.');
mEKF=column(L.mL_EKF_kg,n,'mL_EKF_kg');
mHLC=column(L.mL_HLC_kg,n,'mL_HLC_kg');
assert(all(mEKF>0) && all(mHLC>0), ...
    '[R47] Logged masses must be positive; no fallback mass permitted.');
qd=sqrt(sum(q.^2,2));
assert(all(qd>0),'[R47] Zero cable-direction vector found.');
d=pL-pQ; dn=sqrt(sum(d.^2,2));
qhat=q./qd;
axial=sum(d.*qhat,2);
perp=sqrt(sum((d-axial.*qhat).^2,2));
A=struct();
A.schema='AF7R58R47_TOWING_SAMPLED_LOG_AUDIT_V1';
A.samples=n;
A.first_t_s=t(1);A.last_t_s=t(end);
A.max_cable_length_abs_residual_m=max(abs(dn-len));
A.max_cable_collinearity_residual_m=max(perp);
A.max_cable_unit_norm_residual=max(abs(qd-1));
A.min_cable_signed_projection_m=min(axial);
A.max_sampled_UAV_secant_speed_mps=max(sqrt(sum((diff(pQ)./diff(t)).^2,2)));
A.max_sampled_load_secant_speed_mps=max(sqrt(sum((diff(pL)./diff(t)).^2,2)));
A.latest_EKF_payload_mass_kg=mEKF(end);
A.latest_HLC_logged_mass_kg=mHLC(end);
A.max_logged_mass_channel_difference_kg=max(abs(mEKF-mHLC));
A.mass_authority_overridden=false;
A.samples_are_not_continuous_time_bounds=true;
A.execution_or_obstacle_certificate=false;
A.production_modified=false;
if isfield(L,'qdot_logged_s')
    qdot=vec3(L.qdot_logged_s,n,'qdot_logged_s');
    A.max_logged_qdot_per_s=max(sqrt(sum(qdot.^2,2)));
else
    A.max_logged_qdot_per_s=NaN;
end
if isfield(L,'measured_tension_N')
    tn=column(L.measured_tension_N,n,'measured_tension_N');
    A.min_measured_tension_N=min(tn);
    A.max_measured_tension_N=max(tn);
else
    A.min_measured_tension_N=NaN;
    A.max_measured_tension_N=NaN;
end
if isfield(L,'pQ_ref_m')
    er=pQ-vec3(L.pQ_ref_m,n,'pQ_ref_m');
    A.max_sampled_UAV_position_error_m=max(sqrt(sum(er.^2,2)));
else
    A.max_sampled_UAV_position_error_m=NaN;
end
if isfield(L,'pL_ref_m')
    er=pL-vec3(L.pL_ref_m,n,'pL_ref_m');
    A.max_sampled_load_position_error_m=max(sqrt(sum(er.^2,2)));
else
    A.max_sampled_load_position_error_m=NaN;
end
if isfield(L,'thrust_command_N')
    A.max_logged_thrust_command_N=max(abs(column(L.thrust_command_N,n,'thrust_command_N')));
else
    A.max_logged_thrust_command_N=NaN;
end
if isfield(L,'torque_command_Nm')
    m=vec3(L.torque_command_Nm,n,'torque_command_Nm');
    A.max_logged_abs_torque_components_Nm=max(abs(m),[],1);
else
    A.max_logged_abs_torque_components_Nm=[NaN NaN NaN];
end
A.status='SAMPLED_TOWING_DIAGNOSTIC_ONLY__NO_CONTINUOUS_CERTIFICATE';
fprintf('\n[R47/TOWING] samples=%d | [%.9f,%.9f] s\n',n,t(1),t(end));
fprintf('[R47/TOWING] cable length residual max %.12g m; lateral %.12g m\n', ...
    A.max_cable_length_abs_residual_m,A.max_cable_collinearity_residual_m);
fprintf('[R47/TOWING] UAV/load sampled secant speed maxima %.9f / %.9f m/s\n', ...
    A.max_sampled_UAV_secant_speed_mps,A.max_sampled_load_secant_speed_mps);
fprintf('[R47/TOWING] latest EKF/HLC mass %.9f / %.9f kg (distinct channels)\n', ...
    A.latest_EKF_payload_mass_kg,A.latest_HLC_logged_mass_kg);
fprintf('[R47/TOWING] %s\n',A.status);
end

function x=vec3(v,n,name)
x=double(v);
assert(ismatrix(x) && size(x,1)==n && size(x,2)==3 && ...
    all(isfinite(x(:))), '[R47] Field %s must be finite Nx3.',name);
end
function x=column(v,n,name)
x=double(v(:));
assert(numel(x)==n && all(isfinite(x)), ...
    '[R47] Field %s must be finite Nx1.',name);
end


function A = r47_timing_source_split(report_file)
% AF7R58R47_TIMING_SOURCE_SPLIT  Read-only R23 main-loop attribution.
% Input: original AF7R58R23_SPIKE_REPORT.txt from SAME simulation run.
% The TOP 30 lines are ranked events, not a complete per-cycle trace.
% Recorded P50/P95/MAX are parsed from the original report; no averaging
% of spikes, no arbitrary tail trimming and no production instrumentation.

assert(ischar(report_file) || (isstring(report_file)&&isscalar(report_file)), ...
    '[R47] Supply explicit R23 report filename.');
report_file=char(report_file);
assert(isfile(report_file),'[R47] Timing report not found: %s',report_file);
raw=fileread(report_file);
assert(contains(raw,'AF7R58R23 | FULL ITERATION SPIKE ATTRIBUTION'), ...
    '[R47] The input is not the expected original R23 cycle timing report.');
ls=splitlines(string(raw));
needle='time_s full online acquire sensor estimator reference controller input plant logger other';
i=find(contains(strtrim(ls),needle),1,'first');
assert(~isempty(i),'[R47] R23 TOP spikes column schema not found.');
X=zeros(0,12);
for k=i+1:numel(ls)
    s=strtrim(char(ls(k)));
    if isempty(s) || startsWith(s,'REPLANNING EVENTS'),break;end
    v=sscanf(s,'%f').';
    if isempty(v),break;end
    assert(numel(v)==12,'[R47] Malformed R23 spike row at line %d.',k);
    X(end+1,:)=v; %#ok<AGROW>
end
assert(~isempty(X) && all(isfinite(X(:))),'[R47] No finite cycle timing samples.');
components={'sensor_acquisition','sensor','estimator', ...
    'reference_replanner','controller','input_transform', ...
    'simulation_plant','logger','other_overhead'};
recordNames=[components,{'online_before_plant','full_iteration'}];
A=struct();
A.schema='AF7R58R47_TIMING_SOURCE_SPLIT_FROM_R23_V1';
A.report_file=report_file;
A.top_rows=X;
A.top_row_count=size(X,1);
A.component_names=components;
A.meaning='Top reported cycle rows only; summary from the complete recorded flight.';
A.top_row_max_budget_exceed_count=sum(X(:,2)>25); % 25 ms is an explicit system period.
A.component_sum_residual_ms=X(:,2)-sum(X(:,4:12),2);
A.max_abs_component_sum_residual_ms=max(abs(A.component_sum_residual_ms));
[~,idx]=max(X(:,4:12),[],2);
A.dominant_components=components(idx);
A.max_spike_time_s=X(1,1);
A.max_spike_full_ms=X(1,2);
A.max_spike_dominant_component=A.dominant_components{1};
A.max_spike_dominant_component_ms=X(1,idx(1)+3);
A.production_modified=false;
A.report_is_measurement_not_WCET_proof=true;
% Separate complete-flight summarized statistics from top-30-only rows.
summary=struct();
for j=1:numel(recordNames)
    key=recordNames{j};
    re=['(?m)^' regexptranslate('escape',key) ...
        '\s+p50=\s*([+\-0-9.eE]+)\s+p95=\s*([+\-0-9.eE]+)\s+max=\s*([+\-0-9.eE]+)'];
    m=regexp(raw,re,'tokens','once');
    assert(~isempty(m),'[R47] Summary bucket missing: %s',key);
    summary.(key)=str2double(m);
end
A.all_flight_summary_ms=summary;
m=regexp(raw,'Recorded Flight cycles\s*=\s*(\d+)','tokens','once');
assert(~isempty(m),'[R47] Recorded Flight cycle count missing.');
A.recorded_flight_cycles=str2double(m{1});
assert(A.recorded_flight_cycles>=A.top_row_count, ...
    '[R47] Timing report has inconsistent cycle count.');
A.stage1_native_returned_ms=[];
A.stage1_native_event_t_s=[];
for k=1:numel(ls)
    s=char(ls(k));
    if ~contains(s,'STAGE1_NATIVE_RETURNED'),continue;end
    m=regexp(s,'sim_t=([+\-0-9.eE]+)\s*\|\s*stage=([+\-0-9.eE]+)', ...
        'tokens','once');
    if ~isempty(m)
        A.stage1_native_event_t_s(end+1,1)=str2double(m{1}); %#ok<AGROW>
        A.stage1_native_returned_ms(end+1,1)=str2double(m{2}); %#ok<AGROW>
    end
end
A.status='READ_ONLY_FULL_CYCLE_SOURCE_ATTRIBUTION__NOT_A_HARD_RT_CERTIFICATE';
fprintf('\n[R47/TIMING] %d reported flight cycles; %d top spike rows\n', ...
    A.recorded_flight_cycles,A.top_row_count);
fprintf('[R47/TIMING] reference p50/p95/max %.6f / %.6f / %.6f ms\n', ...
    summary.reference_replanner);
fprintf('[R47/TIMING] HLC p50/p95/max %.6f / %.6f / %.6f ms\n', ...
    summary.controller);
fprintf('[R47/TIMING] full p50/p95/max %.6f / %.6f / %.6f ms\n', ...
    summary.full_iteration);
fprintf('[R47/TIMING] largest cycle %.9f s = %.6f ms, dominated by %s (%.6f ms)\n', ...
    A.max_spike_time_s,A.max_spike_full_ms, ...
    A.max_spike_dominant_component,A.max_spike_dominant_component_ms);
fprintf('[R47/TIMING] sum closure worst residual %.6g ms\n', ...
    A.max_abs_component_sum_residual_ms);
fprintf('[R47/TIMING] %s\n',A.status);
end


function A = r47_c7_bezier_diag(segments, signed_axis)
% AF7R58R47_C7_BEZIER_DIAG  Piecewise BEZIER correction C7 diagnostics.
% Each element of segments MUST be a nonoverlapping, time-ordered piece of
% the *already combined* correction, with fields:
%   t0_s, t1_s, cp_m (16x3 for degree 15 Bernstein in physical time).
% The axis is an EXPLICIT measured signed direction (3x1); no side guess.
% This verifies double-precision junction jets and Bernstein coefficient
% hulls. It does not check UAV/cable/payload safety, yaw, or plant dynamics.
assert(isstruct(segments) && ~isempty(segments), ...
    '[R47/C7] Supply actual combined correction Bezier segments.');
axis=double(signed_axis(:));
assert(numel(axis)==3 && all(isfinite(axis)) && norm(axis)>0, ...
    '[R47/C7] Explicit measured signed axis must be a nonzero 3-vector.');
axis=axis/norm(axis);
n=numel(segments);
starts=zeros(n,8,3);
ends=zeros(n,8,3);
v_bounds=zeros(n,1);
signed_lows=zeros(n,1);
signed_highs=zeros(n,1);
t0=zeros(n,1);t1=zeros(n,1);
for j=1:n
    seg=segments(j);
    assert(all(isfield(seg,{'t0_s','t1_s','cp_m'})), ...
        '[R47/C7] Missing t0_s, t1_s or cp_m in segment %d.',j);
    t0(j)=double(seg.t0_s); t1(j)=double(seg.t1_s);
    assert(isscalar(t0(j)) && isscalar(t1(j)) && ...
        isfinite(t0(j)) && isfinite(t1(j)) && t1(j)>t0(j), ...
        '[R47/C7] Bad time interval at segment %d.',j);
    c=double(seg.cp_m);
    assert(isequal(size(c),[16,3]) && all(isfinite(c(:))), ...
        '[R47/C7] Segment %d needs degree-15 16x3 combined CP.',j);
    signed_lows(j)=min(c*axis);
    signed_highs(j)=max(c*axis);
    h=t1(j)-t0(j);
    d=c;
    for r=0:7
        starts(j,r+1,:)=d(1,:);
        ends(j,r+1,:)=d(end,:);
        if r==1
            v_bounds(j)=max(sqrt(sum(d.^2,2)));
        end
        d=(15-r)*(diff(d,1,1))/h;
    end
end
residual=zeros(max(0,n-1),8);
time_gap=zeros(max(0,n-1),1);
for j=1:n-1
    time_gap(j)=t0(j+1)-t1(j);
    for r=0:7
        l=reshape(ends(j,r+1,:),[1 3]);
        rr=reshape(starts(j+1,r+1,:),[1 3]);
        residual(j,r+1)=norm(l-rr);
    end
end
terminal_jet=zeros(8,1);
for r=0:7
    terminal_jet(r+1)=norm(reshape(ends(end,r+1,:),[3 1]));
end
A=struct();
A.schema='AF7R58R47_COMBINED_CORRECTION_C7_NUMERIC_DIAG_V1';
A.segment_count=n;
A.start_time_s=t0(1);A.end_time_s=t1(end);
A.time_gaps_s=time_gap;
A.junction_C7_residual_by_order_m_per_s_r=residual;
A.max_junction_C7_jet_residual=max([0;residual(:)]);
A.terminal_jet_norm_by_order=terminal_jet;
A.max_terminal_jet_norm=max(terminal_jet);
A.signed_axis=axis;
A.min_signed_CP_m=min(signed_lows);
A.max_signed_CP_m=max(signed_highs);
A.segment_velocity_CP_upper_mps=v_bounds;
A.max_velocity_CP_upper_mps=max(v_bounds);
A.exact_real_arithmetic_proof=false;
A.head_tangent_yaw_C6_proved=false;
A.whole_body_obstacle_certificate=false;
A.execution_certificate=false;
A.production_modified=false;
A.status='DOUBLE_PRECISION_SHAPE_DIAGNOSTIC_ONLY';
fprintf('\n[R47/C7] %d combined degree-15 Bezier intervals [%g, %g] s\n',n,t0(1),t1(end));
fprintf('[R47/C7] max time gap=%.12g s | C7 junction=%.12g | terminal jets=%.12g\n', ...
    max([0;abs(time_gap)]),A.max_junction_C7_jet_residual, A.max_terminal_jet_norm);
fprintf('[R47/C7] signed CP range=[%.12g, %.12g] m\n', ...
    A.min_signed_CP_m,A.max_signed_CP_m);
fprintf('[R47/C7] correction-speed CP bound max %.12g m/s\n', ...
    A.max_velocity_CP_upper_mps);
fprintf('[R47/C7] %s\n',A.status);
end

