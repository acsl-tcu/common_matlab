function OUT = AF7R36_TOWING_DIAG_CAPTURE_V2_INSTALL()
% AF7R36_TOWING_DIAG_CAPTURE_V2_INSTALL
% -------------------------------------------------------------------------
% V2 post-run persistence patch for AF7_TOWING_DIAG_V1.
%
% PURPOSE
%   Preserve the last *real runtime* AF7R36 native diagnostic result after
%   the staged replanning pipeline has completed, so no temporary R variable
%   is required after run_flight_headless finishes.
%
% SAFETY / SEMANTICS
%   - Does NOT change trajectory synthesis.
%   - Does NOT change candidate ordering.
%   - Does NOT change speed/rejoin/whole-body certificates.
%   - Does NOT change adaptive rejoin selection or commit logic.
%   - Stores only the already-computed RAW result in the existing
%     generator_last_status property, outside preflight/integration priming.
%
% PREREQUISITE
%   AF7R36_TOWING_DIAG_INSTALL (V1) must already be installed.

fprintf('\n====================================================================================================\n');
fprintf(' AF7R36 TOWING DIAGNOSTIC CAPTURE V2 INSTALL\n');
fprintf(' trajectory/certificate semantics modified = 0\n');
fprintf('====================================================================================================\n');

rf = which('REPLANNING_SYSTEM');
assert(~isempty(rf) && isfile(rf),'[TOWDIAG-V2] REPLANNING_SYSTEM.m not resolved.');
refDir = fileparts(rf);
hpp = fullfile(refDir,'af7r36_core.hpp');
cpp = fullfile(refDir,'af7r36_single_core_mex.cpp');
assert(isfile(hpp) && isfile(cpp),'[TOWDIAG-V2] AF7R36 sources are missing.');

h = normalize_lf(fileread(hpp));
c = normalize_lf(fileread(cpp));
s = normalize_lf(fileread(rf));

assert(contains(h,'AF7_TOWING_DIAG_V1'), ...
    '[TOWDIAG-V2] V1 core diagnostics are not installed in af7r36_core.hpp.');
assert(contains(c,'towing_diag_revision'), ...
    '[TOWDIAG-V2] V1 MEX diagnostics are not installed in af7r36_single_core_mex.cpp.');

if contains(s,'AF7_TOWING_DIAG_CAPTURE_V2')
    fprintf('[TOWDIAG-V2] Already installed: %s\n',rf);
    OUT=struct('pass',true,'already_installed',true,'source',rf, ...
        'trajectory_semantics_modified',false,'certificate_semantics_modified',false);
    return;
end

old = [ ...
    '            pkt=af7r34_make_core_packet(obj,CTX,[]);' newline ...
    '            RAW=af7r36_single_core_mex(1,pkt);' newline newline ...
    '            P=struct();' newline];

new = [ ...
    '            pkt=af7r34_make_core_packet(obj,CTX,[]);' newline ...
    '            RAW=af7r36_single_core_mex(1,pkt);' newline newline ...
    '            % AF7_TOWING_DIAG_CAPTURE_V2 -- diagnostic persistence only.' newline ...
    '            % Never used by synthesis, certification, selection or commit.' newline ...
    '            % Ignore setup/integration prime calls so post-run data refers' newline ...
    '            % to an actual runtime replanning request.' newline ...
    '            if isfield(RAW,''towing_diag_revision'') && ...' newline ...
    '                    ~obj.setup_exact_do_prime_active && ...' newline ...
    '                    ~obj.generator_integration_prime_active' newline ...
    '                obj.generator_last_status=RAW;' newline ...
    '            end' newline newline ...
    '            P=struct();' newline];

hits=strfind(s,old);
assert(numel(hits)==1, ...
    '[TOWDIAG-V2] Source anchor count=%d; expected exactly 1. No edit made.',numel(hits));

stamp=char(datetime('now','Format','yyyyMMdd_HHmmss'));
bak=[rf '.towdiag_capture_v2_backup_' stamp];
copyfile(rf,bak,'f');

try
    s=strrep(s,old,new);
    write_text_lf(rf,s);
    fprintf('[TOWDIAG-V2] PASS: post-run RAW persistence installed.\n');
    fprintf('[TOWDIAG-V2] REPLANNING_SYSTEM = %s\n',rf);
    fprintf('[TOWDIAG-V2] backup            = %s\n',bak);
    fprintf('[TOWDIAG-V2] IMPORTANT: run `clear classes; clear mex; rehash` before the next simulation.\n');
    OUT=struct('pass',true,'already_installed',false,'source',rf,'backup',bak, ...
        'trajectory_semantics_modified',false,'certificate_semantics_modified',false);
catch ME
    try, copyfile(bak,rf,'f'); catch, end
    fprintf(2,'[TOWDIAG-V2] FAILED; REPLANNING_SYSTEM restored from backup.\n');
    rethrow(ME);
end
end

function s=normalize_lf(s)
s=strrep(s,sprintf('\r\n'),sprintf('\n'));
s=strrep(s,sprintf('\r'),sprintf('\n'));
end

function write_text_lf(path,txt)
f=fopen(path,'wt','n','UTF-8');
assert(f>=0,'[TOWDIAG-V2] Cannot write %s',path);
c=onCleanup(@()fclose(f)); %#ok<NASGU>
fwrite(f,txt,'char');
end
