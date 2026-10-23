function A = AF7R58R47_RETURN_AUDIT(source)
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
