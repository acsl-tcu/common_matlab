function [ground_active,phase]=TRAJECTORY_GROUND_SCOPE(options)
%TRAJECTORY_GROUND_SCOPE Flight-only z=0 safety policy.
% Geometry is independent of phase in GROUND_INFORMATION.definition().
% A trajectory certificate may omit ground ONLY when the authoritative
% caller explicitly labels the entire certified piece a/t/l. A piece that
% overlaps the f-phase MUST be split at the phase boundary and checked
% as f on its f-part (no mixed-phase exemption).
% Without a phase annotation the conservative historic policy remains ON.
% Do not derive phase from t_s by comparing against guessed time bounds.
assert(isstruct(options)&&isscalar(options), ...
    'TRAJECTORY_GROUND_SCOPE:Options', ...
    'Caller proof options must be a scalar struct.');
if ~isfield(options,'trajectory_phase')
    phase='UNSPECIFIED_CONSERVATIVE';
    ground_active=true;
    return
end
v=options.trajectory_phase;
if isstring(v)&&isscalar(v),v=char(v);end
assert(ischar(v)&&isrow(v)&&numel(v)==1&& ...
    any(strcmp(v,{'a','t','f','l'})), ...
    'TRAJECTORY_GROUND_SCOPE:Phase', ...
    'Use the real phase a/t/f/l for the WHOLE certified interval; split mixed intervals.');
phase=v;
ground_active=strcmp(v,'f');
end
