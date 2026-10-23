function R=AF7R58R33_LOGGER_RETURN_AUDIT(logger,runtime_profile)
% Run after the same headless simulation: no need to regenerate trajectories.
% Must use the exact sample correspondence; refuse if sizes disagree.
assert(isa(logger,'LOGGER'), 'First input must be the live LOGGER.');
assert(isstruct(runtime_profile) && isfield(runtime_profile,'t') && ...
    isfield(runtime_profile,'phase'), 'Runtime profile unavailable.');
[raw,~]=logger.data(1, ...
    'reference.result.state.rtime_correction_xyz', '', 'phase','f');
D=squeeze(double(raw));
mask=(runtime_profile.phase(:)=='f');
t=double(runtime_profile.t(mask));
% Source ordering is strictly sim-time synchronized for these scalar/vector
% samples, but verify the numbers before using them as trajectory evidence.
if ismatrix(D) && size(D,1)==numel(t) && size(D,2)==3
    C=D;
elseif ismatrix(D) && size(D,1)==3 && size(D,2)==numel(t)
    C=D.';
else
    error('AF7R58R33:LOGGER_ALIGNMENT', ...
       'LOGGER sample count/orientation does not match Flight timestamps.');
end
R=AF7R58R33_RETURN_VECTOR_AUDIT(t,C);
end
