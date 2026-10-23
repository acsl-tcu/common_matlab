function C = AF7R58R31_C7_RETURN_SEED(t_start, duration, offset)
% AF7R58R31_C7_RETURN_SEED
% Candidate-only C7 terminal offset reduction, NOT a flight plan/certificate.
% Terminal offset c0 continues from previously certified reference; this seed
% transitions c0 to zero without componentwise overshoot on the return itself.
% Duration is an EXTERNAL variable to be searched; not a fixed maneuver time.
% A complete continuous-time UAV/payload/cable, speed, yaw and physical-lead
% certificate is MANDATORY before any future use of such a candidate.
validateattributes(t_start, {'double'}, {'real','finite','scalar'});
validateattributes(duration, {'double'}, {'real','finite','scalar','positive'});
validateattributes(offset, {'double'}, {'real','finite','numel',3});
c0 = reshape(offset,3,1);
% Degree-15 Bernstein control polygon: B0,...,B7=c0; B8,...,B15=0.
% Therefore derivatives 1..7 vanish at both endpoints exactly in real
% arithmetic; adjoining constant holds are C7. Shape = 1 - I_u(8,8).
cp = [repmat(c0,1,8), zeros(3,8)];
C = struct('schema','AF7R58R31_C7_RETURN_SEED_V1', ...
    't_start',t_start,'duration',duration,'t_end',t_start+duration, ...
    'control_points',cp,'degree',15,'offset_start',c0, ...
    'speed_certified',false,'heading_certified',false, ...
    'wholebody_certified',false,'committable',false, ...
    'stage','CANDIDATE_ONLY_NOT_CONNECTED');
assert(isfinite(C.t_end) && C.t_end>t_start, ...
    'Finite nonzero end time is required.');
end
