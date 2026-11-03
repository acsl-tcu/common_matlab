function C=TRIGGER_PICARD_INTERVAL_OBLIGATIONS(B)
% CONDITIONAL one-interval existence/enclosure test for x'=F(t,x,u,w).
%
% For a true complete 24-state initial set X0 and rectangular Z, assume a
% rigorously outward-rounded interval implementation proves, for EVERY
% t in [0,h], every x in Z, and EVERY admissible held HLC input,
% parameter and disturbance: F(t,x,u,w) in [f_lower,f_upper].
% If X0 + [0,h] * [f_lower,f_upper] lies STRICTLY inside Z, then Picard
% existence/enclosure holds under the standard continuity / Lipschitz
% hypotheses. This executable function tests only the algebraic inclusion.
%
% It DOES NOT derive or independently authenticate the 24-state interval
% RHS, EKF error, held-input set, mass variation, taut-cable tension,
% or rounding guarantees. It ALWAYS returns safe_to_commit=false and
% continuous_closed_loop_safe_certified=false, even on algebraic PASS.
C=struct('status','UNKNOWN_PICARD_RHS_PROOF_MISSING', ...
    'conditional_strict_inclusion',false,'conditional_min_margin',NaN, ...
    'requires_verified_initial_set',true, ...
    'requires_outward_interval_rhs',true, ...
    'requires_ekf_and_hlc_input_enclosure',true, ...
    'requires_mass_disturbance_bounds',true, ...
    'requires_tension_and_hybrid_guards',true, ...
    'safe_to_commit',false, ...
    'continuous_closed_loop_safe_certified',false,'reason','');
try
    assert(isstruct(B)&&isscalar(B), ...
        'TRIGGER_PICARD_INTERVAL_OBLIGATIONS:Type','One interval is required.');
    fields={'h_s','x0_lower','x0_upper','tube_lower','tube_upper', ...
        'rhs_lower','rhs_upper'};
    assert(all(isfield(B,fields)), ...
        'TRIGGER_PICARD_INTERVAL_OBLIGATIONS:Fields','Missing required interval data.');
    h=double(B.h_s);
    assert(isscalar(h)&&isreal(h)&&isfinite(h)&&h>0, ...
        'TRIGGER_PICARD_INTERVAL_OBLIGATIONS:Step','Positive duration required.');
    lo=double(B.x0_lower(:)); hi=double(B.x0_upper(:));
    zl=double(B.tube_lower(:));zu=double(B.tube_upper(:));
    fl=double(B.rhs_lower(:));fu=double(B.rhs_upper(:));
    V=[lo hi zl zu fl fu];
    assert(isequal(size(V),[24 6])&&isreal(V)&&all(isfinite(V(:))) && ...
        all(lo<=hi)&&all(zl<zu)&&all(fl<=fu), ...
        'TRIGGER_PICARD_INTERVAL_OBLIGATIONS:Dimensions', ...
        'Only ordered finite 24-state interval boxes are supported.');
    % Set multiplication [0,h] * [fl,fu], with zero included.
    % IMPORTANT: operations below use ordinary double rounding; the
    % conditional result is NOT a rigorous interval arithmetic certificate.
    candidate_lower=lo+h.*min(0,fl);
    candidate_upper=hi+h.*max(0,fu);
    gap=min([candidate_lower-zl;zu-candidate_upper]);
    C.conditional_min_margin=min(gap);
    C.conditional_strict_inclusion=all(candidate_lower>zl) && ...
        all(candidate_upper<zu);
    if C.conditional_strict_inclusion
        C.status='CONDITIONAL_PICARD_INCLUSION__OUTWARD_RHS_NOT_AUDITED';
        C.reason=['Strict algebraic inclusion holds in ordinary double; ' ...
            'actual model, initial uncertainty, hybrid input, mass, and directed rounding are not proven.'];
    else
        C.status='UNKNOWN_PICARD_INCLUSION_NOT_SHOWN';
        C.reason='No strict inclusion; different tube, step or verified RHS is necessary.';
    end
catch ME
    C.status='UNKNOWN_PICARD_INPUT_OR_ARITHMETIC';
    C.reason=[ME.identifier ' | ' ME.message];
end
end
