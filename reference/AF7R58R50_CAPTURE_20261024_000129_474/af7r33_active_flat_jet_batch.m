function [Y,valid,meta] = af7r33_active_flat_jet_batch(replanner,tg)
% AF7R33 repeated-replan-safe active-reference flat-jet batch evaluator.
%
% NOMINAL:
%   exact current nominal_flat_derivative_fn{1:7}, batched.
%
% REPAIRED:
%   exact nominal batch + every committed piecewise degree-15 correction
%   stored in active_repaired_reference.batch_flat_model.
%
% Each committed correction has zero endpoint jets and is additive to the
% previous active flat reference.  Therefore the complete repaired chain is
% exactly nominal + sum(committed corrections).
%
% If a repaired reference predates AF7R33 metadata, this function fails
% closed rather than silently substituting the nominal reference.

assert(isa(replanner,'REPLANNING_SYSTEM'), ...
    '[AF7R33] replanner must be REPLANNING_SYSTEM.');
tg=double(tg(:).');
assert(all(isfinite(tg)),'[AF7R33] global times must be finite.');

mode=upper(char(replanner.active_reference_mode));
switch mode
    case 'NOMINAL'
        model=af7r33_nominal_batch_model(replanner);
    case 'REPAIRED'
        R=replanner.active_repaired_reference;
        assert(isstruct(R)&&isfield(R,'batch_flat_model')&& ...
            isstruct(R.batch_flat_model)&&isfield(R.batch_flat_model,'certified')&& ...
            logical(R.batch_flat_model.certified), ...
            ['[AF7R33] repaired active reference lacks the explicit batch flat model. ' ...
             'Failing closed prevents accidental nominal substitution.']);
        model=R.batch_flat_model;
    otherwise
        error('[AF7R33] unsupported active reference mode %s.',mode);
end

[Y,valid,meta]=af7r33_flat_jet_batch_model(replanner,tg,model);
end
