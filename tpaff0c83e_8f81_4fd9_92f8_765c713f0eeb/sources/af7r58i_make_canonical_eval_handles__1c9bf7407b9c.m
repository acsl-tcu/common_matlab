function [evalFn,flatEvalFn] = af7r58i_make_canonical_eval_handles(replanner,model)
% Bounded nonrecursive evaluator handles.
assert(isa(replanner,'REPLANNING_SYSTEM'));
assert(isstruct(model)&&isfield(model,'certified')&&logical(model.certified)&& ...
    isfield(model,'corrections')&&isfield(model,'flight_reference_time_origin'), ...
    '[AF7R58I] canonical batch model is invalid.');
assert(isa(replanner.base_ref,'TIME_VARYING_REFERENCE')&& ...
    isprop(replanner.base_ref,'func')&&isa(replanner.base_ref.func,'function_handle'), ...
    '[AF7R58I] nominal raw reference authority unavailable.');
F=replanner.nominal_flat_derivative_fn;
assert(iscell(F)&&numel(F)>=7&&all(cellfun(@(q)isa(q,'function_handle'),F(1:7))), ...
    '[AF7R58I] nominal flat derivative authority unavailable.');

rawfh=replanner.base_ref.func;
flatfh=F(1:7);
tf0=double(model.flight_reference_time_origin);
assert(isscalar(tf0)&&isfinite(tf0));

evalFn=@(tg)af7r58i_eval_canonical_reference(rawfh,flatfh,tf0,model,double(tg),false);
flatEvalFn=@(tg)af7r58i_eval_canonical_reference(rawfh,flatfh,tf0,model,double(tg),true);
end