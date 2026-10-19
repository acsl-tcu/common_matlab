function model = af7r33_make_batch_model_from_proposal(replanner,P,targetVersion)
% AF7R58I canonical bounded batch-model builder.
% No object state is modified here.

assert(isa(replanner,'REPLANNING_SYSTEM'));
[ok,reason,model,~]=af7r58i_prepare_batch_model( ...
    replanner,P,targetVersion,double(P.t0));
assert(ok,'[AF7R33] canonical resource gate rejected: %s',reason);
end
