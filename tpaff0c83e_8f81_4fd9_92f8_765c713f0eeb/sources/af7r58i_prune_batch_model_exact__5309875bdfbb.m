function [model,npruned] = af7r58i_prune_batch_model_exact(model,t_now)
% Exact future-semantics-preserving expired correction prune.
t_now=double(t_now);
assert(isscalar(t_now)&&isfinite(t_now),'[AF7R58I] prune time must be finite scalar.');
assert(isstruct(model)&&isfield(model,'corrections')&&isstruct(model.corrections), ...
    '[AF7R58I] invalid canonical batch model.');

G=numel(model.corrections);
keep=true(1,G);
for g=1:G
    t1=double(model.corrections(g).t1);
    assert(isscalar(t1)&&isfinite(t1),'[AF7R58I] correction t1 must be finite.');
    if t1<=t_now
        keep(g)=false;
    end
end
npruned=sum(~keep);
model.corrections=model.corrections(keep);
end