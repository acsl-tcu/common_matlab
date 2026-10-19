function xd = af7r58i_eval_canonical_reference(rawfh,flatfh,tf0,model,tg,useFlat)
% Exact additive canonical evaluator.
assert(isa(rawfh,'function_handle'));
assert(iscell(flatfh)&&numel(flatfh)>=7);
tg=double(tg); tf0=double(tf0);
assert(isscalar(tg)&&isfinite(tg)&&isscalar(tf0)&&isfinite(tf0));

tau=tg-tf0;
if tau<0, tau=0; end
xd=rawfh(tau);
xd=double(xd(:));
if numel(xd)<28, xd=[xd;zeros(28-numel(xd),1)]; end
assert(numel(xd)>=28&&all(isfinite(xd(1:28))),'[AF7R58I] nominal raw reference invalid.');

if logical(useFlat)
    for r=0:6
        z=double(flatfh{r+1}(tau));
        z=z(:);
        assert(numel(z)==3&&all(isfinite(z)),'[AF7R58I] nominal flat derivative invalid.');
        xd(4*r+(1:3))=z;
    end
end

assert(isstruct(model)&&isfield(model,'corrections')&&isstruct(model.corrections));
for g=1:numel(model.corrections)
    D=af7r58i_correction_derivatives(tg,model.corrections(g),6);
    for r=0:6
        xd(4*r+(1:3))=xd(4*r+(1:3))+D(:,r+1);
    end
end
assert(all(isfinite(xd(1:28))),'[AF7R58I] canonical reference nonfinite.');
end