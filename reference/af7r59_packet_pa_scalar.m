function [p,a,valid] = af7r59_packet_pa_scalar(replanner,tg,model)
% AF7R59 exact p/a-only canonical repaired-reference evaluator.
% It is the projection of the existing 0..6 flat-jet evaluator onto
% derivative orders 0 and 2. No interpolation or sampled safety test.

assert(isa(replanner,'REPLANNING_SYSTEM'));
assert(isstruct(model)&&isfield(model,'certified')&&logical(model.certified)&& ...
    isfield(model,'corrections')&&isstruct(model.corrections)&& ...
    isfield(model,'flight_reference_time_origin'), ...
    '[AF7R59] canonical batch model invalid.');
F=replanner.nominal_flat_derivative_fn;
assert(iscell(F)&&numel(F)>=3&&isa(F{1},'function_handle')&&isa(F{3},'function_handle'), ...
    '[AF7R59] nominal derivative-0/2 authority unavailable.');

tg=double(tg); tf0=double(model.flight_reference_time_origin);
assert(isscalar(tg)&&isfinite(tg)&&isscalar(tf0)&&isfinite(tf0));
tau=tg-tf0; if tau<0, tau=0; end
p=double(F{1}(tau)); p=p(:);
a=double(F{3}(tau)); a=a(:);
valid=numel(p)==3&&numel(a)==3&&all(isfinite([p;a]));
if ~valid, return; end

for g=1:numel(model.corrections)
    C=model.corrections(g);
    [d0,d2,ok]=corr_pa(tg,C);
    if ~ok, valid=false; return; end
    p=p+d0;
    a=a+d2;
end
valid=all(isfinite([p;a]));
end

function [d0,d2,ok]=corr_pa(t,C)
d0=zeros(3,1); d2=zeros(3,1); ok=true;
if t<=C.t0||t>=C.t1, return; end
s0=double(C.segment_t0(:)); s1=double(C.segment_t1(:));
is=find(t<=s1,1,'first');
if isempty(is), return; end
if t<s0(is)
    if is==1, return; end
    is=is-1;
end
T=s1(is)-s0(is);
if ~(isfinite(T)&&T>0), ok=false; return; end
u=(t-s0(is))/T; u=max(0,min(1,u));
Q=double(C.control_points{is});
if ~(isequal(size(Q),[3 16])&&all(isfinite(Q),'all')), ok=false; return; end
d0=decasteljau(Q,u);
deg=15;
Q=(deg/T)*diff(Q,1,2); deg=deg-1;
Q=(deg/T)*diff(Q,1,2);
d2=decasteljau(Q,u);
ok=all(isfinite([d0;d2]));
end

function y=decasteljau(C,u)
while size(C,2)>1
    C=(1-u)*C(:,1:end-1)+u*C(:,2:end);
end
y=C(:,1);
end
