function [p,tau,d] = AF7R58R60_EVAL(C,t,nominal_position_fn)
% Evaluate an R60 candidate; numeric diagnostic, NOT a safety certificate.
assert(isa(nominal_position_fn,'function_handle'), ...
    'R60:Nominal','Nominal evaluator must be provided by the caller');
assert(isstruct(C)&&isfield(C,'t0_s')&&isfield(C,'t1_s')&& ...
    isfield(C,'phase_bezier17')&&isfield(C,'offset_bezier15'), ...
    'R60:Candidate','Invalid candidate');
T=C.t1_s-C.t0_s;
assert(isscalar(T)&&isfinite(T)&&T>0,'R60:Duration','Bad candidate');
tau=zeros(1,numel(t));d=zeros(3,numel(t));p=zeros(3,numel(t));
for i=1:numel(t)
    ti=double(t(i));
    if ti<C.t0_s
        error('R60:OutsideDomain', ...
            'Candidate evaluation before t0 requires existing active history');
    elseif ti<=C.t1_s
        u=(ti-C.t0_s)/T;
        tau(i)=casteljau(C.phase_bezier17,u);
        d(:,i)=casteljau(C.offset_bezier15,u);
    else
        tau(i)=ti-C.phase_delay_final_s;
        d(:,i)=zeros(3,1);
    end
    q=double(nominal_position_fn(tau(i)));
    assert(isequal(size(q),[3 1])&&all(isfinite(q)), ...
        'R60:NominalPosition','Invalid nominal position');
    p(:,i)=q+d(:,i);
end
end
function v=casteljau(cp,u)
v=cp;n=size(v,2)-1;
for j=1:n
    v(:,1:n-j+1)=(1-u)*v(:,1:n-j+1)+u*v(:,2:n-j+2);
end
v=v(:,1);
end
