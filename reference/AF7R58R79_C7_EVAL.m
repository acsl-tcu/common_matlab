function J=AF7R58R79_C7_EVAL(C,t,order)
% Exact analytical Bezier derivative evaluation, no finite differences.
assert(isstruct(C)&&isfield(C,'payload_cp')&& ...
       isequal(size(C.payload_cp),[3,16]),'[R79] invalid connector.');
assert(isnumeric(t)&&isscalar(t)&&isfinite(t)&&t>=C.t0&&t<=C.t1, ...
       '[R79] evaluation time must be inside connector.');
assert(isnumeric(order)&&isscalar(order)&&order==fix(order)&& ...
       order>=0&&order<=7,'[R79] only derivatives 0..7 supported.');
P=double(C.payload_cp);
T=double(C.duration_s);
for r=1:order
    P=(size(P,2)-1)/T*diff(P,1,2);
end
u=(double(t)-double(C.t0))/T;
while size(P,2)>1
    P=(1-u)*P(:,1:end-1)+u*P(:,2:end);
end
J=P(:,1);
end
