function y = AF7R58R31_C7_RETURN_EVAL(C,t,derivative_order)
% Evaluate the C7 correction seed, including constant holds outside span.
% The function is NEVER a replacement for continuous-time safety proof.
validateattributes(t, {'double'}, {'real','finite','scalar'});
validateattributes(derivative_order, {'double'}, ...
    {'real','finite','integer','scalar','>=',0,'<=',7});
r=derivative_order;
if t <= C.t_start
    y=zeros(3,1);
    if r==0, y=C.offset_start; end
    return
elseif t >= C.t_end
    y=zeros(3,1);
    return
end
B=C.control_points;
for j=1:r
    B=(size(B,2)-1)*diff(B,1,2)/C.duration;
end
u=(t-C.t_start)/C.duration;
% De Casteljau: convex arithmetic for 0<=u<=1.
for k=size(B,2)-1:-1:1
    B=(1-u)*B(:,1:k)+u*B(:,2:k+1);
end
y=B(:,1);
end
