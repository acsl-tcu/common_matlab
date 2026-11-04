function y=TRIGGER_FLOAT_OUTWARD_ONE_ULP(x,direction)
% TRIGGER_FLOAT_OUTWARD_ONE_ULP  Adjacent finite IEEE-754 binary64 value.
% direction is +1 (toward +Inf) or -1 (toward -Inf).
% Only a single elementary correctly-rounded operation is enclosed by a
% one-ULP expansion.  This is NOT a certificate for composite RHS/ODEs.
assert(isa(x,'double')&&isreal(x)&&all(isfinite(x(:))) && ...
    isscalar(direction)&&any(direction==[-1 1]), ...
    'TRIGGER_FLOAT_OUTWARD_ONE_ULP:Input', ...
    'Finite binary64 array and direction +/-1 are required.');
sz=size(x);
a=reshape(x,1,[]);
b=typecast(a,'uint64');
positive=a>0;negative=a<0;zero=~(positive|negative);
if direction>0
    b(positive)=b(positive)+uint64(1);
    b(negative)=b(negative)-uint64(1);
    b(zero)=uint64(1); % smallest positive subnormal, including -0
else
    b(positive)=b(positive)-uint64(1);
    b(negative)=b(negative)+uint64(1);
    b(zero)=bitor(bitshift(uint64(1),63),uint64(1));
end
y=reshape(typecast(b,'double'),sz);
end
