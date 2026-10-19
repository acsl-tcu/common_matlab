function y = af7r58i_nextdown_positive(x)
x=double(x);
assert(isreal(x)&&isscalar(x)&&isfinite(x)&&x>0,'[AF7R58I] nextdown needs positive finite scalar.');
u=typecast(x,'uint64');
y=typecast(u-uint64(1),'double');
end