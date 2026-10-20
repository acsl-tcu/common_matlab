function y = af7r58i_nextup_nonnegative(x)
x=double(x);
assert(isreal(x)&&isscalar(x)&&x>=0,'[AF7R58I] nextup needs nonnegative scalar.');
if isinf(x), y=x; return; end
if x==0, x=0; end
u=typecast(x,'uint64');
y=typecast(u+uint64(1),'double');
end