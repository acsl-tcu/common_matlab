function DOUT = af7r58i_correction_derivatives(t,C,maxOrder)
DOUT=zeros(3,maxOrder+1);
if t<=double(C.t0)||t>=double(C.t1), return; end
s0=double(C.segment_t0(:));
s1=double(C.segment_t1(:));
CP=C.control_points;
assert(numel(s0)==numel(s1)&&numel(CP)==numel(s0));

idx=find(t<=s1,1,'first');
if isempty(idx), return; end
if t<s0(idx)
    if idx==1, return; end
    idx=idx-1;
end

ta=s0(idx); tb=s1(idx); T=tb-ta;
assert(isfinite(T)&&T>0);
u=(t-ta)/T;
u=max(0,min(1,u));
C0=double(CP{idx});
assert(isequal(size(C0),[3 16])&&all(isfinite(C0(:))));

deg=15;
Q=C0;
DOUT(:,1)=af7r58i_decasteljau(Q,u);
for r=1:maxOrder
    Q=(deg/T)*diff(Q,1,2);
    deg=deg-1;
    DOUT(:,r+1)=af7r58i_decasteljau(Q,u);
end
end