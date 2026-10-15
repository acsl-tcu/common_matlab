function OUT = AF7R19A_R8_BEZIER_C7_ALGEBRA_AUDIT()
% Independent algebra audit for degree-15 endpoint jets and affine towing segment.
% Does not call private REPLANNING_SYSTEM methods and does not modify production source.

T1=4.225;
T2=3.175;
T3=7.35;
a=[0.72;-1.14;0.33];
v=[0.28;-0.41;0.16];

Jz=zeros(3,8);
Je=zeros(3,8); Je(:,1)=a; Je(:,2)=v;
P1=zeros(3,16);
for d=1:3
    P1(d,:)=cp15(Jz(d,:),Je(d,:),T1).';
end

u=(0:15)/15;
P2=a+v*(T2*u);

Js=zeros(3,8);
Js(:,1)=a+v*T2;
Js(:,2)=v;
P3=zeros(3,16);
for d=1:3
    P3(d,:)=cp15(Js(d,:),Jz(d,:),T3).';
end

D1e=endpoint_jets(P1,T1,false,7);
D2s=endpoint_jets(P2,T2,true,7);
D2e=endpoint_jets(P2,T2,false,7);
D3s=endpoint_jets(P3,T3,true,7);
D1s=endpoint_jets(P1,T1,true,7);
D3e=endpoint_jets(P3,T3,false,7);

err12=max(abs(D1e-D2s),[],'all');
err23=max(abs(D2e-D3s),[],'all');
errStart=max(abs(D1s),[],'all');
errEnd=max(abs(D3e),[],'all');
affineHigh=max(abs(D2s(:,3:8)),[],'all');

scale=max(1,max(abs([D1e(:);D2s(:);D2e(:);D3s(:)])));
tol=2^20*eps(scale);
closed=err12<=tol && err23<=tol && errStart<=tol && errEnd<=tol && affineHigh<=tol;

fprintf('\n====================================================================================================\n');
fprintf(' AF7R19A | R8 DEGREE-15 C7 / TOWING-AFFINE ALGEBRA AUDIT\n');
fprintf('====================================================================================================\n');
fprintf(' transition -> unsafe C7 mismatch        = %.16e\n',err12);
fprintf(' unsafe -> transition C7 mismatch        = %.16e\n',err23);
fprintf(' initial zero-jet mismatch               = %.16e\n',errStart);
fprintf(' terminal zero-jet mismatch              = %.16e\n',errEnd);
fprintf(' unsafe affine derivatives r=2..7       = %.16e\n',affineHigh);
fprintf(' floating audit tolerance                = %.16e\n',tol);
fprintf(' algebra closed                          = %d\n',closed);
fprintf('====================================================================================================\n');

OUT=struct('schema','AF7R19A_R8_BEZIER_C7_ALGEBRA_AUDIT_V1', ...
    'err_transition_to_affine',err12,'err_affine_to_transition',err23, ...
    'err_start_zero_jet',errStart,'err_end_zero_jet',errEnd, ...
    'affine_high_derivative_error',affineHigh,'tolerance',tol,'closed',closed);
end

function P=cp15(j0,j1,T)
j0=double(j0(:)); j1=double(j1(:));
ds=zeros(8,1); de=zeros(8,1);
ds(1)=j0(1); de(1)=j1(1);
ff=1;
for r=1:7
    ff=ff*(16-r);
    ds(r+1)=j0(r+1)*T^r/ff;
    de(r+1)=j1(r+1)*T^r/ff;
end
P=zeros(16,1);
for k=0:7
    v0=0; v1=0;
    for r=0:k
        bc=nchoosek(k,r);
        v0=v0+bc*ds(r+1);
        v1=v1+((-1)^r)*bc*de(r+1);
    end
    P(k+1)=v0;
    P(16-k)=v1;
end
end

function J=endpoint_jets(P,T,atStart,maxOrder)
D=double(P);
J=zeros(3,maxOrder+1);
for r=0:maxOrder
    if atStart
        J(:,r+1)=D(:,1);
    else
        J(:,r+1)=D(:,end);
    end
    if r<maxOrder
        deg=15-r;
        D=(deg/T)*diff(D,1,2);
    end
end
end
