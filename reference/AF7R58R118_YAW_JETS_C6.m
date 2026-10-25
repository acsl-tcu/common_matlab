function R=AF7R58R118_YAW_JETS_C6(segment,t,anchorYaw)
% Analytic physical-time jets psi^(0..6), psi=unwrapped atan2(py',px').
% Uses truncated complex logarithm Taylor algebra, never finite differences.
% Caller must first certify horizontal speed nonzero throughout connector.
% anchorYaw selects the 2*pi branch of the continuous lift; application
% must carry the preceding yaw branch through every segment / tick.
assert(isstruct(segment)&&isfield(segment,'payload_cp')&& ...
    isfield(segment,'t0')&&isfield(segment,'t1')&& ...
    isequal(size(segment.payload_cp),[3 16]),'R118:YawSegment');
assert(isscalar(t)&&isfinite(t)&&t>=segment.t0&&t<=segment.t1&& ...
    isscalar(anchorYaw)&&isfinite(anchorYaw), ...
    'R118:TimeAndPreviousYawBranchRequired');
T=segment.t1-segment.t0;u=(t-segment.t0)/T;
D=double(segment.payload_cp);
jets=zeros(3,8);
for k=0:7
    jets(:,k+1)=casteljau(D,u);
    if k<7
        D=(size(D,2)-1)/T*diff(D,1,2);
    end
end
z=complex(jets(1,2:8),jets(2,2:8));
if ~(all(isfinite(z))&&abs(z(1))>0)
    error('R118:YawHorizontalSingularity', ...
        'The C7 path tangent cannot define yaw at zero XY velocity.');
end
% z(t+h)/z(t)=1+u(h), u(0)=0. The coefficient of h^k in
% log(1+u(h)) is obtained exactly to degree6 from formal power series.
uc=zeros(1,7);
for k=1:6
    uc(k+1)=z(k+1)/factorial(k)/z(1);
end
product=zeros(1,7);product(1)=1;lc=zeros(1,7);
for m=1:6
    product=truncate_convolve(product,uc);
    lc=lc+(-1)^(m+1)*product/m;
end
yaw=zeros(1,7);
yaw(1)=angle(z(1));
yaw(1)=yaw(1)+2*pi*round((anchorYaw-yaw(1))/(2*pi));
for k=1:6
    yaw(k+1)=factorial(k)*imag(lc(k+1));
end
R=struct('yaw_jets06',yaw,'payload_jets07',jets, ...
    'horizontal_speed_m_s',abs(z(1)), ...
    'method','ANALYTIC_COMPLEX_LOG_TAYLOR_DEGREE6', ...
    'branch_from_external_continuous_anchor',true, ...
    'continuous_heading_precondition_checked_here',false);
end
function c=truncate_convolve(a,b)
c=zeros(1,7);
for k=0:6
    for j=0:k
        c(k+1)=c(k+1)+a(j+1)*b(k-j+1);
    end
end
end
function v=casteljau(P,u)
while size(P,2)>1
    P=(1-u)*P(:,1:end-1)+u*P(:,2:end);
end
v=P(:,1);
end
