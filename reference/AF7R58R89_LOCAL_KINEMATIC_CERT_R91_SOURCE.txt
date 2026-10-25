function K=AF7R58R89_LOCAL_KINEMATIC_CERT(C,g,L,vLimit,maxNodes)
% R91 - derivative-CP Bernstein subdivision on NORMALIZED Bezier time.
% Crucial difference from R89: NEVER repeatedly subtract nearly identical
% position CP or global timestamps on short leaves. Instead subdivide the
% four PHYSICAL-TIME derivative Bezier control polygons independently.
% Whole-leaf UAV speed bound: ||vQ(mid)|| + (T*leafWidth/2)*sup||aQ||.
% All-positive leaf bounds follow algebraically in exact arithmetic.
% IEEE754 directed-rounding / tracking / torque proof NOT supplied.
% If unresolved, return INCONCLUSIVE or a numeric violation witness; NEVER
% mark a candidate safe based on sampling or an assumed epsilon threshold.
assert(isstruct(C)&&isfield(C,'payload_cp')&& ...
    isequal(size(C.payload_cp),[3 16])&&all(isfinite(C.payload_cp(:)))&& ...
    isfield(C,'duration_s')&&isfinite(C.duration_s)&&C.duration_s>0 && ...
    isfield(C,'t0')&&isscalar(C.t0)&&isfinite(C.t0) && ...
    isfield(C,'t1')&&isscalar(C.t1)&&isfinite(C.t1) && C.t1>C.t0, ...
    'R91:FiniteBezierRequired');
assert(isnumeric(g)&&isscalar(g)&&isfinite(g)&&g>0 && ...
    isnumeric(L)&&isscalar(L)&&isfinite(L)&&L>0 && ...
    isnumeric(vLimit)&&isscalar(vLimit)&&isfinite(vLimit)&&vLimit>0 && ...
    isnumeric(maxNodes)&&isscalar(maxNodes)&&isfinite(maxNodes) && ...
    maxNodes>=1&&fix(maxNodes)==maxNodes,'R91:LiveAuthorityRequired');
T=double(C.duration_s);P=double(C.payload_cp);
assert(abs((double(C.t1)-double(C.t0))-T) <= ...
    64*eps(max([1,abs(double(C.t1)),T])), ...
    'R91:InconsistentPhysicalTime');
D1=(15/T)*diff(P,1,2); D2=(14/T)*diff(D1,1,2);
D3=(13/T)*diff(D2,1,2);D4=(12/T)*diff(D3,1,2);
assert(all(isfinite([D1(:);D2(:);D3(:);D4(:)])), ...
    'R91:InvalidDerivedCP');
K=struct('certified',false,'continuous_time',false, ...
 'status','INCONCLUSIVE_NOT_STARTED','nodes',0,'safe_leaves',0, ...
 'start_vQ_m_s',NaN,'end_vQ_m_s',NaN, ...
 'uncertified_interval_s',[NaN NaN], ...
 'velocity_upper_m_s',NaN,'accel_upper_m_s2',NaN, ...
 'jerk_upper_m_s3',NaN,'specific_force_lower_m_s2',NaN, ...
 'cable_qdot_upper_rad_s',NaN,'uav_speed_upper_m_s',NaN, ...
 'finite_sensing_limit_m_s',vLimit,'reference_only',true, ...
 'production_commit_authorized',false, ...
 'ieee754_outward_closed',false, ...
 'violation_witness_time_s',NaN,'violation_witness_speed_m_s',NaN, ...
 'certificate_method','NORMALIZED_DERIVATIVE_CP_PLUS_SPEED_LIPSCHITZ', ...
 'limitations','Real-arithmetic Bernstein enclosure; IEEE outward rounding, HLC tube, torque and thrust not certified');
[K.start_vQ_m_s,okStart]=speed_at(D1(:,1),D2(:,1),D3(:,1),g,L);
[K.end_vQ_m_s,okEnd]=speed_at(D1(:,end),D2(:,end),D3(:,end),g,L);
if ~okStart||~okEnd
  K.status='INCONCLUSIVE_DEGENERATE_ENDPOINT_SPECIFIC_FORCE';return;
end
slack=128*eps(max([1,vLimit,K.start_vQ_m_s,K.end_vQ_m_s]));
if K.start_vQ_m_s >= vLimit-slack
 K.status='NECESSARY_START_SPEED_LIMIT_VIOLATED';
 K.uncertified_interval_s=[C.t0 C.t0];return;
end
if K.end_vQ_m_s >= vLimit-slack
 K.status='NECESSARY_END_SPEED_LIMIT_VIOLATED';
 K.uncertified_interval_s=[C.t1 C.t1];return;
end
stack=struct('V',D1,'A',D2,'J',D3,'S',D4,'ua',0,'ub',1);
maxV=0;maxA=0;maxJ=0;maxQ=0;maxVQ=0;minSig=Inf;
while ~isempty(stack)
 Z=stack(end);stack(end)=[];
 if K.nodes>=maxNodes
  K.status='INCONCLUSIVE_WORK_BUDGET';
  K.uncertified_interval_s=physical_times(C,T,Z.ua,Z.ub);return;
 end
 K.nodes=K.nodes+1;
 [ok,stats,witness]=bound_leaf(Z,g,L,vLimit,T);
 if ok
  K.safe_leaves=K.safe_leaves+1;
  maxV=max(maxV,stats.V);maxA=max(maxA,stats.A);
  maxJ=max(maxJ,stats.J);maxQ=max(maxQ,stats.qdot);
  maxVQ=max(maxVQ,stats.vQ);minSig=min(minSig,stats.sigma);
  continue;
 end
 if witness.valid
  K.status='NUMERICAL_INTERIOR_SPEED_VIOLATION_WITNESS';
  K.violation_witness_time_s=double(C.t0)+T*(Z.ua+(Z.ub-Z.ua)/2);
  K.violation_witness_speed_m_s=witness.speed;
  K.uncertified_interval_s=physical_times(C,T,Z.ua,Z.ub);return;
 end
 mid=Z.ua+0.5*(Z.ub-Z.ua);
 if ~(isfinite(mid)&&mid>Z.ua&&mid<Z.ub)
  K.status='INCONCLUSIVE_NORMALIZED_FLOAT_RESOLUTION';
  K.uncertified_interval_s=physical_times(C,T,Z.ua,Z.ub);return;
 end
 [v0,v1]=split_half(Z.V);[a0,a1]=split_half(Z.A);
 [j0,j1]=split_half(Z.J);[s0,s1]=split_half(Z.S);
 stack(end+1)=struct('V',v1,'A',a1,'J',j1,'S',s1,'ua',mid,'ub',Z.ub); %#ok<AGROW>
 stack(end+1)=struct('V',v0,'A',a0,'J',j0,'S',s0,'ua',Z.ua,'ub',mid); %#ok<AGROW>
end
K.certified=true;K.continuous_time=true;
K.status='CERTIFIED_NUMERIC_BERNSTEIN_REFERENCE_KINEMATICS__NOT_IEEE_PROOF';
K.velocity_upper_m_s=maxV;K.accel_upper_m_s2=maxA;
K.jerk_upper_m_s3=maxJ;K.specific_force_lower_m_s2=minSig;
K.cable_qdot_upper_rad_s=maxQ;K.uav_speed_upper_m_s=maxVQ;
end

function [ok,D,witness]=bound_leaf(Z,g,L,vLimit,T)
 ok=false; witness=struct('valid',false,'speed',NaN);
 D=struct('V',Inf,'A',Inf,'J',Inf,'sigma',NaN,'qdot',Inf,'vQ',Inf);
 D.V=max(vecnorm(Z.V,2,1));D.A=max(vecnorm(Z.A,2,1));
 D.J=max(vecnorm(Z.J,2,1)); S=max(vecnorm(Z.S,2,1));
 F=Z.A+[zeros(2,size(Z.A,2));g*ones(1,size(Z.A,2))];
 fMid=decasteljau(Z.A,0.5)+[0;0;g];sigMid=norm(fMid);
 if ~(isfinite(sigMid)&&sigMid>0),return;end
 n=fMid/sigMid;
 % Since F(u) belongs to convex hull(F_i), n'*F(u)>=min(n'*F_i).
 D.sigma=min(n'*F);
 sigmaSlack=128*eps(max([1,g,max(vecnorm(F,2,1))]));
 if ~(isfinite(D.sigma)&&D.sigma>sigmaSlack),return;end
 D.qdot=D.J/D.sigma;
 % q''=(I-qq')*snap/sigma-2*(q'j)*q'/sigma-q*||q'||^2
 % Hence ||aQ||<=||aL||+L*(||snap||/sigma+3*(J/sigma)^2).
 aqBound=D.A+L*(S/D.sigma+3*(D.J/D.sigma)^2);
 if ~(isfinite(aqBound)&&aqBound>=0),return;end
 [vMid,good]=speed_at(decasteljau(Z.V,0.5), ...
   decasteljau(Z.A,0.5),decasteljau(Z.J,0.5),g,L);
 if ~good,return;end
 width=T*(Z.ub-Z.ua);
 D.vQ=vMid+0.5*width*aqBound;
 tol=128*eps(max([1,vLimit,D.vQ]));
 if isfinite(D.vQ)&&D.vQ < vLimit-tol
   ok=true;return;
 end
 % A witness of excess speed is diagnostic only, NOT positive safety proof.
 if vMid > vLimit+tol
   witness.valid=true;witness.speed=vMid;
 end
end

function [s,ok]=speed_at(v,a,j,g,L)
 f=a+[0;0;g];nf=norm(f);s=NaN;ok=false;
 if ~(isfinite(nf)&&nf>128*eps(max([1,g,norm(a)]))),return;end
 q=f/nf;
 qdot=(j-q*(q'*j))/nf;
 s=norm(v+L*qdot);ok=isfinite(s);
end
function v=decasteljau(P,u)
 for k=size(P,2)-1:-1:1
  P=(1-u)*P(:,1:k)+u*P(:,2:k+1);
 end
 v=P(:,1);
end
function [A,B]=split_half(P)
 N=size(P,2);A=zeros(size(P));B=zeros(size(P));
 A(:,1)=P(:,1);B(:,end)=P(:,end);
 for k=2:N
  P=0.5*(P(:,1:end-1)+P(:,2:end));
  A(:,k)=P(:,1);B(:,N-k+1)=P(:,end);
 end
end
function v=physical_times(C,T,a,b)
 v=[double(C.t0)+T*a,double(C.t0)+T*b];
end
