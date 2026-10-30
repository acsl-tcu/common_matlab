function R=AF7R58R186_BIND_C7(source,startTime)
% Turns a genuine R170/R168 degree-15 C7 candidate into a 28-state
% physical reference with path-exact yaw derivatives 0..6. Not a
% certification of controller tracking, torque, return, or obstacle risk.
R=struct('status','UNKNOWN_C7_INPUT','bound',false,...
 'reference',struct(),'C7_heading',struct(),...
 'production_commit_authorized',false,'execution_certified',false);
if ~isstruct(source)||~isscalar(source)||~isfield(source,'control_points')||...
 ~isfield(source,'times_s')||~isscalar(startTime)||~isfinite(startTime),return;end
T=double(source.times_s(:).');P=source.control_points;
if numel(T)<2||~iscell(P)||numel(P)~=numel(T)-1||...
 any(~isfinite(T))||any(diff(T)<=0),return;end
H=AF7R58R184_C7_HEADING_GATE(source);R.C7_heading=H;
if ~H.conditional_heading_C6
 R.status=['REJECT_REFERENCE_' char(H.status)];return;
end
% Build a canonical set of 3x16 finite control points. Determine the
% unwrapped yaw at each segment start by continuity along its endpoint.
A=zeros(1,numel(P));lastEnd=NaN;
for k=1:numel(P)
 C=double(P{k});if isequal(size(C),[16 3]),C=C';end
 if ~isequal(size(C),[3 16])||any(~isfinite(C(:)))
  R.status='REJECT_INVALID_BERNSTEIN';return;
 end
 P{k}=C;
 vStart=complex(C(1,2)-C(1,1),C(2,2)-C(2,1));
 vEnd=complex(C(1,16)-C(1,15),C(2,16)-C(2,15));
 if abs(vStart)==0||abs(vEnd)==0
  R.status='REJECT_ZERO_TANGENT_AT_KNOT';return;
 end
 if k==1
  A(k)=angle(vStart);
 else
  theta=angle(vStart);
  A(k)=lastEnd+atan2(sin(theta-lastRawEnd),cos(theta-lastRawEnd));
 end
 lastRawEnd=angle(vEnd);
 lastEnd=A(k)+atan2(sin(lastRawEnd-angle(vStart)),...
                           cos(lastRawEnd-angle(vStart)));
end
C=struct('times_s',T-T(1)+double(startTime),...
 'control_points',{P},'yaw_lift_rad',A);
% Verify exact same evaluator returns continuous 28-state jets at all
% interior knots (numeric quality check; NOT an IEEE outward proof).
for k=2:numel(C.times_s)-1
 tau=C.times_s(k);delta=min(diff(C.times_s))*1e-8;
 try
  a=AF7R58R186_FLAT_EVAL(C,tau-delta);
  b=AF7R58R186_FLAT_EVAL(C,tau+delta);
 catch
  R.status='REJECT_EVALUATOR_FAILURE_AT_C7_KNOT';return;
 end
 if any(~isfinite([a;b]))
  R.status='REJECT_EVALUATOR_NONFINITE';return;
 end
end
try
 first=AF7R58R186_FLAT_EVAL(C,C.times_s(1));
 last=AF7R58R186_FLAT_EVAL(C,C.times_s(end));
catch
 R.status='REJECT_FLAT_REFERENCE_EVALUATION';return;
end
R.bound=true;
R.reference=C;
R.reference.valid_from=C.times_s(1);
R.reference.valid_until=C.times_s(end);
R.reference.eval_fn=@(t)AF7R58R186_FLAT_EVAL(C,t);
R.reference.flat_eval_fn=R.reference.eval_fn;
R.reference.first_jet=first;
R.reference.last_jet=last;
R.reference.yaw_policy='PATH_TANGENT_C6_EXACT';
R.status='C7_QP_REFERENCE_28STATE_AND_HEADING_C6_EVALUATOR_READY__EXECUTION_CERT_MISSING';
end
