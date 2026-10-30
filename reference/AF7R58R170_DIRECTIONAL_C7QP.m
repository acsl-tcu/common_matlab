function R=AF7R58R170_DIRECTIONAL_C7QP(route,J0,Jend,tseg,limits,packet,opts)
% Construction-stage C7-QP with exact directional support of the taut
% cable cone. Only bodies are balls of their own physical radii. The full
% cable is a STRAIGHT LINE SEGMENT with radius rC, never a sphere of L.
% Unknown live values or insufficient tube => fail closed. Separate source-
% obstacle/torque/yaw/HLC/return certificates are required for commit.
if nargin<7||isempty(opts),opts=struct();end
if ~isfield(opts,'robust_tube_on'),opts.robust_tube_on=true;end
R=struct('status','UNKNOWN_MISSING_AUTHORITY','qp_pass',false,'feasible',false,...
 'production_commit_authorized',false,'conditional_reference_geometry',false,...
 'used_cable_sphere_radius_L',false,'tube_present',false,'robust_tube_missing',false,...
 'q_cone_half_angle_rad',NaN,'face_margin_m',NaN,...
 'qp',struct(),'candidate',struct());
req={'live_physical_parameters_valid','source_cable_length_m',...
 'source_gravity_mps2','source_radii_m'};
if ~isstruct(packet)||~isscalar(packet)||~all(isfield(packet,req))||...
 ~isequal(packet.live_physical_parameters_valid,true)||...
 ~isstruct(route)||~isscalar(route)||~isfield(route,'cells')||...
 ~iscell(route.cells)||~isnumeric(limits)||numel(limits)~=4,return;end
L=double(packet.source_cable_length_m);g=double(packet.source_gravity_mps2);
r=double(packet.source_radii_m(:));lim=double(limits(:));
if numel(r)~=3||any(~isfinite([L;g;r;lim]))||...
 L<=0||g<=0||any(r<0)||any(lim<0),return;end
% acceleration CP L2 cap A implies F belongs to ball(g*e3,A).
% A>=g permits F=0 (slack/singular taut model): NEVER accept.
A=lim(2);
if A>=g,R.status='UNKNOWN_TAUT_CONE_SINGULAR_ACCEL_ENVELOPE';return;end
inflation=0;
if opts.robust_tube_on
 if ~isfield(packet,'certified_tracking_tube_m')||...
 ~isnumeric(packet.certified_tracking_tube_m)||...
 ~isscalar(packet.certified_tracking_tube_m)||...
 ~isfinite(packet.certified_tracking_tube_m)||...
 packet.certified_tracking_tube_m<0||...
 ~isfield(packet,'certified_tracking_tube_source')||...
 isempty(packet.certified_tracking_tube_source)
  R.robust_tube_missing=true;
  R.status='UNKNOWN_TRACKING_OBSERVER_TUBE_REQUIRED';return;
 end
 inflation=double(packet.certified_tracking_tube_m);R.tube_present=true;
end
inner=route;faceworst=Inf;
for k=1:numel(route.cells)
 cell0=route.cells{k};
 if ~isstruct(cell0)||~all(isfield(cell0,{'A','b'})),return;end
 H=double(cell0.A);b=double(cell0.b(:));
 if size(H,2)~=3||size(H,1)~=numel(b)||size(H,1)<4||...
 any(~isfinite([H(:);b(:)])),return;end
 [qu,~,ok,angle]=AF7R58R170_DIRECTIONAL_SUPPORT(H,[0;0;g],A);
 if ~ok,R.status='UNKNOWN_TAUT_DIRECTION_SUPPORT';return;end
 R.q_cone_half_angle_rad=angle;
 norms=sqrt(sum(H.^2,2));
 % Support of three occupied components, relative to pL:
 % payload: rL; UAV: L*q+rQ; cable: lambda*L*q+rC, lambda in [0,1].
 offset=max([norms*(r(3)+inflation),...
     L*qu+norms*(r(1)+inflation),...
     max(0,L*qu)+norms*(r(2)+inflation)],[],2);
 inner.cells{k}.b=b-offset;
 faceworst=min(faceworst,min(b-inner.cells{k}.b));
end
Q=AF7R58R168_CONSTRAINED_C7QP(inner,J0,Jend,tseg,lim);
R.qp=Q;
if ~isfield(Q,'feasible')||~Q.feasible
 R.status=['INCONCLUSIVE_DIRECTIONAL_FULLBODY_QP_' Q.status];return;
end
C=Q;C.feasible=true;
C.actuator_certified=false;C.uav_cable_certified=false;
C.yaw_C6_certified=false;C.tail_certified=false;
C.production_commit_authorized=false;
route.C7=C;
R.candidate=route;R.qp_pass=true;R.feasible=true;
R.conditional_reference_geometry=true;R.face_margin_m=faceworst;
R.status='CONDITIONAL_DIRECTIONAL_TAUT_C7QP__NO_FLIGHT_CERT';
end
