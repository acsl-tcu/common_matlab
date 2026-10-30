function C=AF7R58R189_C7_LIVE_TRIAL(obj,t,mask,ev)
% Flight-event-bound, simulated C7 corridor/QP *candidate* (no commit).
% Uses actual known SO(3) ellipsoids, current reference jets and event time.
% Never asserts continuous-time closed-loop execution safety.
tstart=tic;
C=struct('status','UNKNOWN_LIVE_SOURCE','generated',false,...
 'c7_conditional',false,'heading_conditional',false,...
 'geometric_reference_sufficient',false,'physical_execution_certified',false,...
 'production_commit_authorized',false,'reference_version',NaN,...
 'time_s',double(t),'elapsed_ms',NaN,'amplitude_m',NaN,...
 'selected_side',NaN,'route',struct(),'binding',struct(),...
 'reason','NO_CERTIFIED_EXECUTION_TUBE');
try
 assert(isa(obj,'REPLANNING_SYSTEM') && isstruct(ev) && ...
    isfield(ev,'new_event')&&isequal(ev.new_event,true), ...
    'R189:WrongEvent');
 assert(isfinite(t)&&isfinite(ev.first_ttc_s) && ...
    ev.first_ttc_s>=0 && ev.first_ttc_s<=obj.horizon_s,...
    'R189:BadHorizon');
 C.reference_version=double(obj.active_reference_version);
 S=obj.r86_live_serialized_c7_authority(t);
 assert(isstruct(S) && logical(S.valid) && ...
    isequal(double(S.reference_version),C.reference_version) && ...
    isequal(size(S.active_jets_0_to_7),[3,8]),'R189:NoActiveC7');
 K=obj.obstacle_packet;mask=logical(mask(:).');
 assert(numel(mask)==numel(K.ids) && any(mask), 'R189:NoKnownSet');
 ids=find(mask);assert(all(isfinite(K.C(:))) && ...
   all(isfinite(K.Araw(:))) && all(K.Araw(:)>0), 'R189:InvalidEllipsoids');
 J0=double(S.active_jets_0_to_7);
 % Initial position and velocity are the SAME-TICK EKF observations, not
 % a fictitious future nominal state. Higher source jets are a proposal
 % only; no unproved physical C7 switch is ever committed.
 est=obj.self.estimator.result.state;
 initP=double(est.pL(:));initV=double(est.vL(:));
 assert(numel(initP)==3&&numel(initV)==3&&...
  all(isfinite([initP;initV])),'R189:InvalidSameTickEKF');
 % R190: raw reference lives in UAV-flat coordinates. The unchanged
 % SUSPENDED_LOAD_REF_ADJUST shifts only the commanded payload Z by -L.
 % At the physical EKF event, the raw starting target therefore must be
 % pL_EKF + L e3, never pL_EKF. No actual UAV/payload tautness is inferred.
 Lnow=double(obj.self.parameter.get("cableL"));
 assert(isscalar(Lnow)&&isfinite(Lnow)&&Lnow>0 && ...
        isequal(Lnow,double(obj.L_cable)), 'R190:UnboundCableLength');
 J0(:,1)=initP+[0;0;Lnow];J0(:,2)=initV;
 v=J0(1:2,2);normv=norm(v);
 assert(normv>0 && isfinite(normv),'R189:ZeroHorizontalSpeed');
 side=[-v(2);v(1);0]/normv;
 % Finite sensing horizon fixes trigger prediction only, not maneuver time.
 % Proposed duration uses spatial progress past *all known* ellipsoids.
 p0=double(J0(:,1));centers=double(K.C(:,ids));
 ahead=max(0, max((centers-p0)'*([v;0]/normv)));
 aMax=max(double(K.Araw(:,ids)),[],'all');
 % Distance necessary to pass the furthest obstacle and clear its body.
 travel=ahead + aMax + max(double([obj.r_drone,obj.r_load,obj.r_cable]));
 T=max(double(obj.horizon_s),2*travel/normv);
 % The unknown true closed-loop prediction is *not* used to certify geometry.
 T=max(T,4*double(obj.prediction_dt));
 ts=double(t)+[0,T/3,2*T/3,T];
 pos=zeros(3,4);J=cell(1,4);
 J{1}=J0;
 for k=2:4
   J{k}=double(obj.r63_nominal_position_jet_global(ts(k)));
   assert(isequal(size(J{k}),[3,8]) && all(isfinite(J{k}(:))), ...
      'R189:MissingNominalJets');
   pos(:,k)=J{k}(:,1);
 end
 pos(:,1)=J0(:,1);
 % A fixed side over all obstacles is a single coupled 3D corridor decision.
 % For each side this solves min (1/2) A^2 subject to A >= A_j >= 0:
 % the exact 1-dimensional convex QP solution is max_j A_j.
 % A_j depends on the true oriented-ellipsoid support, not axis-aligned radii.
 impactT=min(ts(3),max(ts(2),t+ev.first_ttc_s));
 xx=double(obj.r63_nominal_position_jet_global(impactT));xx=xx(:,1);
 rad=max([double(obj.r_drone),double(obj.r_load),double(obj.r_cable)])+...
     double(obj.d_margin);
 assert(isfinite(rad)&&rad>=0,'R189:BadBodyRadius');
 best=Inf;bestSign=NaN;
 for sg=[-1 1]
   n=sg*side;
   lower=0;
   for ii=ids
     rot=reshape(double(K.R(:,ii)),3,3);
     a=double(K.Araw(:,ii));c=double(K.C(:,ii));
     h=n'*c + norm(a.*(rot'*n));
     lower=max(lower,h+rad-n'*xx);
   end
   if lower<best,best=lower;bestSign=sg;end
 end
 amp=max(0,best);
 % Avoid impossible infinite/nonfinite amplitudes, but no arbitrary bound.
 assert(isfinite(amp) && isfinite(bestSign),'R189:NoQP');
 C.amplitude_m=amp;C.selected_side=bestSign;
 % Smooth lateral lane: 0, A, A, 0; all derivatives 1..7 of the
 % lane offset are zero at the three joints. Position jets are stitched
 % using degree-15 Hermite interpolation with the exact source jets.
 offset=double(bestSign)*amp*side;
 for k=2:3,J{k}(:,1)=J{k}(:,1)+offset;end
 B=cell(1,3);
 for k=1:3
   B{k}=r189_hermite_15(J{k},J{k+1},ts(k+1)-ts(k));
 end
 source=struct('times_s',ts,'control_points',{B});
 C.route=source;
 H=AF7R58R184_C7_HEADING_GATE(source);
 C.c7_conditional=logical(H.numeric_jets_C7_close);
 C.heading_conditional=logical(H.conditional_heading_C6);
 if ~C.c7_conditional || ~C.heading_conditional
    C.status=['REJECT_C7_OR_HEADING__' char(H.status)];C.elapsed_ms=1e3*toc(tstart);return;
 end
 % Exact support + convex hull = conditional reference-level sufficient
 % geometry for a *vertical straight-cable proxy*. The actual cable/payload
 % under HLC is NOT the same proxy, hence no execution certificate.
 G=AF7R58R189_REFERENCE_HULL(source,K,mask,rad,obj);
 C.geometric_reference_sufficient=G.sufficient;
 C.reference_hull=G;
 Q=AF7R58R186_BIND_C7(source,double(t));
 C.binding=Q;
 if ~Q.bound,C.status=['REJECT_R186_BINDING_' char(Q.status)];C.elapsed_ms=1e3*toc(tstart);return;end
 C.generated=true;
 C.status='SIM_EKF_INITIAL_C7_QP_GENERATED__NO_VALIDATED_HLC_REACH_OR_SPATIAL_RETURN';
catch ME
 C.status=['UNKNOWN_OR_REJECT_' char(ME.identifier)];
 C.reason=char(ME.message);
end
C.elapsed_ms=1e3*toc(tstart);
end

function B=r189_hermite_15(A,Z,h)
% Endpoint k-th derivatives k=0..7 in physical seconds, n=15.
assert(isequal(size(A),[3 8])&&isequal(size(Z),[3 8])&&...
 all(isfinite([A(:);Z(:)]))&&isfinite(h)&&h>0,'R189:BadJets');
B=zeros(3,16);
for k=0:7
 s=zeros(3,1);e=zeros(3,1);
 for j=0:k
  ff=prod(16-j:15); % falling(15,j), product([])=1 for j=0
  term=nchoosek(k,j)*h^j/ff;
  s=s+term*A(:,j+1);
  e=e+(-1)^j*term*Z(:,j+1);
 end
 B(:,k+1)=s;B(:,16-k)=e;
end
end
