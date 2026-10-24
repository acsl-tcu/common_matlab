function A=AF7R58R76_COLLISION_AUDIT(c,P)
% Event-bound, sample-grid physical/reference collision comparison.
% Static rotated enclosing ellipsoids and UAV / whole cable / payload.
% 4 seconds is ONLY a collision-trigger query horizon, not a maneuver time.
% A hit means intersection with a CONSERVATIVE inflated outer geometry.
% No-hit at the samples does NOT prove separation between samples.
A=struct('status','NOT_RUN','samples',0,'known_count',NaN, ...
    'physical_sample_outer_hit',false,'physical_first_time_s',NaN, ...
    'physical_first_obstacle_id',NaN,'physical_first_component','NONE', ...
    'min_outer_normalized_clearance',NaN, ...
    'reference_sample_outer_hit',false,'reference_first_time_s',NaN, ...
    'reference_first_obstacle_id',NaN,'reference_first_component','NONE', ...
    'existing_trigger_hit',false,'existing_trigger_ttc_s',NaN, ...
    'native_physical_nonseparation',NaN,'native_physical_jhit',NaN, ...
    'native_physical_obstacle_id',NaN,'native_physical_component_code',NaN, ...
    'native_reference_nonseparation',NaN,'native_reference_jhit',NaN, ...
    'native_reference_obstacle_id',NaN,'native_reference_component_code',NaN, ...
    'native_reference_matches_original',false, ...
    'packet_version_matches',false,'physical_sample_gap_max_m',NaN, ...
    'sampled_numeric_only',true,'inter_sample_safety_proved',false, ...
    'execution_limits_proved',false,'tension_limits_proved',false, ...
    'eligible_for_trigger_replacement',false,'error_message','');
if ~isstruct(c) || ~all(isfield(c,{'r76_packet','r76_known_mask', ...
        'r76_reference_pL','r76_reference_q','r76_original_trigger', ...
        'r76_radii_m','r76_d_margin_m'}))
    A.status='NO_SAME_EVENT_GEOMETRY__R75_SNAPSHOT_REQUIRES_NEW_FLIGHT';
    return;
end
assert(isstruct(P) && isfield(P,'t') && isfield(P,'pQ') && isfield(P,'pL'), ...
    '[R76] existing HLC+plant prediction is unavailable.');
t=double(P.t(:).'); pQ=double(P.pQ);pL=double(P.pL);
N=numel(t); L=double(c.cable_length_m);
assert(N>1 && size(pQ,1)==3 && size(pL,1)==3 && ...
    size(pQ,2)==N && size(pL,2)==N && ...
    all(isfinite([t(:);pQ(:);pL(:)])) && ...
    all(diff(t)>0) && isfinite(L) && L>0, ...
    '[R76] invalid same-event numeric flight geometry.');
assert(abs((t(end)-t(1))-double(c.horizon_s)) <= ...
    1e-8*max(1,double(c.horizon_s)) && ...
    abs(t(1)-double(c.time_s)) <= 1e-8*max(1,abs(t(1))), ...
    '[R76] predicted physical future does not match event 4-s horizon.');
B=c.r76_packet; mask=logical(c.r76_known_mask(:).');
req={'C','R','Araw','Apay','Auav','Acable','ids','reference_version', ...
    'delta_payload_m','delta_uav_m','delta_cable_m'};
assert(all(isfield(B,req)),'[R76] incomplete same-event obstacle packet.');
M=numel(B.ids);
assert(isequal(size(B.C),[3 M]) && isequal(size(B.R),[9 M]) && ...
    isequal(size(B.Araw),[3 M]) && isequal(size(B.Apay),[3 M]) && ...
    isequal(size(B.Auav),[3 M]) && isequal(size(B.Acable),[3 M]) && ...
    numel(mask)==M && all(isfinite([B.C(:);B.R(:);B.Araw(:); ...
      B.Apay(:);B.Auav(:);B.Acable(:);B.ids(:)])) && ...
    all([B.Araw(:);B.Apay(:);B.Auav(:);B.Acable(:)]>0), ...
    '[R76] obstacle packet shape/geometry inconsistent.');
A.packet_version_matches=isequal(double(B.reference_version), ...
    double(c.reference_version));
assert(A.packet_version_matches, ...
    '[R76] obstacle packet references a different trajectory version.');
qref=double(c.r76_reference_q);pref=double(c.r76_reference_pL);
assert(isequal(size(pref),[3 N]) && isequal(size(qref),[3 N]) && ...
    all(isfinite([pref(:);qref(:)])), ...
    '[R76] same-event original future reference packet mismatch.');
A.samples=N; A.known_count=nnz(mask);
A.existing_trigger_hit=logical(c.r76_original_trigger);
A.existing_trigger_ttc_s=double(c.r76_original_ttc_s);
A.physical_sample_gap_max_m=max(abs(sqrt(sum((pQ-pL).^2,1))-L));
% Exact signed normalized radial OUTER-ellipsoid clearance. For the cable,
% minimize the full segment quadratic, not a finite set of cable points.
[hp,jp,ip,kp,dp]=r76_sample_gate(pQ,pL,t,B,mask);
prQ=pref-L*qref;
[hr,jr,ir,kr,~]=r76_sample_gate(prQ,pref,t,B,mask);
A.physical_sample_outer_hit=hp; A.reference_sample_outer_hit=hr;
A.min_outer_normalized_clearance=dp;
if hp
    A.physical_first_time_s=t(jp)-t(1);
    A.physical_first_obstacle_id=double(B.ids(ip));
    A.physical_first_component=kp;
end
if hr
    A.reference_first_time_s=t(jr)-t(1);
    A.reference_first_obstacle_id=double(B.ids(ir));
    A.reference_first_component=kr;
end
% Same old production support-plane kernel, queried independently on
% physical numeric rollout and on the exact earlier reference packet.
% Do NOT mistake either negative result for continuous-time certification.
if exist('g26_future_trigger_kernel_mex','file')==3
    d=double(c.r76_radii_m(:))+double(c.r76_d_margin_m)+ ...
       double([B.delta_payload_m;B.delta_uav_m;B.delta_cable_m]);
    assert(numel(d)==3 && all(isfinite(d)) && all(d>=0), ...
        '[R76] invalid event physical protection authority.');
    dt=double(c.prediction_dt); H=double(c.horizon_s);
    [nh,nj,ni,nk]=g26_future_trigger_kernel_mex(H,dt,L, ...
        pL,(pL-pQ)/L,double(B.C),double(B.R),double(B.Araw), ...
        d,double(B.ids),double(mask));
    [rh,rj,ri,rk]=g26_future_trigger_kernel_mex(H,dt,L, ...
        pref,qref,double(B.C),double(B.R),double(B.Araw), ...
        d,double(B.ids),double(mask));
    A.native_physical_nonseparation=logical(nh);
    A.native_physical_jhit=double(nj);
    A.native_physical_obstacle_id=double(ni);
    A.native_physical_component_code=double(nk);
    A.native_reference_nonseparation=logical(rh);
    A.native_reference_jhit=double(rj);
    A.native_reference_obstacle_id=double(ri);
    A.native_reference_component_code=double(rk);
    A.native_reference_matches_original=logical(rh)==logical(c.r76_original_trigger);
    if isfield(c,'r76_was_stage_latched') && logical(c.r76_was_stage_latched)
        % Old trigger is held from an earlier cycle, not a new MEX call.
        A.native_reference_matches_original=false;
        A.status='SAME_EVENT_PIPELINE_LATCHED__NOT_A_NEW_REFERENCE_KERNEL_CHECK';
    elseif ~A.native_reference_matches_original
        A.status='SAME_EVENT_REFERENCE_TRIGGER_DISAGREEMENT__INVESTIGATE';
    else
        A.status='SAME_EVENT_REFERENCE_VS_EKF_HLC_PLANT_SAMPLED__NO_CONTINUOUS_PROOF';
    end
else
    A.status='SAME_EVENT_SAMPLED_GEOMETRY_ONLY__NATIVE_MEX_NOT_AVAILABLE';
end
end

function [hit,jhit,ihit,khit,minMargin]=r76_sample_gate(pQ,pL,t,B,mask)
hit=false;jhit=NaN;ihit=NaN;khit='NONE';minMargin=Inf;
for j=1:numel(t)
  for i=find(mask)
    R=reshape(double(B.R(:,i)),3,3); c=double(B.C(:,i));
    assert(norm(R.'*R-eye(3),'fro')<1e-8 && abs(det(R)-1)<1e-8,...
       '[R76] invalid obstacle orientation.');
    azQ=double(B.Auav(:,i)); azL=double(B.Apay(:,i));
    azC=double(B.Acable(:,i));
    zQ=R.'*(pQ(:,j)-c); zL=R.'*(pL(:,j)-c);
    mq=sqrt(sum((zQ./azQ).^2))-1;
    ml=sqrt(sum((zL./azL).^2))-1;
    u=zQ./azC; v=zL./azC; d=v-u; dd=sum(d.^2);
    lam=0;if dd>0,lam=max(0,min(1,-dot(u,d)/dd));end
    mc=norm(u+lam*d)-1;
    [margin,k]=min([ml,mq,mc]);
    minMargin=min(minMargin,margin);
    if ~hit && margin<=0
      hit=true;jhit=j;ihit=i;
      labels={'PAYLOAD','UAV','CABLE'};khit=labels{k};
    end
  end
end
if isinf(minMargin),minMargin=NaN;end
end
