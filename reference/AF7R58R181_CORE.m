function R=AF7R58R181_CORE(P,obstacles,radii,origin)
% R181 numerical current-EKF-anchored, actual HLC+plant-rollout collision
% witness. TEST/TRIGGER WARNING, not a closed-loop reachability certificate.
% UAV, load = independent balls. Cable = straight [pQ,pL] capsule.
% No L-radius sphere and NO global fmax*T^2 position inflation.
R=struct('status','UNKNOWN_INPUT','numeric_request',false, ...
 'first_obstacle_id',NaN,'first_component_code',0, ...
 'first_sample_index',NaN,'first_collision_time_s',NaN, ...
 'first_ttc_s',NaN,'numeric_min_margin_m',NaN, ...
 'checked_samples',0,'checked_obstacles',0, ...
 'sampled_preview_only',true,'continuous_time_collision_proved',false,...
 'continuous_time_clearance_proved',false,'execution_certified',false, ...
 'commit_authorized',false,'body_shape','BALL_CAPSULE_BALL', ...
 'used_length_sphere',false,'error_message','');
if ~isstruct(P)||~isscalar(P)||~all(isfield(P, ...
 {'t_s','Q_m','L_m','source_time_s','estimated_mass_kg','source_EKF_Q','source_EKF_L'})) || ...
 ~isstruct(origin)||~isscalar(origin)||~all(isfield(origin, ...
 {'t_s','estimated_mL_kg','pQ','pL','horizon_s'}))||~isnumeric(radii)||...
 numel(radii)~=3||~isstruct(obstacles),return;end
if ~(isscalar(origin.horizon_s) && double(origin.horizon_s)==4),return;end
t=double(P.t_s(:).');Q=double(P.Q_m);L=double(P.L_m);rad=double(radii(:));
if isequal(size(Q),[numel(t) 3]),Q=Q.';end
if isequal(size(L),[numel(t) 3]),L=L.';end
if numel(t)<2||~isequal(size(Q),[3 numel(t)])||~isequal(size(L),size(Q))||...
 any(~isfinite([t(:);Q(:);L(:);rad]))||any(rad<0)||any(diff(t)<=0),return;end
if abs(t(1)-double(origin.t_s))>32*eps(max(1,abs(t(1))))||...
 abs(t(end)-double(origin.t_s)-4)>256*eps(max([1;abs(t(:))]))||...
 ~isequal(double(P.source_time_s),double(origin.t_s))||...
 ~isequal(double(P.estimated_mass_kg),double(origin.estimated_mL_kg)),return;end
pQ=double(origin.pQ(:));pL=double(origin.pL(:));
if numel(pQ)~=3||numel(pL)~=3||any(~isfinite([pQ;pL])),return;end
% Roundoff check for the *same source*; not a cable tautness check.
v1=double(P.source_EKF_Q(:));v2=double(P.source_EKF_L(:));
if numel(v1)~=3||numel(v2)~=3 ||...
 any(abs(v1-pQ)>1024*eps(max([1;abs(pQ)]))) ||...
 any(abs(v2-pL)>1024*eps(max([1;abs(pL)]))) ||...
 any(abs(Q(:,1)-pQ)>1024*eps(max([1;abs(pQ)]))) ||...
 any(abs(L(:,1)-pL)>1024*eps(max([1;abs(pL)]))),return;end
if isempty(obstacles)
 R.status='NUMERIC_NO_KNOWN_OBSTACLE__NOT_GLOBAL_SAFETY';return;end
if ~all(isfield(obstacles,{'id','p_center','R_obs','ellipsoid_radii'})),return;end
ids=double([obstacles.id]);if any(~isfinite(ids))||numel(unique(ids))~=numel(ids),return;end
R.checked_obstacles=numel(obstacles);
R.numeric_min_margin_m=Inf;
% Search time FIRST, obstacle SECOND: earliest witnessed time independent
% of obstacle ordering; ties resolved using immutable original obstacle IDs.
for i=1:numel(t)
 R.checked_samples=R.checked_samples+1;
 hits=[];
 for j=1:numel(obstacles)
  o=obstacles(j);c=double(o.p_center(:));Rot=double(o.R_obs);
  a=double(o.ellipsoid_radii(:));
  if numel(c)~=3||numel(a)~=3||~isequal(size(Rot),[3 3])||...
    any(~isfinite([c;a;Rot(:)]))||any(a<=0)||...
    norm(Rot'*Rot-eye(3),'fro')>1e-7||det(Rot)<0.9
   R.status='UNKNOWN_BAD_ORIGINAL_OBSTACLE';return;
  end
  q=Q(:,i);l=L(:,i);
  % Exact containment in a spherical bound around this ellipsoid is used
  % ONLY to skip distant numerical distance evaluations. The actual cable
  % remains a finite-width line segment and never an L-radius ball.
  u=q-l;den=u'*u;
  if den>0,lam=min(1,max(0,(c-l)'*u/den));else,lam=0;end
  segpoint=l+lam*u;
  dQ=norm(q-c)-(max(a)+rad(1));
  dC=norm(segpoint-c)-(max(a)+rad(2));
  dL=norm(l-c)-(max(a)+rad(3));
  if min([dQ dC dL])>0
   R.numeric_min_margin_m=min(R.numeric_min_margin_m,min([dQ dC dL]));
   continue;
  end
  D=AF7R58R177_ELLIPSOID_SEGMENT(c,Rot,a,q,l,rad(1),rad(2),rad(3));
  if ~startsWith(char(D.status),'NUMERICAL_')||~isfinite(double(D.minimum_m))
   R.status='UNKNOWN_NUMERIC_ORIGINAL_SHAPE_EVALUATION';return;
  end
  R.numeric_min_margin_m=min(R.numeric_min_margin_m,double(D.minimum_m));
  if D.minimum_m<=0
   ds=[D.uav_m D.cable_m D.payload_m];
   [~,ix]=min(ds); % 1=UAV 2=CABLE 3=PAYLOAD
   native=[2 3 1];
   hits(end+1,:)=[double(o.id),native(ix)]; %#ok<AGROW>
  end
 end
 if ~isempty(hits)
  [~,ii]=min(hits(:,1));
  R.numeric_request=true;
  R.first_obstacle_id=hits(ii,1);
  R.first_component_code=hits(ii,2);
  R.first_sample_index=i;
  R.first_collision_time_s=t(i);
  R.first_ttc_s=max(0,t(i)-double(origin.t_s));
  R.status='NUMERIC_CURRENT_EKF_HLC_PREVIEW_CONTACT_WITNESS__NOT_PROOF';
  return;
 end
end
R.status='NUMERIC_CURRENT_EKF_HLC_PREVIEW_NO_CONTACT_SAMPLES__UNKNOWN_BETWEEN';
end
