function G=AF7R58R179_TAUT_PLANE(H,P,qLower,qUpper,L,radii)
% The SAME spherical UAV + straight finite-width cable + spherical load
% contract applied to the planned ideal taut trajectory.
% q_up=(payload_acceleration+g*e3)/norm(...) points LOAD->UAV.
% pQ=pL+L*q_up, pT (model UAV->LOAD) = -q_up.
% For every face n, lower/upper bounds of n*x for each component.
% Columns: [UAV, CABLE, PAYLOAD] from independent radii [rQ;rC;rL].
% Directional q bounds are certified over a WHOLE interval, not samples.
G=struct('valid',false,'status','UNKNOWN_TAUT_FACE_INPUT',...
 'lower',[],'upper',[],...
 'cable_is_segment_capsule',true,'uses_length_radius_sphere',false);
if ~isnumeric(H)||size(H,2)~=3||~isnumeric(P)||size(P,1)~=3||...
 isempty(P)||~isnumeric(L)||~isscalar(L)||...
 ~isfinite(L)||L<=0||numel(radii)~=3,return;end
H=double(H);P=double(P);r=double(radii(:));
if any(~isfinite([H(:);P(:);r]))||any(r<0),return;end
m=size(H,1);lo=nan(m,1);hi=nan(m,1);
if ~isempty(qLower),lo=double(qLower(:));end
if ~isempty(qUpper),hi=double(qUpper(:));end
if numel(lo)~=m||numel(hi)~=m||...
 any(~isfinite(lo(~isnan(lo))))||any(~isfinite(hi(~isnan(hi)))),return;end
nrm=sqrt(sum(H.^2,2));a=H*P;
loP=min(a,[],2);hiP=max(a,[],2);
G.lower=[loP+L*lo-r(1)*nrm,...
         loP+min(0,L*lo)-r(2)*nrm,...
         loP-r(3)*nrm];
G.upper=[hiP+L*hi+r(1)*nrm,...
         hiP+max(0,L*hi)+r(2)*nrm,...
         hiP+r(3)*nrm];
G.valid=true;G.status='REAL_ARITHMETIC_TAUT_FACE_BALL_CAPSULE_BOUNDS';
end
