function C=AF7R58R121_FINITE_TAIL_CERT(A)
% Reference-only finite tail closure. An exact source-bound polynomial
% representation is REQUIRED; interpolation/samples cannot provide one.
% A.tail_segments and A.tail_knot_jets come from the SAME source-bound
% degree-15 Hermite representation. All seams must share exactly identical
% 0..7 endpoint jets; high-order finite differencing of rounded CP is NOT
% used to decide formal C7 continuity.
% A.tail_segments are BEZIER representations obtained from the live source,
% with external proof of exact equivalence; this routine independently
% checks seamlessness, coverage and the full three-body geometry on ALL
% connector and tail segments. It does not certify actual HLC tracking.
req={'connector','tail_segments','tail_source_certificate', ...
 'finite_tail_end_s','nominal_source_domain_end_s', ...
 'nominal_shift_s','known_obstacles','live','radii_m', ...
 'certificate_nodes','reference_version','tail_knot_jets'};
for k=1:numel(req)
 assert(isfield(A,req{k}),'R120:TailSourceAuthority', ...
  'Missing %s.',req{k});
end
C=struct('status','INCONCLUSIVE_TAIL_NOT_CERTIFIED', ...
 'finite_tail_reference_certified',false, ...
 'connector_and_tail_c7',false,'connector_and_tail_yaw_c6',false, ...
 'continuous_uav_cable_payload',false,'nominal_source_identity_verified',false, ...
 'execution_certified',false,'production_commit_authorized',false, ...
 'first_bad_part','', 'first_bad_index',NaN, ...
 'maximum_c7_seam_relative_error',NaN, ...
 'external_source_attestation_present',false, ...
 'independent_source_proof_verified',false, ...
 'ideal_shared_jets_c7',false, ...
 'c7_basis','DEGREE15_HERMITE_SHARED_KNOT_JETS', ...
 'max_hermite_cp_residual_m',NaN, ...
 'max_hermite_cp_allowed_m',NaN, ...
 'finite_tail_end_s',A.finite_tail_end_s,'segment_records',struct([]));
Q=A.connector;
assert(isstruct(Q)&&isfield(Q,'segments')&&isfield(Q,'times_s')&& ...
 isfield(Q,'knot_jets')&&isfield(Q,'num_segments')&& ...
 numel(Q.segments)==Q.num_segments && ...
 isequal(size(Q.knot_jets),[3 8 Q.num_segments+1]), ...
 'R121:ConnectorSchema','Connector schema invalid.');
source=A.tail_source_certificate;
requiredSource={'verified','reference_version','exact_for_entire_domain', ...
 'source_kind','finite_domain_end_s'};
if ~isstruct(source)||~all(isfield(source,requiredSource))|| ...
 ~isequal(source.verified,true)|| ...
 ~isequal(source.exact_for_entire_domain,true)|| ...
 ~isequal(source.reference_version,A.reference_version)|| ...
 ~strcmp(char(source.source_kind),'EXACT_SOURCE_BOUND_BERNSTEIN')|| ...
 ~isequal(source.finite_domain_end_s,A.nominal_source_domain_end_s)|| ...
 ~isequal(A.nominal_source_domain_end_s+A.nominal_shift_s, ...
            A.finite_tail_end_s)
 C.status='BLOCKED_NO_EXACT_SOURCE_EQUIVALENCE_PROOF';return;
end
C.nominal_source_identity_verified=true; % means trusted caller attestation, NOT independent source audit
C.external_source_attestation_present=true;
S=Q.segments;
T=A.tail_segments;
if ~isstruct(T),C.status='BLOCKED_INVALID_TAIL_SCHEMA';return;end
if isempty(T)
 if Q.times_s(end)~=A.finite_tail_end_s
    C.status='BLOCKED_TAIL_HAS_UNCOVERED_TIME';return;
 end
else
 if abs(T(1).t0-Q.times_s(end))>64*eps(max(1,abs(Q.times_s(end))))|| ...
   abs(T(end).t1-A.finite_tail_end_s)>64*eps(max(1,abs(A.finite_tail_end_s)))
   C.status='BLOCKED_TAIL_TIME_COVERAGE';return;
 end
end
% Require the exact source-tail representation's knot jets. Geometry
% samples alone can never certify all the required derivative seams.
TJ=A.tail_knot_jets;
if ~isnumeric(TJ) || size(TJ,1)~=3 || size(TJ,2)~=8 || ...
   size(TJ,3)~=numel(T)+1 || any(~isfinite(TJ(:)))
 C.status='BLOCKED_TAIL_KNOT_JETS_MISSING_OR_INVALID';return;
end
allSeg=[S T];N=numel(allSeg);
if N<1,C.status='BLOCKED_NO_SEGMENTS';return;end
% The canonical piecewise curve is EXACTLY the degree-15 Hermite polynomial
% of each (start,end) jet pair (in ideal real arithmetic). Continuity is an
% algebraic consequence of ONE shared knot jet; finite differences of
% rounded cp values are unsuitable for 7th-derivative acceptance.
JQ=Q.knot_jets;
if ~isequal(JQ(:,:,end),TJ(:,:,1))
 C.status='BLOCKED_EXACT_C7_JET_MISMATCH';
 C.first_bad_part='CONNECTOR_TO_TAIL';C.first_bad_index=numel(S)+1;return;
end
JJ=cat(3,JQ,TJ(:,:,2:end));
maxCP=0;maxBound=0;
for k=1:N
 seg=allSeg(k);jk=JJ(:,:,k);jk1=JJ(:,:,k+1);
 if ~isstruct(seg) || ~all(isfield(seg,{'t0','t1','payload_cp','duration_s'})) || ...
    ~isequal(size(seg.payload_cp),[3 16]) || ...
    any(~isfinite(seg.payload_cp(:))) || ...
    ~isfield(seg,'trajectory_frame') || ...
    ~strcmp(char(seg.trajectory_frame),'HLC_PAYLOAD_WORLD') || ...
    ~(seg.t1>seg.t0) || ...
    abs(seg.t1-seg.t0-seg.duration_s)>64*eps(max(1,abs(seg.duration_s)))
    C.status='BLOCKED_INVALID_SEGMENT';C.first_bad_index=k;return;
 end
 expected=r121_hermite(jk,jk1,seg.duration_s);
 % Small representational mismatch is a diagnostic of floating Hermite
 % coefficient construction, NOT a proof of IEEE754 interval enclosure.
 % Scale the comparison by the actual Bernstein coefficient construction.
 scale=max([1; abs(expected(:)); abs(seg.payload_cp(:))]);
 bound=1024*eps(scale);
 err=max(abs(expected(:)-seg.payload_cp(:)));
 maxCP=max(maxCP,err);maxBound=max(maxBound,bound);
 if ~(isfinite(err)&&err<=bound)
   C.status='BLOCKED_SEGMENT_NOT_SOURCE_JET_HERMITE';
   C.first_bad_index=k;
   if k<=numel(S),C.first_bad_part='CONNECTOR';
   else,C.first_bad_part='TAIL';end
   C.max_hermite_cp_residual_m=maxCP;
   C.max_hermite_cp_allowed_m=maxBound;return;
 end
 if k>1
    previous=allSeg(k-1);
    if abs(previous.t1-seg.t0)>64*eps(max(1,abs(seg.t0)))
      C.status='BLOCKED_TIME_GAP_BETWEEN_SEGMENTS';C.first_bad_index=k;return;
    end
 end
end
C.ideal_shared_jets_c7=true;
C.max_hermite_cp_residual_m=maxCP;
C.max_hermite_cp_allowed_m=maxBound;
v=double(A.radii_m(:));
assert(numel(v)==3&&all(isfinite(v))&&all(v>=0), ...
 'R121:LiveBodyRadii','Invalid live body radii.');
needed={'gravity_m_s2','cable_length_m','finite_sensing_vQ_limit_m_s'};
assert(all(isfield(A.live,needed)), 'R121:LiveDynamicsAuthority', 'Missing live dynamics.');
live=A.live;g=double(live.gravity_m_s2);L=double(live.cable_length_m);
% For every segment, the structural jet identity was already checked.
% The numerical Bernstein CP still undergoes independent all-time checks.
maxerr=0;
log=repmat(struct('geometry',struct([]),'kinematic',struct([]), ...
 'heading',struct([]),'interval',[NaN NaN]),1,N);
for k=1:N
 seg=allSeg(k);
 % Frame was validated above; never rewrite source frame metadata.
 G=AF7R58R80_CONTINUOUS_SEPARATE_BODY_CERT( ...
   seg,A.known_obstacles,g,L,v(1),v(2),v(3),A.certificate_nodes);
 K=AF7R58R91_STABLE_KINEMATIC_CERT( ...
   seg,g,L,live.finite_sensing_vQ_limit_m_s,A.certificate_nodes);
 Y=AF7R58R118_HEADING_C6_CERT(seg,A.certificate_nodes);
 log(k)=struct('geometry',G,'kinematic',K,'heading',Y, ...
     'interval',[seg.t0 seg.t1]);
 if ~(G.certified && K.certified && Y.certified)
    C.status='INCONCLUSIVE_REFERENCE_CERTIFICATE';
    C.first_bad_index=k;
    if k<=numel(S),C.first_bad_part='CONNECTOR';
    else,C.first_bad_part='TAIL';end
    C.segment_records=log(1:k);return;
 end
end
C.status='FINITE_TAIL_CONDITIONAL_EXACT_SOURCE_REFERENCE_CERTIFIED_C7_SHARED_JETS';
C.finite_tail_reference_certified=true;
C.connector_and_tail_c7=true;
C.connector_and_tail_yaw_c6=true;
C.continuous_uav_cable_payload=true;
C.maximum_c7_seam_relative_error=maxerr;
C.segment_records=log;
end
function P=r121_hermite(A,B,T)
% Same degree-15 endpoint-jet Hermite transform used by R92 and R120.
P=zeros(size(A,1),16);
for j=0:7
 u=zeros(size(A,1),1);v=u;
 for k=0:j
    falling=prod(15:-1:(16-k));
    z=nchoosek(j,k)*T^k/falling;
    u=u+z*A(:,k+1);
    v=v+(-1)^k*z*B(:,k+1);
 end
 P(:,j+1)=u;P(:,16-j)=v;
end
end
