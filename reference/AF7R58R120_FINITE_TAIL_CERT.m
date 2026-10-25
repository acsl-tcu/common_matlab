function C=AF7R58R120_FINITE_TAIL_CERT(A)
% Reference-only finite tail closure. An exact source-bound polynomial
% representation is REQUIRED; interpolation/samples cannot provide one.
% A.tail_segments are BEZIER representations obtained from the live source,
% with external proof of exact equivalence; this routine independently
% checks seamlessness, coverage and the full three-body geometry on ALL
% connector and tail segments. It does not certify actual HLC tracking.
req={'connector','tail_segments','tail_source_certificate', ...
 'finite_tail_end_s','nominal_source_domain_end_s', ...
 'nominal_shift_s','known_obstacles','live','radii_m', ...
 'certificate_nodes','reference_version'};
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
 'finite_tail_end_s',A.finite_tail_end_s,'segment_records',struct([]));
Q=A.connector;
assert(isstruct(Q)&&isfield(Q,'segments')&&isfield(Q,'times_s')&& ...
 isfield(Q,'knot_jets')&&isfield(Q,'num_segments')&& ...
 numel(Q.segments)==Q.num_segments && ...
 isequal(size(Q.knot_jets),[3 8 Q.num_segments+1]), ...
 'R120:ConnectorSchema');
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
C.nominal_source_identity_verified=true;
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
allSeg=[S T];N=numel(allSeg);
if N<1,C.status='BLOCKED_NO_SEGMENTS';return;end
v=double(A.radii_m(:));
assert(numel(v)==3&&all(isfinite(v))&&all(v>=0), ...
 'R120:LiveBodyRadii');
needed={'gravity_m_s2','cable_length_m','finite_sensing_vQ_limit_m_s'};
assert(all(isfield(A.live,needed)), 'R120:LiveDynamicsAuthority');
live=A.live;g=double(live.gravity_m_s2);L=double(live.cable_length_m);
maxerr=0;
log=repmat(struct('geometry',struct([]),'kinematic',struct([]), ...
 'heading',struct([]),'interval',[NaN NaN]),1,N);
for k=1:N
 seg=allSeg(k);
 if ~isstruct(seg)||~all(isfield(seg,{'t0','t1','payload_cp','duration_s'}))|| ...
   ~isequal(size(seg.payload_cp),[3 16]) || ...
   any(~isfinite(seg.payload_cp(:)))|| ...
   ~(seg.t1>seg.t0)|| ...
   abs(seg.t1-seg.t0-seg.duration_s)>64*eps(max(1,abs(seg.duration_s)))
   C.status='BLOCKED_INVALID_SEGMENT';C.first_bad_index=k;return;
 end
 if k>1
   before=allSeg(k-1);
   if abs(before.t1-seg.t0)>64*eps(max(1,abs(seg.t0)))
    C.status='BLOCKED_TIME_GAP_BETWEEN_SEGMENTS';C.first_bad_index=k;return;
   end
   Jl=jets(before.payload_cp,before.duration_s,false);
   Jr=jets(seg.payload_cp,seg.duration_s,true);
   % Equal jets up to numerical endpoint construction errors.
   % Scale every derivative by the natural knot time scale dt^d.
   dt=min(before.duration_s,seg.duration_s);
   relative=0;
   for d=0:7
       w=dt^d;
       a=Jl(:,d+1)*w;b=Jr(:,d+1)*w;
       tol=8192*eps(max([1;abs(a);abs(b)]));
       relative=max(relative,max(abs(a-b))/tol);
   end
   maxerr=max(maxerr,relative);
   if relative>1
      C.status='BLOCKED_C7_SEAM';C.first_bad_index=k;
      C.maximum_c7_seam_relative_error=maxerr;return;
   end
 end
 seg.trajectory_frame='HLC_PAYLOAD_WORLD';
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
C.status='FINITE_TAIL_CONDITIONAL_EXACT_SOURCE_REFERENCE_CERTIFIED';
C.finite_tail_reference_certified=true;
C.connector_and_tail_c7=true;
C.connector_and_tail_yaw_c6=true;
C.continuous_uav_cable_payload=true;
C.maximum_c7_seam_relative_error=maxerr;
C.segment_records=log;
end
function J=jets(P,T,start)
J=zeros(3,8);
for d=0:7
 if start,J(:,d+1)=P(:,1);else,J(:,d+1)=P(:,end);end
 if d<7,P=(size(P,2)-1)/T*diff(P,1,2);end
end
end
