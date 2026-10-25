function R=AF7R58R122_C7_TO_RAW_XD28(segment,tGlobal,cableL,yawAnchor)
% Emit historical 28-element PRE-SLOAD reference packet from a payload C7
% segment. Heading psi is path-tangent exact, with analytic jets 0..6.
% Apply +cableL to RAW z POSITION only; SLOAD will subtract L once.
% The caller must independently certify the entire trajectory, yaw tangent
% non-singularity, three-body safety and all segment seams. No commit here.
assert(isstruct(segment)&&all(isfield(segment, ...
    {'payload_cp','t0','t1','duration_s','trajectory_frame'})) && ...
    strcmp(char(segment.trajectory_frame),'HLC_PAYLOAD_WORLD') && ...
    isequal(size(segment.payload_cp),[3 16]), ...
    'R122:PayloadSegmentRequired','Expected certified-format C7 payload segment.');
assert(isnumeric(cableL)&&isscalar(cableL)&&isfinite(cableL)&&cableL>0, ...
    'R122:LiveCableLRequired','Supply current externally authoritative cableL.');
Y=AF7R58R118_YAW_JETS_C6(segment,double(tGlobal),double(yawAnchor));
P=double(Y.payload_jets07);
assert(isequal(size(P),[3 8]) && all(isfinite(P(:))) && ...
    numel(Y.yaw_jets06)==7 && all(isfinite(Y.yaw_jets06)), ...
    'R122:InvalidC7AndYawC6','Nonfinite payload/yaw analytic derivatives.');
raw28=zeros(28,1);
for r=0:6
    raw28(4*r+(1:3))=P(:,r+1);
    raw28(4*r+4)=Y.yaw_jets06(r+1);
end
raw28(3)=raw28(3)+double(cableL);
assert(all(isfinite(raw28))&&numel(raw28)==28, ...
    'R122:InvalidHlcPacket','HLC raw packet must have 28 finite elements.');
R=struct();
R.status='C7_PAYLOAD_TO_RAW_XD28__ADAPTER_ONLY_NOT_CERTIFICATE';
R.raw_xd28=raw28;
R.payload_jets07=P;
R.payload_yaw_jets06=double(Y.yaw_jets06(:).');
R.unwrapped_yaw_anchor_used=double(yawAnchor);
R.payload_height_after_sload_m=raw28(3)-double(cableL);
R.source_frame='HLC_PAYLOAD_WORLD';
R.consumer_frame='PRE_SUSPENDED_LOAD_REF_ADJUST_WORLD';
R.hlc_width=28;
R.production_commit_authorized=false;
R.heading_global_certification_required=true;
end
