function P = af7r77_compile_correction_packet_R25(model,targetVersion)
% AF7R58ZZY-R25 exact compiled-correction builder.
%
% Output schema and floating-point recurrence are intentionally identical to
% AF7R58ZZY-R23:
%   Q_{r+1}=((15-r)/T)*diff(Q_r,1,2), r=0..5.
%
% R25 only moves the derivative-control-point recurrence into a tiny MEX.
% Group/segment order, support bounds, metadata and zero padding are unchanged.

assert(isstruct(model) && isfield(model,'schema') && ...
    strcmp(char(model.schema),'AF7R33_BATCH_FLAT_MODEL_V1') && ...
    isfield(model,'certified') && logical(model.certified) && ...
    isfield(model,'corrections') && isstruct(model.corrections), ...
    '[R25] canonical batch model is invalid.');
assert(isnumeric(targetVersion) && isscalar(targetVersion) && ...
    isfinite(targetVersion) && targetVersion>=0, ...
    '[R25] invalid target reference version.');

C=model.corrections;
G=numel(C);
S=0;
for g=1:G
    assert(isfield(C(g),'t0')&&isfield(C(g),'t1')&& ...
        isfield(C(g),'segment_t0')&&isfield(C(g),'segment_t1')&& ...
        isfield(C(g),'control_points'), ...
        '[R25] correction schema mismatch.');
    s0=double(C(g).segment_t0(:).');
    s1=double(C(g).segment_t1(:).');
    assert(~isempty(s0) && numel(s0)==numel(s1) && ...
        iscell(C(g).control_points) && numel(C(g).control_points)==numel(s0), ...
        '[R25] correction segment/control-point count mismatch.');
    S=S+numel(s0);
end

gt0=zeros(1,G);
gt1=zeros(1,G);
gfirst=zeros(1,G);
gcount=zeros(1,G);
s0all=zeros(1,S);
s1all=zeros(1,S);
cp0=zeros(3,16,S);
duration=zeros(1,S);

cursor=1;
for g=1:G
    gt0(g)=double(C(g).t0);
    gt1(g)=double(C(g).t1);
    assert(isfinite(gt0(g))&&isfinite(gt1(g))&&gt1(g)>gt0(g), ...
        '[R25] correction support is invalid.');

    s0=double(C(g).segment_t0(:).');
    s1=double(C(g).segment_t1(:).');
    nS=numel(s0);
    gfirst(g)=cursor;
    gcount(g)=nS;

    for k=1:nS
        idx=cursor+k-1;
        ta=s0(k); tb=s1(k); T=tb-ta;
        assert(isfinite(ta)&&isfinite(tb)&&isfinite(T)&&T>0, ...
            '[R25] correction segment duration is invalid.');
        Q=double(C(g).control_points{k});
        assert(isequal(size(Q),[3 16])&&all(isfinite(Q),'all'), ...
            '[R25] correction control points must be finite 3x16.');
        s0all(idx)=ta;
        s1all(idx)=tb;
        duration(idx)=T;
        cp0(:,:,idx)=Q;
    end
    cursor=cursor+nS;
end
assert(cursor==S+1,'[R25] compiled correction cursor mismatch.');

if S==0
    dcp=zeros(3,16,7,0);
else
    assert(exist('af7r77_compile_dcp_mex','file')==3, ...
        '[R25] af7r77_compile_dcp_mex is unavailable.');
    dcp=af7r77_compile_dcp_mex(cp0,duration);
    dcp=double(dcp);
    assert(size(dcp,1)==3&&size(dcp,2)==16&&size(dcp,3)==7&&size(dcp,4)==S&& ...
        all(isfinite(dcp),'all'), ...
        '[R25] compiled derivative-control-point MEX output is invalid.');
end

P=struct();
P.schema='AF7R58ZZY_R23_COMPILED_CORRECTION_V1';
P.reference_version=double(targetVersion);
P.group_count=double(G);
P.segment_count=double(S);
P.group_t0=gt0;
P.group_t1=gt1;
P.group_first=gfirst;
P.group_segments=gcount;
P.segment_t0=s0all;
P.segment_t1=s1all;
P.derivative_control_points=dcp;
P.max_derivative_order=6;
P.degree=15;
P.strict_support=true;
P.group_addition_order='ASCENDING_CANONICAL_GROUP_INDEX';
P.certified_numeric_representation=true;
end
