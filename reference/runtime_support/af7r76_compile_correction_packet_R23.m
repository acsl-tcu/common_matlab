function P = af7r76_compile_correction_packet_R23(model,targetVersion)
% AF7R58ZZY-R23 compiled correction authority.
% Converts the already-certified canonical correction model into contiguous
% numeric arrays. No trajectory mathematics is changed.
%
% Derivative control points are generated ONCE with exactly the same
% recurrence used by af7r58i_correction_derivatives:
%   Q_{r+1} = ((15-r)/T) * diff(Q_r,1,2), r=0..5.
% Runtime evaluation then uses the same degree-(15-r) de Casteljau recursion.

assert(isstruct(model) && isfield(model,'schema') && ...
    strcmp(char(model.schema),'AF7R33_BATCH_FLAT_MODEL_V1') && ...
    isfield(model,'certified') && logical(model.certified) && ...
    isfield(model,'corrections') && isstruct(model.corrections), ...
    '[R23] canonical batch model is invalid.');
assert(isnumeric(targetVersion) && isscalar(targetVersion) && ...
    isfinite(targetVersion) && targetVersion>=0, ...
    '[R23] invalid target reference version.');

C=model.corrections;
G=numel(C);
S=0;
for g=1:G
    assert(isfield(C(g),'t0')&&isfield(C(g),'t1')&& ...
        isfield(C(g),'segment_t0')&&isfield(C(g),'segment_t1')&& ...
        isfield(C(g),'control_points'), ...
        '[R23] correction schema mismatch.');
    s0=double(C(g).segment_t0(:).');
    s1=double(C(g).segment_t1(:).');
    assert(~isempty(s0) && numel(s0)==numel(s1) && ...
        iscell(C(g).control_points) && numel(C(g).control_points)==numel(s0), ...
        '[R23] correction segment/control-point count mismatch.');
    S=S+numel(s0);
end

gt0=zeros(1,G);
gt1=zeros(1,G);
gfirst=zeros(1,G);
gcount=zeros(1,G);
s0all=zeros(1,S);
s1all=zeros(1,S);
% Fixed storage: [xyz, max 16 control points, derivative order 0..6, segment].
% For derivative order r, only columns 1:(16-r) are authoritative.
dcp=zeros(3,16,7,S);

cursor=1;
for g=1:G
    gt0(g)=double(C(g).t0);
    gt1(g)=double(C(g).t1);
    assert(isfinite(gt0(g))&&isfinite(gt1(g))&&gt1(g)>gt0(g), ...
        '[R23] correction support is invalid.');

    s0=double(C(g).segment_t0(:).');
    s1=double(C(g).segment_t1(:).');
    nS=numel(s0);
    gfirst(g)=cursor; % 1-based by contract
    gcount(g)=nS;

    for s=1:nS
        ta=s0(s); tb=s1(s); T=tb-ta;
        assert(isfinite(ta)&&isfinite(tb)&&isfinite(T)&&T>0, ...
            '[R23] correction segment duration is invalid.');
        Q=double(C(g).control_points{s});
        assert(isequal(size(Q),[3 16])&&all(isfinite(Q),'all'), ...
            '[R23] correction control points must be finite 3x16.');

        idx=cursor+s-1;
        s0all(idx)=ta;
        s1all(idx)=tb;
        dcp(:,:,1,idx)=Q;

        deg=15;
        for r=1:6
            % EXACT production derivative-control recurrence.
            Q=(deg/T)*diff(Q,1,2);
            deg=deg-1;
            assert(isequal(size(Q),[3 16-r])&&all(isfinite(Q),'all'), ...
                '[R23] derivative control-point construction failed.');
            dcp(:,1:(16-r),r+1,idx)=Q;
        end
    end
    cursor=cursor+nS;
end
assert(cursor==S+1,'[R23] compiled correction cursor mismatch.');

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
