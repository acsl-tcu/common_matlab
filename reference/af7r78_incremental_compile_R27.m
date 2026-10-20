function Pout = af7r78_incremental_compile_R27(baseCP,P,targetVersion,t_commit)
% AF7R58ZZY-R27 exact incremental compiled-correction builder.
% -------------------------------------------------------------------------
% Purpose:
%   Produce the exact AF7R58ZZY_R23_COMPILED_CORRECTION_V1 packet for the
%   canonical model that would result from:
%       exact prune at t_commit + append candidate P
%   without recompiling derivative control points for already-active groups.
%
% Exactness basis:
%   1) expired groups are pruned with the same rule: correction.t1 <= t_commit;
%   2) kept groups preserve canonical ascending group order and their already
%      bitwise-certified derivative-control-point arrays;
%   3) only the newly generated group is differentiated, using the already
%      validated bitwise-exact af7r77_compile_dcp_mex recurrence;
%   4) metadata/schema are identical to R23 compiled-correction authority.
%
% This helper is runtime implementation only.  It does not accept/reject a
% trajectory and it does not replace any continuous-time certificate.

assert(isnumeric(targetVersion)&&isscalar(targetVersion)&& ...
    isfinite(targetVersion)&&targetVersion>=0,'[R27] invalid target version.');
assert(isnumeric(t_commit)&&isscalar(t_commit)&&isfinite(t_commit), ...
    '[R27] invalid commit time.');
req={'t0','t1','piecewise_segment_t0','piecewise_segment_t1','piecewise_control_points'};
assert(isstruct(P)&&all(isfield(P,req)),'[R27] candidate compile fields missing.');
assert(isequal(double(P.t0),double(t_commit)), ...
    '[R27] candidate t0/commit mismatch.');

% --------------------------- base authority ------------------------------
if isempty(baseCP) || ~isstruct(baseCP) || ~isfield(baseCP,'schema')
    base=empty_packet(double(targetVersion)-1);
else
    base=baseCP;
    validate_packet(base);
    assert(double(base.reference_version)==double(targetVersion)-1, ...
        '[R27] base compiled reference version is not targetVersion-1.');
end

G0=double(base.group_count);
S0=double(base.segment_count);
assert(G0>=0&&S0>=0&&G0==numel(base.group_t0)&&G0==numel(base.group_t1)&& ...
    G0==numel(base.group_first)&&G0==numel(base.group_segments), ...
    '[R27] base compiled group metadata mismatch.');
assert(S0==numel(base.segment_t0)&&S0==numel(base.segment_t1), ...
    '[R27] base compiled segment metadata mismatch.');

keep=false(1,G0);
for g=1:G0
    keep(g)=double(base.group_t1(g))>double(t_commit); % exact AF7R58I prune complement
end
kg=find(keep);

% Gather kept segments in canonical order.  Do not recompute derivatives.
segKeep=[];
for kk=1:numel(kg)
    g=kg(kk);
    first=round(double(base.group_first(g)));
    nseg=round(double(base.group_segments(g)));
    assert(first>=1&&nseg>=1&&first+nseg-1<=S0, ...
        '[R27] base compiled group segment range invalid.');
    segKeep=[segKeep first:(first+nseg-1)]; %#ok<AGROW>
end

base_gt0=double(base.group_t0(kg));
base_gt1=double(base.group_t1(kg));
base_gc=double(base.group_segments(kg));
base_s0=double(base.segment_t0(segKeep));
base_s1=double(base.segment_t1(segKeep));
if isempty(segKeep)
    base_dcp=zeros(3,16,7,0);
else
    base_dcp=double(base.derivative_control_points(:,:,:,segKeep));
end

% --------------------------- new group -----------------------------------
s0=double(P.piecewise_segment_t0(:).');
s1=double(P.piecewise_segment_t1(:).');
CP=P.piecewise_control_points;
Snew=numel(s0);
assert(Snew>=1&&numel(s1)==Snew&&iscell(CP)&&numel(CP)==Snew, ...
    '[R27] candidate segment/control-point count mismatch.');
assert(isfinite(double(P.t0))&&isfinite(double(P.t1))&&double(P.t1)>double(P.t0), ...
    '[R27] candidate support invalid.');

cp0=zeros(3,16,Snew);
dur=zeros(1,Snew);
for k=1:Snew
    T=s1(k)-s0(k);
    Q=double(CP{k});
    assert(isfinite(s0(k))&&isfinite(s1(k))&&isfinite(T)&&T>0&& ...
        isequal(size(Q),[3 16])&&all(isfinite(Q),'all'), ...
        '[R27] candidate segment data invalid.');
    cp0(:,:,k)=Q;
    dur(k)=T;
end
assert(exist('af7r77_compile_dcp_mex','file')==3, ...
    '[R27] bitwise-exact derivative MEX unavailable.');
new_dcp=double(af7r77_compile_dcp_mex(cp0,dur));
assert(size(new_dcp,1)==3&&size(new_dcp,2)==16&& ...
    size(new_dcp,3)==7&&size(new_dcp,4)==Snew&&all(isfinite(new_dcp),'all'), ...
    '[R27] incremental derivative MEX output invalid.');

% --------------------------- exact concatenation --------------------------
Gk=numel(kg);
Sk=numel(segKeep);
G=Gk+1;
S=Sk+Snew;

gcount=[base_gc,double(Snew)];
gfirst=zeros(1,G);
cursor=1;
for g=1:G
    gfirst(g)=cursor;
    cursor=cursor+gcount(g);
end
assert(cursor==S+1,'[R27] incremental group cursor mismatch.');

Pout=struct();
Pout.schema='AF7R58ZZY_R23_COMPILED_CORRECTION_V1';
Pout.reference_version=double(targetVersion);
Pout.group_count=double(G);
Pout.segment_count=double(S);
Pout.group_t0=[base_gt0,double(P.t0)];
Pout.group_t1=[base_gt1,double(P.t1)];
Pout.group_first=gfirst;
Pout.group_segments=gcount;
Pout.segment_t0=[base_s0,s0];
Pout.segment_t1=[base_s1,s1];
Pout.derivative_control_points=cat(4,base_dcp,new_dcp);
Pout.max_derivative_order=6;
Pout.degree=15;
Pout.strict_support=true;
Pout.group_addition_order='ASCENDING_CANONICAL_GROUP_INDEX';
Pout.certified_numeric_representation=true;
validate_packet(Pout);
end

function P=empty_packet(version)
P=struct('schema','AF7R58ZZY_R23_COMPILED_CORRECTION_V1', ...
    'reference_version',double(version),'group_count',0,'segment_count',0, ...
    'group_t0',zeros(1,0),'group_t1',zeros(1,0), ...
    'group_first',zeros(1,0),'group_segments',zeros(1,0), ...
    'segment_t0',zeros(1,0),'segment_t1',zeros(1,0), ...
    'derivative_control_points',zeros(3,16,7,0), ...
    'max_derivative_order',6,'degree',15,'strict_support',true, ...
    'group_addition_order','ASCENDING_CANONICAL_GROUP_INDEX', ...
    'certified_numeric_representation',true);
end

function validate_packet(P)
assert(isstruct(P)&&isfield(P,'schema')&& ...
    strcmp(char(P.schema),'AF7R58ZZY_R23_COMPILED_CORRECTION_V1')&& ...
    isfield(P,'reference_version')&&isfield(P,'group_count')&& ...
    isfield(P,'segment_count')&&isfield(P,'group_t0')&&isfield(P,'group_t1')&& ...
    isfield(P,'group_first')&&isfield(P,'group_segments')&& ...
    isfield(P,'segment_t0')&&isfield(P,'segment_t1')&& ...
    isfield(P,'derivative_control_points')&& ...
    isfield(P,'certified_numeric_representation')&&logical(P.certified_numeric_representation), ...
    '[R27] compiled packet schema invalid.');
G=double(P.group_count); S=double(P.segment_count);
assert(G>=0&&S>=0&&G==floor(G)&&S==floor(S)&& ...
    numel(P.group_t0)==G&&numel(P.group_t1)==G&& ...
    numel(P.group_first)==G&&numel(P.group_segments)==G&& ...
    numel(P.segment_t0)==S&&numel(P.segment_t1)==S, ...
    '[R27] compiled packet metadata size invalid.');
D=double(P.derivative_control_points);
assert(size(D,1)==3&&size(D,2)==16&&size(D,3)==7&&size(D,4)==S&& ...
    all(isfinite(D),'all'),'[R27] compiled packet derivative array invalid.');
assert(double(P.degree)==15&&double(P.max_derivative_order)==6&& ...
    logical(P.strict_support)&&strcmp(char(P.group_addition_order), ...
    'ASCENDING_CANONICAL_GROUP_INDEX'), ...
    '[R27] compiled packet representation contract invalid.');
end
