function [ok,reason,model,meta] = af7r80_fast_batch_append_R29( ...
        baseModel,P,CP,targetVersion,t_commit,RC)
% AF7R58ZZY-R29 exact canonical append using the already-closed R28 native
% representation invariant.
%
% This function removes repeated validation/counting of already-certified
% historical control points. It does NOT change the resource theorem.
%
% Authority:
%   - baseModel is the currently committed canonical model.
%   - CP is the R28 native compiled packet constructed from the same active
%     compiled base and the same new native PlanResult.
%   - CP prunes exactly old groups with t1<=t_commit.
%   - R28 install selftest established exact D0..D6 construction.
%
% Safety/resource acceptance remains:
%   exact prune -> count -> cap check -> append one certified candidate group.
ok=false; reason=''; model=struct(); meta=struct();

if ~(isstruct(baseModel)&&isfield(baseModel,'certified')&&logical(baseModel.certified)&& ...
        isfield(baseModel,'corrections')&&isstruct(baseModel.corrections))
    reason='R29_BASE_CANONICAL_MODEL_INVALID';
    return;
end
if ~(isstruct(P)&&isfield(P,'representation_code')&&double(P.representation_code)==2&& ...
        isfield(P,'r28_native_compiled_exact')&&logical(P.r28_native_compiled_exact))
    reason='R29_NATIVE_CANDIDATE_AUTHORITY_INVALID';
    return;
end
req={'t0','t1','piecewise_segment_t0','piecewise_segment_t1','piecewise_control_points'};
if ~all(isfield(P,req))
    reason='R29_CANDIDATE_RESOURCE_FIELDS_MISSING';
    return;
end
if ~(isstruct(CP)&&isfield(CP,'schema')&& ...
        strcmp(char(CP.schema),'AF7R58ZZY_R23_COMPILED_CORRECTION_V1')&& ...
        isfield(CP,'certified_numeric_representation')&& ...
        logical(CP.certified_numeric_representation))
    reason='R29_COMPILED_PACKET_INVALID';
    return;
end
if ~(isstruct(RC)&&isfield(RC,'closed')&&logical(RC.closed))
    reason='R29_RESOURCE_CONTRACT_OPEN';
    return;
end

pt0=double(P.t0); pt1=double(P.t1); t_commit=double(t_commit);
if ~(isscalar(pt0)&&isscalar(pt1)&&isscalar(t_commit)&& ...
        all(isfinite([pt0 pt1 t_commit]))&&pt1>pt0&&isequal(pt0,t_commit))
    reason='R29_PROPOSAL_TIME_INVALID';
    return;
end
if pt0<double(RC.flight_t0)||pt1>double(RC.reference_tend)
    reason='R29_CORRECTION_OUTSIDE_FINITE_REFERENCE_DOMAIN';
    return;
end
if double(CP.reference_version)~=double(targetVersion)
    reason='R29_COMPILED_PACKET_VERSION_MISMATCH';
    return;
end

% Exact same prune boundary as AF7R58I: discard t1<=t_commit.
model=baseModel;
Gbase=numel(model.corrections);
keep=true(1,Gbase);
for g=1:Gbase
    q=double(model.corrections(g).t1);
    if ~(isscalar(q)&&isfinite(q))
        reason='R29_STORED_CORRECTION_SCHEMA_INVALID';
        return;
    end
    if q<=pt0
        keep(g)=false;
    end
end
npruned=sum(~keep);
model.corrections=model.corrections(keep);
G0=numel(model.corrections);

s0=double(P.piecewise_segment_t0(:).');
s1=double(P.piecewise_segment_t1(:).');
CPC=P.piecewise_control_points;
Snew=numel(CPC);
if ~(numel(s0)==numel(s1)&&Snew==numel(s0)&&Snew>=1&& ...
        all(isfinite(s0))&&all(isfinite(s1))&&all(s1>s0))
    reason='R29_CANDIDATE_SEGMENT_SCHEMA_INVALID';
    return;
end

% The native compiled packet already performed finite 3x16 validation of
% every new control point before compiled_native_exact could become true.
% Bind the resource count to that exact packet rather than rewalking CP cells.
G1=G0+1;
if ~(isfield(CP,'group_count')&&isfield(CP,'segment_count')&& ...
        isfield(CP,'group_segments')&&isfield(CP,'group_t0')&& ...
        isfield(CP,'group_t1')&&isfield(CP,'group_first')&& ...
        isfield(CP,'segment_t0')&&isfield(CP,'segment_t1'))
    reason='R29_COMPILED_RESOURCE_METADATA_MISSING';
    return;
end
if double(CP.group_count)~=G1
    reason='R29_GROUP_COUNT_BINDING_FAILED';
    return;
end
S1=double(CP.segment_count);
if ~(isscalar(S1)&&isfinite(S1)&&S1==round(S1)&&S1>=Snew)
    reason='R29_SEGMENT_COUNT_BINDING_FAILED';
    return;
end
if double(CP.group_segments(end))~=Snew
    reason='R29_NEW_GROUP_SEGMENT_COUNT_BINDING_FAILED';
    return;
end
if ~isequal(typecast(double(CP.group_t0(end)),'uint64'),typecast(pt0,'uint64')) || ...
        ~isequal(typecast(double(CP.group_t1(end)),'uint64'),typecast(pt1,'uint64'))
    reason='R29_NEW_GROUP_SUPPORT_BINDING_FAILED';
    return;
end
first=round(double(CP.group_first(end)));
ix=first:(first+Snew-1);
if ~(numel(ix)==Snew&&first>=1&&ix(end)<=S1&& ...
        isequal(typecast(double(CP.segment_t0(ix)),'uint64'),typecast(s0,'uint64'))&& ...
        isequal(typecast(double(CP.segment_t1(ix)),'uint64'),typecast(s1,'uint64')))
    reason='R29_NEW_SEGMENT_SUPPORT_BINDING_FAILED';
    return;
end

if Snew>double(RC.Sgroup_max)
    reason='AF7R58I_MAX_SINGLE_GROUP_SEGMENTS';
    return;
end
if G1>double(RC.Gmax)
    reason='AF7R58I_MAX_REPAIR_GROUPS';
    return;
end
if S1>double(RC.Smax)
    reason='AF7R58I_MAX_REPAIR_SEGMENTS';
    return;
end
S0=S1-Snew;

C=struct('t0',pt0,'t1',pt1,'segment_t0',s0,'segment_t1',s1, ...
    'control_points',{CPC});
model.corrections(end+1)=C;
model.reference_version=double(targetVersion);
model.certified=true;
model.schema='AF7R33_BATCH_FLAT_MODEL_V1';

meta=struct('contract',RC,'pruned_groups',npruned, ...
    'G_before',G0,'S_before',S0,'G_after',G1,'S_after',S1, ...
    'candidate_segments',Snew);
model.resource_certificate=RC;
model.resource_state=meta;

ok=true;
reason='R29_FAST_CANONICAL_RESOURCE_APPEND_CLOSED';
end
