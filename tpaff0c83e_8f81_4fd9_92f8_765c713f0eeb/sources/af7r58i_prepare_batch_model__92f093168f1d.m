function [ok,reason,model,meta] = af7r58i_prepare_batch_model(replanner,P,targetVersion,t_commit)
% PRUNE -> COUNT -> ORDINARY RETURN -> APPEND canonical resource gate.
assert(isa(replanner,'REPLANNING_SYSTEM'));
ok=false; reason=''; model=struct(); meta=struct();

if ~(isstruct(P)&&isfield(P,'representation_code')&&double(P.representation_code)==2)
    reason='AF7R58I_NONCANONICAL_REPRESENTATION';
    return;
end

req={'t0','t1','piecewise_segment_t0','piecewise_segment_t1','piecewise_control_points'};
if ~all(isfield(P,req))
    reason='AF7R58I_PROPOSAL_RESOURCE_FIELDS_MISSING';
    return;
end

pt0=double(P.t0);
pt1=double(P.t1);
t_commit=double(t_commit);
if ~(isscalar(pt0)&&isscalar(pt1)&&isscalar(t_commit)&&all(isfinite([pt0 pt1 t_commit]))&&pt1>pt0)
    reason='AF7R58I_PROPOSAL_TIME_INVALID';
    return;
end
if ~isequal(pt0,t_commit)
    reason='AF7R58I_T0_COMMIT_TIME_MISMATCH';
    return;
end

RC=af7r58i_resource_contract(replanner);
if ~RC.closed
    reason=['AF7R58I_RESOURCE_CONTRACT_OPEN__' RC.reason];
    meta=struct('contract',RC);
    return;
end
if pt0<RC.flight_t0 || pt1>RC.reference_tend
    reason='AF7R58I_CORRECTION_OUTSIDE_FINITE_REFERENCE_DOMAIN';
    meta=struct('contract',RC);
    return;
end

if strcmpi(char(replanner.active_reference_mode),'NOMINAL')
    if replanner.nominal_batch_model_cache_valid && ...
            isstruct(replanner.nominal_batch_model_cache) && ...
            isfield(replanner.nominal_batch_model_cache,'certified') && ...
            logical(replanner.nominal_batch_model_cache.certified) && ...
            double(replanner.nominal_batch_model_cache.reference_version)==double(replanner.active_reference_version)
        model=replanner.nominal_batch_model_cache;
    else
        model=af7r33_nominal_batch_model(replanner);
    end
elseif strcmpi(char(replanner.active_reference_mode),'REPAIRED')
    R=replanner.active_repaired_reference;
    if ~(isstruct(R)&&isfield(R,'batch_flat_model')&&isstruct(R.batch_flat_model)&& ...
            isfield(R.batch_flat_model,'certified')&&logical(R.batch_flat_model.certified))
        reason='AF7R58I_PREVIOUS_CANONICAL_MODEL_UNAVAILABLE';
        meta=struct('contract',RC);
        return;
    end
    model=R.batch_flat_model;
else
    reason='AF7R58I_UNSUPPORTED_ACTIVE_REFERENCE_MODE';
    meta=struct('contract',RC);
    return;
end

[model,npruned]=af7r58i_prune_batch_model_exact(model,pt0);

G0=numel(model.corrections);
S0=0;
for g=1:G0
    if ~isfield(model.corrections(g),'control_points')
        reason='AF7R58I_STORED_CORRECTION_SCHEMA_INVALID';
        meta=struct('contract',RC,'pruned_groups',npruned);
        return;
    end
    S0=S0+numel(model.corrections(g).control_points);
end

s0=double(P.piecewise_segment_t0(:).');
s1=double(P.piecewise_segment_t1(:).');
CP=P.piecewise_control_points;
Snew=numel(CP);
if ~(numel(s0)==numel(s1)&&Snew==numel(s0)&&Snew>=1)
    reason='AF7R58I_CANDIDATE_SEGMENT_SCHEMA_INVALID';
    meta=struct('contract',RC,'pruned_groups',npruned);
    return;
end
if Snew>RC.Sgroup_max
    reason='AF7R58I_MAX_SINGLE_GROUP_SEGMENTS';
    meta=struct('contract',RC,'pruned_groups',npruned,'G_before',G0,'S_before',S0,'S_new',Snew);
    return;
end

G1=G0+1;
S1=S0+Snew;
meta=struct('contract',RC,'pruned_groups',npruned, ...
    'G_before',G0,'S_before',S0,'G_after',G1,'S_after',S1,'candidate_segments',Snew);

if G1>RC.Gmax
    reason='AF7R58I_MAX_REPAIR_GROUPS';
    return;
end
if S1>RC.Smax
    reason='AF7R58I_MAX_REPAIR_SEGMENTS';
    return;
end

C=struct();
C.t0=pt0;
C.t1=pt1;
C.segment_t0=s0;
C.segment_t1=s1;
C.control_points=CP;

if ~(all(isfinite([C.t0 C.t1]))&&C.t1>C.t0&&all(isfinite(C.segment_t0))&&all(isfinite(C.segment_t1)))
    reason='AF7R58I_CANDIDATE_CORRECTION_NONFINITE';
    return;
end
for k=1:numel(C.control_points)
    cp=double(C.control_points{k});
    if ~(isequal(size(cp),[3 16])&&all(isfinite(cp(:))))
        reason='AF7R58I_CANDIDATE_CONTROL_POINTS_INVALID';
        return;
    end
end

model.corrections(end+1)=C;
model.reference_version=double(targetVersion);
model.certified=true;
model.schema='AF7R33_BATCH_FLAT_MODEL_V1';
model.resource_certificate=RC;
model.resource_state=meta;

ok=true;
reason='AF7R58I_CANONICAL_RESOURCE_APPEND_CLOSED';
end