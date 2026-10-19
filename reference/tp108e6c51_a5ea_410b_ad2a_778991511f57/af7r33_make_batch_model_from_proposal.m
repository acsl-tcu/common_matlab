function model = af7r33_make_batch_model_from_proposal(replanner,P,targetVersion)
% Build the immutable flat model that will represent the reference after P.
% No object state is modified here.

assert(isa(replanner,'REPLANNING_SYSTEM'));
assert(isstruct(P)&&isfield(P,'representation_code')&& ...
    double(P.representation_code)==2, ...
    '[AF7R33] committed batch model requires piecewise representation_code=2.');

if strcmpi(char(replanner.active_reference_mode),'NOMINAL')
    model=af7r33_nominal_batch_model(replanner);
elseif strcmpi(char(replanner.active_reference_mode),'REPAIRED')
    R=replanner.active_repaired_reference;
    assert(isstruct(R)&&isfield(R,'batch_flat_model')&& ...
        isstruct(R.batch_flat_model)&&isfield(R.batch_flat_model,'certified')&& ...
        logical(R.batch_flat_model.certified), ...
        '[AF7R33] previous repaired reference lacks batch model.');
    model=R.batch_flat_model;
else
    error('[AF7R33] unsupported current active reference mode.');
end

C=struct();
C.t0=double(P.t0);
C.t1=double(P.t1);
C.segment_t0=double(P.piecewise_segment_t0(:).');
C.segment_t1=double(P.piecewise_segment_t1(:).');
C.control_points=P.piecewise_control_points;

assert(isfinite(C.t0)&&isfinite(C.t1)&&C.t1>C.t0);
assert(numel(C.segment_t0)==numel(C.segment_t1)&& ...
    numel(C.control_points)==numel(C.segment_t0)&&~isempty(C.control_points));
for k=1:numel(C.control_points)
    cp=double(C.control_points{k});
    assert(isequal(size(cp),[3 16])&&all(isfinite(cp),'all'));
end

model.corrections(end+1)=C;
model.reference_version=double(targetVersion);
model.certified=true;
model.schema='AF7R33_BATCH_FLAT_MODEL_V1';
end
