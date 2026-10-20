function [P0,A2,valid] = af7r75_posaccel_core_R19(P0,A2,tvec,model)
% AF7R58ZZY-R19 exact correction r={0,2} evaluator.
% Segment mapping + derivative-control construction + de Casteljau are one
% strict-FP native call. Group accumulation remains in MATLAB, in the same
% group order and only on active columns, preserving R17 addition semantics.

assert(isstruct(model) && isfield(model,'certified') && logical(model.certified) && ...
    isfield(model,'schema') && strcmp(char(model.schema),'AF7R33_BATCH_FLAT_MODEL_V1') && ...
    isfield(model,'corrections') && isstruct(model.corrections), ...
    '[R19] invalid canonical batch model.');
assert(exist('af7r75_correction_posaccel_mex','file')==3, ...
    '[R19] native correction evaluator unavailable.');

tvec=double(tvec(:).');
N=numel(tvec);
P0=double(P0); A2=double(A2);
assert(N>=1 && all(isfinite(tvec)) && ...
    isequal(size(P0),[3 N]) && isequal(size(A2),[3 N]) && ...
    all(isfinite(P0),'all') && all(isfinite(A2),'all'), ...
    '[R19] invalid nominal base batch.');
valid=true(1,N);

G=numel(model.corrections);
if G>0
    [D0,D2,mask]=af7r75_correction_posaccel_mex(tvec,model.corrections);
    assert(isequal(size(D0),[3 N G]) || (G==1 && isequal(size(D0),[3 N])), ...
        '[R19] native position correction shape invalid.');
    assert(isequal(size(D2),[3 N G]) || (G==1 && isequal(size(D2),[3 N])), ...
        '[R19] native acceleration correction shape invalid.');
    assert(isequal(size(mask),[G N]), '[R19] native correction mask shape invalid.');
    if G==1
        D0=reshape(D0,3,N,1);
        D2=reshape(D2,3,N,1);
    end
    for g=1:G
        idx=find(mask(g,:));
        if isempty(idx), continue; end
        P0(:,idx)=P0(:,idx)+D0(:,idx,g);
        A2(:,idx)=A2(:,idx)+D2(:,idx,g);
    end
end

assert(isequal(size(P0),[3 N]) && isequal(size(A2),[3 N]) && ...
    all(isfinite(P0),'all') && all(isfinite(A2),'all'), ...
    '[R19] native reduced flat-jet evaluation produced invalid output.');
end
