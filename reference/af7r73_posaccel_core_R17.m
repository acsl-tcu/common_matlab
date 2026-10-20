function [P0,A2,valid] = af7r73_posaccel_core_R17(P0,A2,tvec,model)
% AF7R58ZZY-R17 exact correction-only r={0,2} canonical batch evaluator.
% P0/A2 are exact nominal base columns on the same global query times.

assert(isstruct(model) && isfield(model,'certified') && logical(model.certified) && ...
    isfield(model,'schema') && strcmp(char(model.schema),'AF7R33_BATCH_FLAT_MODEL_V1') && ...
    isfield(model,'corrections') && isstruct(model.corrections), ...
    '[R17] invalid canonical batch model.');
assert(exist('af7r71_decasteljau_batch_mex','file')==3, ...
    '[R17] exact de Casteljau MEX unavailable.');

tvec=double(tvec(:).');
N=numel(tvec);
P0=double(P0); A2=double(A2);
assert(N>=1 && all(isfinite(tvec)) && ...
    isequal(size(P0),[3 N]) && isequal(size(A2),[3 N]) && ...
    all(isfinite(P0),'all') && all(isfinite(A2),'all'), ...
    '[R17] invalid nominal base batch.');
valid=true(1,N);

for g=1:numel(model.corrections)
    C=model.corrections(g);
    req={'t0','t1','segment_t0','segment_t1','control_points'};
    assert(all(isfield(C,req)), '[R17] correction schema invalid.');
    ct0=double(C.t0); ct1=double(C.t1);
    s0=double(C.segment_t0(:).');
    s1=double(C.segment_t1(:).');
    CP=C.control_points;
    assert(isscalar(ct0)&&isscalar(ct1)&&isfinite(ct0)&&isfinite(ct1)&&ct1>ct0, ...
        '[R17] correction support invalid.');
    assert(numel(s0)==numel(s1) && numel(CP)==numel(s0) && ~isempty(CP) && ...
        all(isfinite(s0)) && all(isfinite(s1)), ...
        '[R17] correction segment schema invalid.');

    active=find(tvec>ct0 & tvec<ct1);
    if isempty(active), continue; end

    seg=zeros(1,numel(active));
    for jj=1:numel(active)
        tt=tvec(active(jj));
        is=find(tt<=s1,1,'first');
        assert(~isempty(is), '[R17] active correction time outside segment authority.');
        if tt<s0(is)
            if is==1
                seg(jj)=0;
                continue;
            end
            is=is-1;
        end
        seg(jj)=is;
    end

    useSeg=unique(seg(seg>0),'stable');
    for uu=1:numel(useSeg)
        is=useSeg(uu);
        idx=active(seg==is);
        ta=s0(is); tb=s1(is); T=tb-ta;
        assert(isfinite(T)&&T>0, '[R17] correction segment duration invalid.');
        u=(tvec(idx)-ta)/T;
        u=max(0,min(1,u));
        Q0=double(CP{is});
        assert(isequal(size(Q0),[3 16]) && all(isfinite(Q0),'all'), ...
            '[R17] correction control points invalid.');

        D0=af7r71_decasteljau_batch_mex(Q0,u);
        Q1=(15/T)*diff(Q0,1,2);
        Q2=(14/T)*diff(Q1,1,2);
        D2=af7r71_decasteljau_batch_mex(Q2,u);

        P0(:,idx)=P0(:,idx)+D0;
        A2(:,idx)=A2(:,idx)+D2;
    end
end

assert(isequal(size(P0),[3 N]) && isequal(size(A2),[3 N]) && ...
    all(isfinite(P0),'all') && all(isfinite(A2),'all'), ...
    '[R17] reduced flat-jet evaluation produced invalid output.');
end
