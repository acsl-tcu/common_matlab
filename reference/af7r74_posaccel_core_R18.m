function [P0,A2,valid,T] = af7r74_posaccel_core_R18(P0,A2,tvec,model)
% AF7R58ZZY-R18 diagnostic-only exact correction r={0,2} evaluator.
% Numerical operations and ordering match af7r73_posaccel_core_R17.
% T contains lightweight tic/toc substage timings only.

T=zero_timing();
totalTic=tic;

assert(isstruct(model) && isfield(model,'certified') && logical(model.certified) && ...
    isfield(model,'schema') && strcmp(char(model.schema),'AF7R33_BATCH_FLAT_MODEL_V1') && ...
    isfield(model,'corrections') && isstruct(model.corrections), ...
    '[R18] invalid canonical batch model.');
assert(exist('af7r71_decasteljau_batch_mex','file')==3, ...
    '[R18] exact de Casteljau MEX unavailable.');

tvec=double(tvec(:).');
N=numel(tvec);
P0=double(P0); A2=double(A2);
assert(N>=1 && all(isfinite(tvec)) && ...
    isequal(size(P0),[3 N]) && isequal(size(A2),[3 N]) && ...
    all(isfinite(P0),'all') && all(isfinite(A2),'all'), ...
    '[R18] invalid nominal base batch.');
valid=true(1,N);

for g=1:numel(model.corrections)
    tg=tic;
    C=model.corrections(g);
    req={'t0','t1','segment_t0','segment_t1','control_points'};
    assert(all(isfield(C,req)), '[R18] correction schema invalid.');
    ct0=double(C.t0); ct1=double(C.t1);
    s0=double(C.segment_t0(:).');
    s1=double(C.segment_t1(:).');
    CP=C.control_points;
    assert(isscalar(ct0)&&isscalar(ct1)&&isfinite(ct0)&&isfinite(ct1)&&ct1>ct0, ...
        '[R18] correction support invalid.');
    assert(numel(s0)==numel(s1) && numel(CP)==numel(s0) && ~isempty(CP) && ...
        all(isfinite(s0)) && all(isfinite(s1)), ...
        '[R18] correction segment schema invalid.');
    T.schema_ms=T.schema_ms+1e3*toc(tg);

    tg=tic;
    active=find(tvec>ct0 & tvec<ct1);
    T.active_scan_ms=T.active_scan_ms+1e3*toc(tg);
    if isempty(active), continue; end

    tg=tic;
    seg=zeros(1,numel(active));
    for jj=1:numel(active)
        tt=tvec(active(jj));
        is=find(tt<=s1,1,'first');
        assert(~isempty(is), '[R18] active correction time outside segment authority.');
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
    T.segment_map_ms=T.segment_map_ms+1e3*toc(tg);

    for uu=1:numel(useSeg)
        is=useSeg(uu);
        tg=tic;
        idx=active(seg==is);
        ta=s0(is); tb=s1(is); dur=tb-ta;
        assert(isfinite(dur)&&dur>0, '[R18] correction segment duration invalid.');
        u=(tvec(idx)-ta)/dur;
        u=max(0,min(1,u));
        Q0=double(CP{is});
        assert(isequal(size(Q0),[3 16]) && all(isfinite(Q0),'all'), ...
            '[R18] correction control points invalid.');
        T.segment_setup_ms=T.segment_setup_ms+1e3*toc(tg);

        tg=tic;
        D0=af7r71_decasteljau_batch_mex(Q0,u);
        T.d0_mex_ms=T.d0_mex_ms+1e3*toc(tg);

        tg=tic;
        Q1=(15/dur)*diff(Q0,1,2);
        Q2=(14/dur)*diff(Q1,1,2);
        T.d2_control_ms=T.d2_control_ms+1e3*toc(tg);

        tg=tic;
        D2=af7r71_decasteljau_batch_mex(Q2,u);
        T.d2_mex_ms=T.d2_mex_ms+1e3*toc(tg);

        tg=tic;
        P0(:,idx)=P0(:,idx)+D0;
        A2(:,idx)=A2(:,idx)+D2;
        T.add_ms=T.add_ms+1e3*toc(tg);
        T.segment_count=T.segment_count+1;
        T.point_count=T.point_count+numel(idx);
    end
end

tg=tic;
assert(isequal(size(P0),[3 N]) && isequal(size(A2),[3 N]) && ...
    all(isfinite(P0),'all') && all(isfinite(A2),'all'), ...
    '[R18] reduced flat-jet evaluation produced invalid output.');
T.final_validate_ms=1e3*toc(tg);
T.total_ms=1e3*toc(totalTic);
T.accounted_ms=T.schema_ms+T.active_scan_ms+T.segment_map_ms+ ...
    T.segment_setup_ms+T.d0_mex_ms+T.d2_control_ms+T.d2_mex_ms+ ...
    T.add_ms+T.final_validate_ms;
T.residual_ms=T.total_ms-T.accounted_ms;
end

function T=zero_timing()
T=struct('schema_ms',0,'active_scan_ms',0,'segment_map_ms',0, ...
    'segment_setup_ms',0,'d0_mex_ms',0,'d2_control_ms',0, ...
    'd2_mex_ms',0,'add_ms',0,'final_validate_ms',0, ...
    'total_ms',0,'accounted_ms',0,'residual_ms',0, ...
    'segment_count',0,'point_count',0);
end
