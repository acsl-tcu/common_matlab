function [P0,A2,valid] = af7r72_posaccel_core_R16(flat0,flat2,tf0,tvec,model)
% AF7R58ZZY-R16-R2 exact r={0,2} restriction of canonical AF7R33 flat-jet algebra.
% No approximation, interpolation, or sampled acceptance.

assert(isa(flat0,'function_handle')&&isa(flat2,'function_handle'), ...
    '[R16] flat derivative function authority unavailable.');
assert(isstruct(model) && isfield(model,'certified') && logical(model.certified) && ...
    isfield(model,'schema') && strcmp(char(model.schema),'AF7R33_BATCH_FLAT_MODEL_V1') && ...
    isfield(model,'flight_reference_time_origin') && ...
    isfield(model,'corrections') && isstruct(model.corrections), ...
    '[R16] invalid canonical batch model.');
assert(exist('af7r71_decasteljau_batch_mex','file')==3, ...
    '[R16] exact de Casteljau MEX unavailable.');

tvec=double(tvec(:).');
N=numel(tvec);
assert(N>=1 && all(isfinite(tvec)), '[R16] invalid query times.');
tf0=double(tf0);
assert(isscalar(tf0)&&isfinite(tf0) && ...
    isequal(tf0,double(model.flight_reference_time_origin)), ...
    '[R16] Flight time-origin authority mismatch.');

tau=tvec-tf0;
tau(tau<0)=0;
P0=evaluate_triplet_scalar_batch_r16(flat0,tau,N);
A2=evaluate_triplet_scalar_batch_r16(flat2,tau,N);
valid=true(1,N);

for g=1:numel(model.corrections)
    C=model.corrections(g);
    req={'t0','t1','segment_t0','segment_t1','control_points'};
    assert(all(isfield(C,req)), '[R16] correction schema invalid.');
    ct0=double(C.t0); ct1=double(C.t1);
    s0=double(C.segment_t0(:).');
    s1=double(C.segment_t1(:).');
    CP=C.control_points;
    assert(isscalar(ct0)&&isscalar(ct1)&&isfinite(ct0)&&isfinite(ct1)&&ct1>ct0, ...
        '[R16] correction support invalid.');
    assert(numel(s0)==numel(s1) && numel(CP)==numel(s0) && ~isempty(CP) && ...
        all(isfinite(s0)) && all(isfinite(s1)), ...
        '[R16] correction segment schema invalid.');

    active=find(tvec>ct0 & tvec<ct1);
    if isempty(active), continue; end

    % Match canonical scalar segment selection exactly at segment joins.
    seg=zeros(1,numel(active));
    for jj=1:numel(active)
        tt=tvec(active(jj));
        is=find(tt<=s1,1,'first');
        assert(~isempty(is), '[R16] active correction time outside segment authority.');
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
        assert(isfinite(T)&&T>0, '[R16] correction segment duration invalid.');
        u=(tvec(idx)-ta)/T;
        u=max(0,min(1,u));
        Q0=double(CP{is});
        assert(isequal(size(Q0),[3 16]) && all(isfinite(Q0),'all'), ...
            '[R16] correction control points invalid.');

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
    '[R16] reduced flat-jet evaluation produced invalid output.');
end

function X=evaluate_triplet_scalar_batch_r16(f,tau,N)
% The nominal symbolic matlabFunction handles are scalar-time authorities.
% Some valid handles (e.g. @(t)[0;0;1.5*t+3]) cannot accept a vector t
% because MATLAB vertical concatenation would mix scalar and vector rows.
% Evaluate exactly as the canonical scalar evaluator does: one time at a time.
assert(isa(f,'function_handle'),'[R16-R2] nominal flat derivative handle invalid.');
X=zeros(3,N);
for k=1:N
    z=double(f(double(tau(k))));
    z=z(:);
    assert(numel(z)==3 && all(isfinite(z)), ...
        '[R16-R2] nominal flat derivative scalar output invalid.');
    X(:,k)=z;
end
end
