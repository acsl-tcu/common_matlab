function [Y,valid,meta] = af7r33_flat_jet_batch_model(replanner,tg,model)
% Evaluate a stored AF7R33 model at global times tg.

persistent CACHE
assert(isa(replanner,'REPLANNING_SYSTEM'));
assert(isstruct(model)&&isfield(model,'certified')&&logical(model.certified));
tg=double(tg(:).');
assert(all(isfinite(tg)));

F=replanner.nominal_flat_derivative_fn(1:7);
S=cellfun(@func2str,F,'UniformOutput',false);
signature=sha256_text(strjoin(S,char(10)));
assert(isfield(model,'nominal_signature')&&strcmp(signature,model.nominal_signature), ...
    '[AF7R33] nominal source signature changed; batch model is stale.');

if isempty(CACHE)||~isfield(CACHE,'signature')||~strcmp(CACHE.signature,signature)
    CACHE=build_nominal_cache(F,S,signature);
end

tf0=double(model.flight_reference_time_origin);
assert(isfinite(tf0));
tau=max(0,tg-tf0);
N=numel(tg);
Y=zeros(21,N);

for r=1:7
    rows=(3*r-2):(3*r);
    if CACHE.direct_vector_ok(r)
        Z=normalize_triplet_output(double(F{r}(tau)),N);
    else
        Z=zeros(3,N);
        for j=1:3
            Z(j,:)=normalize_scalar_output(double(CACHE.scalar_fn{r,j}(tau)),N);
        end
    end
    Y(rows,:)=Z;
end

% Add every committed correction.  Expired/future corrections contribute zero.
if isfield(model,'corrections')&&~isempty(model.corrections)
    for ic=1:numel(model.corrections)
        C=model.corrections(ic);
        mask=tg>double(C.t0) & tg<double(C.t1);
        if ~any(mask), continue; end
        tc=tg(mask);
        D=piecewise_derivative_batch(tc,C,6);
        for rr=0:6
            rows=3*rr+(1:3);
            Y(rows,mask)=Y(rows,mask)+D(:,:,rr+1);
        end
    end
end

valid=all(isfinite(Y),1);
meta=struct('schema','AF7R33_FLAT_BATCH_META_V1', ...
    'signature',signature,'direct_vector_ok',CACHE.direct_vector_ok, ...
    'correction_count',numel(model.corrections));
end


function C=build_nominal_cache(F,S,signature)
C=struct('signature',signature,'direct_vector_ok',false(1,7), ...
    'scalar_fn',{cell(7,3)});
probe=[0.123456789,0.987654321];
for r=1:7
    try
        normalize_triplet_output(double(F{r}(probe)),numel(probe));
        C.direct_vector_ok(r)=true;
    catch
        C.direct_vector_ok(r)=false;
    end
    if ~C.direct_vector_ok(r)
        [zsym,tSym]=parse_symbolic_triplet(S{r});
        for j=1:3
            C.scalar_fn{r,j}=matlabFunction(zsym(j),'Vars',tSym);
        end
    end
end
end


function DOUT=piecewise_derivative_batch(t,C,maxOrder)
t=double(t(:).');
N=numel(t);
DOUT=zeros(3,N,maxOrder+1);
s0=double(C.segment_t0(:));
s1=double(C.segment_t1(:));
CP=C.control_points;

for is=1:numel(CP)
    idx=find(t>=s0(is) & t<=s1(is));
    if isempty(idx), continue; end
    T=s1(is)-s0(is);
    assert(isfinite(T)&&T>0);
    u=(t(idx)-s0(is))/T;
    u=max(0,min(1,u));
    B=double(CP{is});
    deg=15;
    D=B;
    DOUT(:,idx,1)=decasteljau_batch(D,u);
    for r=1:maxOrder
        D=(deg/T)*diff(D,1,2);
        deg=deg-1;
        DOUT(:,idx,r+1)=decasteljau_batch(D,u);
    end
end
end


function Y=decasteljau_batch(C,u)
global AF7R61_DECASTELJAU_CAPTURE;
if isempty(AF7R61_DECASTELJAU_CAPTURE), AF7R61_DECASTELJAU_CAPTURE={}; end
AF7R61_DECASTELJAU_CAPTURE{end+1}={C,u};
Y=af7r61_decasteljau_batch_exact_original(C,u);
end
