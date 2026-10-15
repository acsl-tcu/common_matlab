function [Y,valid,meta] = af7r31_flat_jet_batch_v3(replanner,tg)
% AF7R31 flat-jet batch evaluator without MATLAB Coder or ccode.
%
% It is source-bound to the CURRENT nominal_flat_derivative_fn{1:7}.
% For each derivative vector it first uses the original handle directly if
% that handle is vector-input compatible.  If not, it reconstructs the exact
% symbolic 3-vector from func2str() exactly as REPLANNING_SYSTEM does, then
% creates three scalar matlabFunction handles.  Those scalar expressions are
% vectorized, so a batch of N times requires at most 21 MATLAB function calls,
% not 7*N scalar calls.
%
% No production source is modified and no safety decision is made here.

persistent CACHE
assert(isa(replanner,'REPLANNING_SYSTEM'), ...
    '[AF7R31V3] replanner must be REPLANNING_SYSTEM.');

tg=double(tg(:).');
assert(all(isfinite(tg)),'[AF7R31V3] global times must be finite.');

F=replanner.nominal_flat_derivative_fn;
assert(iscell(F)&&numel(F)>=7&& ...
    all(cellfun(@(f)isa(f,'function_handle'),F(1:7))), ...
    '[AF7R31V3] nominal flat derivative handles unavailable.');

S=cellfun(@func2str,F(1:7),'UniformOutput',false);
signature=sha256_text(strjoin(S,char(10)));

if isempty(CACHE) || ~isfield(CACHE,'signature') || ...
        ~strcmp(CACHE.signature,signature)
    CACHE=build_cache(F(1:7),S,signature);
end

tf0=double(replanner.flight_reference_time_origin);
assert(isfinite(tf0),'[AF7R31V3] flight reference time origin unavailable.');
tau=max(0,tg-tf0);
N=numel(tau);
Y=zeros(21,N);

for r=1:7
    rows=(3*r-2):(3*r);
    if CACHE.direct_vector_ok(r)
        z=double(F{r}(tau));
        Z=normalize_triplet_output(z,N);
    else
        Z=zeros(3,N);
        for j=1:3
            v=double(CACHE.scalar_fn{r,j}(tau));
            Z(j,:)=normalize_scalar_output(v,N);
        end
    end
    Y(rows,:)=Z;
end

valid=all(isfinite(Y),1);
meta=struct();
meta.schema='AF7R31_FLAT_JET_BATCH_V3_META';
meta.signature=signature;
meta.direct_vector_ok=CACHE.direct_vector_ok;
meta.scalar_fallback_count=nnz(~CACHE.direct_vector_ok);
end


function C=build_cache(F,S,signature)
C=struct();
C.signature=signature;
C.direct_vector_ok=false(1,7);
C.scalar_fn=cell(7,3);

probe=[0.123456789,0.987654321];

for r=1:7
    % First determine whether the original MATLAB-generated vector handle
    % already supports vector time input exactly.
    try
        z=double(F{r}(probe));
        normalize_triplet_output(z,numel(probe));
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


function [z,tSym]=parse_symbolic_triplet(fstr)
s=strtrim(char(fstr));
tok=regexp(s,'^@\(([A-Za-z]\w*)\)(.*)$','tokens','once');
assert(~isempty(tok)&&numel(tok)==2, ...
    '[AF7R31V3] flat handle must be a one-variable anonymous function.');
varName=char(tok{1});
body=strtrim(char(tok{2}));

% Match REPLANNING_SYSTEM.function_handle_to_symbolic_vector().
body=strrep(body,'.^','^');
body=strrep(body,'.*','*');
body=strrep(body,'./','/');

z=str2sym(body);
z=z(:);
assert(numel(z)==3,'[AF7R31V3] flat derivative must be a 3-vector.');

tSym=sym(varName,'real');
sv=symvar(z);
for k=1:numel(sv)
    assert(strcmp(char(sv(k)),varName), ...
        '[AF7R31V3] unexpected symbolic variable %s.',char(sv(k)));
end
end


function Z=normalize_triplet_output(z,N)
z=double(z);
if N==1
    assert(numel(z)==3,'[AF7R31V3] 3-vector output expected.');
    Z=reshape(z,3,1);
    return;
end
if isequal(size(z),[3,N])
    Z=z;
elseif isequal(size(z),[N,3])
    Z=z.';
elseif numel(z)==3
    % Constant 3-vector independent of time.
    Z=repmat(reshape(z,3,1),1,N);
else
    error('[AF7R31V3] vector handle output shape is not 3xN/Nx3/constant-3.');
end
end


function v=normalize_scalar_output(v,N)
v=double(v);
if isscalar(v)
    v=repmat(v,1,N);
elseif numel(v)==N
    v=reshape(v,1,N);
else
    error('[AF7R31V3] scalar component output is neither scalar nor length N.');
end
end


function h=sha256_text(s)
md=java.security.MessageDigest.getInstance('SHA-256');
md.update(uint8(unicode2native(char(s),'UTF-8')));
d=typecast(md.digest(),'uint8');
h=lower(reshape(dec2hex(d,2).',1,[]));
end
