function OUT = af7r36_runtime_prime(replanner)
% Silent pre-Flight prime for the R14D production native core.
%
% Compiles the CURRENT nominal flat-jet expressions into the persistent C++
% AST.  This is setup-only.  The existing pre-motion
% prime_locked_backend_kernel_variants() subsequently executes the actual
% selected callback on synthetic detected-obstacle variants, warming the full
% episode->QP->certificate native path before Flight.

persistent LAST_SIGNATURE LAST_CLOSED

assert(isa(replanner,'REPLANNING_SYSTEM'), ...
    '[AF7R36 PROD] replanner must be REPLANNING_SYSTEM.');
assert(exist('af7r36_single_core_mex','file')==3, ...
    '[AF7R36 PROD] canonical single-core MEX missing.');

F=replanner.nominal_flat_derivative_fn(1:7);
assert(all(cellfun(@(f)isa(f,'function_handle'),F)), ...
    '[AF7R36 PROD] nominal flat derivative handles unavailable.');

S=cellfun(@func2str,F,'UniformOutput',false);
signature=sha256_text(strjoin(S,char(10)));

if ~isempty(LAST_SIGNATURE) && strcmp(LAST_SIGNATURE,signature) && ...
        ~isempty(LAST_CLOSED) && LAST_CLOSED
    OUT=struct('schema','AF7R36_RUNTIME_PRIME_V1', ...
        'closed',true,'signature',signature,'cached',true);
    return;
end

expr=cell(21,1);
kk=0;
for r=1:7
    s=strtrim(S{r});
    tok=regexp(s,'^@\(([A-Za-z]\w*)\)(.*)$','tokens','once');
    assert(~isempty(tok)&&numel(tok)==2, ...
        '[AF7R36 PROD] derivative handle %d is not one-variable anonymous.',r-1);
    body=strtrim(char(tok{2}));
    body=strrep(body,'.^','^');
    body=strrep(body,'.*','*');
    body=strrep(body,'./','/');

    z=str2sym(body);
    z=z(:);
    assert(numel(z)==3,'[AF7R36 PROD] derivative %d is not 3-vector.',r-1);

    t=sym('t','real');
    sv0=symvar(z);
    assert(numel(sv0)<=1, ...
        '[AF7R36 PROD] derivative %d has more than one free symbolic variable.',r-1);
    if ~isempty(sv0)
        z=subs(z,sv0(1),t);
    end
    sv=symvar(z);
    for j=1:numel(sv)
        assert(strcmp(char(sv(j)),'t'), ...
            '[AF7R36 PROD] unexpected symbolic variable %s.',char(sv(j)));
    end

    for j=1:3
        kk=kk+1;
        expr{kk}=char(z(j));
    end
end

ok=af7r36_single_core_mex(0,expr,signature);
assert(logical(ok),'[AF7R36 PROD] native nominal AST compile failed.');

% Small exact source-bound post-compile equivalence check during setup.
tg=unique([double(replanner.nominal_table_t0), ...
           double(replanner.flight_reference_time_origin), ...
           double(replanner.nominal_table_tend)]);
model=af7r33_nominal_batch_model(replanner);
pkt=af7r34_make_core_packet(replanner,[],[],model);
YC=double(af7r36_single_core_mex(2,pkt,tg));

tf0=double(replanner.flight_reference_time_origin);
tau=max(0,tg-tf0);
YM=zeros(21,numel(tg));
for k=1:numel(tg)
    for r=1:7
        q=double(F{r}(tau(k))); q=q(:);
        assert(numel(q)==3&&all(isfinite(q)));
        YM(3*r-2:3*r,k)=q;
    end
end

d=max(abs(YC-YM),[],'all');
scale=max(1,max(abs(YM),[],'all'));
tol=4096*eps(scale);
closed=d<=tol;
assert(closed, ...
    '[AF7R36 PROD] setup AST source equivalence failed: diff=%.17g tol=%.17g.',d,tol);

LAST_SIGNATURE=signature;
LAST_CLOSED=true;

OUT=struct('schema','AF7R36_RUNTIME_PRIME_V1', ...
    'closed',true,'signature',signature,'cached',false, ...
    'max_abs_diff',d,'tolerance',tol);
end


function h=sha256_text(s)
md=java.security.MessageDigest.getInstance('SHA-256');
md.update(uint8(unicode2native(char(s),'UTF-8')));
d=typecast(md.digest(),'uint8');
h=lower(reshape(dec2hex(d,2).',1,[]));
end
