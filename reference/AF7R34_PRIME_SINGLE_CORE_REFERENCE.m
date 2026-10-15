function OUT = AF7R34_PRIME_SINGLE_CORE_REFERENCE(replanner)
% Compile the CURRENT nominal flat jet into the persistent R14 expression AST.
% No MATLAB Coder / ccode is used. The parser lives inside the C++ MEX.

clc;
fprintf('\n====================================================================================================\n');
fprintf(' AF7R34 | PRIME SOURCE-BOUND NOMINAL REFERENCE AST\n');
fprintf('====================================================================================================\n');
fprintf('[AUTHORITY] sourceBound=1 | setupOnly=1 | productionModification=0\n');

assert(isa(replanner,'REPLANNING_SYSTEM'));
assert(exist('af7r34_single_core_mex','file')==3,'[AF7R34] core MEX missing.');

F=replanner.nominal_flat_derivative_fn(1:7);
assert(all(cellfun(@(f)isa(f,'function_handle'),F)), ...
    '[AF7R34] nominal flat derivative handles unavailable.');

S=cellfun(@func2str,F,'UniformOutput',false);
signature=sha256_text(strjoin(S,char(10)));

expr=cell(21,1);
kk=0;
for r=1:7
    s=strtrim(S{r});
    tok=regexp(s,'^@\(([A-Za-z]\w*)\)(.*)$','tokens','once');
    assert(~isempty(tok)&&numel(tok)==2, ...
        '[AF7R34] derivative handle %d is not single-variable anonymous.',r-1);
    varName=char(tok{1});
    body=strtrim(char(tok{2}));
    body=strrep(body,'.^','^');
    body=strrep(body,'.*','*');
    body=strrep(body,'./','/');
    z=str2sym(body); z=z(:);
    assert(numel(z)==3,'[AF7R34] derivative %d is not 3-vector.',r-1);
    t=sym('t','real');
    sv0=symvar(z);
    assert(numel(sv0)<=1, ...
        '[AF7R34] derivative %d contains more than one free symbolic variable.',r-1);
    if ~isempty(sv0)
        z=subs(z,sv0(1),t);
    end
    sv=symvar(z);
    for j=1:numel(sv)
        assert(strcmp(char(sv(j)),'t'), ...
            '[AF7R34] unexpected free symbolic variable %s.',char(sv(j)));
    end
    for j=1:3
        kk=kk+1;
        expr{kk}=char(z(j));
    end
end

ok=af7r34_single_core_mex(0,expr,signature);
assert(logical(ok),'[AF7R34] C++ AST prime failed.');

% Verify the AST evaluator against the current MATLAB nominal authority.
model=af7r33_nominal_batch_model(replanner);
P=af7r34_make_core_packet(replanner,[],[],model);
g0=double(replanner.nominal_table_t0);
g1=double(replanner.nominal_table_tend);
tg=unique([g0,g1,double(replanner.flight_reference_time_origin),linspace(g0,g1,257)]);
Ym=af7r34_single_core_mex(2,P,tg);
Yr=zeros(21,numel(tg));
tf0=double(replanner.flight_reference_time_origin);
tau=max(0,tg-tf0);
for k=1:numel(tg)
    for r=1:7
        q=double(F{r}(tau(k))); q=q(:);
        Yr(3*r-2:3*r,k)=q;
    end
end
maxDiff=max(abs(double(Ym)-Yr),[],'all');
scale=max(1,max(abs(Yr),[],'all'));
tol=4096*eps(scale);
closed=maxDiff<=tol;

fprintf(' nominal signature                       = %s\n',signature);
fprintf(' scalar expressions compiled             = %d\n',numel(expr));
fprintf(' AST max |C++-MATLAB|                    = %.17g\n',maxDiff);
fprintf(' AST equivalence tolerance               = %.17g\n',tol);
fprintf(' AST source equivalence closed           = %d\n',closed);
fprintf(' production source modified              = 0\n');
fprintf('====================================================================================================\n');

assert(closed,'[AF7R34] nominal AST equivalence did not close.');

root=fileparts(which('REPLANNING_SYSTEM'));
token=struct('schema','AF7R34_AST_TOKEN_V1','closed',true, ...
    'signature',signature,'max_diff',maxDiff,'tol',tol);
save(fullfile(root,'AF7R34_AST_TOKEN.mat'),'token');

OUT=struct('schema','AF7R34_PRIME_SINGLE_CORE_REFERENCE_V1', ...
    'closed',closed,'signature',signature,'max_diff',maxDiff,'tol',tol);
assignin('base','AF7R34_PRIME_SINGLE_CORE_REFERENCE_OUT',OUT);
end

function h=sha256_text(s)
md=java.security.MessageDigest.getInstance('SHA-256');
md.update(uint8(unicode2native(char(s),'UTF-8')));
d=typecast(md.digest(),'uint8');
h=lower(reshape(dec2hex(d,2).',1,[]));
end
