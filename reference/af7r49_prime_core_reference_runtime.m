function OUT = af7r49_prime_core_reference_runtime(replanner)
% Setup-only compilation of the CURRENT nominal flat jet into AF7R49 MEX.
% No production state is mutated other than the MEX's internal AST cache.

assert(isa(replanner,'REPLANNING_SYSTEM'));
assert(exist('af7r49_single_core_mex','file')==3, ...
    '[AF7R49] native core MEX missing.');

F=replanner.nominal_flat_derivative_fn(1:7);
assert(all(cellfun(@(f)isa(f,'function_handle'),F)), ...
    '[AF7R49] nominal flat derivative handles unavailable.');
S=cellfun(@func2str,F,'UniformOutput',false);
signature=sha256_text(strjoin(S,char(10)));
expr=cell(21,1); kk=0;
for r=1:7
    ss=strtrim(S{r});
    tok=regexp(ss,'^@\(([A-Za-z]\w*)\)(.*)$','tokens','once');
    assert(~isempty(tok)&&numel(tok)==2, ...
        '[AF7R49] derivative handle %d is not single-variable anonymous.',r-1);
    body=strtrim(char(tok{2}));
    body=strrep(body,'.^','^'); body=strrep(body,'.*','*'); body=strrep(body,'./','/');
    z=str2sym(body); z=z(:);
    assert(numel(z)==3,'[AF7R49] derivative %d is not a 3-vector.',r-1);
    t=sym('t','real'); sv0=symvar(z);
    assert(numel(sv0)<=1,'[AF7R49] derivative %d has multiple free variables.',r-1);
    if ~isempty(sv0), z=subs(z,sv0(1),t); end
    sv=symvar(z);
    for j=1:numel(sv), assert(strcmp(char(sv(j)),'t')); end
    for j=1:3, kk=kk+1; expr{kk}=char(z(j)); end
end
ok=af7r49_single_core_mex(0,expr,signature);
assert(logical(ok),'[AF7R49] native AST compilation failed.');
OUT=struct('schema','AF7R49_AST_PRIME_V1','closed',true,'signature',signature);
end

function h=sha256_text(s)
md=java.security.MessageDigest.getInstance('SHA-256');
md.update(uint8(unicode2native(char(s),'UTF-8')));
d=typecast(md.digest(),'uint8');
h=lower(reshape(dec2hex(d,2).',1,[]));
end
