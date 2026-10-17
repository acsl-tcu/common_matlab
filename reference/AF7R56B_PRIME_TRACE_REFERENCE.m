function OUT = AF7R56B_PRIME_TRACE_REFERENCE(replanner,M)
% Prime ONLY the persistent nominal AST inside af7r56_trace_mex.
% No production source/state is modified.
%
% The newly built shadow MEX starts with G_compiled=false.  Operation 1
% therefore cannot be called until operation 0 compiles the CURRENT nominal
% 0..6 derivative expressions into that MEX.  This is the same setup
% contract used by AF7R36.

clc;
fprintf('\n====================================================================================================\n');
fprintf(' AF7R56B | PRIME SHADOW BRANCH MEX NOMINAL AST\n');
fprintf('====================================================================================================\n');
fprintf('[AUTHORITY] setupOnly=1 | sourceBound=1 | productionModified=0\n');

assert(isa(replanner,'REPLANNING_SYSTEM'));
assert(exist('af7r56_trace_mex','file')==3, ...
    '[AF7R56B] af7r56_trace_mex missing.');
assert(isstruct(M)&&isfield(M,'certified')&&logical(M.certified)&& ...
    isfield(M,'nominal_signature'), ...
    '[AF7R56B] saved nominal batch model invalid.');

F=replanner.nominal_flat_derivative_fn(1:7);
assert(iscell(F)&&numel(F)>=7&&all(cellfun(@(f)isa(f,'function_handle'),F)), ...
    '[AF7R56B] current nominal derivative handles unavailable.');

S=cellfun(@func2str,F,'UniformOutput',false);
signature=sha256_text(strjoin(S,char(10)));

% Before priming the MEX, bind it to exactly the same nominal source that
% produced the saved true-trigger theorem snapshot.
assert(strcmp(signature,char(M.nominal_signature)), ...
    ['[AF7R56B] current nominal signature differs from the saved true-trigger ' ...
     'authority. No shadow replay permitted.']);

expr=cell(21,1);
kk=0;
for r=1:7
    ss=strtrim(S{r});
    tok=regexp(ss,'^@\(([A-Za-z]\w*)\)(.*)$','tokens','once');
    assert(~isempty(tok)&&numel(tok)==2, ...
        '[AF7R56B] derivative handle %d is not single-variable anonymous.',r-1);

    body=strtrim(char(tok{2}));
    body=strrep(body,'.^','^');
    body=strrep(body,'.*','*');
    body=strrep(body,'./','/');

    z=str2sym(body);
    z=z(:);
    assert(numel(z)==3, ...
        '[AF7R56B] derivative %d is not a 3-vector.',r-1);

    t=sym('t','real');
    sv0=symvar(z);
    assert(numel(sv0)<=1, ...
        '[AF7R56B] derivative %d has multiple free variables.',r-1);
    if ~isempty(sv0)
        z=subs(z,sv0(1),t);
    end
    sv=symvar(z);
    for j=1:numel(sv)
        assert(strcmp(char(sv(j)),'t'), ...
            '[AF7R56B] unexpected free symbolic variable %s.',char(sv(j)));
    end

    for j=1:3
        kk=kk+1;
        expr{kk}=char(z(j));
    end
end

ok=af7r56_trace_mex(0,expr,signature);
assert(logical(ok),'[AF7R56B] shadow AST prime failed.');

fprintf(' nominal signature                       = %s\n',signature);
fprintf(' scalar expressions compiled             = %d\n',numel(expr));
fprintf(' saved/current signature equal           = 1\n');
fprintf(' production source modified              = 0\n');
fprintf('====================================================================================================\n');

OUT=struct('schema','AF7R56B_PRIME_TRACE_REFERENCE_V1', ...
    'closed',true,'signature',signature,'expression_count',numel(expr));
assignin('base','AF7R56B_PRIME_TRACE_REFERENCE_OUT',OUT);
end

function h=sha256_text(s)
md=java.security.MessageDigest.getInstance('SHA-256');
md.update(uint8(unicode2native(char(s),'UTF-8')));
d=typecast(md.digest(),'uint8');
h=lower(reshape(dec2hex(d,2).',1,[]));
end
