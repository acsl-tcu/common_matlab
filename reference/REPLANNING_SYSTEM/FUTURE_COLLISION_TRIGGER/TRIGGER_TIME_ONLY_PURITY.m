function R = TRIGGER_TIME_ONLY_PURITY(fn)
% Strict, side-effect-free eligibility check for exact-time reference reuse.
% Accept only an anonymous scalar-time expression consisting of literal
% numeric arithmetic and a finite, explicitly whitelisted set of MATLAB
% numeric elementary functions, with numeric immutable captured values.
% An unrecognized expression is NOT cached. This never evaluates fn.
%
% Limitations: this recognizes a deliberately small arithmetic subset; it
% is not a general MATLAB analyzer or a proof of arbitrary user functions.
R=struct('eligible',false,'status','NOT_PROVEN_PURE', ...
    'reason','','captured_names',{{}},'expression','');
if ~isa(fn,'function_handle')
    R.reason='Input is not a function handle.';
    return
end
try
    info=functions(fn);
    if ~isfield(info,'type') || ~strcmp(info.type,'anonymous') || ...
            ~isfield(info,'workspace')
        R.reason='Only explicit anonymous expressions can be audited.';
        return
    end
    source=func2str(fn);
    % Forbid nested handles and unparsed function-handle syntax.
    tok=regexp(source,'^@\(([a-zA-Z]\w*)\)(.+)$','tokens','once');
    if isempty(tok)
        R.reason='Not an explicit one-argument anonymous expression.';
        return
    end
    variable=tok{1}; expression=strtrim(tok{2});
    R.expression=expression;
    if isempty(expression) || ~isempty(regexp(expression, ...
            '[^a-zA-Z0-9_\s\.\+\-\*\/\^,:;()\[\]]','once'))
        R.reason='Expression contains syntax outside the permitted arithmetic subset.';
        return
    end
    % Disallow field access, transpose, strings, logical/relational
    % operators, array concatenation side effects, and dynamic dispatch.
    if ~isempty(regexp(expression,'\.[a-zA-Z_]','once'))
        R.reason='Field access is not part of the purity proof.';
        return
    end
    workspace=info.workspace;
    captured={};
    if ~isempty(workspace)
        if ~iscell(workspace)
            R.reason='Anonymous workspace metadata is not a cell array.';
            return
        end
        for j=1:numel(workspace)
            st=workspace{j};
            if ~isstruct(st)||~isscalar(st)
                R.reason='Captured workspace is not a scalar structure.';
                return
            end
            names=fieldnames(st);
            for k=1:numel(names)
                val=st.(names{k});
                if ~isnumeric(val) || ~isreal(val) || ...
                        ~isa(val,'double') || ~all(isfinite(val(:))) || ...
                        issparse(val) || ~isvarname(names{k})
                    R.reason='Captured value is not an immutable finite double array.';
                    return
                end
            end
            captured=[captured;names]; %#ok<AGROW>
        end
    end
    if numel(unique(captured))~=numel(captured)
        R.reason='Ambiguous captured variable name.';
        return
    end
    % Lexical whitelist: no user-defined, random, stateful or I/O calls.
    builtin={'sin','cos','tan','asin','acos','atan','atan2', ...
        'sinh','cosh','tanh','asinh','acosh','atanh', ...
        'sqrt','abs','exp','log','log10','floor','ceil','round', ...
        'mod','rem','sign','min','max','sum','prod', ...
        'power','reshape','repmat','zeros','ones','pi','eps'};
    permitted=[{variable},captured.',builtin];
    % Remove scientific-notation exponent markers, not variable tokens.
    lex=regexprep(expression,'(?<=\d)[eE][\+\-]?\d+','0');
    names=regexp(lex,'[a-zA-Z]\w*','match');
    if ~all(ismember(names,permitted))
        R.reason=['Unknown token: ' strjoin(unique(names(~ismember(names,permitted))),',')];
        return
    end
    % Captured constants with the same name as a builtin are ambiguous.
    if any(ismember(captured,builtin)) || any(strcmp(captured,variable))
        R.reason='Captured name shadows a builtin or the time variable.';
        return
    end
    % Reject path-shadowed and external functions: arithmetic may be
    % reused only if each invoked function resolves to a MATLAB builtin.
    invoked=regexp(lex,'([a-zA-Z]\w*)\s*\(','tokens');
    calls=unique(cellfun(@(x)x{1},invoked,'UniformOutput',false));
    for j=1:numel(calls)
        name=calls{j};
        if strcmp(name,variable) || ismember(name,captured)
            % Built-in numeric array indexing is side-effect free.
            continue
        end
        resolution=which(name);
        if ~ischar(resolution) || ...
                ~strncmpi(resolution,'built-in',8)
            R.reason=['Function is not resolved to a builtin: ' name];
            return
        end
    end
    R.eligible=true;
    R.status='PROVEN_FOR_STRICT_ARITHMETIC_SUBSET__EXACT_TIME_ONLY';
    R.reason='Only numeric captured constants and pure arithmetic tokens.';
    R.captured_names=captured.';
catch ME
    R.status='NOT_PROVEN_PURE';
    R.reason=[ME.identifier ' | ' ME.message];
end
end
