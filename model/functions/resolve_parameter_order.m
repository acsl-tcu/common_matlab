function parameter_order = resolve_parameter_order(method, expected_count)
% Resolve parameter order from model/substance input-order comments.
% model実体（substance内）ファイルのInput orderに記載のparameter順序をparameter_orderに反映するための関数
arguments
    method
    expected_count = []
end

parameter_order = string.empty(1, 0);
if isempty(method)
    if ~isempty(expected_count)
        error("ACSL: method is empty while strict parameter_order enforcement is enabled.");
    end
    return
end

method = string(method);
root_dir = fileparts(fileparts(mfilename('fullpath')));
substance_path = fullfile(root_dir, "substance", method + ".m");
if ~isfile(substance_path)
    matches = dir(fullfile(root_dir, "substance", "**", method + ".m"));
    if isempty(matches)
        matches = find_substance_file(fullfile(root_dir, "substance"), method + ".m");
    end
    if isempty(matches)
        if ~isempty(expected_count) && expected_count > 0
            error("ACSL: missing substance file for %s (expected %d parameters).", ...
                method, expected_count);
        end
        return
    end
    if isstruct(matches)
        substance_path = fullfile(matches(1).folder, matches(1).name);
    else
        substance_path = matches;
    end
end

text = fileread(substance_path);
token = regexp(text, '(?m)%\\s*\\w+\\s*\\(parameter\\)\\s*=\\s*\\[([^\\n\\r]*)\\];?', 'tokens', 'once');
if isempty(token)
    token = fallback_parameter_token(text);
end
if isempty(token)
    if ~isempty(expected_count) && expected_count > 0
        error("ACSL: parameter_order comment missing in %s (expected %d parameters).", ...
            method, expected_count);
    end
    return
end

% =======================================================================
function path = find_substance_file(root_dir, target_name)
    path = "";
    if isstring(target_name)
        target_name = char(target_name);
    end
    stack = {root_dir};
    while ~isempty(stack)
        current = stack{end};
        stack(end) = [];
        listing = dir(current);
        for i = 1:numel(listing)
            name = listing(i).name;
            if strcmp(name, ".") || strcmp(name, "..")
                continue
            end
            full = fullfile(current, name);
            if listing(i).isdir
                stack{end + 1} = full; %#ok<AGROW>
            else
                if strcmp(name, target_name)
                    path = full;
                    return
                end
            end
        end
    end
end
% =======================================================================

raw = token{1};
matches = regexp(raw, '"([^"]*)"', 'tokens');
if isempty(matches)
    return
end

names = cellfun(@(c) c{1}, matches{1}, 'UniformOutput', false);
parameter_order = reshape(string(names), 1, []);

if ~isempty(expected_count)
    if expected_count == 0 && ~isempty(parameter_order)
        error("ACSL: parameter_order should be empty for %s (expected 0, got %d).", ...
            method, numel(parameter_order));
    elseif expected_count > 0 && numel(parameter_order) ~= expected_count
        error("ACSL: parameter_order size mismatch for %s (expected %d, got %d).", ...
            method, expected_count, numel(parameter_order));
    end
end

% =======================================================================
function token = fallback_parameter_token(text)
    token = {};
    lines = splitlines(text);
    for i = 1:numel(lines)
        line = lines(i);
        if contains(line, "(parameter)")
            inner = regexp(line, '\[(.*)\]', 'tokens', 'once');
            if ~isempty(inner)
                token = inner;
                return
            end
        end
    end
end
% =======================================================================
end
