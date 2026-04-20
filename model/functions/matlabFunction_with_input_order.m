function matlabFunction_with_input_order(expr, varargin)
% Wrapper for matlabFunction that writes Input order comments to the output file.

[file_path, vars] = parse_matlab_function_args(varargin{:});
matlabFunction(expr, varargin{:});

if isempty(file_path) || isempty(vars)
    return
end

target = string(file_path);
if ~endsWith(lower(target), ".m")
    target = target + ".m";
end
target_dir = fileparts(target);
if target_dir ~= "" && ~isfolder(target_dir)
    mkdir(target_dir);
end

if ~isfile(target)
    return
end

text = fileread(target);
text = regexprep(text, '(?m)^% Input order:\\s*\\n(?:%.*\\n)+\\n?', '');

[arg_names, insert_line] = find_insertion_point(text);
comment_block = build_input_order_block(arg_names, vars);
if comment_block == ""
    return
end

text = insert_block(text, insert_line, comment_block);
fid = fopen(target, 'w');
if fid == -1
    return
end
fwrite(fid, text);
fclose(fid);
end

function [file_path, vars] = parse_matlab_function_args(varargin)
file_path = "";
vars = {};
for i = 1:2:numel(varargin)
    key = varargin{i};
    if isstring(key) || ischar(key)
        key = lower(string(key));
        if key == "file"
            file_path = string(varargin{i + 1});
        elseif key == "vars"
            vars = varargin{i + 1};
        end
    end
end
end

function [arg_names, insert_line] = find_insertion_point(text)
lines = splitlines(text);
fn_idx = find(startsWith(strtrim(lines), "function"), 1, "first");
if isempty(fn_idx)
    arg_names = string.empty(1, 0);
    insert_line = 1;
    return
end
sig = lines{fn_idx};
arg_names = parse_function_args(sig);

insert_line = fn_idx + 1;
while insert_line <= numel(lines)
    trimmed = strtrim(lines{insert_line});
    if startsWith(trimmed, "%")
        insert_line = insert_line + 1;
        continue
    end
    break
end
end

function arg_names = parse_function_args(sig)
arg_names = string.empty(1, 0);
tokens = regexp(sig, '^function\\s+[^=]*=\\s*\\w+\\s*\\(([^)]*)\\)', 'tokens', 'once');
if isempty(tokens)
    tokens = regexp(sig, '^function\\s+\\w+\\s*\\(([^)]*)\\)', 'tokens', 'once');
end
if isempty(tokens)
    return
end
raw = strtrim(tokens{1});
if raw == ""
    return
end
parts = strsplit(raw, ",");
arg_names = string(strtrim(parts));
end

function block = build_input_order_block(arg_names, vars)
block = "% Input order:" + newline;
for i = 1:numel(vars)
    if i <= numel(arg_names) && arg_names(i) ~= ""
        arg = arg_names(i);
    else
        arg = "in" + string(i);
    end
    entries = var_names(vars{i});
    label = arg_label(i, entries, numel(vars));
    if isempty(entries)
        list = "[]";
    else
        quoted = """" + string(entries) + """";
        list = "[" + strjoin(quoted, ", ") + "]";
    end
    block = block + "% " + arg + " (" + label + ") = " + list + ";" + newline;
end
block = block + newline;
end

function label = arg_label(idx, entries, total_vars)
if idx == 1
    label = "state";
elseif idx == 2
    if total_vars == 2
        if is_input_like(entries)
            label = "input";
        else
            label = "parameter";
        end
    else
        label = "input";
    end
elseif idx == 3
    label = "parameter";
else
    label = "arg";
end
end

function names = var_names(v)
names = string.empty(1, 0);
if isa(v, "sym")
    names = string(arrayfun(@char, v(:).', 'UniformOutput', false));
elseif iscell(v)
    try
        sv = [v{:}];
        if isa(sv, "sym")
            names = string(arrayfun(@char, sv(:).', 'UniformOutput', false));
        end
    catch
        names = string.empty(1, 0);
    end
end
end

function text = insert_block(text, insert_line, block)
lines = splitlines(text);
block_lines = splitlines(block);
if insert_line <= 1
    lines = [block_lines; lines];
else
    lines = [lines(1:insert_line-1); block_lines; lines(insert_line:end)];
end
text = strjoin(lines, newline);
end

function tf = is_input_like(entries)
tf = false;
if isempty(entries)
    return
end
names = lower(string(entries));
for i = 1:numel(names)
    n = names(i);
    if startsWith(n, "u") || startsWith(n, "t")
        tf = true;
        return
    end
end
end
