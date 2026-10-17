function R = AF7R57_BINDING_AND_COMMON_DYADIC_PRECHECK()
% AF7R57_BINDING_AND_COMMON_DYADIC_PRECHECK
% -----------------------------------------------------------------------------
% Shadow-only bridge from AF7R56 -> AF7R57.
%
% Purpose
%   1) Locate the *actual* AF7R56B MATLAB source that calls
%        af7r56_trace_mex
%        af7r56_interval_kkt_mex
%      without inventing any MEX signature.
%   2) Print the exact call statements and bind their source SHA-256.
%   3) Verify that production source bytes remain unchanged.
%   4) Close the source-independent mathematical part of the next theorem:
%      exact common refinement of two dyadic leaf partitions.
%
% This file deliberately DOES NOT:
%   - select a production t1,
%   - modify production source,
%   - relax the finite-sensing gate,
%   - convert the proof to sampled acceptance,
%   - guess MEX inputs/outputs.
%
% Next step after this audit:
%   AF7R57_COMMON_DYADIC_REFINEMENT_INTERVAL_STAGEB
%   using the exact call interfaces printed here.
% -----------------------------------------------------------------------------

clc;
fprintf('\n====================================================================================================\n');
fprintf(' AF7R57 | SOURCE BINDING + COMMON DYADIC REFINEMENT PRECHECK\n');
fprintf('====================================================================================================\n');
fprintf('[AUTHORITY] shadowOnly=1 | sourceBound=1 | exactDyadic=1 | sampledAcceptance=0 | productionModified=0\n');

R = struct();
R.shadowOnly = true;
R.sampledAcceptance = false;
R.productionModified = false;
R.nextTheorem = 'AF7R57_COMMON_DYADIC_REFINEMENT_INTERVAL_STAGEB';

% -------------------------------------------------------------------------
% A. Resolve repository/reference roots from existing authority.
% -------------------------------------------------------------------------
roots = af7r57_candidate_roots();
[srcFile, hits] = af7r57_find_af7r56b_source(roots);

if isempty(srcFile)
    fprintf('\n[A. AF7R56 SOURCE BINDING]\n');
    fprintf(' AF7R56B source found                    = 0\n');
    fprintf(' exact MEX signature guessed             = 0\n');
    fprintf(' production source modified              = 0\n');
    fprintf(' binding closed                          = 0\n');
    fprintf('\nSearch roots:\n');
    for k = 1:numel(roots)
        fprintf('  %s\n', roots{k});
    end
    error('AF7R57:AF7R56BSourceNotFound', ...
        ['Could not locate an AF7R56 MATLAB source containing both ', ...
         'af7r56_trace_mex and af7r56_interval_kkt_mex. ', ...
         'No MEX signature was guessed.']);
end

src0 = fileread(srcFile);
hash0 = af7r57_sha256_file(srcFile);

traceCalls = af7r57_extract_call_statements(src0, 'af7r56_trace_mex');
kktCalls   = af7r57_extract_call_statements(src0, 'af7r56_interval_kkt_mex');

fprintf('\n[A. AF7R56 SOURCE BINDING]\n');
fprintf(' AF7R56B source found                    = 1\n');
fprintf(' source                                  = %s\n', srcFile);
fprintf(' source SHA-256                          = %s\n', hash0);
fprintf(' trace-MEX call statements               = %d\n', numel(traceCalls));
fprintf(' interval-KKT call statements            = %d\n', numel(kktCalls));
fprintf(' exact MEX signature guessed             = 0\n');

fprintf('\n[B. EXACT TRACE-MEX CALL SITES]\n');
for k = 1:numel(traceCalls)
    fprintf(' ---- trace call %d ----\n%s\n', k, traceCalls{k});
end

fprintf('\n[C. EXACT INTERVAL-KKT CALL SITES]\n');
for k = 1:numel(kktCalls)
    fprintf(' ---- KKT call %d ----\n%s\n', k, kktCalls{k});
end

bindingClosed = ~isempty(traceCalls) && ~isempty(kktCalls);

% -------------------------------------------------------------------------
% D. Source-independent common-dyadic theorem self-test.
%
% All endpoints below are binary dyadic numbers.  The routine itself uses
% exact IEEE-754 equality only; no epsilon/tolerance changes topology.
% -------------------------------------------------------------------------
L = [ ...
    1, 0,    1/2;
    1, 1/2,  3/4;
    1, 3/4,  1;
    2, 0,    1      ];

U = [ ...
    1, 0,    1/4;
    1, 1/4,  1/2;
    1, 1/2,  1;
    2, 0,    1/2;
    2, 1/2,  1      ];

[C, A] = af7r57_common_dyadic_refinement(L, U);

expected = [ ...
    1, 0,    1/4;
    1, 1/4,  1/2;
    1, 1/2,  3/4;
    1, 3/4,  1;
    2, 0,    1/2;
    2, 1/2,  1      ];

selfExact = isequal(C, expected);
selfClosed = A.exact_cover && A.lower_refined_exactly && ...
             A.upper_refined_exactly && selfExact;

fprintf('\n[D. COMMON DYADIC REFINEMENT KERNEL]\n');
fprintf(' exact-double topology only              = 1\n');
fprintf(' tolerance-based merge                   = 0\n');
fprintf(' self-test exact common partition        = %d\n', selfExact);
fprintf(' self-test exact cover                    = %d\n', A.exact_cover);
fprintf(' lower partition refined exactly          = %d\n', A.lower_refined_exactly);
fprintf(' upper partition refined exactly          = %d\n', A.upper_refined_exactly);
fprintf(' common dyadic kernel closed              = %d\n', selfClosed);

% -------------------------------------------------------------------------
% E. Three-valued interval sign gate self-test.
% -------------------------------------------------------------------------
c1 = af7r57_interval_sign_classify([-2, -1]);
c2 = af7r57_interval_sign_classify([ 1,  3]);
c3 = af7r57_interval_sign_classify([-1,  2]);

signClosed = strcmp(c1,'CERTIFIED_SAFE') && ...
             strcmp(c2,'CERTIFIED_VIOLATION') && ...
             strcmp(c3,'INCONCLUSIVE_FAIL_CLOSED');

fprintf('\n[E. INTERVAL SIGN GATE]\n');
fprintf(' safe box classification                 = %s\n', c1);
fprintf(' violation box classification            = %s\n', c2);
fprintf(' zero-crossing box classification         = %s\n', c3);
fprintf(' unresolved interval policy               = FAIL CLOSED\n');
fprintf(' interval sign gate closed                = %d\n', signClosed);

% -------------------------------------------------------------------------
% F. Verify no production byte changed.
% -------------------------------------------------------------------------
hash1 = af7r57_sha256_file(srcFile);
productionUnchanged = strcmpi(hash0, hash1);

fprintf('\n[F. FORMAL GATE]\n');
fprintf(' source binding closed                    = %d\n', bindingClosed);
fprintf(' common dyadic kernel closed              = %d\n', selfClosed);
fprintf(' interval sign gate closed                = %d\n', signClosed);
fprintf(' production source SHA unchanged          = %d\n', productionUnchanged);
fprintf(' production source modified               = %d\n', ~productionUnchanged);
fprintf(' production t1 selected                   = 0\n');
fprintf(' continuous terminal-boundary proof       = 0\n');
fprintf(' next theorem                             = COMMON_DYADIC_REFINEMENT_INTERVAL_STAGEB\n');
fprintf('====================================================================================================\n');

R.source = srcFile;
R.source_sha256 = hash0;
R.source_hits = hits;
R.trace_calls = traceCalls;
R.interval_kkt_calls = kktCalls;
R.binding_closed = bindingClosed;
R.common_dyadic_selftest = A;
R.common_dyadic_kernel_closed = selfClosed;
R.interval_sign_gate_closed = signClosed;
R.production_source_unchanged = productionUnchanged;
R.productionModified = ~productionUnchanged;
R.continuous_terminal_boundary_proof = false;
R.production_t1_selected = false;

save('AF7R57_BINDING_REPORT.mat', 'R');

fid = fopen('AF7R57_BINDING_REPORT.txt','w');
if fid >= 0
    c = onCleanup(@() fclose(fid));
    fprintf(fid, 'AF7R57 exact source binding\n');
    fprintf(fid, 'source=%s\n', srcFile);
    fprintf(fid, 'sha256=%s\n', hash0);
    fprintf(fid, 'trace_calls=%d\n', numel(traceCalls));
    for k = 1:numel(traceCalls)
        fprintf(fid, '\n[TRACE %d]\n%s\n', k, traceCalls{k});
    end
    fprintf(fid, 'interval_kkt_calls=%d\n', numel(kktCalls));
    for k = 1:numel(kktCalls)
        fprintf(fid, '\n[KKT %d]\n%s\n', k, kktCalls{k});
    end
end
end


function roots = af7r57_candidate_roots()
roots = {};

% Current working tree.
roots{end+1} = pwd;

% Existing AF7R56 precheck, if on path.
w = which('AF7R56_PRECHECK');
if ~isempty(w)
    roots{end+1} = fileparts(w);
    roots{end+1} = fileparts(fileparts(w));
end

% Existing MEX authority, if on path.
w = which('af7r56_trace_mex');
if ~isempty(w)
    roots{end+1} = fileparts(w);
    roots{end+1} = fileparts(fileparts(w));
end

% This audit's own location.
here = fileparts(mfilename('fullpath'));
if ~isempty(here)
    roots{end+1} = here;
    roots{end+1} = fileparts(here);
end

% Canonical repository location used by the current project, when present.
u = getenv('USERPROFILE');
if ~isempty(u)
    p = fullfile(u, 'Documents', 'GitHub', 'common_matlab');
    if isfolder(p)
        roots{end+1} = p;
        roots{end+1} = fullfile(p, 'reference');
    end
end

% Stable unique, preserving order.
keep = false(size(roots));
seen = {};
for k = 1:numel(roots)
    p = roots{k};
    if isempty(p) || ~isfolder(p), continue; end
    p = char(java.io.File(p).getCanonicalPath());
    if ~any(strcmpi(seen,p))
        seen{end+1} = p; %#ok<AGROW>
        roots{k} = p;
        keep(k) = true;
    end
end
roots = roots(keep);
end


function [srcFile, hits] = af7r57_find_af7r56b_source(roots)
srcFile = '';
hits = struct('file',{},'hasTrace',{},'hasKKT',{},'hasBoundaryText',{},'score',{});

cand = {};
for ir = 1:numel(roots)
    r = roots{ir};
    d = [ ...
        dir(fullfile(r,'AF7R56*.m')); ...
        dir(fullfile(r,'*AF7R56*.m'))  ...
        ];
    rr = fullfile(r,'reference');
    if isfolder(rr)
        d = [d; dir(fullfile(rr,'AF7R56*.m')); dir(fullfile(rr,'*AF7R56*.m'))]; %#ok<AGROW>
    end
    for k = 1:numel(d)
        if d(k).isdir, continue; end
        cand{end+1} = fullfile(d(k).folder,d(k).name); %#ok<AGROW>
    end
end

% Unique exact paths.
uc = {};
for k = 1:numel(cand)
    p = char(java.io.File(cand{k}).getCanonicalPath());
    if ~any(strcmpi(uc,p)), uc{end+1}=p; end %#ok<AGROW>
end

bestScore = -Inf;
for k = 1:numel(uc)
    p = uc{k};
    try
        s = fileread(p);
    catch
        continue;
    end
    hT = contains(s,'af7r56_trace_mex');
    hK = contains(s,'af7r56_interval_kkt_mex');
    hB = contains(s,'actual fixed boundary bracket') || ...
         contains(s,'STAGE-B ENDPOINT LEAF TOPOLOGY') || ...
         contains(s,'Stage-B');
    score = 100*double(hT) + 100*double(hK) + 10*double(hB);
    hits(end+1) = struct('file',p,'hasTrace',hT,'hasKKT',hK, ...
        'hasBoundaryText',hB,'score',score); %#ok<AGROW>
    if hT && hK && score > bestScore
        srcFile = p;
        bestScore = score;
    end
end
end


function calls = af7r57_extract_call_statements(src, token)
% Return exact source statements containing TOKEN.
% This is source binding only; it does not interpret or change the call.
lines = regexp(src, '\r\n|\n|\r', 'split');
idx = find(contains(lines, token));
calls = cell(0,1);

for q = 1:numel(idx)
    i0 = idx(q);
    a = i0;
    while a > 1
        prev = strtrim(lines{a-1});
        if endsWith(prev,'...')
            a = a - 1;
        else
            break;
        end
    end

    b = i0;
    block = strjoin(lines(a:b), newline);
    while b < numel(lines)
        if ~endsWith(strtrim(lines{b}),'...') && ~af7r57_unclosed_delims(block)
            break;
        end
        b = b + 1;
        block = strjoin(lines(a:b), newline);
    end

    block = strtrim(block);
    if ~any(strcmp(calls,block))
        calls{end+1,1} = block; %#ok<AGROW>
    end
end
end


function tf = af7r57_unclosed_delims(s)
% Lightweight lexical balance sufficient only for displaying a complete
% MATLAB call statement.  Strings/comments are not semantically interpreted.
op = count(s,'(') - count(s,')');
ob = count(s,'[') - count(s,']');
oc = count(s,'{') - count(s,'}');
tf = (op > 0) || (ob > 0) || (oc > 0);
end


function h = af7r57_sha256_file(fn)
md = java.security.MessageDigest.getInstance('SHA-256');
fid = fopen(fn,'rb');
if fid < 0
    error('AF7R57:OpenFile','Cannot open %s',fn);
end
c = onCleanup(@() fclose(fid));
while true
    b = fread(fid, 1024*1024, '*uint8');
    if isempty(b), break; end
    md.update(b);
end
raw = typecast(md.digest(),'uint8');
h = lower(reshape(dec2hex(raw,2).',1,[]));
end
