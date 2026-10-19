function OUT = AF7R58F_ONEFILE_LIVE_LIFETIME_FORENSIC(packet)
% AF7R58F: source-bound forensic capture for the next AF7R58 closure step.
%
% Put this file in common_matlab/reference and run:
%     AF7R58F_ONEFILE_LIVE_LIFETIME_FORENSIC
%
% Optional explicit packet (NO guessing, NO defaults selected as production):
%     P = struct();
%     P.tree = struct('t0',...,'t1',...,'dt',...);
%     AF7R58F_ONEFILE_LIVE_LIFETIME_FORENSIC(P)
%
% Return the two files printed at the end:
%     AF7R58F_REPORT.txt
%     AF7R58F_OUT.mat
%
% PURPOSE
%   1) Re-bind the reviewed AF7R36 scalar-tree source used by AF7R58E.
%   2) Capture exact current source evidence for live t0/t1/dt authority.
%   3) Capture exact current source evidence for commit/version lifetime,
%      retention, pruning/release, and candidate/group/segment storage.
%   4) Capture source/build evidence relevant to the floating-point contract.
%   5) Preserve the runtime-parameter contract:
%        - preflight tunables / actuator limits are externally supplied;
%        - payload mass mL is NOT frozen here and must come from the latest
%          in-flight estimator authority when actuator/tension feasibility is
%          audited later.
%
% IMPORTANT
%   V2 FAST: one-pass combined-regex source scan; bounded evidence storage.
%   This is a READ-ONLY FORENSIC audit. Regex/source hits are evidence capture,
%   not a proof by themselves. It does not modify production, fly, simulate,
%   accept safety by samples, claim 4 ms, or close actuator/tension feasibility.
%   Missing/ambiguous evidence stays OPEN; nothing is guessed SAFE.

if nargin < 1
    packet = [];
end

root = fileparts(mfilename('fullpath'));      % common_matlab/reference
repo = fileparts(root);                       % common_matlab
outdir = tempname(root);
[ok,msg] = mkdir(outdir);
assert(ok,'%s',msg);

reportPath = fullfile(outdir,'AF7R58F_REPORT.txt');
f = fopen(reportPath,'wt','n','UTF-8');
assert(f >= 0,'[AF7R58F] cannot open report.');
cleanup = onCleanup(@()fclose(f)); %#ok<NASGU>

say(f,'====================================================================================================');
say(f,' AF7R58F | LIVE AUTHORITY + COMMIT/LIFETIME + FP-ENV FORENSIC CAPTURE');
say(f,'====================================================================================================');
say(f,'[AUTHORITY] readOnly=1 | productionModified=0 | noFlight=1 | noSimulation=1 | sampledSafetyAcceptance=0');
say(f,' root = %s',root);
say(f,' repo = %s',repo);

%% A. Explicit design contract carried forward -- declared, not source-closed here.
CONTRACT = struct();
CONTRACT.preflight_tunables_external = true;
CONTRACT.actuator_limits_external_preflight = true;
CONTRACT.payload_mass_live_estimator_authority = true;
CONTRACT.payload_mass_not_frozen_by_this_audit = true;
CONTRACT.no_numeric_actuator_limit_selected = true;
CONTRACT.no_numeric_vehicle_parameter_selected = true;
CONTRACT.no_tracking_margin_invented = true;
CONTRACT.failure_backup_deferred = true;

say(f,'');
say(f,'[A. RUNTIME PARAMETER AUTHORITY CONTRACT -- DECLARED / NOT YET SOURCE-CLOSED]');
say(f,' preflight tunables supplied externally       = %d',CONTRACT.preflight_tunables_external);
say(f,' actuator/input limits supplied externally    = %d',CONTRACT.actuator_limits_external_preflight);
say(f,' payload mass mL uses live estimator authority= %d',CONTRACT.payload_mass_live_estimator_authority);
say(f,' this audit freezes no mL value               = %d',CONTRACT.payload_mass_not_frozen_by_this_audit);
say(f,' numeric thrust/moment limits selected here   = %d',~CONTRACT.no_numeric_actuator_limit_selected);
say(f,' numeric vehicle parameters selected here     = %d',~CONTRACT.no_numeric_vehicle_parameter_selected);

%% B. Re-bind the exact AF7R58E-reviewed core.
coreExpected = 'ad5ce1970583c0313637dcac58b8cf949092fe53e201efc61e55754fb2ea79be';
corePath = fullfile(root,'af7r36_core.hpp');
corePresent = isfile(corePath);
coreHash = '';
if corePresent
    coreHash = file_hash(corePath);
end
coreBound = corePresent && strcmp(coreHash,coreExpected);

say(f,'');
say(f,'[B. AF7R58E CORE RE-BINDING]');
say(f,' af7r36_core.hpp present                   = %d',corePresent);
say(f,' reviewed SHA-256 match                    = %d',coreBound);
say(f,' expected SHA-256                          = %s',coreExpected);
say(f,' current  SHA-256                          = %s',coreHash);

%% C. Scan current source without assuming where the lifetime authority lives.
[files,skipped] = collect_code_files(repo);
say(f,'');
say(f,'[C. CURRENT SOURCE INVENTORY]');
say(f,' scanned code files                        = %d',numel(files));
say(f,' skipped files                             = %d',numel(skipped));

% Exact source names are useful anchors, but absence is not auto-failure because
% authority may have moved. All matching paths/hashes are returned in OUT.
ANCHORS = {'af7r36_core.hpp','REPLANNING_SYSTEM.m'};
anchors = struct('name',{},'paths',{},'hashes',{},'count',{},'unique',{});
for i = 1:numel(ANCHORS)
    a = find_anchor(files,ANCHORS{i});
    anchors(end+1) = a; %#ok<AGROW>
    say(f,' anchor %-28s count=%d unique=%d',a.name,a.count,a.unique);
    for j = 1:a.count
        say(f,'   [%d] %s | SHA-256=%s',j,a.paths{j},a.hashes{j});
    end
end

% Keep full text of a small, deterministic authority-candidate set in the MAT so
% the next audit can inspect exact semantics without requesting many extra files.
snapshots = capture_authority_snapshots(files);
say(f,' authority-source snapshots captured        = %d',numel(snapshots));
for i = 1:numel(snapshots)
    say(f,'   SNAPSHOT[%d] %s | SHA-256=%s',i,relative_path(snapshots(i).path,repo),snapshots(i).sha256);
end

%% D. Forensic source classes.
% Deliberately broad enough to find moved/renamed authority. The MAT stores ALL
% hits, subject only to an explicit per-file evidence-storage cap.  Any truncation
% is recorded and remains OPEN; it is never evidence of absence.  The TXT prints
% bounded excerpts so it remains practical to return.
CLASS = struct([]);
CLASS(1).name = 'TREE_INTERVAL_AUTHORITY';
CLASS(1).patterns = { ...
    '\<build_candidate\>', '\<t0\>', '\<t1\>', '\<dt\>', ...
    'episode', 'collision.*interval', 'interval.*collision', 'tree'};
CLASS(2).name = 'COMMIT_VERSION_LIFETIME';
CLASS(2).patterns = { ...
    'commit', 'activate', 'active[_A-Za-z0-9]*version', 'version[_A-Za-z0-9]*active', ...
    'append', 'push_back', 'emplace_back', 'erase', 'clear\s*\(', 'prune', ...
    'garbage', 'retire', 'release', 'delete', 'remove', 'lifetime', 'history'};
CLASS(3).name = 'GROUP_SEGMENT_STORAGE';
CLASS(3).patterns = { ...
    'group', 'segment', 'candidate', 'episode', 'branch', 'version', ...
    'reserve\s*\(', 'resize\s*\(', 'capacity\s*\(', 'vector\s*<', 'cell\s*\('};
CLASS(4).name = 'FLOAT_BUILD_ENVIRONMENT';
CLASS(4).patterns = { ...
    'mex', 'mexcuda', 'CXXFLAGS', 'CFLAGS', 'LDFLAGS', 'fast-math', 'ffast-math', ...
    '/fp:', 'frounding', 'rounding', 'fesetround', 'fegetround', 'FLT_ROUNDS', ...
    'is_iec559', 'IEEE', 'binary64', 'fp-contract', 'ffp-contract'};
CLASS(5).name = 'RUNTIME_PARAMETER_BINDING';
CLASS(5).patterns = { ...
    'config', 'parameter', 'runtime', 'thrust', 'moment', 'inertia', ...
    '\<mQ\>', '\<mL\>', 'payload.*mass', 'mass.*payload', 'cable.*length', ...
    '\<gravity\>', '\<sensor_range\>', '\<horizon\>'};

% FAST V2 SCAN: each source file is read once.  Each class uses one combined
% whole-file regex; matching offsets are then mapped back to exact source lines.
% To keep the forensic artifact bounded, stored hits are capped per class/file;
% truncation is explicitly reported and can NEVER be interpreted as absence.
say(f,'');
say(f,'[D0. FAST SOURCE SCAN]');
say(f,' each file read once; whole-file combined-regex classes=%d',numel(CLASS));
tScan = tic;
[allHits,classSummary,scanMeta] = scan_all_classes_fast(files,CLASS);
say(f,' scan elapsed [s]                          = %.6f',toc(tScan));
say(f,' scan truncated anywhere                    = %d',scanMeta.any_truncated);
say(f,' total omitted hits after caps              = %d',scanMeta.total_omitted_hits);
say(f,' regex calls                                = %d',scanMeta.regex_calls);

say(f,'');
say(f,'[D. FORENSIC HIT SUMMARY -- HITS ARE NOT SAFETY PROOF]');
for c = 1:numel(classSummary)
    say(f,' %-30s stored_hits=%d files=%d omitted=%d truncated=%d',classSummary(c).name, ...
        classSummary(c).hit_count,classSummary(c).file_count, ...
        classSummary(c).omitted_hit_count,classSummary(c).truncated);
end

%% E. Print focused snippets.
% Priority files are printed first. Stored hits remain in AF7R58F_OUT.mat; any
% storage truncation is explicitly recorded and never used as absence evidence.
MAX_PRINT_PER_CLASS = 80;
say(f,'');
say(f,'[E. SOURCE EXCERPTS -- EXACT LINE TEXT; STORED HIT SET IS IN MAT]');
for c = 1:numel(CLASS)
    idx = find(strcmp({allHits.class},CLASS(c).name));
    idx = prioritize_hits(allHits,idx,root);
    say(f,'');
    say(f,'--- %s | showing %d of %d hits ---',CLASS(c).name, ...
        min(numel(idx),MAX_PRINT_PER_CLASS),numel(idx));
    nshow = min(numel(idx),MAX_PRINT_PER_CLASS);
    for k = 1:nshow
        h = allHits(idx(k));
        say(f,'%s:%d | %s',relative_path(h.file,repo),h.line,h.text);
    end
    if numel(idx) > MAX_PRINT_PER_CLASS
        say(f,'[TRUNCATED IN TXT] %d additional STORED hits retained in AF7R58F_OUT.mat.', ...
            numel(idx)-MAX_PRINT_PER_CLASS);
    end
end

%% F. Conservative forensic classifications.
% These are deliberately NOT closure claims. They only state what evidence was
% observed by this capture. A regex absence cannot prove semantic absence.
lifeIdx = find(strcmp({allHits.class},'COMMIT_VERSION_LIFETIME'));
lifeText = lower(strjoin({allHits(lifeIdx).text},sprintf('\n')));
hasAppendLike = any_token(lifeText,{'append','push_back','emplace_back','history'});
hasReleaseLike = any_token(lifeText,{'erase','clear(','prune','garbage','retire','release','delete','remove'});

fpIdx = find(strcmp({allHits.class},'FLOAT_BUILD_ENVIRONMENT'));
fpText = lower(strjoin({allHits(fpIdx).text},sprintf('\n')));
hasFastMathToken = any_token(fpText,{'fast-math','ffast-math','/fp:fast'});
hasStrictFpToken = any_token(fpText,{'/fp:strict','frounding','fesetround','fegetround','ffp-contract=off'});

say(f,'');
say(f,'[F. CONSERVATIVE FORENSIC FLAGS -- NOT CLOSURE]');
say(f,' append/history-like evidence observed       = %d',hasAppendLike);
say(f,' release/prune-like evidence observed        = %d',hasReleaseLike);
say(f,' fast-math-related token observed            = %d',hasFastMathToken);
say(f,' strict/rounding-related token observed      = %d',hasStrictFpToken);
say(f,' NOTE: absence of a regex token never proves semantic absence.');
say(f,' evidence-storage truncation observed           = %d',scanMeta.any_truncated);
say(f,' NOTE: truncation also never proves semantic absence; exact source remains authority.');

%% G. Optional explicit tree packet: compute only what AF7R58E already justified.
packetSupplied = ~isempty(packet);
packetTreeSupplied = false;
packetTree = struct();
packetFields = {};
if packetSupplied
    assert(isstruct(packet) && isscalar(packet),'[AF7R58F] packet must be a scalar struct.');
    packetFields = fieldnames(packet);
    if isfield(packet,'tree')
        assert(isstruct(packet.tree) && isscalar(packet.tree) && ...
            all(isfield(packet.tree,{'t0','t1','dt'})), ...
            '[AF7R58F] packet.tree requires explicit scalar t0,t1,dt.');
        packetTreeSupplied = true;
        packetTree = tree_bound(packet.tree.t0,packet.tree.t1,packet.tree.dt);
    elseif all(isfield(packet,{'t0','t1','dt'}))
        % Backward-compatible explicit form matching AF7R58E. Still explicit.
        packetTreeSupplied = true;
        packetTree = tree_bound(packet.t0,packet.t1,packet.dt);
    end
end

say(f,'');
say(f,'[G. EXPLICIT PACKET -- NO RUNTIME GUESSING]');
say(f,' packet supplied                            = %d',packetSupplied);
if packetSupplied
    say(f,' packet top-level fields                   = %s',join_cells(packetFields,', '));
end
say(f,' explicit tree packet supplied              = %d',packetTreeSupplied);
if packetTreeSupplied
    say(f,' tree status                               = %s',packetTree.status);
    say(f,' K / leaf exponent                        = %g / %g',packetTree.depth,packetTree.leaf_bound_exponent);
    say(f,' component exponent                       = %g',packetTree.component_bound_exponent);
    if packetTree.numeric_counts_exact
        say(f,' leaf / component / S_group bounds        = %.17g / %.17g / %.17g', ...
            packetTree.leaf_bound,packetTree.component_bound,packetTree.segment_bound);
    else
        say(f,' numeric counts exact                     = 0 | symbolic exponents retained');
    end
else
    say(f,' current production t0/t1/dt are NOT guessed and no production numeric cap is selected.');
end
say(f,' payload mass mL from packet is NOT consumed by this audit.');
say(f,' thrust/moment limits from packet are NOT consumed by this audit.');

%% H. Closure ledger.
% Source capture can tell the next audit exactly where to bind, but cannot turn
% source regex or a user declaration into a semantic lifetime/floating-point proof.
ledger = struct();
ledger.core_source_rebound = coreBound;
ledger.source_inventory_captured = ~isempty(files);
ledger.live_interval_semantics_closed = false;
ledger.arithmetic_environment_closed = false;
ledger.commit_lifetime_semantics_closed = false;
ledger.Gmax_closed = false;
ledger.Smax_closed = false;
ledger.production_4ms_closed = false;
ledger.actuator_feasibility_closed = false;
ledger.tension_feasibility_closed = false;
ledger.payload_mass_live_binding_closed = false;
ledger.failure_backup_closed = false;

if ~coreBound
    status = 'OPEN_CORE_SOURCE_BINDING';
elseif isempty(files)
    status = 'OPEN_SOURCE_INVENTORY';
else
    status = 'FORENSIC_CAPTURE_COMPLETE__RETURN_TXT_AND_MAT_FOR_EXACT_LIFETIME_BINDING';
end

say(f,'');
say(f,'[H. CLOSURE LEDGER]');
say(f,' core source re-bound                       = %d',ledger.core_source_rebound);
say(f,' source inventory captured                  = %d',ledger.source_inventory_captured);
say(f,' live interval semantics closed             = %d',ledger.live_interval_semantics_closed);
say(f,' arithmetic environment closed              = %d',ledger.arithmetic_environment_closed);
say(f,' commit/lifetime semantics closed           = %d',ledger.commit_lifetime_semantics_closed);
say(f,' Gmax / Smax closed                         = %d / %d',ledger.Gmax_closed,ledger.Smax_closed);
say(f,' 4 ms closed                                = %d',ledger.production_4ms_closed);
say(f,' tension / actuator feasibility closed      = %d / %d', ...
    ledger.tension_feasibility_closed,ledger.actuator_feasibility_closed);
say(f,' live mL binding closed                     = %d',ledger.payload_mass_live_binding_closed);
say(f,' failure backup closed                      = %d',ledger.failure_backup_closed);
say(f,' status                                     = %s',status);

%% I. Save return artifact.
OUT = struct();
OUT.schema = 'AF7R58F_LIVE_LIFETIME_FORENSIC_V2_FAST';
OUT.contract = CONTRACT;
OUT.root = root;
OUT.repo = repo;
OUT.core_expected_sha256 = coreExpected;
OUT.core_path = corePath;
OUT.core_present = corePresent;
OUT.core_sha256 = coreHash;
OUT.core_bound = coreBound;
OUT.files = files;
OUT.skipped_files = skipped;
OUT.anchors = anchors;
OUT.authority_snapshots = snapshots;
OUT.classes = CLASS;
OUT.class_summary = classSummary;
OUT.scan_meta = scanMeta;
OUT.hits = allHits;
OUT.forensic_flags = struct( ...
    'append_or_history_token_observed',hasAppendLike, ...
    'release_or_prune_token_observed',hasReleaseLike, ...
    'fast_math_token_observed',hasFastMathToken, ...
    'strict_rounding_token_observed',hasStrictFpToken, ...
    'regex_absence_is_not_semantic_proof',true);
OUT.packet_supplied = packetSupplied;
OUT.packet_fields = packetFields;
OUT.packet_tree_supplied = packetTreeSupplied;
OUT.packet_tree_bound = packetTree;
OUT.ledger = ledger;
OUT.production_caps_selected = false;
OUT.production_modified = false;
OUT.status = status;
OUT.next_action = 'RETURN_REPORT_AND_OUT__THEN_SOURCE_BOUND_COMMIT_LIFETIME_AND_LIVE_AUTHORITY_PROOF';
OUT.required_return_files = {'AF7R58F_REPORT.txt','AF7R58F_OUT.mat'};
OUT.output_directory = outdir;

matPath = fullfile(outdir,'AF7R58F_OUT.mat');
save(matPath,'OUT','-v7');
assignin('base','AF7R58F_OUT',OUT);

say(f,'');
say(f,'[RETURN THESE TWO FILES]');
say(f,' RETURN: %s',reportPath);
say(f,' RETURN: %s',matPath);
say(f,'');
say(f,'Next audit after return: use the captured exact source/lifetime/build authority;');
say(f,'do not hardcode preflight parameters, actuator limits, or payload mass mL.');
say(f,'Payload mass remains live-estimator authority for the later tension/thrust/moment certificate.');
end

%% ================================================================================================
function [F,skipped] = collect_code_files(repo)
% Source-only inventory. Avoid known generated/metadata directories.
exts = {'*.m','*.hpp','*.h','*.hh','*.cpp','*.cc','*.cxx','*.c','*.cu','*.mk','*.cmake','CMakeLists.txt'};
F = struct('path',{},'name',{},'bytes',{},'sha256',{});
skipped = struct('path',{},'reason',{});
seen = containers.Map('KeyType','char','ValueType','logical');
MAX_BYTES = 8*1024*1024;
for e = 1:numel(exts)
    D = dir(fullfile(repo,'**',exts{e}));
    for i = 1:numel(D)
        if D(i).isdir, continue; end
        p = fullfile(D(i).folder,D(i).name);
        if strcmpi(D(i).name,'AF7R58F_ONEFILE_LIVE_LIFETIME_FORENSIC.m')
            continue; % never use this auditor as evidence about production semantics
        end
        key = lower(p);
        if isKey(seen,key), continue; end
        seen(key) = true;
        if path_excluded(p)
            skipped(end+1) = struct('path',p,'reason','excluded_generated_or_metadata_path'); %#ok<AGROW>
            continue;
        end
        if D(i).bytes > MAX_BYTES
            skipped(end+1) = struct('path',p,'reason','source_file_over_8MiB'); %#ok<AGROW>
            continue;
        end
        h = '';
        try
            h = file_hash(p);
        catch ME
            skipped(end+1) = struct('path',p,'reason',['hash_failed:' ME.identifier]); %#ok<AGROW>
            continue;
        end
        F(end+1) = struct('path',p,'name',D(i).name,'bytes',D(i).bytes,'sha256',h); %#ok<AGROW>
    end
end
if ~isempty(F)
    keys = cellfun(@lower,{F.path},'UniformOutput',false);
    [~,ord] = sort(keys);
    F = F(ord);
end
end

function tf = path_excluded(p)
q = lower(strrep(p,'\','/'));
bad = {'/.git/','/node_modules/','/__pycache__/','/.cache/','/build/','/dist/'};
tf = false;
for i = 1:numel(bad)
    if contains(q,bad{i})
        tf = true;
        return;
    end
end
% AF7R temporary return folders created by tempname commonly start with tp*.
% Do not exclude arbitrary reference files; only child directories beginning /tp.
parts = strsplit(q,'/');
for i = 1:numel(parts)
    s = parts{i};
    if numel(s) >= 2 && strcmp(s(1:2),'tp') && numel(s) > 12
        % Long temp-like directory names only.
        tf = true;
        return;
    end
end
end

function A = find_anchor(F,name)
idx = find(strcmpi({F.name},name));
A = struct();
A.name = name;
A.paths = {F(idx).path};
A.hashes = {F(idx).sha256};
A.count = numel(idx);
A.unique = (A.count == 1);
end

function S = capture_authority_snapshots(F)
% Exact full-text snapshots of known/current authority candidates only.
% Bounded count avoids turning the MAT into a repository archive.
S = struct('path',{},'sha256',{},'text',{});
idx = [];
for i = 1:numel(F)
    n = lower(F(i).name);
    if strcmp(n,'af7r36_core.hpp') || contains(n,'replanning_system') || ...
            startsWith(n,'af7r36')
        idx(end+1) = i; %#ok<AGROW>
    end
end
MAX_SNAPSHOTS = 24;
idx = idx(1:min(numel(idx),MAX_SNAPSHOTS));
for k = 1:numel(idx)
    i = idx(k);
    try
        txt = fileread(F(i).path);
    catch
        txt = '';
    end
    S(end+1) = struct('path',F(i).path,'sha256',F(i).sha256,'text',txt); %#ok<AGROW>
end
end

function [H,SUM,META] = scan_all_classes_fast(F,CLASS)
% V2 fast scanner: each source file is read once, then each class is matched
% against the WHOLE file once.  Matching character offsets are mapped back to
% exact source lines.  This avoids regex calls inside the line loop entirely.
% A storage cap bounds MAT/report growth.  Truncation is evidence-preserving:
% it sets META.any_truncated and NEVER supports a semantic-absence conclusion.
H = struct('class',{},'file',{},'sha256',{},'line',{},'text',{},'context',{});
SUM = struct('name',{},'hit_count',{},'file_count',{},'omitted_hit_count',{},'truncated',{});
NC = numel(CLASS);
expr = cell(1,NC);
for c = 1:NC
    expr{c} = ['(?:' strjoin(CLASS(c).patterns,'|') ')'];
    SUM(c).name = CLASS(c).name;
    SUM(c).hit_count = 0;
    SUM(c).file_count = 0;
    SUM(c).omitted_hit_count = 0;
    SUM(c).truncated = false;
end

% Evidence-storage bound only.  It does not cap production objects and is not
% used for a SAFE decision.
MAX_STORED_HITS_PER_CLASS_FILE = 160;
fileHit = false(NC,numel(F));
META = struct();
META.scanner = 'ONE_READ__ONE_WHOLE_FILE_REGEX_PER_CLASS_V2';
META.max_stored_hits_per_class_file = MAX_STORED_HITS_PER_CLASS_FILE;
META.any_truncated = false;
META.total_omitted_hits = 0;
META.files_read = 0;
META.read_failures = 0;
META.regex_calls = 0;

for i = 1:numel(F)
    p = F(i).path;
    try
        txt = fileread(p);
        META.files_read = META.files_read + 1;
    catch
        META.read_failures = META.read_failures + 1;
        continue;
    end

    % Canonicalize line endings once so character offsets and line starts agree.
    txt = strrep(txt,sprintf('\r\n'),sprintf('\n'));
    txt = strrep(txt,sprintf('\r'),sprintf('\n'));
    classLines = cell(1,NC);
    anyHit = false;
    nl = find(txt == sprintf('\n'));
    lineStarts = [1 nl+1];
    edges = [lineStarts numel(txt)+2];

    for c = 1:NC
        pos = regexpi(txt,expr{c},'start');
        META.regex_calls = META.regex_calls + 1;
        if isempty(pos)
            classLines{c} = [];
            continue;
        end
        anyHit = true;
        fileHit(c,i) = true;
        ln = discretize(pos,edges);
        ln = unique(ln(~isnan(ln)),'stable');
        classLines{c} = ln;
    end
    if ~anyHit
        continue;
    end

    L = regexp(txt,'\n','split');
    for c = 1:NC
        ln = classLines{c};
        if isempty(ln), continue; end
        nStore = min(numel(ln),MAX_STORED_HITS_PER_CLASS_FILE);
        for k = 1:nStore
            n = ln(k);
            lo = max(1,n-3);
            hi = min(numel(L),n+3);
            C = cell(1,hi-lo+1);
            z = 0;
            for q = lo:hi
                z = z + 1;
                C{z} = sprintf('%d | %s',q,L{q});
            end
            ctx = strjoin(C,sprintf('\n'));
            H(end+1) = struct('class',CLASS(c).name,'file',p,'sha256',F(i).sha256, ... %#ok<AGROW>
                'line',n,'text',strtrim(L{n}),'context',ctx);
        end
        omitted = numel(ln)-nStore;
        if omitted > 0
            SUM(c).omitted_hit_count = SUM(c).omitted_hit_count + omitted;
            SUM(c).truncated = true;
            META.any_truncated = true;
            META.total_omitted_hits = META.total_omitted_hits + omitted;
        end
    end
end

for c = 1:NC
    idx = strcmp({H.class},CLASS(c).name);
    SUM(c).hit_count = sum(idx);
    SUM(c).file_count = sum(fileHit(c,:));
end
end

function idx = prioritize_hits(H,idx,root)
if isempty(idx), return; end
score = zeros(size(idx));
for k = 1:numel(idx)
    p = lower(strrep(H(idx(k)).file,'\','/'));
    r = lower(strrep(root,'\','/'));
    if strcmpi(H(idx(k)).file,fullfile(root,'af7r36_core.hpp')), score(k)=score(k)-1000; end
    if strcmpi(H(idx(k)).text,'') , score(k)=score(k)+10; end
    if contains(lower(H(idx(k)).file),'replanning_system'), score(k)=score(k)-500; end
    if startsWith(p,[r '/']), score(k)=score(k)-100; end
    score(k)=score(k)+k*1e-9; % stable deterministic tie breaker
end
[~,o] = sort(score);
idx = idx(o);
end

function B = tree_bound(a,b,dt)
% Same conservative scalar recurrence used in AF7R58E.
B = struct('computed',false,'status','OPEN_INVALID_INPUT','depth',NaN, ...
    'leaf_bound',NaN,'component_bound',NaN,'segment_bound',NaN, ...
    'leaf_bound_exponent',NaN,'component_bound_exponent',NaN, ...
    'threshold',NaN,'midpoint_error_bound',NaN,'width_bounds',[], ...
    'numeric_counts_exact',false);
if ~(isa(a,'double') && isa(b,'double') && isa(dt,'double') && ...
        isreal(a) && isreal(b) && isreal(dt) && ...
        isscalar(a) && isscalar(b) && isscalar(dt) && ...
        isfinite(a) && isfinite(b) && isfinite(dt) && b>a && dt>0)
    return;
end
M = max([1 abs(a) abs(b)]);
if M > realmax/4
    B.status = 'OPEN_ARITHMETIC_RANGE';
    return;
end
eta = 2*eps(M);
tol = 64*eps(1)*max([1 abs(a) abs(b) abs(dt)]);
tau = dt + tol;
if ~isfinite(tau)
    B.status = 'OPEN_THRESHOLD_OVERFLOW';
    return;
end
W = up_nonnegative(b-a);
K = 0;
hist = W;
while up_nonnegative(W) > tau
    V = up_nonnegative(up_nonnegative(W/2)+eta);
    if ~isfinite(V) || ~(V < W)
        B.status = 'OPEN_WIDTH_RECURRENCE_NOT_CONTRACTING';
        B.threshold = tau;
        B.midpoint_error_bound = eta;
        B.width_bounds = hist;
        return;
    end
    W = V;
    K = K+1;
    hist(end+1) = W; %#ok<AGROW>
end
B.computed = true;
B.status = 'CONDITIONAL_BOUND_COMPUTED';
B.depth = K;
B.leaf_bound_exponent = K;
B.component_bound_exponent = max(K-1,0);
B.threshold = tau;
B.midpoint_error_bound = eta;
B.width_bounds = hist;
if K <= 52
    B.leaf_bound = 2^K;
    B.component_bound = 2^max(K-1,0);
    B.segment_bound = 2*B.component_bound + 1;
    B.numeric_counts_exact = true;
else
    B.status = 'CONDITIONAL_SYMBOLIC_COUNT_ONLY';
end
end

function y = up_nonnegative(x)
assert(isreal(x) && isscalar(x) && x>=0,'[AF7R58F] up needs nonnegative scalar.');
if isinf(x), y=x; return; end
if x==0, x=0; end
u = typecast(double(x),'uint64');
y = typecast(u+uint64(1),'double');
end

function tf = any_token(s,tokens)
tf = false;
for i = 1:numel(tokens)
    if contains(s,lower(tokens{i}))
        tf = true;
        return;
    end
end
end

function s = relative_path(p,base)
pn = strrep(p,'\','/');
bn = strrep(base,'\','/');
if startsWith(lower(pn),[lower(bn) '/'])
    s = pn(numel(bn)+2:end);
else
    s = pn;
end
end

function s = join_cells(C,sep)
if isempty(C)
    s = '(none)';
    return;
end
s = C{1};
for i = 2:numel(C)
    s = [s sep C{i}]; %#ok<AGROW>
end
end

function say(f,fmt,varargin)
s = sprintf(fmt,varargin{:});
fprintf(1,'%s\n',s);
fprintf(f,'%s\n',s);
end

function h = file_hash(p)
f = fopen(p,'rb');
assert(f>=0,'cannot read %s',p);
c = onCleanup(@()fclose(f)); %#ok<NASGU>
b = fread(f,Inf,'*uint8');
m = java.security.MessageDigest.getInstance('SHA-256');
m.update(b);
d = typecast(m.digest(),'uint8');
h = lower(reshape(dec2hex(d,2).',1,[]));
end
