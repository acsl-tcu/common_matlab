function OUT = AF7R58C_ONEFILE_RESOURCE_BOUND_SOURCE_AUDIT(root)
% AF7R58C: read-only source evidence for Tcorr_max / Dcommit_min / Cmax.
% Place in common_matlab/reference and invoke without arguments.
% Optional root is the directory containing af7r36_core.hpp.
% Does NOT run Flight, simulation, constructors, MEX, or previous audits.
% Does NOT edit production or select caps. Regex hits are evidence locations,
% NOT a parser, control-flow proof, call-graph proof or theorem certificate.
% Full captured source bytes/text are retained in OUT.mat for review.
% Outputs go to a NEW directory under root on every invocation.
%
% CONDITIONAL counting argument (not yet source-validated):
% If each retained correction committed at c has expiry <= c+T,
% commit times are separated by D>0, and expired groups are pruned before
% admission, a conservative capacity including one prospective append is
% ceil(T/D)+1. A bound on t1-t0 alone is insufficient unless the relationship
% between t0 and commit time is also established (no unbounded future start).
% S_group <= 2*Cmax+1 must hold for the ACTUAL production candidate builder.
% Cmax is unsafe connected-component count, NOT obstacle count.
% A finite G/S cap does not bound all adaptive work or prove a 4 ms WCET.

if nargin<1 || isempty(root), root=fileparts(mfilename('fullpath')); end
root=char(root);
assert(isfolder(root),'[AF7R58C] source root does not exist: %s',root);
self=[mfilename('fullpath') '.m'];
outdir=tempname(root); [ok,msg]=mkdir(outdir);
assert(ok,'[AF7R58C] cannot create output directory: %s',msg);
logfile=fullfile(outdir,'AF7R58C_REPORT.txt');
fid=fopen(logfile,'wt','n','UTF-8');
assert(fid>=0,'[AF7R58C] cannot open report.');
cleaner=onCleanup(@()fclose(fid)); %#ok<NASGU>
say(fid,'====================================================================================================');
say(fid,' AF7R58C | RESOURCE-BOUND SOURCE EVIDENCE - NO PRODUCTION INSTALL');
say(fid,'====================================================================================================');
say(fid,'[AUTHORITY] readOnlyProduction=1 | regexIsNotProof=1 | noCapGuess=1 | flightRun=0 | mexRun=0');
say(fid,' entrypoint = %s',self);
say(fid,' entrypoint SHA-256 = %s',file_hash(self));
say(fid,' source root = %s',root);
say(fid,' report directory = %s',outdir);

names={'af7r36_single_core_mex.cpp','af7r36_core.hpp', ...
    'af7r33_make_batch_model_from_proposal.m','af7r34_make_core_packet.m', ...
    'REPLANNING_SYSTEM.m', ...
    'AF7R58B_ONEFILE_EXACT_PRUNE_ORDINARY_RETURN_SHADOW_AUDIT.m'};
expected={ ...
    '4f9778723aa3aa339d5167b90845fcfcaa8564cae68326ff1e4ed504c7e6daea', ...
    'ad5ce1970583c0313637dcac58b8cf949092fe53e201efc61e55754fb2ea79be', ...
    'f0e9f487cd270430d191871366c84595be586b6da0fb7b93eb98f4dfc25b46fd', ...
    'ce1f98dac7b49ec8510c09d5c89b7edd4aea64a6c6320e7d2752431528cf22f8', ...
    '223d34c49d7148b8ea14ff4ab9c420d409560c0650ac19e9b0f99318887090c9', ...
    '30d4fa6555101ec09f2bfc552e0bf000291f23b8b9adcf90f312b07f002ba18d'};
S=repmat(empty_source(),0,1);
say(fid,'\n[A. EXACT FILE CAPTURE / BASELINE SHA COMPARISON]');
for k=1:numel(names)
    [p,candidates]=resolve_file(root,names{k});
    s=capture(p,names{k},expected{k}); s.candidates=candidates;
    S(end+1,1)=s; %#ok<AGROW>
    print_source(fid,s);
end

% Follow quoted native includes relative to their owner. System includes
% are not searched. Missing local includes remain explicit open evidence.
say(fid,'\n[B. QUOTED NATIVE INCLUDE DEPENDENCIES]');
unresolved={}; i=1;
while i<=numel(S)
    s=S(i); i=i+1;
    [~,~,ext]=fileparts(s.path);
    if ~s.present || ~any(strcmpi(ext,{'.hpp','.h','.cpp','.cc','.cxx'})), continue; end
    inc=regexp(s.text,'(?m)^\s*#\s*include\s*"([^"\r\n]+)"','tokens');
    for j=1:numel(inc)
        p=fullfile(fileparts(s.path),inc{j}{1});
        if ~isfile(p)
            unresolved{end+1}=sprintf('%s -> %s',s.path,inc{j}{1}); %#ok<AGROW>
            say(fid,' UNRESOLVED local include: %s',unresolved{end});
            continue;
        end
        p=canonical(p);
        if any(strcmp({S.path},p)), continue; end
        q=capture(p,inc{j}{1},''); S(end+1,1)=q; %#ok<AGROW>
        print_source(fid,q);
    end
end

% Print complete small producers, avoiding an inference from token presence.
say(fid,'\n[C. COMPLETE AF7R33 / AF7R34 PRODUCERS]');
for i=[3 4]
    if S(i).present, print_lines(fid,S(i),1:numel(S(i).lines)); end
end

% Context windows deliberately include comments and literals. They are
% labelled lexical hits; consumers must inspect the actual code and callers.
topics={ ...
 'TCORR: rejoin endpoint, duration and time origin', ...
 'DCOMMIT: commit callers, timestamps, guards and reset paths', ...
 'CMAX: unsafe components and actual segment construction', ...
 'RESOURCE: allocation, pruning and ordinary return ordering'};
patterns={ ...
 'resolve_reference_rejoin_search_end|Tcorr|correction_duration|rejoin|flight_end|flight_t0|flight_reference_time_origin|tUpper|hold_end', ...
 'commit|Dcommit|last_.*(time|stamp)|next_reference_version|active_reference_version\s*=|active_repaired_reference\s*=|cooldown|debounce|reset|prediction_dt', ...
 'Cmax|build_episode|plan_fixed_episode|struct\s+(Episode|Candidate)|\.comp\b|\.ep\b|segment_t[01]|control_points|push_back|resize\(|reserve\(', ...
 'repair_|parse_ref|prun|expired|Gmax|Smax|resource|result_struct|op\s*==\s*1|af7r46_single_core_proposal_compact'};
evidence=struct('topic',{},'source_index',{},'hit_lines',{});
for t=1:numel(topics)
    say(fid,'\n[D%d. %s]',t,topics{t});
    total=0;
    for i=1:numel(S)
        % AF7R58B is test-only and is never evidence for production bounds.
        if ~S(i).present || i==6, continue; end
        hits=find(~cellfun('isempty',regexpi(S(i).lines,patterns{t},'once')));
        if isempty(hits), continue; end
        total=total+numel(hits);
        evidence(end+1)=struct('topic',topics{t},'source_index',i,'hit_lines',hits); %#ok<AGROW>
        say(fid,' lexical hits=%d (NOT semantic proof): %s',numel(hits),S(i).path);
        mask=false(1,numel(S(i).lines));
        for h=hits(:).', mask(max(1,h-5):min(numel(mask),h+8))=true; end
        print_lines(fid,S(i),find(mask));
    end
    say(fid,' total lexical hits = %d; zero means not located, NOT proven absent.',total);
end

say(fid,'\n[E. REQUIRED PROOF OBLIGATIONS - ALL REQUIRE SOURCE REVIEW]');
obligations={ ...
 'T1: Bind expiry t1 to commit time c: t1-c <= Tcorr_max for every admitted group.', ...
 'T2: Prove the endpoint resolver and every alternate proposal path obey that bound.', ...
 'T3: Bind time origin, delayed/future starts, clock resets and live parameter changes.', ...
 'D1: Identify ALL production commit writers and their callers, including repeated same-time calls.', ...
 'D2: Prove successive commit timestamps differ by Dcommit_min>0; dt alone is not proof.', ...
 'D3: Account for reset/reinitialization, old retained groups and changing dt.', ...
 'C1: Define C as unsafe connected components on the actual certified interval.', ...
 'C2: Derive a finite C bound or an explicit fail-closed admission contract; obstacle count is insufficient.', ...
 'C3: Prove S_group<=2*Cmax+1 for AF7R36 production synthesis, not only AF7R57M shadow.', ...
 'G1: Specify prune-before-count-before-append ordering and whether capacity includes a prospective group.', ...
 'G2: Bound future-start groups, retained old versions and in-flight references separately.', ...
 'G3: Bind invalidation/revalidation when any contract dependency changes.', ...
 'R1: Guard MATLAB packet allocations/copies as well as native parse_ref allocations.', ...
 'R2: Bound obstacles, nominal evaluator complexity, adaptive leaves and candidate loops separately.', ...
 'R3: Verify ordinary rejection preserves the active reference and caller failure behavior.', ...
 'R4: A resource rejection is NOT itself proof that the still-active physical trajectory is safe.', ...
 'R5: End-to-end QPC measurement remains separate from a hard worst-case execution-time proof.'};
for k=1:numel(obligations), say(fid,' %s',obligations{k}); end

say(fid,'\n[F. SOURCE IMMUTABILITY / FINAL STATUS]');
unchanged=true;
for i=1:numel(S)
    if ~S(i).present, continue; end
    S(i).unchanged=isfile(S(i).path) && strcmp(S(i).sha256,file_hash(S(i).path));
    unchanged=unchanged && S(i).unchanged;
end
captured=all([S(1:5).present]) && isempty(unresolved);
baseline=all([S(1:6).matches_expected]);
OUT=struct();
OUT.schema='AF7R58C_RESOURCE_BOUND_SOURCE_EVIDENCE_V1';
OUT.entrypoint=self; OUT.entrypoint_sha256=file_hash(self);
OUT.sources=S; OUT.evidence=evidence; OUT.unresolved_includes=unresolved;
OUT.obligations=obligations; OUT.source_capture_complete=captured;
OUT.baseline_sha_match=baseline; OUT.captured_sources_unchanged=unchanged;
OUT.source_inventory_closed=captured && baseline && unchanged;
OUT.semantic_review_complete=false;
OUT.Tcorr_max=NaN; OUT.Dcommit_min=NaN; OUT.Cmax=NaN;
OUT.Gmax=NaN; OUT.Smax=NaN;
OUT.production_bound_theorem_closed=false;
OUT.production_caps_selected=false; OUT.production_modified=false;
OUT.production_4ms_claimed=false; OUT.flight_run=false;
OUT.status='OPEN_SEMANTIC_SOURCE_REVIEW_REQUIRED';
if ~OUT.source_inventory_closed, OUT.status='OPEN_SOURCE_BINDING_OR_CAPTURE'; end
OUT.next_action='REVIEW_CAPTURED_CALLERS_AND_BUILDERS__BIND_THREE_CONTRACTS__DERIVE_CAPS';
OUT.output_directory=outdir; OUT.report_file=logfile;
say(fid,' required source capture complete       = %d',captured);
say(fid,' AF7R58B-era baseline SHA match          = %d',baseline);
say(fid,' captured source bytes unchanged         = %d',unchanged);
say(fid,' source inventory closed                 = %d',OUT.source_inventory_closed);
say(fid,' semantic bound theorem closed           = 0');
say(fid,' numeric production Gmax/Smax selected    = 0');
say(fid,' production source modified by audit     = 0');
say(fid,' production end-to-end 4 ms claimed       = 0');
say(fid,' status = %s',OUT.status);
say(fid,' next action = %s',OUT.next_action);
save(fullfile(outdir,'AF7R58C_OUT.mat'),'OUT','-v7');
assignin('base','AF7R58C_OUT',OUT);
say(fid,' saved report = %s',logfile);
say(fid,' saved source snapshot = %s',fullfile(outdir,'AF7R58C_OUT.mat'));
say(fid,'====================================================================================================');
end

function s=empty_source()
s=struct('name','','path','','candidates',{{}},'present',false, ...
 'expected_sha256','','sha256','','matches_expected',false, ...
 'bytes',uint8([]),'text','','lines',{{}},'unchanged',false);
end

function [p,allpaths]=resolve_file(root,name)
% Prefer the explicit root; record MATLAB path alternatives without executing.
p=''; allpaths={};
w=which(name,'-all');
if ischar(w), w=cellstr(w); end
if ~isempty(w), allpaths=w(:).'; end
local=fullfile(root,name);
if isfile(local)
    p=canonical(local);
else
    q=which(name);
    if ~isempty(q) && isfile(q), p=canonical(q); end
end
end

function s=capture(p,name,expected)
s=empty_source(); s.name=name; s.path=p; s.expected_sha256=expected;
if isempty(p) || ~isfile(p), return; end
s.bytes=read_bytes(p); s.sha256=hash_bytes(s.bytes); s.present=true;
s.matches_expected=~isempty(expected) && strcmp(s.sha256,expected);
s.text=native2unicode(s.bytes(:).','UTF-8');
s.lines=regexp(s.text,'\r\n|\n|\r','split');
end

function print_source(fid,s)
say(fid,' file = %s',s.name);
say(fid,' resolved = %s | present=%d',s.path,s.present);
say(fid,' SHA-256 = %s',s.sha256);
if ~isempty(s.expected_sha256)
    say(fid,' expected = %s | match=%d',s.expected_sha256,s.matches_expected);
end
for j=1:numel(s.candidates), say(fid,' MATLAB path candidate = %s',s.candidates{j}); end
end

function print_lines(fid,s,idx)
say(fid,' ---- %s ----',s.name);
last=0;
for k=idx(:).'
    if last>0 && k>last+1, say(fid,' ... omitted context; full source is in MAT ...'); end
    say(fid,' L%-6d %s',k,s.lines{k}); last=k;
end
end

function say(fid,fmt,varargin)
v=sprintf(fmt,varargin{:}); fprintf(1,'%s\n',v); fprintf(fid,'%s\n',v);
end

function b=read_bytes(p)
f=fopen(p,'rb'); assert(f>=0,'[AF7R58C] cannot read %s',p);
c=onCleanup(@()fclose(f)); %#ok<NASGU>
b=fread(f,Inf,'*uint8');
end

function h=file_hash(p)
h=hash_bytes(read_bytes(p));
end

function h=hash_bytes(b)
md=java.security.MessageDigest.getInstance('SHA-256'); md.update(b);
d=typecast(md.digest(),'uint8'); h=lower(reshape(dec2hex(d,2).',1,[]));
end

function p=canonical(p)
j=java.io.File(p); p=char(j.getCanonicalPath());
end
