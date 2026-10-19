function OUT = AF7R57N_V9_EPISODE_WRAPPER_FORENSIC_AUDIT()
% AF7R57N_V9_EPISODE_WRAPPER_FORENSIC_AUDIT
% Read-only forensic capture of the exact R57K corrected MEX wrapper.
% No source modification, no MEX build, no planner run.

clc;
fprintf('\n====================================================================================================\n');
fprintf(' AF7R57N V9 | EXACT EPISODE-WRAPPER SOURCE FORENSIC\n');
fprintf('====================================================================================================\n');
fprintf('[AUTHORITY] readOnly=1 | exactSourceSHA=1 | noPatch=1 | productionModified=0\n');

parentCpp=find_source_local('af7r57k_corrected_core_mex.cpp');
expectedCpp='3b774a4aa1b18646182d1f7fa6d01bd2c6669e5ad2e6a8200f702f2f8e6f8597';
sha=sha256_file_local(parentCpp);
assert(strcmpi(sha,expectedCpp), ...
    '[AF7R57N-V9] corrected serializer SHA changed.');

fprintf('\n[A. EXACT SOURCE]\n');
fprintf(' source                                  = %s\n',parentCpp);
fprintf(' SHA-256                                 = %s\n',sha);
fprintf(' expected SHA match                      = 1\n');

cpp=normalize_lf_local(fileread(parentCpp));
L=regexp(cpp,'\n','split');

hit=find(contains(L,'fixed episode unavailable'));
fprintf('\n[B. FIXED-EPISODE THROW INVENTORY]\n');
fprintf(' token count                              = %d\n',numel(hit));
assert(~isempty(hit),'[AF7R57N-V9] fixed-episode token not found.');
for j=1:numel(hit)
    fprintf('\n---- THROW %d | line %d ----\n',j,hit(j));
    print_lines_local(L,max(1,hit(j)-18),min(numel(L),hit(j)+28));
end

mexLine=find(contains(L,'void mexFunction'),1,'first');
assert(~isempty(mexLine),'[AF7R57N-V9] mexFunction not found.');

fprintf('\n[C. MEXFUNCTION DISPATCH / OUTPUT TOKENS]\n');
fprintf(' mexFunction line                         = %d\n',mexLine);
patterns={ ...
    'mxGetScalar(prhs[0])', ...
    'op==', ...
    'op ==', ...
    'case 4', ...
    'fixed episode', ...
    'plhs[0]', ...
    'result_struct(', ...
    'branch_trace_struct(', ...
    'plan_fixed', ...
    'fixed_episode', ...
    'EpisodeCert', ...
    'PlanResult' ...
    };
mask=false(size(L));
for p=1:numel(patterns)
    mask = mask | contains(L,patterns{p});
end
idx=find(mask & ((1:numel(L))>=mexLine-120));
for k=idx(:)'
    fprintf(' L%-5d %s\n',k,L{k});
end

fprintf('\n[D. MEXFUNCTION SOURCE WINDOW]\n');
% Print a bounded but generous window. The exact wrapper has historically
% been compact; this is intentionally source capture, not a semantic guess.
print_lines_local(L,mexLine,min(numel(L),mexLine+260));

fprintf('\n[E. SERIALIZER FUNCTION DECLARATIONS]\n');
for k=1:numel(L)
    if contains(L{k},'static mxArray*') || contains(L{k},'mxArray* result_struct') || ...
            contains(L{k},'mxArray* branch_trace_struct')
        fprintf(' L%-5d %s\n',k,L{k});
    end
end

fprintf('\n[F. READ-ONLY GATE]\n');
fprintf(' exact source bound                       = 1\n');
fprintf(' wrapper source captured                  = 1\n');
fprintf(' source modified                          = 0\n');
fprintf(' production modified                      = 0\n');
fprintf(' next action                              = BUILD_EXACT_OP4_202_203_RETURN_PATCH_FROM_CAPTURE\n');
fprintf('====================================================================================================\n');

OUT=struct();
OUT.schema='AF7R57N_V9_EPISODE_WRAPPER_FORENSIC_AUDIT_V1';
OUT.closed=true;
OUT.source=parentCpp;
OUT.sha256=sha;
OUT.throw_lines=hit(:).';
OUT.mex_line=mexLine;
OUT.production_modified=false;
assignin('base','AF7R57N_V9_EPISODE_WRAPPER_FORENSIC_AUDIT_OUT',OUT);
end

function print_lines_local(L,a,b)
for k=a:b
    fprintf(' L%-5d %s\n',k,L{k});
end
end

function p=find_source_local(name)
q=which(name);
if ~isempty(q) && isfile(q),p=q;return;end
root=fileparts(mfilename('fullpath'));
q=fullfile(root,name);
if isfile(q),p=q;return;end
error('[AF7R57N-V9] source not found: %s',name);
end

function s=normalize_lf_local(s)
s=strrep(char(s),sprintf('\r\n'),sprintf('\n'));
s=strrep(s,sprintf('\r'),sprintf('\n'));
end

function h=sha256_file_local(fn)
fid=fopen(fn,'rb');
assert(fid>=0,'Cannot open %s',fn);
c=onCleanup(@()fclose(fid));
b=fread(fid,Inf,'*uint8');
md=java.security.MessageDigest.getInstance('SHA-256');
md.update(b);
d=typecast(md.digest(),'uint8');
h=lower(reshape(dec2hex(d,2).',1,[]));
end
