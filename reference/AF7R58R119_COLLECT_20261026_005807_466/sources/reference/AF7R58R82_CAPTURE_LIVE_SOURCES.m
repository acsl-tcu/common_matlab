function R=AF7R58R82_CAPTURE_LIVE_SOURCES()
% R82 read-only source-bound capture. Does not modify production/MEX/HLC.
% Outputs are written beside THIS script, never the MATLAB main directory.
root=fileparts(mfilename('fullpath'));
R=struct('status','SOURCE_CAPTURE_ONLY_NO_TRIGGER_OR_COMMIT_CHANGE', ...
    'production_modified',false,'trigger_switched',false, ...
    'geometry_certified',false,'continuous_execution_proved',false, ...
    'source_count',0,'source_paths',{{}},'source_sha256',{{}}, ...
    'missing',{{}},'report_file','','zip_file','','mat_file','');
S={}; N={};
% Current, actually resolved authority. Do not select dated backups.
mainNames={'REPLANNING_SYSTEM','SUSPENDED_LOAD_REF_ADJUST', ...
 'HLC_SUSPENDED_LOAD','Controller_HL_Suspended_Load', ...
 'Model_Quat13','MODEL_CLASS','SimHL'};
for i=1:numel(mainNames)
    p=which(mainNames{i});
    if ~isempty(p) && isfile(p)
        S{end+1}=p; N{end+1}=mainNames{i}; %#ok<AGROW>
    else
        R.missing{end+1}=mainNames{i}; %#ok<AGROW>
    end
end
rpl=which('REPLANNING_SYSTEM');
if ~isempty(rpl)
    refdir=fileparts(rpl);
else
    refdir=root;
end
% Current native & R79/R80/R81 named MATLAB sources. No MEX binary copied.
pat={'af7r36_core.hpp','af7r36_single_core_mex.cpp', ...
 'AF7R58R78_*.m','AF7R58R79_*.m','AF7R58R80_*.m','AF7R58R81_*.m'};
for i=1:numel(pat)
    L=dir(fullfile(refdir,pat{i}));
    if isempty(L)
       R.missing{end+1}=pat{i}; %#ok<AGROW>
    end
    for k=1:numel(L)
       if L(k).isdir, continue; end
       p=fullfile(L(k).folder,L(k).name);
       if ~any(strcmp(S,p))
          S{end+1}=p; N{end+1}=L(k).name; %#ok<AGROW>
       end
    end
end
R.source_paths=S; R.source_count=numel(S);
R.source_sha256=cell(size(S));
report=fullfile(root,'AF7R58R82_SOURCE_CAPTURE.txt');
R.report_file=report;
fid=fopen(report,'wt');
assert(fid>=0,'R82:UnableToOpenReport');
c=onCleanup(@()fclose(fid));
fprintf(fid,'R82 SOURCE-BOUND CAPTURE | read-only | no production changes\n');
fprintf(fid,'Active replanner: %s\n',rpl);
for i=1:numel(S)
    R.source_sha256{i}=local_hash(S{i});
    fprintf(fid,'SOURCE[%d] %s | SHA-256=%s\n',i,S{i},R.source_sha256{i});
end
fprintf(fid,'\nMISSING (not invented):\n');
for i=1:numel(R.missing), fprintf(fid,' %s\n',R.missing{i});end
keys={'get_current_physical_geometry','get_active_reference_packet', ...
    'trigger_core','g26_future_trigger_kernel_mex','requestIdentity', ...
    'AF7R58R81','r78_correction_jet7_identity', ...
    'try_commit_reference_only_proposal','atomic','known_mask'};
if ~isempty(rpl) && isfile(rpl)
    src=fileread(rpl);
    ln=regexp(src,'\r\n|\n|\r','split');
    fprintf(fid,'\nACTIVE REPLANNER SOURCE STRING OCCURRENCES (not execution proof):\n');
    for k=1:numel(keys)
        hits=find(contains(ln,keys{k}));
        fprintf(fid,'  %s: count=%d',keys{k},numel(hits));
        if ~isempty(hits)
            fprintf(fid,' first_lines=');fprintf(fid,'%d ',hits(1:min(numel(hits),12)));
        end
        fprintf(fid,'\n');
    end
end
fprintf(fid,'\nCAVEAT: presence/absence of a symbol cannot prove trigger switch.\n');
fprintf(fid,'R77 had production trigger NOT replaced. R81 was reference-only.\n');
fprintf(fid,'Do not promote sampled physical forecast into continuous-time safety.\n');
clear c; % onCleanup closes the report before zipping
work=tempname; mkdir(work);
cleanWork=onCleanup(@()local_rmdir(work)); %#ok<NASGU>
relative=cell(size(S));
for i=1:numel(S)
    [~,name,ext]=fileparts(S{i});
    relative{i}=sprintf('%03d_%s%s',i,name,ext);
    copyfile(S{i},fullfile(work,relative{i}));
end
R.zip_file=fullfile(root,'AF7R58R82_LIVE_SOURCES.zip');
if ~isempty(relative), zip(R.zip_file,relative,work); else, R.zip_file='';end
R.mat_file=fullfile(root,'AF7R58R82_SOURCE_CAPTURE.mat');
save(R.mat_file,'R');
fprintf('R82: %d current source files captured.\n',R.source_count);
fprintf('R82 report: %s\nR82 ZIP: %s\n',R.report_file,R.zip_file);
end

function s=local_hash(file)
s='SHA_UNAVAILABLE';
try
    fid=fopen(file,'rb');
    if fid<0,return;end
    c=onCleanup(@()fclose(fid)); %#ok<NASGU>
    b=fread(fid,inf,'*uint8');
    d=java.security.MessageDigest.getInstance('SHA-256');
    d.update(typecast(b,'int8'));
    raw=typecast(d.digest(),'uint8');
    s=lower(reshape(dec2hex(raw,2).',1,[]));
catch
    s='SHA_UNAVAILABLE';
end
end
function local_rmdir(p)
if isfolder(p),rmdir(p,'s');end
end
