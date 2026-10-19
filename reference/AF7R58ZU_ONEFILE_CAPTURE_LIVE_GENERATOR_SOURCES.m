function OUT = AF7R58ZU_ONEFILE_CAPTURE_LIVE_GENERATOR_SOURCES()
% AF7R58ZU
% READ-ONLY live-source capture.
%
% PURPOSE
%   Stop guessing source layout. Capture the exact live generator callback
%   and every plausible continuous-certificate source that the current
%   MATLAB session / dependency graph / nearby project roots can resolve.
%
% PRODUCTION MODIFICATION: NONE.
% No source file is edited, moved, deleted, or placed on MATLAB path.
%
% OUTPUT
%   AF7R58ZU_REPORT.txt
%   AF7R58ZU_LIVE_SOURCE_BUNDLE.zip
%
% Send the report/zip back for the next source-bound patch.

OUT=struct();
OUT.schema='AF7R58ZU_LIVE_SOURCE_CAPTURE_V1';
OUT.production_modified=false;
OUT.status='OPEN';
OUT.error=struct();

fprintf('\n====================================================================================================\n');
fprintf(' AF7R58ZU | READ-ONLY LIVE GENERATOR SOURCE CAPTURE\n');
fprintf('====================================================================================================\n');
fprintf('[AUTHORITY] readOnly=1 | productionModified=0 | liveCallbackFirst=1 | noPatch=1\n');

try
    % ---------------------------------------------------------------------
    % A. Resolve the exact callback actually frozen into replanner_instance.
    % ---------------------------------------------------------------------
    liveName='';
    liveFile='';
    resolveHow='';

    if evalin('base','exist(''replanner_instance'',''var'')')==1
        R=evalin('base','replanner_instance');
        try
            fh=R.generator_backend_locked_callback;
        catch
            fh=[];
        end

        if isa(fh,'function_handle')
            info=functions(fh);
            liveName=func2str(fh);
            if isfield(info,'file') && ischar(info.file) && isfile(info.file)
                liveFile=info.file;
                resolveHow='LIVE_LOCKED_CALLBACK_HANDLE';
            end
        end
    end

    if isempty(liveFile)
        names={ ...
            'G27V74R23_BEZIER_QARC_CTGEO_SHADOW_GENERATOR', ...
            'G27V74R21_BEZIER_D15_C7_FAST_SHADOW_GENERATOR'};
        for k=1:numel(names)
            p=which(names{k});
            if ~isempty(p) && isfile(p)
                liveName=names{k};
                liveFile=p;
                resolveHow='WHICH_FALLBACK';
                break;
            end
        end
    end

    assert(~isempty(liveFile) && isfile(liveFile), ...
        '[AF7R58ZU] exact live generator callback file could not be resolved.');

    OUT.live_callback_name=liveName;
    OUT.live_callback_file=liveFile;
    OUT.live_callback_resolution=resolveHow;
    OUT.live_callback_sha=file_hash(liveFile);

    fprintf('\n[A. EXACT LIVE CALLBACK]\n');
    fprintf(' name       = %s\n',liveName);
    fprintf(' resolution = %s\n',resolveHow);
    fprintf(' file       = %s\n',liveFile);
    fprintf(' SHA-256    = %s\n',OUT.live_callback_sha);

    % ---------------------------------------------------------------------
    % B. Build bounded search roots from current live state only.
    % ---------------------------------------------------------------------
    roots={};
    roots{end+1}=fileparts(liveFile);

    pR=which('REPLANNING_SYSTEM');
    if ~isempty(pR) && isfile(pR)
        roots{end+1}=fileparts(pR);
        roots{end+1}=fileparts(fileparts(pR));
    end

    roots{end+1}=pwd;

    up=getenv('USERPROFILE');
    if ~isempty(up)
        d1=fullfile(up,'Downloads');
        d2=fullfile(up,'Documents','GitHub');
        if isfolder(d1), roots{end+1}=d1; end
        if isfolder(d2), roots{end+1}=d2; end
    end

    roots=unique_existing_dirs(roots);

    fprintf('\n[B. SEARCH ROOTS]\n');
    for k=1:numel(roots)
        fprintf(' root[%d] = %s\n',k,roots{k});
    end

    % ---------------------------------------------------------------------
    % C. Dependency analyzer for the exact live wrapper.
    % ---------------------------------------------------------------------
    depFiles={};
    depErr='';

    try
        [F,~]=matlab.codetools.requiredFilesAndProducts(liveFile);
        if ischar(F), F={F}; end
        if isstring(F), F=cellstr(F(:)); end
        depFiles=F(:).';
    catch MEdep
        depErr=sprintf('%s | %s',MEdep.identifier,MEdep.message);
    end

    depFiles=unique_existing_files(depFiles);

    fprintf('\n[C. MATLAB DEPENDENCY ANALYZER]\n');
    fprintf(' dependency count = %d\n',numel(depFiles));
    if ~isempty(depErr)
        fprintf(' dependency analyzer error = %s\n',depErr);
    end

    for k=1:numel(depFiles)
        if is_relevant_name(depFiles{k})
            fprintf(' relevant dependency = %s\n',depFiles{k});
        end
    end

    % ---------------------------------------------------------------------
    % D. Resolve exact historically used names via which -all semantics.
    % ---------------------------------------------------------------------
    exactNames={ ...
        'G27V74R23_CONTINUOUS_WHOLE_BODY_CERT', ...
        'G27V74R41I_CONTINUOUS_WHOLE_BODY_CERT_FAST', ...
        'G27V74R23_BEZIER_QARC_CTGEO_SHADOW_GENERATOR', ...
        'G27V74R21_BEZIER_D15_C7_FAST_SHADOW_GENERATOR'};

    nameHits={};
    fprintf('\n[D. EXACT-NAME RESOLUTION]\n');

    for i=1:numel(exactNames)
        nm=exactNames{i};
        hits=which_all_files(nm);
        fprintf(' %s : %d hit(s)\n',nm,numel(hits));
        for j=1:numel(hits)
            fprintf('   %s\n',hits{j});
            nameHits{end+1}=hits{j}; %#ok<AGROW>
        end
    end

    % ---------------------------------------------------------------------
    % E. Bounded exact-filename filesystem search.
    % Only exact known filenames; not a broad scan of arbitrary user files.
    % ---------------------------------------------------------------------
    fsHits={};
    searchFiles={ ...
        'G27V74R23_CONTINUOUS_WHOLE_BODY_CERT.m', ...
        'G27V74R41I_CONTINUOUS_WHOLE_BODY_CERT_FAST.m'};

    fprintf('\n[E. BOUNDED EXACT-FILENAME SEARCH]\n');

    for ir=1:numel(roots)
        for in=1:numel(searchFiles)
            try
                D=dir(fullfile(roots{ir},'**',searchFiles{in}));
            catch
                D=[];
            end
            for k=1:numel(D)
                if ~D(k).isdir
                    p=fullfile(D(k).folder,D(k).name);
                    fprintf(' found = %s\n',p);
                    fsHits{end+1}=p; %#ok<AGROW>
                end
            end
        end
    end

    % ---------------------------------------------------------------------
    % F. Gather candidates and inspect source markers.
    % ---------------------------------------------------------------------
    candidates=[{liveFile},depFiles,nameHits,fsHits];
    candidates=unique_existing_files(candidates);

    certCandidates={};
    sourceRows=struct('file',{},'sha',{},'has_support_helper',{}, ...
        'has_math_pass',{},'has_continuous_time',{},'has_whole_body',{}, ...
        'mentions_r23_cert',{},'mentions_fast_cert',{});

    fprintf('\n[F. SOURCE-MARKER AUDIT]\n');

    for k=1:numel(candidates)
        p=candidates{k};
        if ~endsWith(lower(p),'.m')
            continue;
        end

        try
            t=fileread(p);
        catch
            continue;
        end

        row.file=p;
        row.sha=file_hash(p);
        row.has_support_helper=contains(t,'g27_best_support_margin');
        row.has_math_pass=contains(t,'mathematical_pass');
        row.has_continuous_time=contains(t,'continuous_time');
        row.has_whole_body=contains(t,'whole_body');
        row.mentions_r23_cert=contains(t,'G27V74R23_CONTINUOUS_WHOLE_BODY_CERT');
        row.mentions_fast_cert=contains(t,'G27V74R41I_CONTINUOUS_WHOLE_BODY_CERT_FAST');

        relevant=row.has_support_helper || row.has_math_pass || ...
            row.mentions_r23_cert || row.mentions_fast_cert || strcmp(p,liveFile);

        if relevant
            sourceRows(end+1)=row; %#ok<AGROW>
            fprintf(' FILE = %s\n',p);
            fprintf('   SHA=%s\n',row.sha);
            fprintf('   supportHelper=%d mathPass=%d continuous=%d wholeBody=%d mentionsR23=%d mentionsFast=%d\n', ...
                row.has_support_helper,row.has_math_pass,row.has_continuous_time, ...
                row.has_whole_body,row.mentions_r23_cert,row.mentions_fast_cert);

            if row.has_support_helper && row.has_math_pass && row.has_continuous_time
                certCandidates{end+1}=p; %#ok<AGROW>
            end
        end
    end

    certCandidates=unique_existing_files(certCandidates);

    OUT.search_roots=roots;
    OUT.dependency_files=depFiles;
    OUT.source_rows=sourceRows;
    OUT.certificate_candidates=certCandidates;

    fprintf('\n[G. CERTIFICATE CANDIDATES]\n');
    fprintf(' candidate count = %d\n',numel(certCandidates));
    for k=1:numel(certCandidates)
        fprintf(' candidate[%d] = %s\n',k,certCandidates{k});
    end

    % ---------------------------------------------------------------------
    % H. Print exact live wrapper lines that matter for dispatch/certificate.
    % ---------------------------------------------------------------------
    W=normalize_newlines(fileread(liveFile));
    WL=regexp(W,'\n','split');

    fprintf('\n[H. LIVE WRAPPER RELEVANT LINES]\n');
    printed=0;
    keys={'CERT','cert','geometry','shadow','mathematical_pass', ...
          'feval','function_handle','CONTINUOUS','R23','R41I','do('};

    for k=1:numel(WL)
        hit=false;
        for q=1:numel(keys)
            if contains(WL{k},keys{q})
                hit=true;
                break;
            end
        end
        if hit
            fprintf(' L%d | %s\n',k,WL{k});
            printed=printed+1;
        end
    end

    if printed==0
        fprintf(' no keyword lines found; full wrapper is included in source bundle.\n');
    end

    % ---------------------------------------------------------------------
    % I. Create a local read-only source bundle.
    % Copies are created only in a new diagnostic output folder.
    % ---------------------------------------------------------------------
    stamp=datestr(now,'yyyymmdd_HHMMSS');
    baseOut=fullfile(pwd,['AF7R58ZU_CAPTURE_' stamp]);
    [ok,msg]=mkdir(baseOut); assert(ok,'%s',msg);

    manifest=fullfile(baseOut,'AF7R58ZU_REPORT.txt');
    mf=fopen(manifest,'wt','n','UTF-8'); assert(mf>=0);
    cm=onCleanup(@()safe_fclose(mf)); %#ok<NASGU>

    fprintf(mf,'AF7R58ZU READ-ONLY LIVE SOURCE CAPTURE\n');
    fprintf(mf,'productionModified=0\n');
    fprintf(mf,'liveCallbackName=%s\n',liveName);
    fprintf(mf,'liveCallbackResolution=%s\n',resolveHow);
    fprintf(mf,'liveCallbackFile=%s\n',liveFile);
    fprintf(mf,'liveCallbackSHA=%s\n',OUT.live_callback_sha);
    fprintf(mf,'certificateCandidateCount=%d\n',numel(certCandidates));
    for k=1:numel(certCandidates)
        fprintf(mf,'certificateCandidate[%d]=%s\n',k,certCandidates{k});
    end
    fprintf(mf,'\nSOURCE ROWS\n');
    for k=1:numel(sourceRows)
        r=sourceRows(k);
        fprintf(mf,'%s\n SHA=%s\n support=%d math=%d continuous=%d whole=%d mentionsR23=%d mentionsFast=%d\n', ...
            r.file,r.sha,r.has_support_helper,r.has_math_pass,r.has_continuous_time, ...
            r.has_whole_body,r.mentions_r23_cert,r.mentions_fast_cert);
    end
    fprintf(mf,'\nLIVE WRAPPER SOURCE\n');
    fprintf(mf,'%s\n',W);

    copied={};
    copyList=unique_existing_files([{liveFile},certCandidates,depFiles]);

    for k=1:numel(copyList)
        p=copyList{k};
        if ~endsWith(lower(p),'.m'), continue; end
        [~,nm,ext]=fileparts(p);
        dst=fullfile(baseOut,sprintf('%03d__%s%s',k,nm,ext));
        try
            copyfile(p,dst);
            copied{end+1}=dst; %#ok<AGROW>
        catch
        end
    end

    zipPath=fullfile(pwd,'AF7R58ZU_LIVE_SOURCE_BUNDLE.zip');
    if isfile(zipPath), delete(zipPath); end
    zip(zipPath,baseOut);

    OUT.output_directory=baseOut;
    OUT.report_path=manifest;
    OUT.zip_path=zipPath;
    OUT.copied_file_count=numel(copied);
    OUT.status='LIVE_SOURCE_CAPTURED__NO_PRODUCTION_CHANGE__SEND_REPORT_OR_ZIP';

    assignin('base','AF7R58ZU_OUT',OUT);

    fprintf('\n[I. RESULT]\n');
    fprintf(' production modified = 0\n');
    fprintf(' copied source files = %d\n',numel(copied));
    fprintf(' REPORT = %s\n',manifest);
    fprintf(' ZIP    = %s\n',zipPath);
    fprintf(' status = %s\n',OUT.status);

catch ME
    OUT.production_modified=false;
    OUT.status='CAPTURE_FAILED__NO_PRODUCTION_CHANGE';
    OUT.error=struct('identifier',ME.identifier,'message',ME.message,'stack',{ME.stack});
    assignin('base','AF7R58ZU_OUT',OUT);

    fprintf('\n[CAPTURE ERROR]\n');
    fprintf(' identifier = %s\n',ME.identifier);
    fprintf(' message    = %s\n',ME.message);
    fprintf(' production modified = 0\n');
    fprintf(' status = %s\n',OUT.status);
end
end


function hits=which_all_files(name)
hits={};
try
    w=which(name,'-all');
catch
    w={};
end
if ischar(w)
    if isempty(w), w={}; else, w={w}; end
elseif isstring(w)
    w=cellstr(w(:));
end
for k=1:numel(w)
    p=w{k};
    if ischar(p) && isfile(p)
        hits{end+1}=p; %#ok<AGROW>
    end
end
hits=unique_existing_files(hits);
end


function tf=is_relevant_name(p)
q=upper(p);
tf=contains(q,'G27V74R23') || contains(q,'G27V74R41I') || ...
   contains(q,'G27V74R21') || contains(q,'CERT') || ...
   contains(q,'REPLANNING_SYSTEM');
end


function x=unique_existing_dirs(x)
out={};
for k=1:numel(x)
    p=x{k};
    if ischar(p) && isfolder(p)
        if ~any(strcmpi(out,p))
            out{end+1}=p; %#ok<AGROW>
        end
    end
end
x=out;
end


function x=unique_existing_files(x)
out={};
for k=1:numel(x)
    p=x{k};
    if isstring(p)&&isscalar(p),p=char(p);end
    if ischar(p) && isfile(p)
        if ~any(strcmpi(out,p))
            out{end+1}=p; %#ok<AGROW>
        end
    end
end
x=out;
end


function s=normalize_newlines(s)
s=strrep(s,sprintf('\r\n'),sprintf('\n'));
s=strrep(s,sprintf('\r'),sprintf('\n'));
end


function h=file_hash(p)
fid=fopen(p,'rb'); assert(fid>=0);
c=onCleanup(@()fclose(fid)); %#ok<NASGU>
b=fread(fid,Inf,'*uint8');
m=java.security.MessageDigest.getInstance('SHA-256');
m.update(b);
d=typecast(m.digest(),'uint8');
h=lower(reshape(dec2hex(d,2).',1,[]));
end


function safe_fclose(fid)
try
    if fid>=0,fclose(fid);end
catch
end
end
