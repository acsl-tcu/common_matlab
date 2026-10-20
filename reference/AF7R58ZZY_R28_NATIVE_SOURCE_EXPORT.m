function OUT = AF7R58ZZY_R28_NATIVE_SOURCE_EXPORT()
% AF7R58ZZY-R28 exact native-source export for theory-preserving integration.
% READ-ONLY: does not compile, run Flight, edit production, or change MATLAB path.
%
% Purpose:
%   Export the exact current AF7R36 native generator sources and their MATLAB
%   producer/build interfaces so the next implementation can integrate the
%   compiled derivative representation INSIDE the generator without guessing
%   from audit excerpts.
%
% Output:
%   AF7R58ZZY_R28_NATIVE_SOURCE_<timestamp>.zip
%
% Return that ZIP to ChatGPT.

clc;
fprintf('\n====================================================================================================\n');
fprintf(' AF7R58ZZY-R28 | EXACT NATIVE SOURCE EXPORT\n');
fprintf('====================================================================================================\n');
fprintf('[AUTHORITY] readOnly=1 | noFlight=1 | noMexBuild=1 | productionModified=0 | pathModified=0\n');

expectedProd='310cfdd01f9554a80b71e8d90464816a721859709ad7c4b9137cdc0f3a6eaf49';

% Find the canonical production source by exact SHA. Do not trust first which().
w=which('REPLANNING_SYSTEM','-all');
if ischar(w), w=cellstr(w); end
prod='';
for k=1:numel(w)
    p=w{k};
    if isfile(p)
        try
            if strcmp(sha_file(p),expectedProd)
                prod=p; break;
            end
        catch
        end
    end
end
assert(~isempty(prod), ...
    '[R28 EXPORT] canonical production REPLANNING_SYSTEM with expected SHA not found.');

refdir=fileparts(prod);
root=fileparts(refdir);
fprintf(' canonical root                          = %s\n',root);
fprintf(' production                              = %s\n',prod);
fprintf(' production SHA                          = %s\n',sha_file(prod));

names={ ...
    'af7r36_single_core_mex.cpp', ...
    'af7r36_core.hpp', ...
    'AF7R36_BUILD_CANONICAL_SINGLE_CORE_MEX.m', ...
    'af7r36_single_core_proposal.m', ...
    'af7r34_make_core_packet.m', ...
    'af7r33_make_batch_model_from_proposal.m', ...
    'af7r33_nominal_batch_model.m'};

paths=cell(size(names));
for k=1:numel(names)
    p=fullfile(refdir,names{k});
    if ~isfile(p)
        q=which(names{k});
        if ~isempty(q)&&isfile(q), p=q; end
    end
    assert(isfile(p),'[R28 EXPORT] required source missing: %s',names{k});
    paths{k}=p;
end

stamp=char(datetime('now','Format','yyyyMMdd_HHmmss_SSS'));
outdir=fullfile(root,['AF7R58ZZY_R28_NATIVE_SOURCE_' stamp]);
[ok,msg]=mkdir(outdir); assert(ok,'%s',msg);

manifest=cell(numel(names)+1,1);
for k=1:numel(names)
    dst=fullfile(outdir,names{k});
    copy_binary(paths{k},dst);
    h=sha_file(dst);
    manifest{k}=sprintf('%s  %s',h,names{k});
    fprintf(' %-44s SHA=%s\n',names{k},h);
end

% Small authority note, generated from current bytes only.
note=fullfile(outdir,'R28_SOURCE_AUTHORITY.txt');
fid=fopen(note,'wt','n','UTF-8'); assert(fid>=0,'cannot write authority note');
c=onCleanup(@()fclose(fid)); %#ok<NASGU>
fprintf(fid,'AF7R58ZZY-R28 EXACT NATIVE SOURCE AUTHORITY\n');
fprintf(fid,'capture_time=%s\n',char(datetime('now','Format','yyyy-MM-dd HH:mm:ss.SSS')));
fprintf(fid,'canonical_root=%s\n',root);
fprintf(fid,'production=%s\n',prod);
fprintf(fid,'production_sha=%s\n',sha_file(prod));
fprintf(fid,'read_only=1\nflight_run=0\nmex_build=0\nproduction_modified=0\n');
for k=1:numel(names)
    fprintf(fid,'%s=%s\n',names{k},sha_file(fullfile(outdir,names{k})));
end
clear c;

manifest{end}=sprintf('%s  %s',sha_file(note),'R28_SOURCE_AUTHORITY.txt');
mf=fullfile(outdir,'SHA256_MANIFEST.txt');
fid=fopen(mf,'wt','n','UTF-8'); assert(fid>=0,'cannot write manifest');
for k=1:numel(manifest), fprintf(fid,'%s\n',manifest{k}); end
fclose(fid);

zipPath=[outdir '.zip'];
files=dir(outdir);
items={};
for k=1:numel(files)
    if ~files(k).isdir
        items{end+1}=fullfile(outdir,files(k).name); %#ok<AGROW>
    end
end
zip(zipPath,items,outdir);

OUT=struct();
OUT.schema='AF7R58ZZY_R28_NATIVE_SOURCE_EXPORT_V1';
OUT.closed=true;
OUT.canonical_root=root;
OUT.production_source=prod;
OUT.production_sha=sha_file(prod);
OUT.output_directory=outdir;
OUT.zip=zipPath;
OUT.names=names;
OUT.paths=paths;
OUT.sha=cellfun(@sha_file,paths,'UniformOutput',false);

fprintf('\n====================================================================================================\n');
fprintf(' R28 NATIVE SOURCE EXPORT COMPLETE\n');
fprintf('====================================================================================================\n');
fprintf('ZIP = %s\n',zipPath);
fprintf('Return this ZIP to ChatGPT.\n');
fprintf('====================================================================================================\n');
assignin('base','AF7R58ZZY_R28_NATIVE_SOURCE_EXPORT_OUT',OUT);
end

function copy_binary(src,dst)
f=fopen(src,'rb'); assert(f>=0,'cannot open %s',src);
c=onCleanup(@()fclose(f)); %#ok<NASGU>
b=fread(f,Inf,'*uint8');
clear c;
f=fopen(dst,'wb'); assert(f>=0,'cannot write %s',dst);
c2=onCleanup(@()fclose(f)); %#ok<NASGU>
n=fwrite(f,b,'uint8'); assert(n==numel(b),'short write %s',dst);
end

function h=sha_file(p)
f=fopen(p,'rb'); assert(f>=0,'cannot open %s',p);
c=onCleanup(@()fclose(f)); %#ok<NASGU>
b=fread(f,Inf,'*uint8');
md=java.security.MessageDigest.getInstance('SHA-256');
md.update(typecast(uint8(b(:)),'int8'));
d=typecast(md.digest(),'uint8');
h=lower(reshape(dec2hex(d,2).',1,[]));
end
