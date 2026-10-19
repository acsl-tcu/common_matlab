function OUT = AF7R58ZA_ONEFILE_RUNTIME_BINDING_CACHE_AUDIT()
% AF7R58ZA
% Read-only runtime-binding + AF7R58Z-R5 cache-effect audit.
%
% Answers:
%   1) Was the production REPLANNING_SYSTEM.m actually modified?
%   2) Which REPLANNING_SYSTEM.m does MATLAB resolve right now?
%   3) Does a live REPLANNING_SYSTEM object have the R5 cache properties?
%   4) Did the clean run actually store / hit / consume the cache?
%
% NO source modification. NO class clear. NO synthetic replan.

root=fileparts(mfilename('fullpath'));
outdir=tempname(root);
[ok,msg]=mkdir(outdir); assert(ok,'%s',msg);

report=fullfile(outdir,'AF7R58ZA_REPORT.txt');
f=fopen(report,'wt','n','UTF-8');
assert(f>=0,'[AF7R58ZA] cannot open report.');
c=onCleanup(@()fclose(f)); %#ok<NASGU>

say(f,'====================================================================================================');
say(f,' AF7R58ZA | RUNTIME BINDING + R5 CACHE EFFECT AUDIT');
say(f,'====================================================================================================');
say(f,'[AUTHORITY] readOnly=1 | productionModified=0 | noClearClasses=1 | noSyntheticReplan=1');

OUT=struct();
OUT.schema='AF7R58ZA_RUNTIME_BINDING_CACHE_AUDIT_V1';
OUT.status='OPEN';
OUT.error=struct();

try
    expectedSha='3e1cd5df64bf893c0d9c4d4fd6502d8a26f395ada08b8ad32abd12d076c9f901';

    localProd=fullfile(root,'REPLANNING_SYSTEM.m');
    OUT.local_production_path=localProd;
    OUT.local_production_exists=isfile(localProd);

    say(f,'');
    say(f,'[A. INSTALLER-DIRECTORY PRODUCTION FILE]');
    say(f,' installer root                         = %s',root);
    say(f,' local REPLANNING_SYSTEM exists         = %d',OUT.local_production_exists);

    if OUT.local_production_exists
        localSha=file_hash(localProd);
        OUT.local_production_sha=localSha;
        OUT.local_sha_exact=strcmp(localSha,expectedSha);

        txt=fileread(localProd);
        OUT.local_has_r5_cache_props= ...
            contains(txt,'reference_only_prepared_packet_cache_valid') && ...
            contains(txt,'reference_only_prepared_packet_cache_hit_count') && ...
            contains(txt,'reference_only_prepared_packet_cache_store_count') && ...
            contains(txt,'reference_only_prepared_packet_cache_commit_count');
        OUT.local_has_r5_schema= ...
            contains(txt,'AF7R58Z_R5_ONE_ENTRY_PREPARED_PACKET_CACHE_V1');

        say(f,' local SHA                             = %s',localSha);
        say(f,' local SHA is exact R5                  = %d',OUT.local_sha_exact);
        say(f,' local R5 cache properties present      = %d',OUT.local_has_r5_cache_props);
        say(f,' local R5 cache schema present          = %d',OUT.local_has_r5_schema);
    else
        OUT.local_production_sha='';
        OUT.local_sha_exact=false;
        OUT.local_has_r5_cache_props=false;
        OUT.local_has_r5_schema=false;
    end

    % MATLAB path resolution.
    resolved=which('REPLANNING_SYSTEM');
    OUT.resolved_path=resolved;
    OUT.resolved_exists=~isempty(resolved) && isfile(resolved);

    say(f,'');
    say(f,'[B. MATLAB RUNTIME NAME RESOLUTION]');
    say(f,' which(REPLANNING_SYSTEM)               = %s',safe_text(resolved));

    allWhich=evalc('which REPLANNING_SYSTEM -all');
    OUT.which_all_text=allWhich;
    say(f,' which REPLANNING_SYSTEM -all:');
    print_multiline(f,allWhich);

    rf=which('run_flight_headless');
    OUT.run_flight_headless_path=rf;
    say(f,' which(run_flight_headless)              = %s',safe_text(rf));

    if OUT.resolved_exists
        resolvedSha=file_hash(resolved);
        OUT.resolved_sha=resolvedSha;
        OUT.resolved_sha_exact=strcmp(resolvedSha,expectedSha);
        OUT.resolved_same_as_local=same_path(resolved,localProd);

        rtxt=fileread(resolved);
        OUT.resolved_has_r5_cache_props= ...
            contains(rtxt,'reference_only_prepared_packet_cache_valid') && ...
            contains(rtxt,'reference_only_prepared_packet_cache_hit_count') && ...
            contains(rtxt,'reference_only_prepared_packet_cache_store_count') && ...
            contains(rtxt,'reference_only_prepared_packet_cache_commit_count');

        say(f,' resolved SHA                          = %s',resolvedSha);
        say(f,' resolved SHA is exact R5               = %d',OUT.resolved_sha_exact);
        say(f,' resolved path == installer production  = %d',OUT.resolved_same_as_local);
        say(f,' resolved R5 cache properties present   = %d',OUT.resolved_has_r5_cache_props);
    else
        OUT.resolved_sha='';
        OUT.resolved_sha_exact=false;
        OUT.resolved_same_as_local=false;
        OUT.resolved_has_r5_cache_props=false;
    end

    % In-memory source inventory.
    mem=inmem('-completenames');
    hits=mem(contains(lower(string(mem)),'replanning_system'));
    OUT.inmem_replanning_hits=cellstr(hits);

    say(f,'');
    say(f,'[C. IN-MEMORY REPLANNING FILES]');
    say(f,' matching inmem entries                 = %d',numel(hits));
    for k=1:numel(hits)
        say(f,'   %s',char(hits(k)));
    end

    % Live object.
    [obj,objNames]=find_live_replanner();
    OUT.live_object_names=objNames;
    OUT.live_object_available=~isempty(obj);

    say(f,'');
    say(f,'[D. LIVE REPLANNING_SYSTEM OBJECT]');
    say(f,' direct base-workspace objects found    = %d',numel(objNames));
    if ~isempty(objNames)
        say(f,' names                                  = %s',strjoin(objNames,', '));
    end

    if ~isempty(obj)
        pnames={ ...
            'reference_only_prepared_packet_cache_valid', ...
            'reference_only_prepared_packet_cache_hit_count', ...
            'reference_only_prepared_packet_cache_store_count', ...
            'reference_only_prepared_packet_cache_invalidate_count', ...
            'reference_only_prepared_packet_cache_commit_count', ...
            'reference_only_runtime_packet_reuse_count', ...
            'reference_only_runtime_packet_rebuild_count', ...
            'reference_only_fast_packet_rebuild_count', ...
            'generator_request_count', ...
            'generator_deadline_fail_count', ...
            'replan_commit_count'};

        vals=struct();
        missing={};
        for k=1:numel(pnames)
            nm=pnames{k};
            if isprop(obj,nm)
                vals.(nm)=obj.(nm);
            else
                missing{end+1}=nm; %#ok<AGROW>
            end
        end
        OUT.live_values=vals;
        OUT.live_missing_properties=missing;
        OUT.live_has_r5_properties=isempty(intersect( ...
            missing,{ ...
            'reference_only_prepared_packet_cache_valid', ...
            'reference_only_prepared_packet_cache_hit_count', ...
            'reference_only_prepared_packet_cache_store_count', ...
            'reference_only_prepared_packet_cache_invalidate_count', ...
            'reference_only_prepared_packet_cache_commit_count'}));

        say(f,' live object has all R5 cache props     = %d',OUT.live_has_r5_properties);
        if ~isempty(missing)
            say(f,' missing properties                     = %s',strjoin(missing,', '));
        end

        if OUT.live_has_r5_properties
            say(f,' cache valid                            = %d',logical(vals.reference_only_prepared_packet_cache_valid));
            say(f,' cache hit count                        = %.0f',double(vals.reference_only_prepared_packet_cache_hit_count));
            say(f,' cache store count                      = %.0f',double(vals.reference_only_prepared_packet_cache_store_count));
            say(f,' cache invalidate count                 = %.0f',double(vals.reference_only_prepared_packet_cache_invalidate_count));
            say(f,' cache successful-consume commit count  = %.0f',double(vals.reference_only_prepared_packet_cache_commit_count));
        end

        if isfield(vals,'reference_only_runtime_packet_reuse_count')
            say(f,' active runtime packet reuse count      = %.0f',double(vals.reference_only_runtime_packet_reuse_count));
        end
        if isfield(vals,'reference_only_runtime_packet_rebuild_count')
            say(f,' runtime packet rebuild count           = %.0f',double(vals.reference_only_runtime_packet_rebuild_count));
        end
        if isfield(vals,'reference_only_fast_packet_rebuild_count')
            say(f,' fast packet rebuild count              = %.0f',double(vals.reference_only_fast_packet_rebuild_count));
        end
        if isfield(vals,'generator_request_count')
            say(f,' generator request count                = %.0f',double(vals.generator_request_count));
        end
        if isfield(vals,'generator_deadline_fail_count')
            say(f,' generator deadline fail count          = %.0f',double(vals.generator_deadline_fail_count));
        end
        if isfield(vals,'replan_commit_count')
            say(f,' replan commit count                    = %.0f',double(vals.replan_commit_count));
        end
    else
        OUT.live_has_r5_properties=false;
        OUT.live_values=struct();
        OUT.live_missing_properties={};
        say(f,' no direct live REPLANNING_SYSTEM object found in base workspace.');
    end

    % Conservative classification.
    say(f,'');
    say(f,'[E. CLASSIFICATION]');

    if ~OUT.local_sha_exact
        OUT.status='INSTALLER_LOCAL_PRODUCTION_NOT_R5__PATH_OR_FILE_MISMATCH';
        say(f,' RESULT: installer-directory production is NOT the installed R5 SHA.');
    elseif ~OUT.resolved_sha_exact
        OUT.status='MATLAB_RESOLVES_DIFFERENT_REPLANNING_SYSTEM__SHADOWING_CONFIRMED';
        say(f,' RESULT: production file changed, BUT MATLAB resolves a different REPLANNING_SYSTEM.');
    elseif ~OUT.live_object_available
        OUT.status='R5_SOURCE_RESOLVED__LIVE_OBJECT_NOT_DIRECTLY_VISIBLE';
        say(f,' RESULT: MATLAB resolves R5 source, but direct live object was not found.');
    elseif ~OUT.live_has_r5_properties
        OUT.status='R5_SOURCE_RESOLVED__LIVE_OBJECT_IS_PREINSTALL_OR_DIFFERENT_CLASS_INSTANCE';
        say(f,' RESULT: resolved source is R5, but the live object lacks R5 cache properties.');
    else
        hitsN=double(OUT.live_values.reference_only_prepared_packet_cache_hit_count);
        storesN=double(OUT.live_values.reference_only_prepared_packet_cache_store_count);
        commitsN=double(OUT.live_values.reference_only_prepared_packet_cache_commit_count);
        invN=double(OUT.live_values.reference_only_prepared_packet_cache_invalidate_count);

        if storesN==0 && hitsN==0
            OUT.status='R5_ACTIVE__CACHE_PATH_NOT_TRIGGERED_IN_THIS_RUN';
            say(f,' RESULT: R5 is loaded, but this run never produced a qualifying predeadline-reject cache store.');
        elseif storesN>0 && hitsN==0
            OUT.status='R5_ACTIVE__CACHE_STORED_BUT_NO_EXACT_HIT';
            say(f,' RESULT: R5 stored cache data, but no later request passed the exact cache reuse gate.');
        elseif hitsN>0 && commitsN==0
            OUT.status='R5_ACTIVE__CACHE_HIT_OBSERVED__NO_SUCCESSFUL_CACHE_CONSUME_COMMIT';
            say(f,' RESULT: cache hit occurred, but no successful commit consumed it.');
        else
            OUT.status='R5_ACTIVE__CACHE_HIT_AND_COMMIT_OBSERVED';
            say(f,' RESULT: cache path was exercised and at least one successful cache-consume commit occurred.');
        end

        say(f,' stores/hits/commits/invalidates          = %.0f / %.0f / %.0f / %.0f', ...
            storesN,hitsN,commitsN,invN);
    end

    say(f,'');
    say(f,' IMPORTANT: AF7R58Z-R5 intentionally does NOT change trajectory geometry.');
    say(f,' IMPORTANT: unchanged "bloated" avoidance trajectory is therefore expected at this stage.');
    say(f,' IMPORTANT: timing can improve only on requests that qualify for cache store/hit.');

catch ME
    OUT.status='AUDIT_ERROR__NO_PRODUCTION_CHANGE';
    OUT.error=struct('identifier',ME.identifier,'message',ME.message,'stack',{ME.stack});
    say(f,'');
    say(f,'[AUDIT ERROR]');
    say(f,' identifier = %s',ME.identifier);
    say(f,' message    = %s',ME.message);
end

save(fullfile(outdir,'AF7R58ZA_OUT.mat'),'OUT','-v7');
assignin('base','AF7R58ZA_OUT',OUT);

say(f,'');
say(f,'[RETURN THESE TWO FILES]');
say(f,' RETURN: %s',report);
say(f,' RETURN: %s',fullfile(outdir,'AF7R58ZA_OUT.mat'));
say(f,' status = %s',OUT.status);
end


function [obj,names]=find_live_replanner()
obj=[];
names={};
W=evalin('base','whos');

for k=1:numel(W)
    try
        v=evalin('base',W(k).name);
    catch
        continue;
    end
    if isa(v,'REPLANNING_SYSTEM')
        names{end+1}=W(k).name; %#ok<AGROW>
    end
end

if numel(names)==1
    obj=evalin('base',names{1});
end
end


function tf=same_path(a,b)
if ispc
    tf=strcmpi(char(java.io.File(a).getCanonicalPath()), ...
               char(java.io.File(b).getCanonicalPath()));
else
    tf=strcmp(char(java.io.File(a).getCanonicalPath()), ...
              char(java.io.File(b).getCanonicalPath()));
end
end


function s=safe_text(s)
if isempty(s)
    s='<EMPTY>';
end
end


function print_multiline(f,s)
s=strrep(s,sprintf('\r\n'),sprintf('\n'));
s=strrep(s,sprintf('\r'),sprintf('\n'));
C=regexp(s,'\n','split');
for k=1:numel(C)
    if ~isempty(strtrim(C{k}))
        say(f,'   %s',C{k});
    end
end
end


function h=file_hash(p)
fid=fopen(p,'rb'); assert(fid>=0,'cannot read %s',p);
cc=onCleanup(@()fclose(fid)); %#ok<NASGU>
b=fread(fid,Inf,'*uint8');

m=java.security.MessageDigest.getInstance('SHA-256');
m.update(b);
d=typecast(m.digest(),'uint8');
h=lower(reshape(dec2hex(d,2).',1,[]));
end


function say(f,fmt,varargin)
s=sprintf(fmt,varargin{:});
fprintf(1,'%s\n',s);
fprintf(f,'%s\n',s);
end
