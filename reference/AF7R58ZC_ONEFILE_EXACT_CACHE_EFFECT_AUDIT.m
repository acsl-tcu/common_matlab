function OUT = AF7R58ZC_ONEFILE_EXACT_CACHE_EFFECT_AUDIT()
% AF7R58ZC - read-only post-clean-run audit for AF7R58ZB.

root=fileparts(mfilename('fullpath'));
outdir=tempname(root);
[ok,msg]=mkdir(outdir); assert(ok,'%s',msg);
report=fullfile(outdir,'AF7R58ZC_REPORT.txt');
f=fopen(report,'wt','n','UTF-8'); assert(f>=0);
c=onCleanup(@()fclose(f)); %#ok<NASGU>

say(f,'====================================================================================================');
say(f,' AF7R58ZC | EXACT PREPARED-CACHE POST-RUN EFFECT AUDIT');
say(f,'====================================================================================================');
say(f,'[AUTHORITY] readOnly=1 | productionModified=0 | noSyntheticReplan=1');

OUT=struct();
OUT.schema='AF7R58ZC_EXACT_CACHE_EFFECT_AUDIT_V1';
OUT.status='OPEN';
OUT.error=struct();

try
    resolved=which('REPLANNING_SYSTEM');
    OUT.resolved_path=resolved;
    OUT.resolved_sha=file_hash(resolved);

    [obj,names]=find_live_replanner();
    OUT.live_names=names;
    OUT.live_available=~isempty(obj);

    say(f,'');
    say(f,'[A. RUNTIME BINDING]');
    say(f,' resolved source = %s',resolved);
    say(f,' resolved SHA    = %s',OUT.resolved_sha);
    say(f,' live object     = %d',OUT.live_available);

    if isempty(obj)
        OUT.status='NO_DIRECT_LIVE_REPLANNER_OBJECT';
    else
        props={ ...
            'reference_only_prepared_packet_cache_valid', ...
            'reference_only_prepared_packet_cache_hit_count', ...
            'reference_only_prepared_packet_cache_store_count', ...
            'reference_only_prepared_packet_cache_invalidate_count', ...
            'reference_only_prepared_packet_cache_commit_count', ...
            'reference_only_prepared_packet_cache_gate_pass_count', ...
            'reference_only_prepared_packet_cache_last_reason_code', ...
            'reference_only_prepared_packet_cache_miss_reason_count', ...
            'reference_only_runtime_packet_rebuild_count', ...
            'reference_only_fast_packet_rebuild_count', ...
            'generator_request_count', ...
            'generator_deadline_fail_count', ...
            'replan_commit_count'};

        V=struct();
        missing={};
        for k=1:numel(props)
            if isprop(obj,props{k})
                V.(props{k})=obj.(props{k});
            else
                missing{end+1}=props{k}; %#ok<AGROW>
            end
        end
        OUT.values=V;
        OUT.missing=missing;

        say(f,'');
        say(f,'[B. CACHE COUNTERS]');
        if ~isempty(missing)
            say(f,' missing = %s',strjoin(missing,', '));
            OUT.status='ZB_PROPERTIES_MISSING__CLEAN_RERUN_WITH_NEW_CLASS_REQUIRED';
        else
            say(f,' valid             = %d',logical(V.reference_only_prepared_packet_cache_valid));
            say(f,' stores            = %.0f',double(V.reference_only_prepared_packet_cache_store_count));
            say(f,' gate passes       = %.0f',double(V.reference_only_prepared_packet_cache_gate_pass_count));
            say(f,' hits              = %.0f',double(V.reference_only_prepared_packet_cache_hit_count));
            say(f,' successful commits= %.0f',double(V.reference_only_prepared_packet_cache_commit_count));
            say(f,' invalidates       = %.0f',double(V.reference_only_prepared_packet_cache_invalidate_count));
            say(f,' last reason       = %.0f',double(V.reference_only_prepared_packet_cache_last_reason_code));
            say(f,' reason counts     = %s',mat2str(double(V.reference_only_prepared_packet_cache_miss_reason_count)));
            say(f,' runtime rebuilds  = %.0f',double(V.reference_only_runtime_packet_rebuild_count));
            say(f,' fast rebuilds     = %.0f',double(V.reference_only_fast_packet_rebuild_count));
            say(f,' requests          = %.0f',double(V.generator_request_count));
            say(f,' deadline fails    = %.0f',double(V.generator_deadline_fail_count));
            say(f,' commits           = %.0f',double(V.replan_commit_count));

            rc=double(V.reference_only_prepared_packet_cache_miss_reason_count);
            [mx,idx]=max(rc);
            OUT.dominant_miss_reason_code=idx;
            OUT.dominant_miss_reason_count=mx;

            labels={ ...
                'POLICY', ...
                'CACHE_SCHEMA_OR_SHAPE', ...
                'NEW_SPEED_CERT_INVALID', ...
                'CACHED_SPEED_CERT_INVALID', ...
                'PAYLOAD_SPEED_CHANGED', ...
                'UAV_SPEED_CHANGED', ...
                'CACHE_KEY_SHAPE', ...
                'ACTIVE_AUTHORITY_CHANGED', ...
                'LIVE_NUMERIC_DEP_CHANGED', ...
                'OBSTACLE_SOURCE_CHANGED', ...
                'SWEEP_MISMATCH', ...
                'PACKET_INVALID', ...
                'SENSOR_MISMATCH', ...
                'CACHED_DEP_INVALID', ...
                'RESERVED_15', ...
                'RESERVED_16'};
            OUT.reason_labels=labels;

            say(f,'');
            say(f,'[C. MISS REASON MAP]');
            for k=1:numel(labels)
                say(f,' %2d %-30s : %.0f',k,labels{k},rc(k));
            end
            say(f,' dominant miss reason = %d %s (%.0f)',idx,labels{idx},mx);

            if double(V.reference_only_prepared_packet_cache_hit_count)>0
                OUT.status='EXACT_CACHE_HIT_OBSERVED__TIMING_AUDIT_NEXT';
            elseif double(V.reference_only_prepared_packet_cache_store_count)>0 && mx>0
                OUT.status=['EXACT_CACHE_SAFE_MISS_DOMINANT__' labels{idx}];
            else
                OUT.status='EXACT_CACHE_PATH_NOT_EXERCISED';
            end
        end
    end
catch ME
    OUT.status='AUDIT_ERROR__NO_PRODUCTION_CHANGE';
    OUT.error=struct('identifier',ME.identifier,'message',ME.message,'stack',{ME.stack});
    say(f,'');
    say(f,'[AUDIT ERROR] %s | %s',ME.identifier,ME.message);
end

save(fullfile(outdir,'AF7R58ZC_OUT.mat'),'OUT','-v7');
assignin('base','AF7R58ZC_OUT',OUT);
say(f,'');
say(f,'RETURN: %s',report);
say(f,'RETURN: %s',fullfile(outdir,'AF7R58ZC_OUT.mat'));
say(f,'status = %s',OUT.status);
end


function [obj,names]=find_live_replanner()
obj=[]; names={};
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


function h=file_hash(p)
fid=fopen(p,'rb'); assert(fid>=0);
c=onCleanup(@()fclose(fid)); %#ok<NASGU>
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
