function OUT = AF7R58ZE_ONEFILE_MONOTONE_CACHE_EFFECT_AUDIT()
% AF7R58ZE - read-only post-run audit for AF7R58ZD.

root=fileparts(mfilename('fullpath'));
outdir=tempname(root);
[ok,msg]=mkdir(outdir); assert(ok,'%s',msg);
report=fullfile(outdir,'AF7R58ZE_REPORT.txt');
f=fopen(report,'wt','n','UTF-8'); assert(f>=0);
cc=onCleanup(@()fclose(f)); %#ok<NASGU>

say(f,'====================================================================================================');
say(f,' AF7R58ZE | MONOTONE CACHE POST-RUN EFFECT + AVAILABLE TIMING AUDIT');
say(f,'====================================================================================================');
say(f,'[AUTHORITY] readOnly=1 | productionModified=0 | noSyntheticReplan=1');

OUT=struct(); OUT.status='OPEN'; OUT.error=struct();

try
    [obj,names]=find_live_replanner();
    OUT.live_names=names;
    assert(~isempty(obj),'[AF7R58ZE] live replanner object not found.');

    props={ ...
        'reference_only_prepared_packet_cache_store_count', ...
        'reference_only_prepared_packet_cache_gate_pass_count', ...
        'reference_only_prepared_packet_cache_hit_count', ...
        'reference_only_prepared_packet_cache_commit_count', ...
        'reference_only_prepared_packet_cache_invalidate_count', ...
        'reference_only_prepared_packet_cache_last_reason_code', ...
        'reference_only_prepared_packet_cache_miss_reason_count', ...
        'reference_only_runtime_packet_rebuild_count', ...
        'reference_only_fast_packet_rebuild_count', ...
        'generator_request_count', ...
        'generator_deadline_fail_count', ...
        'replan_commit_count', ...
        'generator_last_cycle_critical_ms', ...
        'generator_last_callback_ms', ...
        'generator_last_validation_ms', ...
        'generator_last_critical_ms'};

    V=struct(); missing={};
    for k=1:numel(props)
        if isprop(obj,props{k}), V.(props{k})=obj.(props{k});
        else, missing{end+1}=props{k}; %#ok<AGROW>
        end
    end
    OUT.values=V; OUT.missing=missing;

    say(f,'');
    say(f,'[A. CACHE EFFECT]');
    core={ ...
        'reference_only_prepared_packet_cache_store_count', ...
        'reference_only_prepared_packet_cache_gate_pass_count', ...
        'reference_only_prepared_packet_cache_hit_count', ...
        'reference_only_prepared_packet_cache_commit_count', ...
        'reference_only_prepared_packet_cache_invalidate_count', ...
        'reference_only_runtime_packet_rebuild_count', ...
        'generator_request_count', ...
        'generator_deadline_fail_count', ...
        'replan_commit_count'};
    for k=1:numel(core)
        nm=core{k};
        if isfield(V,nm), say(f,' %-48s = %.0f',nm,double(V.(nm))); end
    end

    rc=double(V.reference_only_prepared_packet_cache_miss_reason_count);
    say(f,' miss reason counts = %s',mat2str(rc));
    say(f,' reason 5 now means CACHED_PAYLOAD_BOUND_TOO_SMALL (current increased).');
    say(f,' reason 6 now means CACHED_UAV_BOUND_TOO_SMALL (current increased).');

    say(f,'');
    say(f,'[B. AVAILABLE LAST-REQUEST TIMING]');
    tprops={'generator_last_cycle_critical_ms','generator_last_callback_ms', ...
        'generator_last_validation_ms','generator_last_critical_ms'};
    for k=1:numel(tprops)
        nm=tprops{k};
        if isfield(V,nm), say(f,' %-38s = %.9g ms',nm,double(V.(nm))); end
    end

    % Discover legacy AF7R58 timing/diagnostic properties still resident.
    P=properties(obj);
    mask=contains(lower(string(P)),'af7r58') | ...
         contains(lower(string(P)),'timing_ring') | ...
         contains(lower(string(P)),'capture_overhead');
    diagNames=cellstr(string(P(mask)));
    OUT.legacy_diagnostic_properties=diagNames;

    say(f,'');
    say(f,'[C. LEGACY DIAGNOSTIC INSTRUMENTATION STILL PRESENT]');
    say(f,' matching diagnostic properties = %d',numel(diagNames));
    for k=1:numel(diagNames)
        say(f,'   %s',diagNames{k});
    end

    hits=double(V.reference_only_prepared_packet_cache_hit_count);
    fails=double(V.generator_deadline_fail_count);
    req=double(V.generator_request_count);

    if hits>0 && fails<req
        OUT.status='MONOTONE_CACHE_HIT_OBSERVED__REMOVE_DIAGNOSTIC_OVERHEAD_NEXT';
    elseif hits>0
        OUT.status='MONOTONE_CACHE_HIT_OBSERVED__DEADLINE_STILL_OPEN__REMOVE_DIAGNOSTICS_NEXT';
    elseif rc(5)>0
        OUT.status='PAYLOAD_BOUND_KEEPS_INCREASING__INCREMENTAL_SPEED_RETARGET_NEXT';
    else
        OUT.status='MONOTONE_CACHE_NO_HIT__OTHER_SAFE_MISS_REASON';
    end

catch ME
    OUT.status='AUDIT_ERROR__NO_PRODUCTION_CHANGE';
    OUT.error=struct('identifier',ME.identifier,'message',ME.message,'stack',{ME.stack});
    say(f,'');
    say(f,'[AUDIT ERROR] %s | %s',ME.identifier,ME.message);
end

save(fullfile(outdir,'AF7R58ZE_OUT.mat'),'OUT','-v7');
assignin('base','AF7R58ZE_OUT',OUT);
say(f,'');
say(f,'RETURN: %s',report);
say(f,'RETURN: %s',fullfile(outdir,'AF7R58ZE_OUT.mat'));
say(f,'status = %s',OUT.status);
end


function [obj,names]=find_live_replanner()
obj=[]; names={};
W=evalin('base','whos');
for k=1:numel(W)
    try, v=evalin('base',W(k).name); catch, continue; end
    if isa(v,'REPLANNING_SYSTEM')
        names{end+1}=W(k).name; %#ok<AGROW>
    end
end
if numel(names)==1, obj=evalin('base',names{1}); end
end


function say(f,fmt,varargin)
s=sprintf(fmt,varargin{:});
fprintf(1,'%s\n',s);
fprintf(f,'%s\n',s);
end
