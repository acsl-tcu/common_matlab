function OUT = AF7R58ZG_ONEFILE_POST_DIAGNOSTIC_REMOVAL_AUDIT()
% AF7R58ZG
% Read-only audit after AF7R58ZF + clean run.

root=fileparts(mfilename('fullpath'));
outdir=tempname(root);
[ok,msg]=mkdir(outdir); assert(ok,'%s',msg);
report=fullfile(outdir,'AF7R58ZG_REPORT.txt');
f=fopen(report,'wt','n','UTF-8'); assert(f>=0);
cc=onCleanup(@()fclose(f)); %#ok<NASGU>

say(f,'====================================================================================================');
say(f,' AF7R58ZG | POST-DIAGNOSTIC-REMOVAL PRODUCTION TIMING AUDIT');
say(f,'====================================================================================================');
say(f,'[AUTHORITY] readOnly=1 | productionModified=0 | noSyntheticReplan=1');

OUT=struct(); OUT.status='OPEN'; OUT.error=struct();

try
    [obj,names]=find_live_replanner();
    assert(~isempty(obj),'[AF7R58ZG] live REPLANNING_SYSTEM object not found.');
    OUT.live_names=names;

    flags={ ...
        'ENABLE_AF7R58M_TIMING_RING', ...
        'ENABLE_AF7R58Q_MIN_TIMING', ...
        'ENABLE_AF7R58T_OTHER_TIMING', ...
        'ENABLE_AF7R58V_R4_PRETRY_TIMING'};

    say(f,'');
    say(f,'[A. LIVE DIAGNOSTIC ENABLE FLAGS]');
    allOff=true;
    F=struct();
    for k=1:numel(flags)
        assert(isprop(obj,flags{k}),'[AF7R58ZG] missing flag %s.',flags{k});
        v=logical(obj.(flags{k}));
        F.(flags{k})=v;
        allOff=allOff && ~v;
        say(f,' %-42s = %d',flags{k},v);
    end
    OUT.flags=F;
    OUT.all_diagnostics_off=allOff;

    say(f,'');
    say(f,'[B. CURRENT REQUEST / DEADLINE STATE]');
    p={ ...
        'generator_request_count', ...
        'generator_deadline_fail_count', ...
        'generator_last_cycle_critical_ms', ...
        'generator_last_callback_ms', ...
        'generator_last_critical_ms', ...
        'reference_only_prepared_packet_cache_hit_count', ...
        'reference_only_prepared_packet_cache_store_count', ...
        'reference_only_prepared_packet_cache_invalidate_count', ...
        'reference_only_prepared_packet_cache_miss_reason_count', ...
        'reference_only_runtime_packet_rebuild_count', ...
        'replan_commit_count'};
    V=struct();
    for k=1:numel(p)
        if isprop(obj,p{k})
            V.(p{k})=obj.(p{k});
            if isnumeric(V.(p{k})) && isscalar(V.(p{k}))
                say(f,' %-48s = %.9g',p{k},double(V.(p{k})));
            elseif isnumeric(V.(p{k}))
                say(f,' %-48s = %s',p{k},mat2str(double(V.(p{k}))));
            end
        end
    end
    OUT.values=V;

    if isfield(V,'generator_last_cycle_critical_ms')
        cycle=double(V.generator_last_cycle_critical_ms);
    else
        cycle=NaN;
    end

    if isfield(V,'generator_request_count') && isfield(V,'generator_deadline_fail_count')
        req=double(V.generator_request_count);
        fail=double(V.generator_deadline_fail_count);
    else
        req=NaN; fail=NaN;
    end

    say(f,'');
    say(f,'[C. CLASSIFICATION]');
    if ~allOff
        OUT.status='LEGACY_DIAGNOSTICS_STILL_ENABLED__SOURCE_OR_CONSTRUCTOR_OVERRIDE';
    elseif isfinite(cycle) && cycle<=4 && fail<req
        OUT.status='DIAGNOSTICS_OFF__LAST_CYCLE_WITHIN_4MS__MULTI_REQUEST_SPIKE_AUDIT_NEXT';
    elseif isfinite(cycle) && cycle>4
        OUT.status='DIAGNOSTICS_OFF__CYCLE_STILL_OVER_4MS__INCREMENTAL_SPEED_RETARGET_NEXT';
    else
        OUT.status='DIAGNOSTICS_OFF__TIMING_EVIDENCE_INCOMPLETE';
    end
    say(f,' status = %s',OUT.status);

catch ME
    OUT.status='AUDIT_ERROR__NO_PRODUCTION_CHANGE';
    OUT.error=struct('identifier',ME.identifier,'message',ME.message,'stack',{ME.stack});
    say(f,'');
    say(f,'[AUDIT ERROR] %s | %s',ME.identifier,ME.message);
end

save(fullfile(outdir,'AF7R58ZG_OUT.mat'),'OUT','-v7');
assignin('base','AF7R58ZG_OUT',OUT);

say(f,'');
say(f,'RETURN: %s',report);
say(f,'RETURN: %s',fullfile(outdir,'AF7R58ZG_OUT.mat'));
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
