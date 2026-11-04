function T=CHECK_TRIGGER_DISCARDED_FULL_UPDATE(replanner_instance)
% Source-grounded inspection after ONE ordinary Flight; no intervention.
T=struct('status','NOT_COMPLETED_DISCARDED_FULL_UPDATE_CHECK', ...
    'preflight_status','','calls',0,'nonflight_calls',0, ...
    'actual_preflight_isolated',false,'reason','');
try
    S=replanner_instance.get_status();
    needed={'isolated_full_update_prepare_status', ...
        'isolated_full_update_prepare_calls', ...
        'isolated_full_update_prepare_nonflight_calls', ...
        'isolated_full_update_prepare_reason'};
    assert(isstruct(S)&&all(isfield(S,needed)), ...
        'CHECK_TRIGGER_DISCARDED_FULL_UPDATE:Status', ...
        'The expected updated REPLANNING_SYSTEM.m is not active.');
    T.preflight_status=S.isolated_full_update_prepare_status;
    T.calls=S.isolated_full_update_prepare_calls;
    T.nonflight_calls=S.isolated_full_update_prepare_nonflight_calls;
    T.actual_preflight_isolated=T.calls==24 && ...
        T.nonflight_calls==16 && ...
        strcmp(T.preflight_status, ...
        'PASS_DISCARDED_FULL_UPDATE_16_NONFLIGHT_8_FLIGHT__NOT_JIT_OR_SAFETY');
    if T.actual_preflight_isolated
        T.status='PASS_DISCARDED_SAME_CLASS_UPDATE_PATH__EFFECT_NOT_PROVEN';
    else
        T.reason=S.isolated_full_update_prepare_reason;
    end
catch ME
    T.reason=[ME.identifier ' | ' ME.message];
end
save('TRIGGER_DISCARDED_FULL_UPDATE_RESULT.mat','T');
f=fopen('TRIGGER_DISCARDED_FULL_UPDATE_RESULT.txt','w');
if f>=0
    fprintf(f,'status=%s\npreflight_status=%s\ncalls=%d\nnonflight_calls=%d\nreason=%s\n', ...
        T.status,T.preflight_status,T.calls,T.nonflight_calls,T.reason);
    fclose(f);
end
end
