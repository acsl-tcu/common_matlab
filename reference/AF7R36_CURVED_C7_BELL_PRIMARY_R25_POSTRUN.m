function S=AF7R36_CURVED_C7_BELL_PRIMARY_R25_POSTRUN()
assert(evalin('base','exist(''replanner_instance'',''var'')==1'));
R=evalin('base','replanner_instance');
P=R.get_staged_pipeline_status();

fprintf('\n====================================================================================================\n');
fprintf(' AF7R36 CURVED EXACT-C7 ZERO-JET BELL PRIMARY R25 RESULT\n');
fprintf('====================================================================================================\n');
fprintf('candidate seen / success                    = %d / %d\n', ...
    logical(P.predeadline_candidate_seen),logical(P.predeadline_candidate_success));
fprintf('reason code                                 = %.0f\n',double(P.predeadline_reason_code));
fprintf('candidate family                            = %s\n',char(P.predeadline_candidate_family));
fprintf('selected exact C7                           = %d\n',logical(P.predeadline_c7_exact));
fprintf('speed / rejoin / geometry counts            = %.0f / %.0f / %.0f\n', ...
    double(P.predeadline_speed_pass_count), ...
    double(P.predeadline_rejoin_pass_count), ...
    double(P.predeadline_geometry_test_count));
fprintf('geometry pass                               = %d\n',logical(P.predeadline_geometry_pass));
fprintf('Stage1 wall time                            = %.9g ms\n',double(P.last_stage1_ms));

if isfield(P,'r24e_family_seen')
    seen=double(P.r24e_family_seen(:).');
    exact=double(P.r24e_family_exact(:).');
    xr=double(P.r24e_family_exact_reject(:).');
    sp=double(P.r24e_family_speed_pass(:).');
    sf=double(P.r24e_family_speed_fail(:).');
    fprintf('\n[FAMILY BREAKDOWN]\n');
    fprintf('%-31s %6s %6s %7s %9s %9s\n', ...
        'family','seen','exact','C7rej','speedPASS','speedFAIL');
    names={'UNKNOWN','MINIMUM_DEVIATION_AFFINE','LEGACY_MINIMUM_SNAP','FAMILY_3', ...
        'TOWING_C7_MIN_NORM_CONSTANT','TOWING_C7_SPEED_OPT_SPLIT', ...
        'CURVED_C7_ZERO_JET_BELL','FAMILY_7'};
    for k=1:min(8,numel(seen))
        fprintf('%d %-29s %6.0f %6.0f %7.0f %9.0f %9.0f\n', ...
            k-1,names{k},seen(k),exact(k),xr(k),sp(k),sf(k));
    end
end

mode=char(R.active_reference_mode);
fprintf('\n[ACTIVE]\n');
fprintf('mode / version                              = %s / %.0f\n',mode,double(R.active_reference_version));
if strcmpi(mode,'REPAIRED')
    AR=R.active_repaired_reference;
    if isfield(AR,'representation')&&isfield(AR.representation,'kind')
        fprintf('representation kind                         = %s\n',char(AR.representation.kind));
    end
    if isfield(R.active_repaired_certificates,'smoothness')
        C=R.active_repaired_certificates.smoothness;
        fprintf('smoothness source                           = %s\n',char(C.source));
        fprintf('smoothness order                            = %.0f\n',double(C.continuity_order));
    end
end

try
    if evalin('base','exist(''logger'',''var'')==1')
        AF7R25_OBSTACLE_PLOTS;
        fprintf('obstacle plots                              = UPDATED\n');
    end
catch MEP
    fprintf(2,'obstacle plot warning                        = %s | %s\n',MEP.identifier,MEP.message);
end

fprintf('====================================================================================================\n');
S=struct('schema','AF7_CURVED_C7_ZERO_JET_BELL_R25','pipeline',P,'active_mode',mode);
end
