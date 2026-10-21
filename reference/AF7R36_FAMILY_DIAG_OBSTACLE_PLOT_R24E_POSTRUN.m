function S=AF7R36_FAMILY_DIAG_OBSTACLE_PLOT_R24E_POSTRUN()
assert(evalin('base','exist(''replanner_instance'',''var'')==1'));
R=evalin('base','replanner_instance');
P=R.get_staged_pipeline_status();

seen=get8(P,'r24e_family_seen');
exact=get8(P,'r24e_family_exact');
xrej=get8(P,'r24e_family_exact_reject');
spass=get8(P,'r24e_family_speed_pass');
sfail=get8(P,'r24e_family_speed_fail');
splitrej=getscalar(P,'r24e_split_representability_reject');

fprintf('\n====================================================================================================\n');
fprintf(' AF7R36 FAMILY-RESOLVED EXACT-C7 / SPEED DIAGNOSTIC R24E\n');
fprintf('====================================================================================================\n');
fprintf('candidate seen / success                    = %d / %d\n', ...
    logical(P.predeadline_candidate_seen),logical(P.predeadline_candidate_success));
fprintf('reason code                                 = %.0f\n',double(P.predeadline_reason_code));
fprintf('candidate family                            = %s\n',char(P.predeadline_candidate_family));
fprintf('selected exact C7                           = %d\n',logical(P.predeadline_c7_exact));

names={'UNKNOWN','MINIMUM_DEVIATION_AFFINE','LEGACY_MINIMUM_SNAP','FAMILY_3', ...
       'TOWING_C7_MIN_NORM_CONSTANT','TOWING_C7_SPEED_OPT_SPLIT','FAMILY_6','FAMILY_7'};

fprintf('\n[FAMILY BREAKDOWN]\n');
fprintf('%-29s %6s %6s %7s %9s %9s\n', ...
    'family','seen','exact','C7rej','speedPASS','speedFAIL');
for k=1:8
    fprintf('%d %-27s %6.0f %6.0f %7.0f %9.0f %9.0f\n', ...
        k-1,names{k},seen(k),exact(k),xrej(k),spass(k),sfail(k));
end
fprintf('split representability rejects              = %.0f\n',splitrej);

if seen(2)>0 && xrej(2)>0 && sum(spass(5:6))==0
    cls='AFFINE_FREEDOM_EXISTS_BUT_CURRENT_EXACT_C7_FAMILIES_DO_NOT_CLOSE_SPEED';
elseif exact(6)>0 && sfail(6)>0
    cls='EXACT_SPLIT_REACHED_SPEED_TEST_BUT_DID_NOT_CLOSE';
elseif splitrej>0 && exact(6)==0
    cls='SPLIT_REPRESENTABILITY_BLOCKED_FAMILY5';
elseif any(spass>0) && double(P.predeadline_rejoin_pass_count)==0
    cls='SPEED_PASSED_BUT_REJOIN_FAILED';
elseif any(spass>0) && double(P.predeadline_geometry_test_count)>0 && ~logical(P.predeadline_geometry_pass)
    cls='SPEED_AND_REJOIN_PASSED_BUT_WHOLE_BODY_FAILED';
else
    cls='COUNTERS_REQUIRE_CASE_REVIEW';
end

fprintf('\nclassification                              = %s\n',cls);
fprintf('\n[PLOTS]\n');
if evalin('base','exist(''logger'',''var'')==1')
    try
        LGR=evalin('base','logger');
        dt=double(R.prediction_dt);
        AF7R24E_OBSTACLE_PLOTS(LGR,dt,R);
        fprintf('obstacle overlay / Figure 12                = CREATED\n');
    catch MEP
        fprintf(2,'obstacle plot warning                        = %s | %s\n',MEP.identifier,MEP.message);
    end
else
    fprintf('logger unavailable                           = plots skipped\n');
end

fprintf('====================================================================================================\n');

S=struct();
S.schema='AF7_FAMILY_DIAG_OBSTACLE_PLOT_R24E';
S.pipeline=P;
S.family_seen=seen;
S.family_exact=exact;
S.family_exact_reject=xrej;
S.family_speed_pass=spass;
S.family_speed_fail=sfail;
S.split_representability_reject=splitrej;
S.classification=cls;
S.obstacle_plot_helper='AF7R24E_OBSTACLE_PLOTS';
end

function v=get8(S,name)
v=nan(1,8);
if isfield(S,name)
    q=double(S.(name));
    q=q(:).';
    if numel(q)==8,v=q;end
end
end

function v=getscalar(S,name)
v=NaN;
if isfield(S,name)&&isnumeric(S.(name))&&isscalar(S.(name))
    v=double(S.(name));
end
end
