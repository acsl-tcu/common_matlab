function S=AF7R36_FAMILY_DIAG_OBSTACLE_PLOT_R24_POSTRUN()
assert(evalin('base','exist(''replanner_instance'',''var'')==1'));
R=evalin('base','replanner_instance');
P=R.get_staged_pipeline_status();

seen=get8(P,'r24_family_seen');
exact=get8(P,'r24_family_exact');
xrej=get8(P,'r24_family_exact_reject');
spass=get8(P,'r24_family_speed_pass');
sfail=get8(P,'r24_family_speed_fail');
fwit=get8(P,'r24_family_finite_speed_witness');
splitrej=getscalar(P,'r24_split_representability_reject');

fprintf('\n====================================================================================================\n');
fprintf(' AF7R36 FAMILY-RESOLVED EXACT-C7 / SPEED DIAGNOSTIC R24\n');
fprintf('====================================================================================================\n');
fprintf('candidate seen / success                    = %d / %d\n', ...
    logical(P.predeadline_candidate_seen),logical(P.predeadline_candidate_success));
fprintf('reason code                                 = %.0f\n',double(P.predeadline_reason_code));
fprintf('candidate family                            = %s\n',char(P.predeadline_candidate_family));
fprintf('selected exact C7                           = %d\n',logical(P.predeadline_c7_exact));
fprintf('t0 / t1                                    = %.9g / %.9g s\n', ...
    double(P.predeadline_t0),double(P.predeadline_t1));

names={'UNKNOWN','MINIMUM_DEVIATION_AFFINE','LEGACY_MINIMUM_SNAP','FAMILY_3', ...
       'TOWING_C7_MIN_NORM_CONSTANT','TOWING_C7_SPEED_OPT_SPLIT','FAMILY_6','FAMILY_7'};

fprintf('\n[FAMILY BREAKDOWN]\n');
fprintf('%-29s %6s %6s %7s %9s %9s %9s\n', ...
    'family','seen','exact','C7rej','speedPASS','speedFAIL','finiteWit');
for k=1:8
    fprintf('%d %-27s %6.0f %6.0f %7.0f %9.0f %9.0f %9.0f\n', ...
        k-1,names{k},seen(k),exact(k),xrej(k),spass(k),sfail(k),fwit(k));
end
fprintf('split representability rejects              = %.0f\n',splitrej);

if seen(2)>0 && xrej(2)>0 && spass(5)==0 && spass(6)==0
    cls='AFFINE_FREEDOM_EXISTS_BUT_IS_BLOCKED_BY_EXACT_C7';
elseif exact(6)>0 && sfail(6)>0 && fwit(6)>0
    cls='EXACT_SPLIT_HAS_REAL_FINITE_POINT_SPEED_VIOLATION';
elseif exact(6)>0 && sfail(6)>0
    cls='EXACT_SPLIT_SPEED_CERTIFICATE_DID_NOT_CLOSE';
elseif splitrej>0 && exact(6)==0
    cls='SPLIT_REPRESENTABILITY_BLOCKED_FAMILY5';
elseif any(spass>0) && double(P.predeadline_rejoin_pass_count)==0
    cls='SPEED_PASSED_BUT_REJOIN_FAILED';
elseif any(spass>0) && double(P.predeadline_geometry_test_count)>0 && ~logical(P.predeadline_geometry_pass)
    cls='SPEED_AND_REJOIN_PASSED_BUT_WHOLE_BODY_FAILED';
else
    cls='COUNTERS_REQUIRE_CASE_REVIEW';
end

fprintf('\n[CLASSIFICATION]\n');
fprintf('%s\n',cls);

fprintf('\n[OBSTACLE MAP]\n');
obs=R.static_obstacle_map;
for k=1:numel(obs)
    o=obs(k);
    id=k;if isfield(o,'id'),id=double(o.id);end
    typ='';if isfield(o,'type'),typ=char(o.type);end
    c=[NaN;NaN;NaN];if isfield(o,'p_center'),c=double(o.p_center(:));end
    rp=[];if isfield(o,'raw_param'),rp=double(o.raw_param(:));end
    fprintf('id=%g type=%s center=[%.6g %.6g %.6g] raw=%s\n', ...
        id,typ,c(1),c(2),c(3),mat2str(rp.',6));
end
fprintf('====================================================================================================\n');

S=struct();
S.schema='AF7_FAMILY_DIAG_OBSTACLE_PLOT_R24';
S.pipeline=P;
S.family_seen=seen;
S.family_exact=exact;
S.family_exact_reject=xrej;
S.family_speed_pass=spass;
S.family_speed_fail=sfail;
S.family_finite_speed_witness=fwit;
S.split_representability_reject=splitrej;
S.classification=cls;
end

function v=get8(S,name)
v=nan(1,8);
if isfield(S,name)
    q=S.(name);
    q=double(q);
    q=q(:).';
    if numel(q)==8
        v=q;
    end
end
end

function v=getscalar(S,name)
v=NaN;
if isfield(S,name)
    q=S.(name);
    if isnumeric(q)&&isscalar(q)
        v=double(q);
    end
end
end
