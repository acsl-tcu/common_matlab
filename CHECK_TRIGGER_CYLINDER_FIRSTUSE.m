function R=CHECK_TRIGGER_CYLINDER_FIRSTUSE(replanner_instance)
% Evidence of PRE-ARM cylinder path exercise, not Flight/JIT certification.
R=struct('status','NOT_COMPLETED_CYLINDER_FIRSTUSE', ...
    'cases',0,'reason','','safety_proven',false);
try
    assert(nargin==1 && isa(replanner_instance,'REPLANNING_SYSTEM'), ...
        'CHECK_TRIGGER_CYLINDER_FIRSTUSE:Input');
    S=replanner_instance.get_status();
    R.cases=S.isolated_cylinder_first_use_cases;
    R.reason=S.isolated_cylinder_first_use_reason;
    assert(strcmp(S.isolated_cylinder_first_use_status, ...
        'PASS_SYNTHETIC_CYLINDER_3_BRANCHES_8_EACH__NOT_FLIGHT_SAFETY') && ...
        R.cases==24, ...
        'CHECK_TRIGGER_CYLINDER_FIRSTUSE:Coverage', ...
        'The support/exact/noncontact/delayed-contact cylinder paths were not all executed.');
    R.status='PASS_CYLINDER_FIRSTUSE_24_CASES__JIT_EFFECT_UNPROVEN';
    R.reason='';
catch ME
    R.reason=[R.reason ' | ' ME.identifier ' | ' ME.message];
end
fid=fopen('TRIGGER_CYLINDER_FIRSTUSE_RESULT.txt','w');
if fid>=0
    fprintf(fid,'status=%s\ncases=%d\nreason=%s\n', ...
        R.status,R.cases,R.reason);
    fclose(fid);
end
end
