function A=CHECK_TRIGGER_PREFLIGHT_GEOMETRY_AUDIT(replanner_instance)
% Disposable post-flight status inspection. No hard-coded Flight time.
A=struct('status','NOT_RUN_PREFLIGHT_GEOMETRY','reason','', ...
    'near_geometry_status','','safe_to_commit',false);
try
    S=replanner_instance.get_status();
    A.near_geometry_status=S.isolated_near_geometry_status;
    assert(strcmp(A.near_geometry_status, ...
        'SYNTHETIC_NONZERO_SUPPORT_AND_DISTANCE_EXERCISED'), ...
        'CHECK_TRIGGER_PREFLIGHT_GEOMETRY_AUDIT:Path', ...
        'Near-geometry preparation did not complete all required paths.');
    A.status='PASS_SOFTWARE_GEOMETRY_PREPARE__NO_JIT_GUARANTEE';
catch ME
    A.reason=[ME.identifier ' | ' ME.message];
end
save('TRIGGER_PREFLIGHT_GEOMETRY_RESULT.mat','A');
f=fopen('TRIGGER_PREFLIGHT_GEOMETRY_RESULT.txt','w');
if f>=0
    fprintf(f,'status=%s\nnear_geometry_status=%s\nreason=%s\n', ...
        A.status,A.near_geometry_status,A.reason);
    fclose(f);
end
end
