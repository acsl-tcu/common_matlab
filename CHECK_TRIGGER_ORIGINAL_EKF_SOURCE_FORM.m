function R=CHECK_TRIGGER_ORIGINAL_EKF_SOURCE_FORM()
% Read-only SOURCE identification for the actual EKF nonlinear update.
% No covariance-to-deterministic-error assumption; not Flight proof.
R=struct('status','UNKNOWN_EKF_SOURCE','reason','', ...
    'active_file','','sha256','', 'exact_source_pinned',false, ...
    'nonlinear_projection_obligation',true, ...
    'bounded_noise_verified',false,'bounded_model_error_verified',false, ...
    'gain_interval_verified',false,'safe_to_commit',false);
try
    p=which('EKF');
    assert(ischar(p)&&~isempty(p)&&isfile(p), ...
        'CHECK_TRIGGER_ORIGINAL_EKF_SOURCE_FORM:NotFound', ...
        'The active original EKF.m must be on the MATLAB path.');
    R.active_file=p;
    f=fopen(p,'rb');
    assert(f>=0,'CHECK_TRIGGER_ORIGINAL_EKF_SOURCE_FORM:Read');
    close_guard=onCleanup(@()fclose(f)); %#ok<NASGU>
    bytes=fread(f,inf,'*uint8');
    md=java.security.MessageDigest.getInstance('SHA-256');
    md.update(typecast(bytes,'int8'));
    h=typecast(int8(md.digest()),'uint8');
    R.sha256=lower(reshape(dec2hex(h,2).',1,[]));
    % From the archived actual Flight EKF.m in R193_USED_FLIGHT, NOT
    % inferred from names, covariance matrices or source text patterns.
    known='54387629518f1ec2080eb03ff1c00a45140e0182ef83f4e236ef256016e23960';
    if ~strcmp(R.sha256,known)
        R.status='UNKNOWN_ACTIVE_EKF_DIFFERS_FROM_AUDITED_SOURCE';
        R.reason='Source differs. Audit current EKF.do, projection, gain and hybrid steps before proof.';
        return
    end
    R.exact_source_pinned=true;
    R.status='PASS_PINNED_EKF_NONLINEAR_UPDATE_SOURCE__NOT_ERROR_PROOF';
    R.reason=['Original EKF.do uses model.do, gain G, innovation y-yh, ' ...
        'then model.projection. Actual bounded process/measurement noise, ' ...
        'projection image and nonlinear gain bounds remain unproved.'];
catch ME
    R.reason=[ME.identifier ' | ' ME.message];
end
f=fopen('TRIGGER_EKF_SOURCE_FORM_RESULT.txt','w');
if f>=0
    fprintf(f,'status=%s\nsource_sha256=%s\nreason=%s\n', ...
        R.status,R.sha256,R.reason);
    fclose(f);
end
end
