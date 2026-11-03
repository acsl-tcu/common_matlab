function R=CHECK_TRIGGER_BASELINE_FINGERPRINT()
% Disposable common_matlab/ CHECK. Does not change any Flight state.
% Fail when user source does not match the supplied experimental baseline.
R=struct('status','SOURCE_NOT_VERIFIED','source_path','','sha256','', ...
    'expected_sha256','5d8f644f39479061f94cee697bcc88729721685dc0d3587e4838c4ed511d9aab','reason','');
try
    p=which('REPLANNING_SYSTEM');
    assert(~isempty(p),'CHECK_TRIGGER_BASELINE_FINGERPRINT:Missing', ...
        'REPLANNING_SYSTEM.m was not found on the MATLAB path.');
    fid=fopen(p,'rb');
    assert(fid>=0,'CHECK_TRIGGER_BASELINE_FINGERPRINT:File','Could not read source.');
    cleanup=onCleanup(@()fclose(fid));
    bytes=fread(fid,inf,'*uint8');
    md=java.security.MessageDigest.getInstance('SHA-256');
    md.update(typecast(uint8(bytes),'int8'));
    digest=typecast(int8(md.digest()),'uint8');
    R.sha256=lower(reshape(dec2hex(digest,2).',1,[]));
    R.source_path=p;
    if strcmp(R.sha256,R.expected_sha256)
        R.status='PASS_EXACT_BASELINE_SOURCE__READY_FOR_EXPERIMENT';
    else
        R.status='SOURCE_DIFFERS__DO_NOT_OVERWRITE_WITH_EXPERIMENT';
        R.reason='The source differs from the frozen pre-experiment source.';
    end
catch ME
    R.status='SOURCE_FINGERPRINT_FAILED';
    R.reason=[ME.identifier ' | ' ME.message];
end
fprintf('%s\n%s\n',R.status,R.sha256);
end
