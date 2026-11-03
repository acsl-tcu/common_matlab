function R=CHECK_TRIGGER_LOG_TEMPLATE_SOURCE()
% Disposable fingerprint for one-file, reversible A/B.
R=struct('status','UNKNOWN_SOURCE','sha256','','reason','');
try
    path=which('REPLANNING_SYSTEM');
    assert(~isempty(path),'CHECK_TRIGGER_LOG_TEMPLATE_SOURCE:Path', ...
        'REPLANNING_SYSTEM.m is not on MATLAB path.');
    f=fopen(path,'rb');
    assert(f>=0,'CHECK_TRIGGER_LOG_TEMPLATE_SOURCE:Open','Cannot open source.');
    close_file=onCleanup(@()fclose(f)); %#ok<NASGU>
    bytes=fread(f,inf,'*uint8');
    md=java.security.MessageDigest.getInstance('SHA-256');
    md.update(typecast(bytes,'int8'));
    raw=typecast(int8(md.digest()),'uint8');
    R.sha256=lower(reshape(dec2hex(raw,2).',1,[]));
    if strcmp(R.sha256,'5d8f644f39479061f94cee697bcc88729721685dc0d3587e4838c4ed511d9aab')
       R.status='EXACT_RESTORED_BASELINE__NO_LOG_TEMPLATE';
    elseif strcmp(R.sha256,'5dfc87dba95209845f6ba0f52b69b691aab5fbe8672c242df1491219b7582198')
       R.status='EXACT_LOG_TEMPLATE_CANDIDATE__NOT_ADOPTED';
    else
       R.status='UNKNOWN_SOURCE__DO_NOT_OVERWRITE';
       R.reason='Unexpected source hash; stop and inspect current version.';
    end
catch ME
    R.status='SOURCE_FINGERPRINT_FAILED';
    R.reason=[ME.identifier ' | ' ME.message];
end
end
