function R=CHECK_TRIGGER_CALLSITE_SOURCE_STATE()
% Disposable source-fingerprint guard for the isolated callsite A/B/A.
% This code changes nothing. Mismatch => STOP instead of overwriting.
R=struct('status','UNKNOWN_SOURCE_STATE','reason','', ...
    'replanner_sha256','','prepare_sha256','');
try
    a=which('REPLANNING_SYSTEM');
    b=which('TRIGGER_TIMEONLY_SOURCE_PREPARE');
    assert(~isempty(a)&&~isempty(b), ...
        'CHECK_TRIGGER_CALLSITE_SOURCE:Missing','Both production functions must resolve.');
    R.replanner_sha256=local_sha(a);
    R.prepare_sha256=local_sha(b);
    if strcmp(R.replanner_sha256,'5d8f644f39479061f94cee697bcc88729721685dc0d3587e4838c4ed511d9aab') && ...
            strcmp(R.prepare_sha256,'46ebdac069a1fc72d4f3f0e480ba89065ad8dee883a37b5881f9466f81d3c8e0')
        R.status='EXACT_CALLSITE_BASELINE_A';
    elseif strcmp(R.replanner_sha256,'76a2fb76a9d210be40f8d5e78d4e75654f1d090e035f63f059c568089c01ec61') && ...
            strcmp(R.prepare_sha256,'1c55731afbb9da35965a5b3f11784da1ca9ee7112244006d75c435359d4556d4')
        R.status='EXACT_CALLSITE_CANDIDATE_B__NOT_ADOPTED';
    else
        R.status='SOURCE_MISMATCH__STOP_AND_INSPECT';
        R.reason='Active source does not match the provided A or B. Do not overwrite blindly.';
    end
catch ME
    R.reason=[ME.identifier ' | ' ME.message];
end
end
function s=local_sha(p)
f=fopen(p,'rb');
assert(f>=0,'CHECK_TRIGGER_CALLSITE_SOURCE:Read','Cannot open %s.',p);
c=onCleanup(@()fclose(f)); %#ok<NASGU>
x=fread(f,inf,'*uint8');
md=java.security.MessageDigest.getInstance('SHA-256');
md.update(typecast(x,'int8'));
s=lower(reshape(dec2hex(typecast(md.digest(),'uint8'),2).',1,[]));
end
