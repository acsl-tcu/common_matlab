function R=TRIGGER_ORIGINAL_PLANT_SOURCE_BINDING()
% Byte-level source identity for the ORIGINAL 24-state plant RHS.
% This is a PRE-FLIGHT/OFFLINE source check; never invoke in a Flight tick.
% The expected bytes are from the R193 source used for the original Flight.
% This proves file identity only, NOT physical parameter provenance, EKF
% true-state containment, interval arithmetic, or source-to-binary parity.
R=struct('status','UNKNOWN_ORIGINAL_PLANT_SOURCE', ...
    'source_path','','source_sha256','','source_verified',false, ...
    'safe_to_commit',false,'reason','');
try
    p=which('with_load_model_euler_for_HL');
    assert(~isempty(p)&&isfile(p), ...
        'TRIGGER_ORIGINAL_PLANT_SOURCE_BINDING:Missing', ...
        'Original with_load_model_euler_for_HL.m not found.');
    f=fopen(p,'rb');
    assert(f>=0,'TRIGGER_ORIGINAL_PLANT_SOURCE_BINDING:Read', ...
        'Cannot open original plant source.');
    guard=onCleanup(@()fclose(f)); %#ok<NASGU>
    bytes=fread(f,inf,'*uint8');
    md=java.security.MessageDigest.getInstance('SHA-256');
    md.update(typecast(bytes,'int8'));
    raw=typecast(int8(md.digest()),'uint8');
    actual=lower(reshape(dec2hex(raw,2).',1,[]));
    expected='183837c484f6ec8953998aacb20b93120ce0d6fde647c0e54bcc27f5ae02164a';
    R.source_path=p;
    R.source_sha256=actual;
    R.source_verified=strcmp(actual,expected);
    if R.source_verified
        R.status='ORIGINAL_24_STATE_PLANT_FILE_PINNED__NOT_NUMERIC_PROOF';
        R.reason=['The audited original plant RHS source file bytes match. ' ...
            'Runtime dispatch, interval extension arithmetic and real ' ...
            'physical uncertainties are still separate obligations.'];
    else
        R.status='UNKNOWN_ORIGINAL_24_STATE_PLANT_SOURCE_MISMATCH';
        R.reason='Source bytes differ from the inspected original plant.';
    end
catch ME
    R.status='UNKNOWN_ORIGINAL_PLANT_SOURCE_EXCEPTION';
    R.reason=[ME.identifier ' | ' ME.message];
end
end
