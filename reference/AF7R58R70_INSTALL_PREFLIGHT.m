function S=AF7R58R70_INSTALL_PREFLIGHT()
% One-time surgical installation into the CURRENT run_flight_headless.m.
% Never replaces the user's current path, nominal reference or parameters.
% The inserted preflight is run once after Arming and before any Flight tick.
    src=which('run_flight_headless');
    assert(~isempty(src)&&isfile(src), '[R70] Current run_flight_headless.m not found.');
    fr=fopen(src,'r');
    assert(fr>0,'[R70] Could not open current runner.');
    raw=fread(fr,Inf,'uint8=>uint8').';
    fclose(fr);
    text=native2unicode(raw,'UTF-8');
    assert(isequal(uint8(unicode2native(text,'UTF-8')),raw), ...
        '[R70] Runner is not UTF-8; no source modification performed.');
    call='R70_PREFLIGHT=AF7R58R70_PREFLIGHT_AUTHORITIES(agent,replanner_instance,time,replan_opts,nominal_ref);';
    if contains(text,call)
        S=struct('status','ALREADY_INSTALLED','source',src,'backup','');return;
    end
    anchor='%% 8. Headless simulation loop';
    assert(numel(strfind(text,anchor))==1, ...
        '[R70] Current runner has different structure. No automatic edit performed.');
    added=sprintf(['%% R70: freshly validate EXTERNAL runtime authorities for THIS Flight.\n' ...
        '%% Never re-use previous Flight snapshots; mass continues to update in replanner.do.\n' ...
        '%s\n\n'],call);
    modified=strrep(text,anchor,[added,anchor]);
    % Save before editing: do not replace the source if the backup fails.
    backup=[src,'.R70_backup_',datestr(now,'yyyymmdd_HHMMSS')];
    [ok,msg]=copyfile(src,backup,'f');
    assert(ok,'[R70] Could not back up current runner: %s',msg);
    fid=fopen(src,'w','n','UTF-8');
    assert(fid>0,'[R70] Could not open runner for modification.');
    cleaner=onCleanup(@()fclose(fid));
    payload=unicode2native(modified,'UTF-8');
    n=fwrite(fid,payload,'uint8');
    assert(n==numel(payload), '[R70] Incomplete runner write, restore from backup.');
    clear cleaner;
    rehash;
    S=struct('status','INSTALLED','source',src,'backup',backup);
end
