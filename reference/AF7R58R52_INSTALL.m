function OUT = AF7R58R52_INSTALL(action)
% AF7R58R52_INSTALL('apply') / AF7R58R52_INSTALL('restore')
% Applies one guarded REPLANNING_SYSTEM.m Stage2 exact packet reuse patch.
% Source bytes must match captured R50 source or R52 patched source.
% Never modifies HLC, controller, MEX, obstacle safety certificate or trigger.
if nargin~=1 || ~(ischar(action)||isstring(action))
    error('R52:Usage','Call AF7R58R52_INSTALL(''apply'') or (''restore'').');
end
root=fileparts(mfilename('fullpath'));
production=fullfile(root,'REPLANNING_SYSTEM.m');
base=fullfile(root,'AF7R58R52_BASELINE_SOURCE.dat');
patched=fullfile(root,'AF7R58R52_PATCHED_SOURCE.dat');
backup=fullfile(root,'REPLANNING_SYSTEM_PRE_R52.m.bak');
assert(isfile(base)&&isfile(patched),'R52:Assets','R52 source .dat files not found next to this installer.');
assert(isfile(production),'R52:Source','REPLANNING_SYSTEM.m not found next to this installer.');
assert(strcmpi(which('REPLANNING_SYSTEM'),production),'R52:Shadow', ...
    'MATLAB is resolving a different REPLANNING_SYSTEM. Change path first.');
baseBytes=readbytes(base);
patchBytes=readbytes(patched);
currentBytes=readbytes(production);
OUT=struct('action',char(action),'source',production,'backup',backup, ...
    'source_changed',false,'status','NOT_RUN');
switch lower(char(action))
    case 'apply'
        if isequal(currentBytes,patchBytes)
            OUT.status='ALREADY_PATCHED';
        else
            assert(isequal(currentBytes,baseBytes),'R52:SourceMismatch', ...
                ['Production source differs from the exact R50 capture. ' ...
                 'No modification was made. Never overwrite newer changes.']);
            assert(~isfile(backup),'R52:BackupExists', ...
                'A pre-R52 backup already exists. No changes made.');
            [ok,msg]=copyfile(production,backup);
            assert(ok,'R52:BackupFailed','%s',msg);
            try
                [ok,msg]=copyfile(patched,production,'f');
                assert(ok,'R52:WriteFailed','%s',msg);
                assert(isequal(readbytes(production),patchBytes), ...
                    'R52:Verify','Patched file verification failed.');
            catch ME
                copyfile(backup,production,'f');
                rethrow(ME);
            end
            OUT.source_changed=true;
            OUT.status='APPLIED_SOURCE_BACKED_UP';
            % Clear only stale root-appdata profiling, never the controller.
            if isappdata(0,'AF7R58R17_LAST_CAPTURE')
                rmappdata(0,'AF7R58R17_LAST_CAPTURE');
            end
        end
    case 'restore'
        assert(isfile(backup),'R52:NoBackup','No original backup exists.');
        assert(isequal(currentBytes,patchBytes),'R52:SourceMismatch', ...
            'Source is not the exact R52 patch. Refusing to erase other edits.');
        assert(isequal(readbytes(backup),baseBytes),'R52:BackupMismatch', ...
            'Backup is not the original captured source. Refusing restore.');
        [ok,msg]=copyfile(backup,production,'f');
        assert(ok,'R52:RestoreFailed','%s',msg);
        assert(isequal(readbytes(production),baseBytes),'R52:Verify', ...
            'Restored source verification failed.');
        OUT.source_changed=true;
        OUT.status='RESTORED_BASELINE';
    otherwise
        error('R52:Usage','Only apply or restore is allowed.');
end
OUT.mat_file=fullfile(root,'AF7R58R52_INSTALL_OUT.mat');
save(OUT.mat_file,'OUT','-v7');
fprintf('[R52] %s\n',OUT.status);
fprintf('[R52] After apply/restore, before starting the next simulation: clear classes; rehash;\n');
end
function b = readbytes(path)
    f=fopen(path,'rb'); assert(f>0,'R52:Read','Cannot read %s.',path);
    c=onCleanup(@()fclose(f));
    b=fread(f,inf,'*uint8');
end
