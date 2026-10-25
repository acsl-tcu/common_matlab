function I=AF7R58R114_INSTALL()
% One deliberate production change: core geometry and its compiled MEX.
% Unchanged: R111 class, actual HLC, simulator, Stage2/3, existing safety gates.
S=AF7R58R114_SOURCE_CHECK();
I=struct('status','NOT_INSTALLED','installed',false, ...
    'core_backup','','mex_source_backup','','mex_binary_backup','', ...
    'compiled_core_test',struct());
if S.already_installed
    I.status='R114_SOURCES_ALREADY_PATCHED__CHECK_BINARY_WITH_TEST';return;
end
assert(S.installable,'R114:SourceDrift', ...
    'Source mismatch: do not overwrite user files or rebuild from unknown source.');
root=fileparts(mfilename('fullpath'));
hpp=fullfile(root,'af7r36_core.hpp');
cpp=fullfile(root,'af7r36_single_core_mex.cpp');
oldDir=pwd; cleanDir=onCleanup(@()cd(oldDir)); %#ok<NASGU>
cd(root);
originalBinary=fullfile(root,['af7r36_single_core_mex.' mexext]);
step='R114_STAGE_COMPILATION';
backupTag=datestr(now,'yyyymmdd_HHMMSSFFF');
bh=fullfile(root,['af7r36_core_pre_R114_' backupTag '.hpp']);
bc=fullfile(root,['af7r36_single_core_mex_pre_R114_' backupTag '.cpp']);
bm=fullfile(root,['af7r36_single_core_mex_pre_R114_' backupTag '.' mexext]);
assert(copyfile(hpp,bh),'R114:HeaderBackupFailed','Core header backup failed.');
assert(copyfile(cpp,bc),'R114:CppBackupFailed','C++ source backup failed.');
previousMex=isfile(originalBinary);
if previousMex
    assert(copyfile(originalBinary,bm),'R114:MexBackupFailed','MEX backup failed.');
end
I.core_backup=bh;I.mex_source_backup=bc;
if previousMex,I.mex_binary_backup=bm;end
stageName='af7r36_single_core_mex_R114_stage';
stageBinary=fullfile(root,[stageName '.' mexext]);
try
    [a,msg]=copyfile(fullfile(root,'AF7R58R114_PATCHED_CORE.hpp'),hpp,'f');
    if ~a,error('R114:CoreCopy','%s',msg);end
    [a,msg]=copyfile(fullfile(root,'AF7R58R114_PATCHED_MEX.cpp'),cpp,'f');
    if ~a,error('R114:CppCopy','%s',msg);end
    S=AF7R58R114_SOURCE_CHECK();
    assert(S.already_installed,'R114:SourceValidationFailed', ...
        'Patched sources failed exact-source validation.');
    if isfile(stageBinary),delete(stageBinary);end
    % R114 C++17 required by both baseline core and exact support geometry.
    % Explicit MSVC compiler flag; no alternate compiler or algorithm fallback.
    mex('-R2018a','COMPFLAGS=$COMPFLAGS /std:c++17',cpp,'-output',stageName);
    rehash;
    coreTest=feval(stageName,26);
    assert(isstruct(coreTest) && ...
        strcmp(char(coreTest.status),'PASS_R114_COMPILED_REAL_CORE_GEOMETRY'), ...
        'R114:CompiledCoreTestFailed', ...
        'Compiled candidate MEX failed the REAL core support-plane tests.');
    assert(logical(feval(stageName,24)), ...
        'R114:NativePrewarmFailed','Native prewarm op 24 failed.');
    I.compiled_core_test=coreTest;
    eval(['clear ' stageName]);
    eval('clear af7r36_single_core_mex');
    [a,msg]=copyfile(stageBinary,originalBinary,'f');
    if ~a,error('R114:BinaryInstall','%s',msg);end
    if isfile(stageBinary),delete(stageBinary);end
    rehash;
    I.status='R114_PRODUCTION_MEX_SEPARATE_SUPPORT_INSTALLED__RESTART_MATLAB';
    I.installed=true;
catch ME
    try, eval(['clear ' stageName]); catch, end
    try, eval('clear af7r36_single_core_mex'); catch, end
    if isfile(stageBinary),delete(stageBinary);end
    [ok1,~]=copyfile(bh,hpp,'f');
    [ok2,~]=copyfile(bc,cpp,'f');
    if previousMex
        [ok3,~]=copyfile(bm,originalBinary,'f');
    else
        ok3=true;
        if isfile(originalBinary),delete(originalBinary);end
    end
    if ~(ok1&&ok2&&ok3)
        error('R114:RollbackFailed', ...
            'Rollback failed; restore header, cpp, mex using the three backup paths in reference.');
    end
    error('R114:RolledBack', ...
        'Compiled R114 was not installed; all production files restored. Cause: %s', ME.message);
end
end
