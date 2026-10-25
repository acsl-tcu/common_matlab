function I=AF7R58R115_INSTALL()
% Install tested source-bound EKF-NOW component geometry into production.
% No changes to trigger_core, Stage1/2/3, HLC, plant or 4-s predictor.
S=AF7R58R115_SOURCE_CHECK();
I=struct('status','NOT_INSTALLED','installed',false, ...
    'main_backup','','mex_source_backup','','mex_binary_backup','', ...
    'staged_test',struct());
if S.already_installed
    I.staged_test=AF7R58R115_MEX_TEST();
    I.status='R115_ALREADY_INSTALLED__NATIVE_TEST_PASSED';return;
end
assert(S.installable,'R115:SourceDrift', ...
    'The exact R111 class and R114 C++ are required.');
root=fileparts(mfilename('fullpath'));
main=fullfile(root,'REPLANNING_SYSTEM.m');
cpp=fullfile(root,'af7r36_single_core_mex.cpp');
hpp=fullfile(root,'af7r36_core.hpp');
originalMex=fullfile(root,['af7r36_single_core_mex.' mexext]);
assert(isfile(originalMex),'R115:CompiledR114Required', ...
    'Install and verify the original R114 production MEX first.');
% Compile and TEST in an isolated stage directory BEFORE touching production.
work=fullfile(tempdir,['AF7R58R115_STAGED_' datestr(now,'yyyymmdd_HHMMSSFFF')]);
assert(mkdir(work),'R115:StageDirectory','Cannot make a staging directory.');
cleanupDir=onCleanup(@()r115_cleanup_stage(work)); %#ok<NASGU>
copyfile(fullfile(root,'af7r36_core_R115_PINNED.hpp'),fullfile(work,'af7r36_core.hpp'));
copyfile(fullfile(root,'af7r36_single_core_mex_R115_PATCHED.cpp'), ...
    fullfile(work,'af7r36_single_core_mex.cpp'));
stageName='af7r36_single_core_mex_R115_stage';
oldDir=pwd; cleanupCwd=onCleanup(@()cd(oldDir)); %#ok<NASGU>
cd(work);
mex('-R2018a','COMPFLAGS=$COMPFLAGS /std:c++17', ...
    fullfile(work,'af7r36_single_core_mex.cpp'), ...
    '-outdir',work,'-output',stageName);
addpath(work,'-begin');
cleanupPath=onCleanup(@()rmpath(work)); %#ok<NASGU>
rehash;
I.staged_test=AF7R58R115_MEX_TEST(stageName);
eval(['clear ' stageName]);
cd(root);
% Revalidate hashes AFTER staging, to prevent concurrent source drift.
S=AF7R58R115_SOURCE_CHECK();
assert(S.installable,'R115:SourceChangedDuringBuild', ...
    'Source changed during isolated compile. Nothing installed.');
tag=datestr(now,'yyyymmdd_HHMMSSFFF');
bMain=fullfile(root,['REPLANNING_SYSTEM_pre_R115_' tag '.m']);
bCpp=fullfile(root,['af7r36_single_core_mex_pre_R115_' tag '.cpp']);
bMex=fullfile(root,['af7r36_single_core_mex_pre_R115_' tag '.' mexext]);
assert(copyfile(main,bMain) && copyfile(cpp,bCpp) && ...
    copyfile(originalMex,bMex), 'R115:BackupFailed', ...
    'Backup failed before any production source was changed.');
I.main_backup=bMain;I.mex_source_backup=bCpp;I.mex_binary_backup=bMex;
try
    assert(copyfile(fullfile(root,'REPLANNING_SYSTEM_R115_PATCHED_SOURCE.txt'),main,'f'), ...
        'R115:MainCopy','Failed to install patched MATLAB class.');
    assert(copyfile(fullfile(root,'af7r36_single_core_mex_R115_PATCHED.cpp'),cpp,'f'), ...
        'R115:CppCopy','Failed to install patched C++.');
    S=AF7R58R115_SOURCE_CHECK();
    assert(S.already_installed,'R115:SourceCheckAfterInstall', ...
        'Installed sources did not match source-bound SHA.');
    eval('clear af7r36_single_core_mex');
    stageBinary=fullfile(work,[stageName '.' mexext]);
    assert(isfile(stageBinary)&&copyfile(stageBinary,originalMex,'f'), ...
        'R115:BinaryCopy','Could not install stage-tested native MEX.');
    rehash;
    I.status='R115_DIRECT_EKF_NOW_THREE_BODY_GEOMETRY_INSTALLED__RESTART_MATLAB';
    I.installed=true;
catch ME
    try,eval('clear af7r36_single_core_mex');catch,end
    ok1=copyfile(bMain,main,'f');
    ok2=copyfile(bCpp,cpp,'f');
    ok3=copyfile(bMex,originalMex,'f');
    if ~(ok1&&ok2&&ok3)
        error('R115:RollbackFailed', ...
            'Rollback failed; manually restore all three backup paths.');
    end
    error('R115:RolledBack', ...
        'R115 was not installed; originals restored. Cause: %s',ME.message);
end
end

function r115_cleanup_stage(work)
if isfolder(work)
    try,rmdir(work,'s');catch,end
end
end
