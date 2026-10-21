function OUT = AF7R36_TOWING_CONSTANT_V1_COMPAT_REPAIR_R3()
% Compatibility repair for the already installed towing-constant V1 branch.
% Restores legacy build-identity outputs required by REPLANNING_SYSTEM.
% No trajectory/certificate logic is changed here.
% No README and no persistent backup files are created.

fprintf('\n====================================================================================================\n');
fprintf(' AF7R36 TOWING CONSTANT V1 | COMPATIBILITY REPAIR R3\n');
fprintf(' trajectory synthesis changed here          = 0\n');
fprintf(' hard certificate semantics changed here    = 0\n');
fprintf(' legacy build identity restored             = 1\n');
fprintf('====================================================================================================\n');

refDir=fileparts(which('af7r34_make_core_packet'));
assert(~isempty(refDir) && isfolder(refDir), ...
    '[TOWCONST-R3] Cannot resolve current reference directory.');

hpp=fullfile(refDir,'af7r36_core.hpp');
cpp=fullfile(refDir,'af7r36_single_core_mex.cpp');
assert(isfile(hpp),'[TOWCONST-R3] Missing %s',hpp);
assert(isfile(cpp),'[TOWCONST-R3] Missing %s',cpp);

hppText=normalize_lf(fileread(hpp));
cpp0=normalize_lf(fileread(cpp));

assert(contains(hppText,'TOWING-SPECIFIC SECONDARY FAMILY') || ...
       contains(hppText,'AF7_TOWING_CONSTANT_V1'), ...
    ['[TOWCONST-R3] Towing constant branch was not found in af7r36_core.hpp. ' ...
     'No edit made.']);
assert(contains(cpp0,'TOWING_CONSTANT_TRANSLATION'), ...
    ['[TOWCONST-R3] Towing candidate-family output was not found in wrapper. ' ...
     'No edit made.']);

cppNew=cpp0;

oldRev='static const char* PLANNER_REV="AF7_TOWING_CONSTANT_V1_FIXED_T1";';
legacyRev='static const char* PLANNER_REV="AF7_REPLANNER_MIN_DEVIATION_V4_FIXED_T1";';

nOld=count_exact(cppNew,oldRev);
nLegacy=count_exact(cppNew,legacyRev);
assert((nOld==1 && nLegacy==0) || (nOld==0 && nLegacy==1), ...
    ['[TOWCONST-R3] planner revision state is ambiguous: towing=%d legacy=%d. ' ...
     'No edit made.'],nOld,nLegacy);
if nOld==1
    cppNew=replace_once(cppNew,oldRev,legacyRev,'planner native revision');
end

dynamicCompact='    set("compact_constant_translation",logicalScalar(R.cand.constant_translation));';
legacyCompact='    set("compact_constant_translation",logicalScalar(false));';

nDynamic=count_exact(cppNew,dynamicCompact);
nLegacyCompact=count_exact(cppNew,legacyCompact);
assert((nDynamic==1 && nLegacyCompact==0) || ...
       (nDynamic==0 && nLegacyCompact==1), ...
    ['[TOWCONST-R3] compact_constant_translation state is ambiguous: ' ...
     'dynamic=%d legacy=%d. No edit made.'],nDynamic,nLegacyCompact);
if nDynamic==1
    cppNew=replace_once(cppNew,dynamicCompact,legacyCompact, ...
        'legacy compact_constant_translation build flag');
end

% Verify the new selected-family reporting remains available separately.
assert(count_exact(cppNew,'"TOWING_CONSTANT_TRANSLATION"')>=1, ...
    '[TOWCONST-R3] Towing selected-family identifier disappeared.');

% Stage and compile before touching production.
stageDir=tempname;
[ok,msg]=mkdir(stageDir);
assert(ok,'[TOWCONST-R3] Cannot create staging directory: %s',msg);
stageCleanup=onCleanup(@()cleanup_stage(stageDir)); %#ok<NASGU>

stagedHpp=fullfile(stageDir,'af7r36_core.hpp');
stagedCpp=fullfile(stageDir,'af7r36_single_core_mex.cpp');
write_text_lf(stagedHpp,hppText);
write_text_lf(stagedCpp,cppNew);

fprintf('[TOWCONST-R3] Compiling staged compatibility build...\n');

oldDir=pwd;
cwdCleanup=onCleanup(@()cd(oldDir)); %#ok<NASGU>
cd(stageDir);
clear mex;

mex('-R2018a','-O', ...
    'COMPFLAGS="$COMPFLAGS /std:c++17"', ...
    ['-I' stageDir], ...
    '-output','af7r36_single_core_mex', ...
    '-outdir',stageDir, ...
    stagedCpp);

stagedMex=fullfile(stageDir,['af7r36_single_core_mex.' mexext]);
assert(isfile(stagedMex), ...
    '[TOWCONST-R3] Staged compile returned without a MEX binary.');

% Staged build succeeded. Only now replace production wrapper and binary.
cd(oldDir);
clear mex;

cppWasWritten=false;
try
    write_text_lf(cpp,cppNew);
    cppWasWritten=true;

    dstMex=fullfile(refDir,['af7r36_single_core_mex.' mexext]);
    [ok,msg]=copyfile(stagedMex,dstMex,'f');
    assert(ok,'[TOWCONST-R3] Cannot install staged MEX: %s',msg);

    clear mex;
    rehash;

    assert(exist('af7r36_single_core_mex','file')==3, ...
        '[TOWCONST-R3] Installed MEX is not visible.');

catch ME
    if cppWasWritten
        try
            write_text_lf(cpp,cpp0);
        catch
        end
    end
    clear mex;
    rehash;
    rethrow(ME);
end

cppCheck=normalize_lf(fileread(cpp));
assert(count_exact(cppCheck,legacyRev)==1, ...
    '[TOWCONST-R3] Legacy planner revision was not retained.');
assert(count_exact(cppCheck,legacyCompact)==1, ...
    '[TOWCONST-R3] Legacy compact build flag was not retained.');
assert(contains(cppCheck,'TOWING_CONSTANT_TRANSLATION'), ...
    '[TOWCONST-R3] Towing selected-family output was lost.');

fprintf('[TOWCONST-R3] PASS.\n');
fprintf('  planner_native_revision = AF7_REPLANNER_MIN_DEVIATION_V4_FIXED_T1\n');
fprintf('  compact_constant_translation build flag = false\n');
fprintf('  TOWING_CONSTANT_TRANSLATION family kept  = 1\n');
fprintf('  speed/rejoin/whole-body gates modified   = 0\n');
fprintf('  README created                           = 0\n');
fprintf('  persistent backup files created          = 0\n');

OUT=struct();
OUT.pass=true;
OUT.reference_dir=refDir;
OUT.repair='AF7R36_TOWING_CONSTANT_V1_COMPAT_REPAIR_R3';
OUT.planner_native_revision='AF7_REPLANNER_MIN_DEVIATION_V4_FIXED_T1';
OUT.compact_constant_translation_build_flag=false;
OUT.towing_family_kept=true;
OUT.hard_certificate_semantics_modified=false;
OUT.readme_created=false;
OUT.persistent_backup_created=false;
end

function n=count_exact(txt,pat)
h=strfind(txt,pat);
n=numel(h);
end

function txt=replace_once(txt,old,new,label)
h=strfind(txt,old);
assert(numel(h)==1, ...
    '[TOWCONST-R3] Anchor "%s" count=%d; expected exactly 1.', ...
    label,numel(h));
txt=[txt(1:h-1) new txt(h+numel(old):end)];
end

function s=normalize_lf(s)
s=strrep(s,sprintf('\r\n'),sprintf('\n'));
s=strrep(s,sprintf('\r'),sprintf('\n'));
end

function write_text_lf(path,txt)
f=fopen(path,'wt','n','UTF-8');
assert(f>=0,'[TOWCONST-R3] Cannot open %s for writing.',path);
c=onCleanup(@()fclose(f)); %#ok<NASGU>
fwrite(f,txt,'char');
end

function cleanup_stage(stageDir)
if isfolder(stageDir)
    try
        rmdir(stageDir,'s');
    catch
    end
end
end
