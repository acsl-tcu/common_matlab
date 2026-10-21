function OUT = AF7R36_R7_FAILCLOSED_FALLTHROUGH_R15_INSTALL()
% AF7R36_R7_FAILCLOSED_FALLTHROUGH_R15_INSTALL
%
% R14C captured the first live Stage1 exception exactly:
%
%     AF7R34:Exception | R7B midpoint Schur solve failed
%
% The R7 family is a secondary candidate family. A midpoint Schur failure
% means only that this R7 candidate model was not constructed. It must NOT
% abort the whole planner before the already-existing fallback family.
%
% R15 changes ONLY exception transport for this one exact R7B failure:
%
%   R7B midpoint Schur solve failed
%       -> R7 family unavailable for this fixed-t1 call
%       -> do not certify/use R7
%       -> continue to the unchanged fallback family
%
% It does NOT:
%   - regularize/perturb the Schur matrix,
%   - loosen finite-sensing speed,
%   - loosen rejoin or continuous whole-body geometry,
%   - weaken C7/C6 requirements,
%   - change primary or fallback mathematics,
%   - promote an uncertified R7 candidate.
%
% Any other runtime_error from c7r7_model is rethrown unchanged.
%
% No README. No persistent backup file.

fprintf('\n====================================================================================================\n');
fprintf(' AF7R36 R7 FAIL-CLOSED FALLTHROUGH R15 INSTALL\n');
fprintf(' exact captured failure                     = R7B midpoint Schur solve failed\n');
fprintf(' Schur regularization / tolerance change     = 0\n');
fprintf(' R7 uncertified candidate acceptance         = 0\n');
fprintf(' primary/fallback mathematics changed        = 0\n');
fprintf(' speed/rejoin/whole-body gates changed       = 0\n');
fprintf(' R7 exact-failure -> fallback continuation   = 1\n');
fprintf('====================================================================================================\n');

refDir=fileparts(which('af7r34_make_core_packet'));
assert(~isempty(refDir)&&isfolder(refDir), ...
    '[R15] Cannot resolve current reference directory.');

hpp=fullfile(refDir,'af7r36_core.hpp');
cpp=fullfile(refDir,'af7r36_single_core_mex.cpp');
assert(isfile(hpp),'[R15] Missing %s',hpp);
assert(isfile(cpp),'[R15] Missing %s',cpp);

hpp0=normlf(fileread(hpp));
cpp0=normlf(fileread(cpp));
hppNew=hpp0;

assert(contains(hpp0,'R7B midpoint Schur solve failed'), ...
    '[R15] Exact captured R7B Schur failure is not present in current core.');
assert(contains(hpp0,'auto MR=c7r7_model(E.comp,t0,t1,L.L,L.g);'), ...
    '[R15] Current R7B model call not found.');
assert(contains(hpp0, ...
    '// FALLBACK: exact previously flown affine/minimum-snap family.'), ...
    '[R15] Existing fallback branch marker not found.');
assert(~contains(hpp0,'AF7_R7_FAILCLOSED_FALLTHROUGH_R15'), ...
    '[R15] R15 already installed.');

try
    %% --------------------------------------------------------------------
    % 1) Replace only the R7 model construction line with exact-message
    %    fail-closed transport. Other R7 errors still throw.
    % ---------------------------------------------------------------------
    oldOpen='        auto MR=c7r7_model(E.comp,t0,t1,L.L,L.g);';
    newOpen=[ ...
        '        // AF7_R7_FAILCLOSED_FALLTHROUGH_R15' sprintf('\n') ...
        '        // Optional R7-family numerical construction failure must not' sprintf('\n') ...
        '        // abort primary/fallback planner authority.' sprintf('\n') ...
        '        C7R7Model MR;' sprintf('\n') ...
        '        bool r7_model_ok=true;' sprintf('\n') ...
        '        try{' sprintf('\n') ...
        '            MR=c7r7_model(E.comp,t0,t1,L.L,L.g);' sprintf('\n') ...
        '        }catch(const std::runtime_error&e){' sprintf('\n') ...
        '            if(std::string(e.what())=="R7B midpoint Schur solve failed"){' sprintf('\n') ...
        '                r7_model_ok=false;' sprintf('\n') ...
        '            }else{' sprintf('\n') ...
        '                throw;' sprintf('\n') ...
        '            }' sprintf('\n') ...
        '        }' sprintf('\n') ...
        '        if(r7_model_ok){'];
    hppNew=replace_once(hppNew,oldOpen,newOpen,'R7 model construction');

    %% --------------------------------------------------------------------
    % 2) Close the new r7_model_ok scope immediately before the existing
    %    R7 branch scope closes and fallback starts.
    % ---------------------------------------------------------------------
    oldClose=[ ...
        '        R.tim.candidate_build+=std::chrono::duration<double,std::milli>(clk::now()-tc).count();' sprintf('\n') ...
        '    }' sprintf('\n') ...
        '' sprintf('\n') ...
        '    // ------------------------------------------------------------------' sprintf('\n') ...
        '    // FALLBACK: exact previously flown affine/minimum-snap family.'];

    newClose=[ ...
        '        R.tim.candidate_build+=std::chrono::duration<double,std::milli>(clk::now()-tc).count();' sprintf('\n') ...
        '        } // r7_model_ok' sprintf('\n') ...
        '    }' sprintf('\n') ...
        '' sprintf('\n') ...
        '    // ------------------------------------------------------------------' sprintf('\n') ...
        '    // FALLBACK: exact previously flown affine/minimum-snap family.'];
    hppNew=replace_once(hppNew,oldClose,newClose,'R7 branch close before fallback');

    %% --------------------------------------------------------------------
    % 3) Source audits.
    % ---------------------------------------------------------------------
    assert(count_exact(hppNew,'AF7_R7_FAILCLOSED_FALLTHROUGH_R15')==1, ...
        '[R15] R15 marker count invalid.');
    assert(count_exact(hppNew, ...
        'if(std::string(e.what())=="R7B midpoint Schur solve failed")')==1, ...
        '[R15] Exact R7B exception match count invalid.');
    assert(count_exact(hppNew,'r7_model_ok=false;')==1, ...
        '[R15] R7 fail-closed assignment count invalid.');
    assert(count_exact(hppNew,'} // r7_model_ok')==1, ...
        '[R15] R7 fail-closed scope close count invalid.');

    % The underlying Schur failure remains a hard failure inside R7 model
    % construction. We are NOT making dense_solve succeed.
    assert(count_exact(hppNew, ...
        'if(!dense_solve(C,rhs,nm,x))throw std::runtime_error("R7B midpoint Schur solve failed");')==1, ...
        '[R15] Underlying R7B Schur semantics changed unexpectedly.');

    % Existing hard acceptance layers must still be present.
    assert(contains(hppNew,'auto tr=tighten_stageA'), ...
        '[R15] Stage-A finite-sensing certificate disappeared.');
    assert(contains(hppNew,'stageB_speed'), ...
        '[R15] Stage-B finite-sensing certificate disappeared.');
    assert(contains(hppNew,'rejoin_safe'), ...
        '[R15] Rejoin hard gate disappeared.');
    assert(contains(hppNew,'wholebody'), ...
        '[R15] Continuous whole-body certificate disappeared.');
    assert(contains(hppNew, ...
        '// FALLBACK: exact previously flown affine/minimum-snap family.'), ...
        '[R15] Existing fallback branch disappeared.');

    %% --------------------------------------------------------------------
    % 4) Stage compile with current wrapper.
    % ---------------------------------------------------------------------
    stageDir=tempname;
    [ok,msg]=mkdir(stageDir);
    assert(ok,'[R15] Cannot create staging directory: %s',msg);
    stageCleanup=onCleanup(@()cleanup_stage(stageDir)); %#ok<NASGU>

    stagedHpp=fullfile(stageDir,'af7r36_core.hpp');
    stagedCpp=fullfile(stageDir,'af7r36_single_core_mex.cpp');
    write_text(stagedHpp,hppNew);
    [ok,msg]=copyfile(cpp,stagedCpp,'f');
    assert(ok,'[R15] Cannot stage wrapper: %s',msg);

    fprintf('[R15] Compiling staged MEX...\n');
    oldDir=pwd;
    cwdCleanup=onCleanup(@()cd(oldDir)); %#ok<NASGU>
    cd(stageDir);
    clear mex
    mex('-R2018a','-O', ...
        'COMPFLAGS="$COMPFLAGS /std:c++17"', ...
        ['-I' stageDir], ...
        '-output','af7r36_single_core_mex', ...
        '-outdir',stageDir, ...
        stagedCpp);

    stagedMex=fullfile(stageDir,['af7r36_single_core_mex.' mexext]);
    assert(isfile(stagedMex),'[R15] Staged MEX missing.');

    %% --------------------------------------------------------------------
    % 5) Install. No clear classes inside installer.
    % ---------------------------------------------------------------------
    dstMex=fullfile(refDir,['af7r36_single_core_mex.' mexext]);
    oldMexTmp=fullfile(stageDir,['af7r36_single_core_mex_OLD.' mexext]);
    hadOldMex=isfile(dstMex);
    if hadOldMex
        [ok,msg]=copyfile(dstMex,oldMexTmp,'f');
        assert(ok,'[R15] Cannot prepare temporary MEX rollback: %s',msg);
    end

    cd(oldDir);
    clear mex
    wroteHpp=false;installedMex=false;
    try
        write_text(hpp,hppNew);wroteHpp=true;
        [ok,msg]=copyfile(stagedMex,dstMex,'f');
        assert(ok,'[R15] Cannot install staged MEX: %s',msg);
        installedMex=true;
        clear mex
        rehash

        assert(exist('af7r36_single_core_mex','file')==3, ...
            '[R15] Installed MEX not visible.');
        assert(contains(normlf(fileread(hpp)), ...
            'AF7_R7_FAILCLOSED_FALLTHROUGH_R15'), ...
            '[R15] Installed core patch not visible.');
    catch ME2
        if wroteHpp,try,write_text(hpp,hpp0);catch,end,end
        if installedMex && hadOldMex
            try,copyfile(oldMexTmp,dstMex,'f');catch,end
        end
        clear mex
        rehash
        rethrow(ME2);
    end

    fprintf('[R15] PASS.\n');
    fprintf('  R7 midpoint Schur failure accepted         = 0\n');
    fprintf('  R7 midpoint Schur failure aborts planner   = 0\n');
    fprintf('  fallback remains independently certified   = 1\n');
    fprintf('  safety/smoothness gates weakened           = 0\n');
    fprintf('  IMPORTANT: clear old objects/classes before next simulation.\n');

    OUT=struct( ...
        'pass',true, ...
        'revision','AF7_R7_FAILCLOSED_FALLTHROUGH_R15', ...
        'r7_schur_regularized',false, ...
        'r7_failure_accepted',false, ...
        'r7_failure_falls_through',true, ...
        'hard_safety_modified',false, ...
        'C7_modified',false);

catch ME
    clear mex
    rehash
    fprintf(2,'[R15] FAILED. No successful production install completed.\n');
    fprintf(2,'[R15] ORIGINAL ERROR: %s | %s\n',ME.identifier,ME.message);
    rethrow(ME);
end
end

function txt=replace_once(txt,old,new,label)
h=strfind(txt,old);
assert(numel(h)==1,'[R15] Anchor "%s" count=%d; expected 1.',label,numel(h));
txt=[txt(1:h-1) new txt(h+numel(old):end)];
end

function n=count_exact(txt,pat)
n=numel(strfind(txt,pat));
end

function s=normlf(s)
s=strrep(s,sprintf('\r\n'),sprintf('\n'));
s=strrep(s,sprintf('\r'),sprintf('\n'));
end

function write_text(path,txt)
f=fopen(path,'wt','n','UTF-8');
assert(f>=0,'[R15] Cannot open %s.',path);
c=onCleanup(@()fclose(f)); %#ok<NASGU>
fwrite(f,txt,'char');
end

function cleanup_stage(d)
if isfolder(d)
    try,rmdir(d,'s');catch,end
end
end
