function OUT = AF7R36_R7_FAILCLOSED_FALLTHROUGH_R15B_INSTALL()
% AF7R36_R7_FAILCLOSED_FALLTHROUGH_R15B_INSTALL
%
% Exact live failure captured by R14C:
%   AF7R34:Exception | R7B midpoint Schur solve failed
%
% R15B changes ONLY transport of this one optional-family construction error:
%
%   R7B midpoint Schur solve failed
%       -> R7 family unavailable for this fixed-t1 call
%       -> jump to the pre-existing minimum-snap fallback
%
% It does NOT:
%   - regularize or perturb the Schur system,
%   - change dense_solve tolerances,
%   - accept an uncertified R7 candidate,
%   - change primary/fallback mathematics,
%   - change speed / rejoin / continuous whole-body gates,
%   - change C7 construction or commit policy.
%
% Any other std::runtime_error from c7r7_model is rethrown unchanged.
%
% R15B intentionally uses a label immediately before the existing fallback,
% avoiding brittle matching of the R7 branch closing brace.
%
% No README. No persistent backup.

fprintf('\n====================================================================================================\n');
fprintf(' AF7R36 R7 FAIL-CLOSED FALLTHROUGH R15B INSTALL\n');
fprintf(' exact captured failure                     = R7B midpoint Schur solve failed\n');
fprintf(' Schur regularization / tolerance change     = 0\n');
fprintf(' R7 uncertified candidate acceptance         = 0\n');
fprintf(' primary/fallback mathematics changed        = 0\n');
fprintf(' speed/rejoin/whole-body gates changed       = 0\n');
fprintf(' exact R7 failure -> existing fallback       = 1\n');
fprintf('====================================================================================================\n');

refDir=fileparts(which('af7r34_make_core_packet'));
assert(~isempty(refDir)&&isfolder(refDir), ...
    '[R15B] Cannot resolve current reference directory.');

hpp=fullfile(refDir,'af7r36_core.hpp');
cpp=fullfile(refDir,'af7r36_single_core_mex.cpp');
assert(isfile(hpp),'[R15B] Missing %s',hpp);
assert(isfile(cpp),'[R15B] Missing %s',cpp);

hpp0=normlf(fileread(hpp));
cpp0=normlf(fileread(cpp));
hppNew=hpp0;

assert(contains(hpp0,'R7B midpoint Schur solve failed'), ...
    '[R15B] Exact R7B Schur failure text not found.');
assert(count_exact(hpp0,'        auto MR=c7r7_model(E.comp,t0,t1,L.L,L.g);')==1, ...
    '[R15B] Current R7 model call count is not one.');
assert(count_exact(hpp0, ...
    ['    // ------------------------------------------------------------------' sprintf('\n') ...
     '    // FALLBACK: exact previously flown affine/minimum-snap family.'])==1, ...
    '[R15B] Existing fallback marker count is not one.');
assert(~contains(hpp0,'AF7_R7_FAILCLOSED_FALLTHROUGH_R15B'), ...
    '[R15B] R15B already installed.');

try
    %% --------------------------------------------------------------------
    % 1) Replace only R7 model construction.
    % ---------------------------------------------------------------------
    old='        auto MR=c7r7_model(E.comp,t0,t1,L.L,L.g);';
    new=[ ...
        '        // AF7_R7_FAILCLOSED_FALLTHROUGH_R15B' sprintf('\n') ...
        '        // R7 is optional. Failure to construct its condensed midpoint' sprintf('\n') ...
        '        // model means R7 is unavailable; it does not invalidate the' sprintf('\n') ...
        '        // independently certified fallback family.' sprintf('\n') ...
        '        C7R7Model MR;' sprintf('\n') ...
        '        try{' sprintf('\n') ...
        '            MR=c7r7_model(E.comp,t0,t1,L.L,L.g);' sprintf('\n') ...
        '        }catch(const std::runtime_error&e){' sprintf('\n') ...
        '            if(std::string(e.what())=="R7B midpoint Schur solve failed"){' sprintf('\n') ...
        '                goto AF7_R15B_FALLBACK;' sprintf('\n') ...
        '            }' sprintf('\n') ...
        '            throw;' sprintf('\n') ...
        '        }'];
    hppNew=replace_once(hppNew,old,new,'R7 model construction');

    %% --------------------------------------------------------------------
    % 2) Label the already-existing fallback. No R7-close-brace matching.
    % ---------------------------------------------------------------------
    fallback=[ ...
        '    // ------------------------------------------------------------------' sprintf('\n') ...
        '    // FALLBACK: exact previously flown affine/minimum-snap family.'];
    labeled=[ ...
        'AF7_R15B_FALLBACK:' sprintf('\n') ...
        '    // ------------------------------------------------------------------' sprintf('\n') ...
        '    // FALLBACK: exact previously flown affine/minimum-snap family.'];
    hppNew=replace_once(hppNew,fallback,labeled,'fallback label');

    %% --------------------------------------------------------------------
    % 3) Source audits.
    % ---------------------------------------------------------------------
    assert(count_exact(hppNew,'AF7_R7_FAILCLOSED_FALLTHROUGH_R15B')==1, ...
        '[R15B] R15B marker count invalid.');
    assert(count_exact(hppNew,'goto AF7_R15B_FALLBACK;')==1, ...
        '[R15B] Exact fail-closed jump count invalid.');
    assert(count_exact(hppNew,'AF7_R15B_FALLBACK:')==1, ...
        '[R15B] Fallback label count invalid.');
    assert(count_exact(hppNew, ...
        'if(std::string(e.what())=="R7B midpoint Schur solve failed")')==1, ...
        '[R15B] Exact exception match count invalid.');

    % Underlying R7 numerical rule must remain unchanged.
    assert(count_exact(hppNew, ...
        'if(!dense_solve(C,rhs,nm,x))throw std::runtime_error("R7B midpoint Schur solve failed");')==1, ...
        '[R15B] Underlying R7 Schur solve semantics changed unexpectedly.');

    % Hard certificates remain present.
    assert(contains(hppNew,'tighten_stageA'), ...
        '[R15B] Stage-A speed certificate disappeared.');
    assert(contains(hppNew,'stageB_speed'), ...
        '[R15B] Stage-B speed certificate disappeared.');
    assert(contains(hppNew,'rejoin_safe'), ...
        '[R15B] Rejoin certificate disappeared.');
    assert(contains(hppNew,'wholebody'), ...
        '[R15B] Whole-body certificate disappeared.');
    assert(contains(hppNew, ...
        'for(auto&b:B)affine.push_back(build_candidate(E.comp,t0,t1,b.z,b.c,b.sg,b.s,2));'), ...
        '[R15B] Existing minimum-snap fallback candidate construction disappeared.');

    %% --------------------------------------------------------------------
    % 4) Stage compile current wrapper + patched core.
    % ---------------------------------------------------------------------
    stageDir=tempname;
    [ok,msg]=mkdir(stageDir);
    assert(ok,'[R15B] Cannot create staging directory: %s',msg);
    stageCleanup=onCleanup(@()cleanup_stage(stageDir)); %#ok<NASGU>

    stagedHpp=fullfile(stageDir,'af7r36_core.hpp');
    stagedCpp=fullfile(stageDir,'af7r36_single_core_mex.cpp');
    write_text(stagedHpp,hppNew);
    [ok,msg]=copyfile(cpp,stagedCpp,'f');
    assert(ok,'[R15B] Cannot stage wrapper: %s',msg);

    fprintf('[R15B] Compiling staged MEX...\n');
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
    assert(isfile(stagedMex),'[R15B] Staged MEX missing.');

    %% --------------------------------------------------------------------
    % 5) Install. Do not clear classes inside installer.
    % ---------------------------------------------------------------------
    dstMex=fullfile(refDir,['af7r36_single_core_mex.' mexext]);
    oldMexTmp=fullfile(stageDir,['af7r36_single_core_mex_OLD.' mexext]);
    hadOldMex=isfile(dstMex);
    if hadOldMex
        [ok,msg]=copyfile(dstMex,oldMexTmp,'f');
        assert(ok,'[R15B] Cannot prepare temporary MEX rollback: %s',msg);
    end

    cd(oldDir);
    clear mex
    wroteHpp=false;installedMex=false;
    try
        write_text(hpp,hppNew);wroteHpp=true;
        [ok,msg]=copyfile(stagedMex,dstMex,'f');
        assert(ok,'[R15B] Cannot install staged MEX: %s',msg);
        installedMex=true;
        clear mex
        rehash

        assert(exist('af7r36_single_core_mex','file')==3, ...
            '[R15B] Installed MEX not visible.');
        assert(contains(normlf(fileread(hpp)), ...
            'AF7_R7_FAILCLOSED_FALLTHROUGH_R15B'), ...
            '[R15B] Installed core patch not visible.');
    catch ME2
        if wroteHpp,try,write_text(hpp,hpp0);catch,end,end
        if installedMex && hadOldMex
            try,copyfile(oldMexTmp,dstMex,'f');catch,end
        end
        clear mex
        rehash
        rethrow(ME2);
    end

    fprintf('[R15B] PASS.\n');
    fprintf('  R7 midpoint Schur failure accepted         = 0\n');
    fprintf('  R7 midpoint Schur failure aborts planner   = 0\n');
    fprintf('  existing fallback remains independently certified = 1\n');
    fprintf('  safety/smoothness gates weakened           = 0\n');
    fprintf('  IMPORTANT: clear old objects/classes before next simulation.\n');

    OUT=struct( ...
        'pass',true, ...
        'revision','AF7_R7_FAILCLOSED_FALLTHROUGH_R15B', ...
        'r7_schur_regularized',false, ...
        'r7_failure_accepted',false, ...
        'r7_failure_falls_through',true, ...
        'hard_safety_modified',false, ...
        'C7_modified',false);

catch ME
    clear mex
    rehash
    fprintf(2,'[R15B] FAILED. No successful production install completed.\n');
    fprintf(2,'[R15B] ORIGINAL ERROR: %s | %s\n',ME.identifier,ME.message);
    rethrow(ME);
end
end

function txt=replace_once(txt,old,new,label)
h=strfind(txt,old);
assert(numel(h)==1,'[R15B] Anchor "%s" count=%d; expected 1.',label,numel(h));
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
assert(f>=0,'[R15B] Cannot open %s.',path);
c=onCleanup(@()fclose(f)); %#ok<NASGU>
fwrite(f,txt,'char');
end

function cleanup_stage(d)
if isfolder(d)
    try,rmdir(d,'s');catch,end
end
end
