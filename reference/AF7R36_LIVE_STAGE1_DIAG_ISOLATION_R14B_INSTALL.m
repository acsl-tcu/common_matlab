function OUT = AF7R36_LIVE_STAGE1_DIAG_ISOLATION_R14B_INSTALL()
% AF7R36_LIVE_STAGE1_DIAG_ISOLATION_R14B_INSTALL
%
% R14B fixes two installer defects in R14:
%   (1) the op=1 diagnostic block was matched as one exact multiline block,
%       but R11 had inserted an intervening comment line;
%   (2) outer catch called "clear classes" before rethrow(ME), which erased
%       the caught MException variable itself.
%
% R14B:
%   - removes known diagnostic EXECUTION statements from native op=1
%     individually, not by brittle block matching;
%   - leaves planner math / C7 / speed / rejoin / whole-body / commit gates
%     unchanged;
%   - persists first/last Stage1 native MATLAB exception details;
%   - NEVER calls clear classes inside the installer.
%
% Existing diagnostic helper code/reader ops may remain compiled but dormant.
% No README. No persistent backup.

fprintf('\n====================================================================================================\n');
fprintf(' AF7R36 LIVE STAGE1 DIAGNOSTIC ISOLATION R14B INSTALL\n');
fprintf(' production planner mathematics modified     = 0\n');
fprintf(' hard safety gates modified                  = 0\n');
fprintf(' translation C7 semantics modified           = 0\n');
fprintf(' commit policy modified                      = 0\n');
fprintf(' live op1 diagnostic execution isolated      = 1\n');
fprintf(' installer clear-classes side effect         = 0\n');
fprintf(' Stage1 native exception persistence         = 1\n');
fprintf('====================================================================================================\n');

refDir=fileparts(which('af7r34_make_core_packet'));
assert(~isempty(refDir)&&isfolder(refDir), ...
    '[R14B] Cannot resolve current reference directory.');

cpp=fullfile(refDir,'af7r36_single_core_mex.cpp');
rps=fullfile(refDir,'REPLANNING_SYSTEM.m');
hpp=fullfile(refDir,'af7r36_core.hpp');
assert(isfile(cpp),'[R14B] Missing %s',cpp);
assert(isfile(rps),'[R14B] Missing %s',rps);
assert(isfile(hpp),'[R14B] Missing %s',hpp);

cpp0=normlf(fileread(cpp));
rps0=normlf(fileread(rps));
cppNew=cpp0;
rpsNew=rps0;

assert(contains(cpp0,'motion11_push(Q,R,t0,tm,searchEnd);'), ...
    '[R14B] R11 live motion hook not found.');
assert(contains(cpp0,'phr9_push(Q,Q7,R,L,t0,tm,searchEnd);'), ...
    '[R14B] R9 live heading hook not found.');
assert(contains(cpp0,'r7shadow_push(Q7,L,t0,tm,searchEnd);'), ...
    '[R14B] R8 live shadow hook not found.');
assert(contains(cpp0,'C7R8ShadowResult Q7=c7r8_shadow_plan(R,L,B,O,t0,tm,searchEnd);'), ...
    '[R14B] R8 live shadow evaluation not found.');
assert(contains(rps0,'STAGE1_NATIVE_ERROR__'), ...
    '[R14B] Current staged Stage1 catch block not found.');
assert(~contains(rps0,'staged_replan_native_error_count = 0;'), ...
    '[R14B] Native-error persistence already installed.');

try
    %% --------------------------------------------------------------------
    % 1) Isolate diagnostics from live op=1.
    %
    % Remove each execution statement individually. This is intentionally
    % independent of comments/blank lines added by previous diagnostics.
    % ---------------------------------------------------------------------
    cppNew=remove_once(cppNew, ...
        '            motion11_push(Q,R,t0,tm,searchEnd);', ...
        'R11 motion execution');
    cppNew=remove_once(cppNew, ...
        '            phr9_push(Q,Q7,R,L,t0,tm,searchEnd);', ...
        'R9 heading execution');
    cppNew=remove_once(cppNew, ...
        '            r7shadow_push(Q7,L,t0,tm,searchEnd);', ...
        'R8 shadow ring execution');
    cppNew=remove_once(cppNew, ...
        '            C7R8ShadowResult Q7=c7r8_shadow_plan(R,L,B,O,t0,tm,searchEnd);', ...
        'R8 shadow planner execution');

    % R4C towmap is also diagnostic-only. Remove its live execution so no
    % diagnostic writer can affect Stage1 proposal success or timing.
    if count_exact(cppNew,'            towmap_push(Q,L,t0,tm,searchEnd);')==1
        cppNew=remove_once(cppNew, ...
            '            towmap_push(Q,L,t0,tm,searchEnd);', ...
            'R4C towmap live execution');
    end

    % Witness resets are diagnostic state only. Remove all standalone live
    % reset lines; reader/clear ops and helper code may remain dormant.
    cppNew=strrep(cppNew, ...
        ['            towmap_core_reset_witness();' sprintf('\n')],'');

    %% --------------------------------------------------------------------
    % 2) Persist first/last Stage1 native MATLAB exception.
    % ---------------------------------------------------------------------
    propAnchor='        staged_replan_last_reason = '''';';
    propNew=[propAnchor sprintf('\n') ...
        '        % AF7R36 R14B diagnostic persistence only.' sprintf('\n') ...
        '        staged_replan_native_error_count = 0;' sprintf('\n') ...
        '        staged_replan_first_native_error_identifier = '''';' sprintf('\n') ...
        '        staged_replan_first_native_error_message = '''';' sprintf('\n') ...
        '        staged_replan_first_native_error_time = NaN;' sprintf('\n') ...
        '        staged_replan_last_native_error_identifier = '''';' sprintf('\n') ...
        '        staged_replan_last_native_error_message = '''';' sprintf('\n') ...
        '        staged_replan_last_native_error_time = NaN;'];
    rpsNew=replace_once(rpsNew,propAnchor,propNew,'R14B diagnostic properties');

    oldCatch=[ ...
        '            catch ME' sprintf('\n') ...
        '                obj.staged_replan_fail_count=obj.staged_replan_fail_count+1;' sprintf('\n') ...
        '                obj.generator_validation_fail_count= ...' sprintf('\n') ...
        '                    obj.generator_validation_fail_count+1;' sprintf('\n') ...
        '                obj.generator_last_status_code=-61;' sprintf('\n') ...
        '                obj.staged_replan_last_reason= ...' sprintf('\n') ...
        '                    [''STAGE1_NATIVE_ERROR__'' ME.identifier];' sprintf('\n') ...
        '                return;' sprintf('\n') ...
        '            end'];

    newCatch=[ ...
        '            catch ME' sprintf('\n') ...
        '                % AF7R36 R14B: preserve the first real Stage1 native error.' sprintf('\n') ...
        '                obj.staged_replan_native_error_count= ...' sprintf('\n') ...
        '                    obj.staged_replan_native_error_count+1;' sprintf('\n') ...
        '                if obj.staged_replan_native_error_count==1' sprintf('\n') ...
        '                    obj.staged_replan_first_native_error_identifier=char(ME.identifier);' sprintf('\n') ...
        '                    obj.staged_replan_first_native_error_message=char(ME.message);' sprintf('\n') ...
        '                    obj.staged_replan_first_native_error_time=double(t_now);' sprintf('\n') ...
        '                end' sprintf('\n') ...
        '                obj.staged_replan_last_native_error_identifier=char(ME.identifier);' sprintf('\n') ...
        '                obj.staged_replan_last_native_error_message=char(ME.message);' sprintf('\n') ...
        '                obj.staged_replan_last_native_error_time=double(t_now);' sprintf('\n') ...
        '                obj.staged_replan_fail_count=obj.staged_replan_fail_count+1;' sprintf('\n') ...
        '                obj.generator_validation_fail_count= ...' sprintf('\n') ...
        '                    obj.generator_validation_fail_count+1;' sprintf('\n') ...
        '                obj.generator_last_status_code=-61;' sprintf('\n') ...
        '                obj.staged_replan_last_reason= ...' sprintf('\n') ...
        '                    [''STAGE1_NATIVE_ERROR__'' ME.identifier];' sprintf('\n') ...
        '                return;' sprintf('\n') ...
        '            end'];
    rpsNew=replace_once(rpsNew,oldCatch,newCatch,'Stage1 native catch');

    statusAnchor='            S.last_reason=char(obj.staged_replan_last_reason);';
    statusNew=[statusAnchor sprintf('\n') ...
        '            S.native_error_count=double(obj.staged_replan_native_error_count);' sprintf('\n') ...
        '            S.first_native_error_identifier=char(obj.staged_replan_first_native_error_identifier);' sprintf('\n') ...
        '            S.first_native_error_message=char(obj.staged_replan_first_native_error_message);' sprintf('\n') ...
        '            S.first_native_error_time=double(obj.staged_replan_first_native_error_time);' sprintf('\n') ...
        '            S.last_native_error_identifier=char(obj.staged_replan_last_native_error_identifier);' sprintf('\n') ...
        '            S.last_native_error_message=char(obj.staged_replan_last_native_error_message);' sprintf('\n') ...
        '            S.last_native_error_time=double(obj.staged_replan_last_native_error_time);'];
    rpsNew=replace_once(rpsNew,statusAnchor,statusNew,'staged status diagnostic exposure');

    %% --------------------------------------------------------------------
    % 3) Audits: no diagnostic execution remains in live op=1.
    % ---------------------------------------------------------------------
    assert(count_exact(cppNew,'motion11_push(Q,R,t0,tm,searchEnd);')==0, ...
        '[R14B] R11 motion execution still active.');
    assert(count_exact(cppNew,'phr9_push(Q,Q7,R,L,t0,tm,searchEnd);')==0, ...
        '[R14B] R9 heading execution still active.');
    assert(count_exact(cppNew,'r7shadow_push(Q7,L,t0,tm,searchEnd);')==0, ...
        '[R14B] R8 ring execution still active.');
    assert(count_exact(cppNew,'C7R8ShadowResult Q7=c7r8_shadow_plan(R,L,B,O,t0,tm,searchEnd);')==0, ...
        '[R14B] R8 shadow planning still active.');
    assert(count_exact(cppNew,'towmap_push(Q,L,t0,tm,searchEnd);')==0, ...
        '[R14B] R4C towmap live execution still active.');

    assert(count_exact(rpsNew,'staged_replan_native_error_count = 0;')==1, ...
        '[R14B] Native-error property count invalid.');
    assert(count_exact(rpsNew,'S.native_error_count=double(obj.staged_replan_native_error_count);')==1, ...
        '[R14B] Native-error getter count invalid.');

    % Production authority/gates must remain present.
    assert(contains(cppNew, ...
        'static const char* PLANNER_REV="AF7_REPLANNER_MIN_DEVIATION_V4_FIXED_T1";'), ...
        '[R14B] Planner identity changed.');
    assert(contains(cppNew,'result_struct(Q,t0,tm,t1,O)'), ...
        '[R14B] Production op=1 result serializer disappeared.');
    assert(contains(rpsNew,'obj.evaluate_reference_only_proposal_certificate(P)'), ...
        '[R14B] Staged reference certificate disappeared.');
    assert(contains(rpsNew,'obj.activate_staged_reference_only_bundle'), ...
        '[R14B] Staged atomic activation disappeared.');

    %% --------------------------------------------------------------------
    % 4) Stage compile BEFORE source installation.
    % ---------------------------------------------------------------------
    stageDir=tempname;
    [ok,msg]=mkdir(stageDir);
    assert(ok,'[R14B] Cannot create stage directory: %s',msg);
    stageCleanup=onCleanup(@()cleanup_stage(stageDir)); %#ok<NASGU>

    stagedCpp=fullfile(stageDir,'af7r36_single_core_mex.cpp');
    stagedHpp=fullfile(stageDir,'af7r36_core.hpp');
    write_text(stagedCpp,cppNew);
    [ok,msg]=copyfile(hpp,stagedHpp,'f');
    assert(ok,'[R14B] Cannot stage header: %s',msg);

    fprintf('[R14B] Compiling staged MEX...\n');
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
    assert(isfile(stagedMex),'[R14B] Staged MEX missing.');

    %% --------------------------------------------------------------------
    % 5) Install. Do NOT clear classes: existing workspace objects may exist.
    % ---------------------------------------------------------------------
    dstMex=fullfile(refDir,['af7r36_single_core_mex.' mexext]);
    oldMexTmp=fullfile(stageDir,['af7r36_single_core_mex_OLD.' mexext]);
    hadOldMex=isfile(dstMex);
    if hadOldMex
        [ok,msg]=copyfile(dstMex,oldMexTmp,'f');
        assert(ok,'[R14B] Cannot prepare temporary MEX rollback: %s',msg);
    end

    cd(oldDir);
    clear mex
    wroteCpp=false;wroteRps=false;installedMex=false;
    try
        write_text(cpp,cppNew);wroteCpp=true;
        write_text(rps,rpsNew);wroteRps=true;
        [ok,msg]=copyfile(stagedMex,dstMex,'f');
        assert(ok,'[R14B] Cannot install staged MEX: %s',msg);
        installedMex=true;
        clear mex
        rehash

        assert(exist('af7r36_single_core_mex','file')==3, ...
            '[R14B] Installed MEX not visible.');
        assert(contains(normlf(fileread(rps)), ...
            'staged_replan_native_error_count = 0;'), ...
            '[R14B] Installed REPLANNING_SYSTEM source patch not visible.');
    catch ME2
        % Preserve ME2: never call clear classes/clear here.
        if wroteCpp,try,write_text(cpp,cpp0);catch,end,end
        if wroteRps,try,write_text(rps,rps0);catch,end,end
        if installedMex && hadOldMex
            try,copyfile(oldMexTmp,dstMex,'f');catch,end
        end
        clear mex
        rehash
        rethrow(ME2);
    end

    fprintf('[R14B] PASS.\n');
    fprintf('  live op1 diagnostic execution              = 0\n');
    fprintf('  production planner/safety/C7/commit gates  = unchanged\n');
    fprintf('  Stage1 native first/last error persistence  = 1\n');
    fprintf('  IMPORTANT: clear old objects/classes before next simulation.\n');

    OUT=struct( ...
        'pass',true, ...
        'revision','AF7_LIVE_STAGE1_DIAG_ISOLATION_R14B', ...
        'production_math_modified',false, ...
        'hard_safety_modified',false, ...
        'C7_modified',false, ...
        'commit_policy_modified',false, ...
        'live_op1_diagnostics_active',false, ...
        'native_error_capture_enabled',true);

catch ME
    % Preserve the actual exception. No clear classes here.
    clear mex
    rehash
    fprintf(2,'[R14B] FAILED. Source installation was not successfully completed.\n');
    fprintf(2,'[R14B] ORIGINAL ERROR: %s | %s\n',ME.identifier,ME.message);
    rethrow(ME);
end
end

function txt=replace_once(txt,old,new,label)
h=strfind(txt,old);
assert(numel(h)==1,'[R14B] Anchor "%s" count=%d; expected 1.',label,numel(h));
txt=[txt(1:h-1) new txt(h+numel(old):end)];
end

function txt=remove_once(txt,line,label)
pat=[line sprintf('\n')];
h=strfind(txt,pat);
assert(numel(h)==1,'[R14B] Line "%s" count=%d; expected 1.',label,numel(h));
txt=[txt(1:h-1) txt(h+numel(pat):end)];
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
assert(f>=0,'[R14B] Cannot open %s.',path);
c=onCleanup(@()fclose(f)); %#ok<NASGU>
fwrite(f,txt,'char');
end

function cleanup_stage(d)
if isfolder(d)
    try,rmdir(d,'s');catch,end
end
end
