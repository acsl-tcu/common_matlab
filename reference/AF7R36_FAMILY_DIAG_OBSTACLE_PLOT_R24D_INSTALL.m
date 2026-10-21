function OUT=AF7R36_FAMILY_DIAG_OBSTACLE_PLOT_R24D_INSTALL()
% AF7R36_FAMILY_DIAG_OBSTACLE_PLOT_R24D_INSTALL
%
% Diagnostic + visualization only.
% No C7/speed/rejoin/whole-body acceptance mathematics are changed.
%
% R24D differs from R24/R24B/R24C:
%   - native instrumentation is inserted only inside certify_candidates.
%   - run_flight_headless.m is NOT modified at all.
%   - obstacle rendering is an independent helper installed beside the replanner.
%   - POSTRUN calls the helper automatically after diagnostics.

fprintf('\n====================================================================================================\n');
fprintf(' AF7R36 FAMILY-DIAGNOSTIC + OBSTACLE-PLOT R24D INSTALL\n');
fprintf(' planner mathematics modified                = 0\n');
fprintf(' hard safety gates modified                  = 0\n');
fprintf(' exact-C7 gate modified                      = 0\n');
fprintf(' family-resolved rejection diagnostics       = 1\n');
fprintf(' runner source modified                       = 0\n');
fprintf(' raw + planning-envelope obstacle rendering  = helper/postrun\n');
fprintf(' local equal-scale obstacle view             = Figure 12\n');
fprintf('====================================================================================================\n');

refDir=fileparts(which('af7r34_make_core_packet'));
assert(~isempty(refDir)&&isfolder(refDir),'[R24D] Cannot resolve reference directory.');
hpp=fullfile(refDir,'af7r36_core.hpp');
cpp=fullfile(refDir,'af7r36_single_core_mex.cpp');
rps=fullfile(refDir,'REPLANNING_SYSTEM.m');
assert(isfile(hpp)&&isfile(cpp)&&isfile(rps),'[R24D] production sources missing.');
helperDst=fullfile(refDir,'AF7R24D_OBSTACLE_PLOTS.m');

hpp0=normlf(fileread(hpp)); cpp0=normlf(fileread(cpp));
rps0=normlf(fileread(rps));
hppNew=hpp0; cppNew=cpp0; rpsNew=rps0;

assert(contains(hpp0,'AF7_EXACT_C7_DYADIC_LATTICE_R22C'), ...
    '[R24D] R22C exact-C7 source is not installed.');
assert(contains(hpp0,'if(!cand.exact_c7)continue;'), ...
    '[R24D] R22C exact-C7 hard gate not found.');
assert(contains(cpp0,'"stored_c7_exact"'), ...
    '[R24D] R22C serializer not found.');
assert(contains(rps0,'staged_replan_predeadline_c7_exact'), ...
    '[R24D] R22C predeadline latch not found.');
assert(~contains(hpp0,'AF7R24D_FAMILY_DIAGNOSTICS'), ...
    '[R24D] R24D already installed.');

try
    %% 1. Diagnostic counters in PlanResult.
    old=[ ...
        '    int fallback_rejoin_pass=0;' sprintf('\n') ...
        '    int fallback_geom_tests=0;'];
    new=[old sprintf('\n') ...
        '' sprintf('\n') ...
        '    // AF7R24D_FAMILY_DIAGNOSTICS -- diagnostic only.' sprintf('\n') ...
        '    std::array<int,8> r24d_family_seen{};' sprintf('\n') ...
        '    std::array<int,8> r24d_family_exact{};' sprintf('\n') ...
        '    std::array<int,8> r24d_family_exact_reject{};' sprintf('\n') ...
        '    std::array<int,8> r24d_family_speed_pass{};' sprintf('\n') ...
        '    std::array<int,8> r24d_family_speed_fail{};' sprintf('\n') ...
        '    int r24d_split_representability_reject=0;'];
    hppNew=replace_once(hppNew,old,new,'PlanResult diagnostics');

    %% 2. Count R22C representability rejection. Behavior unchanged.
    old='                    if(m.rfind("R22C ",0)==0)return;';
    new=[ ...
        '                    if(m.rfind("R22C ",0)==0){' sprintf('\n') ...
        '                        ++R.r24d_split_representability_reject;' sprintf('\n') ...
        '                        return;' sprintf('\n') ...
        '                    }'];
    hppNew=replace_once(hppNew,old,new,'split diagnostic');

    %% 3. Instrument exact-C7 gate inside certify_candidates only.
    old='        if(!cand.exact_c7)continue;';
    new=[ ...
        '        const int r24cfi=(cand.family>=0&&cand.family<8)?cand.family:0;' sprintf('\n') ...
        '        ++R.r24d_family_seen[r24cfi];' sprintf('\n') ...
        '        if(cand.exact_c7)++R.r24d_family_exact[r24cfi];' sprintf('\n') ...
        '        else{' sprintf('\n') ...
        '            ++R.r24d_family_exact_reject[r24cfi];' sprintf('\n') ...
        '            continue;' sprintf('\n') ...
        '        }'];
    hppNew=replace_once_in_region(hppNew, ...
        'auto certify_candidates=[&]','return false;', ...
        old,new,'exact-C7 family gate');

    %% 4. Instrument final speed result only. Stage-A/B code is untouched.
    old='            if(!sc.certified) continue;';
    new=[ ...
        '            if(!sc.certified){' sprintf('\n') ...
        '                ++R.r24d_family_speed_fail[r24cfi];' sprintf('\n') ...
        '                continue;' sprintf('\n') ...
        '            }' sprintf('\n') ...
        '            ++R.r24d_family_speed_pass[r24cfi];'];
    hppNew=replace_once_in_region(hppNew, ...
        'auto certify_candidates=[&]','return false;', ...
        old,new,'speed family result');

    %% 5. Serialize diagnostics.
    old='"spatial_envelope_priority","compact_constant_translation","stored_c7_exact","minimum_deviation_primary",';
    new=[ ...
        '"spatial_envelope_priority","compact_constant_translation","stored_c7_exact",' ...
        '"r24d_family_seen","r24d_family_exact","r24d_family_exact_reject",' ...
        '"r24d_family_speed_pass","r24d_family_speed_fail",' ...
        '"r24d_split_representability_reject","minimum_deviation_primary",'];
    cppNew=replace_once(cppNew,old,new,'MEX field list');

    old='    set("stored_c7_exact",logicalScalar(R.cand.exact_c7));';
    new=[old sprintf('\n') ...
        '    std::vector<int> r24cs(R.r24d_family_seen.begin(),R.r24d_family_seen.end());' sprintf('\n') ...
        '    std::vector<int> r24ce(R.r24d_family_exact.begin(),R.r24d_family_exact.end());' sprintf('\n') ...
        '    std::vector<int> r24cx(R.r24d_family_exact_reject.begin(),R.r24d_family_exact_reject.end());' sprintf('\n') ...
        '    std::vector<int> r24cp(R.r24d_family_speed_pass.begin(),R.r24d_family_speed_pass.end());' sprintf('\n') ...
        '    std::vector<int> r24cf(R.r24d_family_speed_fail.begin(),R.r24d_family_speed_fail.end());' sprintf('\n') ...
        '    set("r24d_family_seen",ivecrow(r24cs));' sprintf('\n') ...
        '    set("r24d_family_exact",ivecrow(r24ce));' sprintf('\n') ...
        '    set("r24d_family_exact_reject",ivecrow(r24cx));' sprintf('\n') ...
        '    set("r24d_family_speed_pass",ivecrow(r24cp));' sprintf('\n') ...
        '    set("r24d_family_speed_fail",ivecrow(r24cf));' sprintf('\n') ...
        '    set("r24d_split_representability_reject",dscalar(R.r24d_split_representability_reject));'];
    cppNew=replace_once(cppNew,old,new,'MEX values');

    %% 6. MATLAB latch/status.
    old='        staged_replan_predeadline_c7_exact = false;';
    new=[old sprintf('\n') ...
        '        % R24D diagnostic-only family decomposition.' sprintf('\n') ...
        '        staged_replan_predeadline_r24d_family_seen = nan(1,8);' sprintf('\n') ...
        '        staged_replan_predeadline_r24d_family_exact = nan(1,8);' sprintf('\n') ...
        '        staged_replan_predeadline_r24d_family_exact_reject = nan(1,8);' sprintf('\n') ...
        '        staged_replan_predeadline_r24d_family_speed_pass = nan(1,8);' sprintf('\n') ...
        '        staged_replan_predeadline_r24d_family_speed_fail = nan(1,8);' sprintf('\n') ...
        '        staged_replan_predeadline_r24d_split_representability_reject = NaN;'];
    rpsNew=replace_once(rpsNew,old,new,'MATLAB properties');

    old=[ ...
        '                if isfield(NR16,''stored_c7_exact'')' sprintf('\n') ...
        '                    obj.staged_replan_predeadline_c7_exact=logical(NR16.stored_c7_exact);' sprintf('\n') ...
        '                end'];
    new=[old sprintf('\n') ...
        '                if isfield(NR16,''r24d_family_seen''),obj.staged_replan_predeadline_r24d_family_seen=reshape(double(NR16.r24d_family_seen),1,[]);end' sprintf('\n') ...
        '                if isfield(NR16,''r24d_family_exact''),obj.staged_replan_predeadline_r24d_family_exact=reshape(double(NR16.r24d_family_exact),1,[]);end' sprintf('\n') ...
        '                if isfield(NR16,''r24d_family_exact_reject''),obj.staged_replan_predeadline_r24d_family_exact_reject=reshape(double(NR16.r24d_family_exact_reject),1,[]);end' sprintf('\n') ...
        '                if isfield(NR16,''r24d_family_speed_pass''),obj.staged_replan_predeadline_r24d_family_speed_pass=reshape(double(NR16.r24d_family_speed_pass),1,[]);end' sprintf('\n') ...
        '                if isfield(NR16,''r24d_family_speed_fail''),obj.staged_replan_predeadline_r24d_family_speed_fail=reshape(double(NR16.r24d_family_speed_fail),1,[]);end' sprintf('\n') ...
        '                if isfield(NR16,''r24d_split_representability_reject''),obj.staged_replan_predeadline_r24d_split_representability_reject=double(NR16.r24d_split_representability_reject);end'];
    rpsNew=replace_once(rpsNew,old,new,'MATLAB native latch');

    old='            S.predeadline_c7_exact=logical(obj.staged_replan_predeadline_c7_exact);';
    new=[old sprintf('\n') ...
        '            S.r24d_family_seen=double(obj.staged_replan_predeadline_r24d_family_seen);' sprintf('\n') ...
        '            S.r24d_family_exact=double(obj.staged_replan_predeadline_r24d_family_exact);' sprintf('\n') ...
        '            S.r24d_family_exact_reject=double(obj.staged_replan_predeadline_r24d_family_exact_reject);' sprintf('\n') ...
        '            S.r24d_family_speed_pass=double(obj.staged_replan_predeadline_r24d_family_speed_pass);' sprintf('\n') ...
        '            S.r24d_family_speed_fail=double(obj.staged_replan_predeadline_r24d_family_speed_fail);' sprintf('\n') ...
        '            S.r24d_split_representability_reject=double(obj.staged_replan_predeadline_r24d_split_representability_reject);'];
    rpsNew=replace_once(rpsNew,old,new,'MATLAB status');

    %% 7. Static invariants.
    assert(count_exact(hppNew,'AF7R24D_FAMILY_DIAGNOSTICS')==1);
    assert(contains(hppNew,'r24d_family_speed_fail'));
    assert(contains(cppNew,'"r24d_family_seen"'));
    assert(contains(rpsNew,'S.r24d_family_seen='));
    assert(contains(hppNew,'tighten_stageA'));
    assert(contains(hppNew,'stageB_speed'));
    assert(contains(hppNew,'rejoin_safe'));
    assert(contains(hppNew,'wholebody'));

    %% 8. Stage compile before any production write.
    stage=tempname;[ok,msg]=mkdir(stage);assert(ok,'%s',msg);
    cleanup=onCleanup(@()cleanup_stage(stage)); %#ok<NASGU>
    sh=fullfile(stage,'af7r36_core.hpp');sc=fullfile(stage,'af7r36_single_core_mex.cpp');
    write_text(sh,hppNew);write_text(sc,cppNew);
    oldDir=pwd;cdguard=onCleanup(@()cd(oldDir)); %#ok<NASGU>
    cd(stage);clear mex
    fprintf('[R24D] Compiling staged MEX...\n');
    mex('-R2018a','-O','COMPFLAGS="$COMPFLAGS /std:c++17"', ...
        ['-I' stage],'-output','af7r36_single_core_mex','-outdir',stage,sc);
    sm=fullfile(stage,['af7r36_single_core_mex.' mexext]);assert(isfile(sm));
    cd(oldDir);clear mex

    %% 9. Transactional install.
    dstMex=fullfile(refDir,['af7r36_single_core_mex.' mexext]);
    oldMex=fullfile(stage,['af7r36_single_core_mex_OLD.' mexext]);
    hadMex=isfile(dstMex);if hadMex,[ok,msg]=copyfile(dstMex,oldMex,'f');assert(ok,'%s',msg);end

    helperSrc=fullfile(fileparts(mfilename('fullpath')),'AF7R24D_OBSTACLE_PLOTS.m');
    assert(isfile(helperSrc),'[R24D] Helper file missing beside installer.');
    helperOld='';
    hadHelper=isfile(helperDst);
    if hadHelper
        helperOld=fullfile(stage,'AF7R24D_OBSTACLE_PLOTS_OLD.m');
        copyfile(helperDst,helperOld,'f');
    end

    wroteH=false;wroteC=false;wroteR=false;wroteHelper=false;installed=false;
    try
        write_text(hpp,hppNew);wroteH=true;
        write_text(cpp,cppNew);wroteC=true;
        write_text(rps,rpsNew);wroteR=true;
        [ok,msg]=copyfile(helperSrc,helperDst,'f');assert(ok,'%s',msg);wroteHelper=true;
        [ok,msg]=copyfile(sm,dstMex,'f');assert(ok,'%s',msg);installed=true;
        clear mex;rehash
        assert(exist('af7r36_single_core_mex','file')==3);
        assert(exist('AF7R24D_OBSTACLE_PLOTS','file')==2);
    catch ME2
        if wroteH,try,write_text(hpp,hpp0);catch,end,end
        if wroteC,try,write_text(cpp,cpp0);catch,end,end
        if wroteR,try,write_text(rps,rps0);catch,end,end
        if wroteHelper
            if hadHelper,try,copyfile(helperOld,helperDst,'f');catch,end
            else,try,delete(helperDst);catch,end
            end
        end
        if installed&&hadMex,try,copyfile(oldMex,dstMex,'f');catch,end,end
        clear mex;rehash;rethrow(ME2);
    end

    fprintf('[R24D] PASS.\n');
    fprintf('  family exact/speed decomposition installed = 1\n');
    fprintf('  runner source modified                      = 0\n');
fprintf('  obstacle helper installed                  = 1\n');
    fprintf('  Figure 12 local equal-scale view            = 1\n');
    fprintf('  hard acceptance behavior changed            = 0\n');
    OUT=struct('pass',true,'revision','AF7_FAMILY_DIAG_OBSTACLE_PLOT_R24D');

catch ME
    clear mex;rehash
    fprintf(2,'[R24D] FAILED: %s | %s\n',ME.identifier,ME.message);
    rethrow(ME);
end
end

function txt=replace_once_in_region(txt,regionStart,regionEnd,old,new,label)
p0=strfind(txt,regionStart);assert(numel(p0)==1,'[R24D] region start "%s" count=%d.',label,numel(p0));
tail=txt(p0:end);p1=strfind(tail,regionEnd);assert(~isempty(p1),'[R24D] region end "%s" missing.',label);
stop=p0+p1(1)-2;sub=txt(p0:stop);q=strfind(sub,old);
assert(numel(q)==1,'[R24D] anchor "%s" count=%d inside region.',label,numel(q));
a=p0+q(1)-1;txt=[txt(1:a-1) new txt(a+numel(old):end)];
end

function txt=replace_once(txt,old,new,label)
p=strfind(txt,old);assert(numel(p)==1,'[R24D] Anchor "%s" count=%d; expected 1.',label,numel(p));
txt=[txt(1:p-1) new txt(p+numel(old):end)];
end
function n=count_exact(txt,pat),n=numel(strfind(txt,pat));end
function s=normlf(s)
s=strrep(s,sprintf('\r\n'),sprintf('\n'));s=strrep(s,sprintf('\r'),sprintf('\n'));
end
function write_text(path,txt)
f=fopen(path,'wt','n','UTF-8');assert(f>=0,'[R24D] Cannot open %s.',path);
c=onCleanup(@()fclose(f)); %#ok<NASGU>
fwrite(f,txt,'char');
end
function cleanup_stage(d)
if isfolder(d),try,rmdir(d,'s');catch,end,end
end
