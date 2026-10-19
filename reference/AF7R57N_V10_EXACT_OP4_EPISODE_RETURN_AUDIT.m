function OUT = AF7R57N_V10_EXACT_OP4_EPISODE_RETURN_AUDIT()
% AF7R57N_V10_EXACT_OP4_EPISODE_RETURN_AUDIT
% =========================================================================
% Adds a LIVE-CONFIGURABLE, FAIL-CLOSED finite resource contract to the
% already closed AF7R57K corrected derivative-first SHADOW core.
%
% This audit DOES NOT select production caps.
%
% It proves the resource-guard mechanism itself by:
%   1) binding exactly to the closed R57K shadow source SHA;
%   2) inserting guards into all major combinatorial/adaptive paths;
%   3) discovery run with very large diagnostic caps;
%   4) rerun with caps exactly equal to observed usage -> identical theorem;
%   5) one-at-a-time cap reduction -> dedicated fail-closed resource reason.
%
% Production source and active reference are never modified.
% =========================================================================

clc;
fprintf('\n====================================================================================================\n');
fprintf(' AF7R57N V10 | EXACT OP4 EXCEPTION-FREE EPISODE RESOURCE RETURN\n');
fprintf('====================================================================================================\n');
fprintf(['[AUTHORITY] correctedParent=R57K | liveCaps=1 | failClosed=1 | ' ...
         'capValuesNotProductionSelected=1 | productionModified=0\n']);

selfFile=[mfilename('fullpath') '.m'];
assert(isfile(selfFile),'[AF7R57N V10] immutable entrypoint file is unavailable.');
selfSHA=sha256_file(selfFile);
fprintf(' immutable entrypoint                    = %s\n',selfFile);
fprintf(' immutable entrypoint SHA-256             = %s\n',selfSHA);

OUT=struct();
OUT.schema='AF7R57N_V10_EXACT_OP4_EPISODE_RETURN_AUDIT_V1';
OUT.entrypoint=selfFile;
OUT.entrypoint_sha256=selfSHA;
OUT.closed=false;
OUT.production_resource_contract_selected=false;
OUT.production_modified=false;

%% A. Closed parent authorities
fprintf('\n[A. CLOSED PARENT AUTHORITIES]\n');

K=get_authority('AF7R57K_CORRECTED_NATIVE_CORE_AUDIT_OUT', ...
                'AF7R57K_CORRECTED_NATIVE_CORE_AUDIT_OUT.mat');
L=get_authority('AF7R57L_QPC_HARD4MS_CURRENT_PACKET_AUDIT_OUT', ...
                'AF7R57L_QPC_HARD4MS_CURRENT_PACKET_AUDIT_OUT.mat');
M=get_authority('AF7R57M_SOURCE_COMPLEXITY_CONTRACT_AUDIT_OUT', ...
                'AF7R57M_SOURCE_COMPLEXITY_CONTRACT_AUDIT_OUT.mat');
C=get_authority('AF7R57C_SNAPSHOT_BOUND_SCHEMA_CAPTURE_OUT', ...
                'AF7R57C_SNAPSHOT_BOUND_SCHEMA_CAPTURE_OUT.mat');

assert(logical(K.closed),'[AF7R57N] R57K missing.');
assert(logical(L.current_packet_hard_4ms),'[AF7R57N] R57L missing.');
assert(logical(M.closed),'[AF7R57N] R57M missing.');
assert(logical(C.closed),'[AF7R57N] R57C missing.');

fprintf(' R57K corrected theorem                  = 1\n');
fprintf(' R57L current-packet hard 4 ms           = 1\n');
fprintf(' R57M structural complexity              = 1\n');
fprintf(' production-general hard 4 ms            = 0\n');

%% B. Exact corrected-parent source binding
fprintf('\n[B. EXACT CORRECTED-PARENT SOURCE BINDING]\n');

parentH=find_source('af7r57k_trace_core.hpp');
parentCpp=find_source('af7r57k_corrected_core_mex.cpp');

expectedH='4f99d3ba7bdd5645a851a1c2236a8239753631d47998760cc6939a0655a344e1';
expectedCpp='3b774a4aa1b18646182d1f7fa6d01bd2c6669e5ad2e6a8200f702f2f8e6f8597';

hsha=sha256_file(parentH);
csha=sha256_file(parentCpp);
assert(strcmpi(hsha,expectedH),'[AF7R57N] R57K corrected header SHA changed.');
assert(strcmpi(csha,expectedCpp),'[AF7R57N] R57K corrected MEX source SHA changed.');

fprintf(' corrected header SHA current            = 1\n');
fprintf(' corrected serializer SHA current        = 1\n');

fprintf('\n[B2. MATLAB SOURCE-PATCH PREFLIGHT]\n');
fprintf(' token counter implementation            = STRFIND_NUMEL_SCALAR\n');
fprintf(' parent PlanResult local count            = %d\n',token_count(normalize_lf(fileread(parentH)),'PlanResult R;'));
fprintf(' parent corrected Stage-B marker count    = %d\n',token_count(normalize_lf(fileread(parentH)), ...
    'std::vector<StableVLeaf> leaves;for(size_t s=0;s<C.cp.size();++s)leaves.push_back(make_stable_parent_leaf'));
fprintf(' char-plus-newline concatenation used     = 0\n');
parentTxt=text_scalar(normalize_lf(fileread(parentH)));
fprintf(' normalized parent row-char              = %d\n', ...
    ischar(parentTxt)&&size(parentTxt,1)==1);
fprintf(' token_count scalar invariant             = %d\n', ...
    isscalar(token_count(parentTxt,'PlanResult R;')));


%% C. Generate guarded shadow source
fprintf('\n[C. GENERATE GUARDED SHADOW SOURCE]\n');

root=fileparts(mfilename('fullpath'));
guardH=fullfile(root,'af7r57n_v10_trace_core.hpp');
guardCpp=fullfile(root,'af7r57n_v10_guarded_core_mex.cpp');
guardName='af7r57n_v10_guarded_core_mex';

src=text_scalar(normalize_lf(fileread(parentH)));
if ~contains(src,'#include <cstdint>')
    src=text_scalar(['#include <cstdint>' newline src]);
end

% -------------------------------------------------------------------------
% Resource contract / status declarations.
% -------------------------------------------------------------------------
resourceLines={
'// ==================== AF7R57N LIVE RESOURCE CONTRACT ===================='
'struct ResourceContract {'
'    bool enabled=false;'
'    uint64_t max_obstacles=0,max_episode_leaves=0,max_episode_splits=0;'
'    uint64_t max_episodes=0,max_components=0,max_sign_branches=0;'
'    uint64_t max_qp_subset_tests=0,max_candidates=0;'
'    uint64_t max_stageA_leaves=0,max_stageA_splits=0;'
'    uint64_t max_stageB_leaves=0,max_stageB_splits=0;'
'    uint64_t max_geometry_leaves=0,max_geometry_splits=0,max_geometry_depth=0;'
'};'
'struct ResourceUsage {'
'    int exceeded_code=0;'
'    uint64_t obstacles=0,episode_leaves=0,episode_splits=0,episodes=0,components=0;'
'    uint64_t sign_branch_upper=0,qp_subset_upper=0,candidates=0;'
'    uint64_t stageA_leaves=0,stageA_splits=0,stageB_leaves=0,stageB_splits=0;'
'    uint64_t geometry_leaves=0,geometry_splits=0,geometry_depth=0;'
'};'
'static ResourceContract G_RC{};'
'static ResourceUsage G_RU{};'
'static inline bool resource_on(){return G_RC.enabled;}'
'static inline bool resource_exceeded(){return G_RU.exceeded_code!=0;}'
'static inline int resource_reason(){return G_RU.exceeded_code;}'
'static inline void resource_fail(int c){if(G_RU.exceeded_code==0)G_RU.exceeded_code=c;}'
'static inline void resource_reset(){G_RU=ResourceUsage{};}'
'static inline uint64_t resource_pow2_cap(int e,uint64_t cap){'
'    if(e<0)return cap+1;uint64_t v=1;'
'    for(int i=0;i<e;++i){if(v>cap/2)return cap+1;v*=2;}return v;'
'}'
'static inline uint64_t resource_comb_sum_cap(int m,int kmax,uint64_t cap){'
'    long double term=1,sum=1;'
'    for(int k=1;k<=kmax;++k){term=term*(long double)(m-k+1)/(long double)k;sum+=term;'
'        if(sum>(long double)cap)return cap+1;}return (uint64_t)sum;'
'}'
'static inline void resource_configure(const ResourceContract&r,uint64_t nobs){'
'    G_RC=r;resource_reset();G_RU.obstacles=nobs;'
'    if(G_RC.enabled && nobs>G_RC.max_obstacles)resource_fail(201);'
'}'
'// ========================================================================'
};
resourceBlock=[strjoin(resourceLines,newline) newline];
resourceBlock=text_scalar(resourceBlock);

% Critical source-integrity conditions for the resource block.
assert(token_count(resourceBlock,sprintf('\n'))>=30, ...
    '[AF7R57N] resource block lost physical newlines.');
assert(token_count(resourceBlock,'\n')==0, ...
    '[AF7R57N] literal backslash-n leaked into C++ resource block.');
assert(token_count(resourceBlock,'struct ResourceContract {')==1);
assert(token_count(resourceBlock,'struct ResourceUsage {')==1);
assert(token_count(resourceBlock,'// ==================== AF7R57N LIVE RESOURCE CONTRACT')==1);

anchor='struct SpeedCert {';
assert(token_count(src,anchor)==1,'[AF7R57N] SpeedCert anchor mismatch.');
src=text_scalar(strrep(text_scalar(src),text_scalar(anchor), ...
    text_scalar([resourceBlock text_scalar(anchor)])));

% Early abort before episode work if packet parser already rejected obstacle count.
[src,nPlanEarly]=replace_all_count(src,'PlanResult R;', ...
    'PlanResult R;if(resource_exceeded()){R.reason=resource_reason();return R;}');
assert(nPlanEarly>=1,'[AF7R57N] no PlanResult locals patched.');

% Episode adaptive resources.
[src,nEpLeaf]=replace_all_count(src,'++E.leaves;', ...
    ['++E.leaves;if(resource_on()){++G_RU.episode_leaves;' ...
     'if(G_RU.episode_leaves>G_RC.max_episode_leaves){resource_fail(202);E.ok=false;return E;}}']);
assert(nEpLeaf>=1,'[AF7R57N] episode leaf anchor missing.');

[src,nEpSplit]=replace_all_count(src, ...
    'stack.push_back({tc,tb});stack.push_back({ta,tc});++E.splits;', ...
    ['stack.push_back({tc,tb});stack.push_back({ta,tc});++E.splits;' ...
     'if(resource_on()){++G_RU.episode_splits;' ...
     'if(G_RU.episode_splits>G_RC.max_episode_splits){resource_fail(203);E.ok=false;return E;}}']);
assert(nEpSplit>=1,'[AF7R57N] episode split anchor missing.');

% Post-episode pre-admission. Occurs in both native plan/trace paths.
admitOld='int Cn=(int)R.episode.comp.size(),E=(int)R.episode.ep.size(),nvar=6*Cn;';
admitNew=[admitOld ...
    'if(resource_on()){' ...
    'G_RU.episodes=(uint64_t)E;G_RU.components=(uint64_t)Cn;' ...
    'if((uint64_t)E>G_RC.max_episodes){resource_fail(204);R.reason=204;return R;}' ...
    'if((uint64_t)Cn>G_RC.max_components){resource_fail(205);R.reason=205;return R;}' ...
    'uint64_t sb=resource_pow2_cap(E,G_RC.max_sign_branches);G_RU.sign_branch_upper=sb;' ...
    'if(sb>G_RC.max_sign_branches){resource_fail(206);R.reason=206;return R;}' ...
    '}'];
[src,nAdmit]=replace_all_count(src,admitOld,admitNew);
assert(nAdmit>=1,'[AF7R57N] post-episode admission anchor missing.');

% QP active-set subset combinatorial pre-admission, cumulative across signs.
qpOld='for(int k=1;k<=std::min(n,m);++k)rec(0,k,k);';
qpNew=[ ...
    'if(resource_on()){' ...
    'uint64_t ub=resource_comb_sum_cap(m,std::min(n,m),G_RC.max_qp_subset_tests);' ...
    'if(G_RU.qp_subset_upper>G_RC.max_qp_subset_tests-std::min(ub,G_RC.max_qp_subset_tests)){' ...
    'resource_fail(207);return best;}' ...
    'G_RU.qp_subset_upper+=ub;' ...
    'if(G_RU.qp_subset_upper>G_RC.max_qp_subset_tests){resource_fail(207);return best;}' ...
    '}' qpOld];
[src,nQP]=replace_all_count(src,qpOld,qpNew);
assert(nQP==1,'[AF7R57N] QP subset anchor count changed.');

% Candidate count before constructing all candidate CPs.
candOld=['ts=clk::now();' newline ...
    '    std::vector<Candidate> candidates;for(auto&b:B)candidates.push_back('];
candNew=['if(resource_on()){G_RU.candidates=(uint64_t)B.size();' ...
    'if(G_RU.candidates>G_RC.max_candidates){resource_fail(208);R.reason=208;' ...
    'R.tim.total=std::chrono::duration<double,std::milli>(clk::now()-T0).count();return R;}}' newline ...
    '    ts=clk::now();' newline ...
    '    std::vector<Candidate> candidates;for(auto&b:B)candidates.push_back('];
[src,nCand]=replace_all_count(src,candOld,candNew);
assert(nCand>=1,'[AF7R57N] candidate admission anchor missing.');

% Stage-A initial leaf and split resources.
stageAWhile=['while(true){' newline ...
    '        for(int r=0;r<3;++r){R.U[r]=0;R.Lw[r]=0;'];
stageAWhileNew=[ ...
    'if(resource_on()){' ...
    'uint64_t init=0;for(int rr=0;rr<3;++rr)init+=(uint64_t)leaves[rr].size();' ...
    'G_RU.stageA_leaves+=init;' ...
    'if(G_RU.stageA_leaves>G_RC.max_stageA_leaves){resource_fail(209);return R;}}' newline ...
    '    while(true){' newline ...
    '        for(int r=0;r<3;++r){R.U[r]=0;R.Lw[r]=0;'];
[src,nStageAInit]=replace_all_count(src,stageAWhile,stageAWhileNew);
assert(nStageAInit==1,'[AF7R57N] Stage-A init anchor mismatch.');

% Sticky top-level abort after tighten_stageA in both planner paths.
stageACall='SpeedCert sc;auto tr=tighten_stageA(cand,base,L.L,lim);';
stageACallNew=[stageACall ...
    'if(resource_exceeded()){R.success=false;R.reason=resource_reason();' ...
    'R.tim.total=std::chrono::duration<double,std::milli>(clk::now()-T0).count();return R;}'];
[src,nStageASticky]=replace_all_count(src,stageACall,stageACallNew);
assert(nStageASticky==2,'[AF7R57N] Stage-A sticky-abort anchor count mismatch.');

stageASplit='++R.splits;any=true;continue;';
stageASplitNew=[ ...
    '++R.splits;' ...
    'if(resource_on()){++G_RU.stageA_splits;++G_RU.stageA_leaves;' ...
    'if(G_RU.stageA_splits>G_RC.max_stageA_splits||G_RU.stageA_leaves>G_RC.max_stageA_leaves){' ...
    'resource_fail(210);return R;}}any=true;continue;'];
[src,nStageASplit]=replace_all_count(src,stageASplit,stageASplitNew);
assert(nStageASplit==1,'[AF7R57N] Stage-A split anchor mismatch.');

% Corrected Stage-B initial leaves (speed path).
sbInit='std::vector<StableVLeaf> leaves;for(size_t s=0;s<C.cp.size();++s)leaves.push_back(make_stable_parent_leaf';
sbInitNew=[ ...
    'std::vector<StableVLeaf> leaves;for(size_t s=0;s<C.cp.size();++s)leaves.push_back(make_stable_parent_leaf'];
% First preserve exact text; then insert guard after completed statement using a unique tail.
assert(token_count(src,sbInit)>=1,'[AF7R57N] corrected Stage-B marker missing.');
tail='C.seg0[s],C.seg1[s],(int)s));';
tailNew=[tail ...
    'if(resource_on()){G_RU.stageB_leaves+=(uint64_t)leaves.size();' ...
    'if(G_RU.stageB_leaves>G_RC.max_stageB_leaves){resource_fail(211);repr=true;return {};}}'];
[src,nSBTail]=replace_first_count(src,tail,tailNew);
assert(nSBTail==1,'[AF7R57N] Stage-B speed initial tail mismatch.');

% Corrected Stage-B split in speed path.
sbSplit='leaves[bi]=std::move(sp.first);leaves.push_back(std::move(sp.second));++splits;';
sbSplitNew=[ ...
    'leaves[bi]=std::move(sp.first);leaves.push_back(std::move(sp.second));++splits;' ...
    'if(resource_on()){++G_RU.stageB_splits;++G_RU.stageB_leaves;' ...
    'if(G_RU.stageB_leaves>G_RC.max_stageB_leaves){resource_fail(211);repr=true;return {};}' ...
    'if(G_RU.stageB_splits>G_RC.max_stageB_splits){resource_fail(212);repr=true;return {};}}'];
[src,nSBSplit]=replace_first_count(src,sbSplit,sbSplitNew);
assert(nSBSplit==1,'[AF7R57N] Stage-B speed split mismatch.');

% Sticky top-level abort immediately after Stage-B fallback call.
stageBCall='if(!sc.certified){sc=stageB_speed(ref,L,base,cand,lim,wit,rp,&wmax);}';
stageBCallNew=[stageBCall ...
    'if(resource_exceeded()){R.success=false;R.reason=resource_reason();' ...
    'R.tim.total=std::chrono::duration<double,std::milli>(clk::now()-T0).count();return R;}'];
[src,nStageBSticky]=replace_all_count(src,stageBCall,stageBCallNew);
assert(nStageBSticky==2,'[AF7R57N] Stage-B sticky-abort anchor count mismatch.');

% Stage-B trace path also guarded, for diagnostic operations.
traceInit='for(size_t s=0;s<C.cp.size();++s){auto z=make_stable_parent_leaf';
assert(token_count(src,traceInit)>=1,'[AF7R57N] corrected Stage-B trace marker missing.');
traceTail='leaves.push_back(std::move(z));}';
traceTailNew=[traceTail ...
    'if(resource_on()){G_RU.stageB_leaves+=(uint64_t)leaves.size();' ...
    'if(G_RU.stageB_leaves>G_RC.max_stageB_leaves){resource_fail(211);R.repr=true;return R;}}'];
[src,nTraceTail]=replace_first_count(src,traceTail,traceTailNew);
assert(nTraceTail==1,'[AF7R57N] Stage-B trace initial mismatch.');

traceSplit='leaves[bi]=std::move(sp.first);leaves.push_back(std::move(sp.second));++R.splits;';
traceSplitNew=[traceSplit ...
    'if(resource_on()){++G_RU.stageB_splits;++G_RU.stageB_leaves;' ...
    'if(G_RU.stageB_leaves>G_RC.max_stageB_leaves){resource_fail(211);R.repr=true;return R;}' ...
    'if(G_RU.stageB_splits>G_RC.max_stageB_splits){resource_fail(212);R.repr=true;return R;}}'];
[src,nTraceSplit]=replace_first_count(src,traceSplit,traceSplitNew);
assert(nTraceSplit==1,'[AF7R57N] Stage-B trace split mismatch.');

% Whole-body geometry adaptive resources.
geomLeaf='Leaf f=st.back();st.pop_back();double tc=0.5*(f.a+f.b),h=0.5*(f.b-f.a);';
geomLeafNew=[geomLeaf ...
    'if(resource_on()){++G_RU.geometry_leaves;G_RU.geometry_depth=std::max(G_RU.geometry_depth,(uint64_t)f.d);' ...
    'if(G_RU.geometry_leaves>G_RC.max_geometry_leaves){resource_fail(213);G.reason=213;return G;}' ...
    'if((uint64_t)f.d>G_RC.max_geometry_depth){resource_fail(215);G.reason=215;return G;}}'];
[src,nGeomLeaf]=replace_all_count(src,geomLeaf,geomLeafNew);
assert(nGeomLeaf>=1,'[AF7R57N] geometry leaf anchor missing.');

geomSplit='st.push_back({tc,f.b,f.d+1});st.push_back({f.a,tc,f.d+1});++G.splits;';
geomSplitNew=[geomSplit ...
    'if(resource_on()){++G_RU.geometry_splits;' ...
    'if(G_RU.geometry_splits>G_RC.max_geometry_splits){resource_fail(214);G.reason=214;return G;}}'];
[src,nGeomSplit]=replace_all_count(src,geomSplit,geomSplitNew);
assert(nGeomSplit>=1,'[AF7R57N] geometry split anchor missing.');

% -------------------------------------------------------------------------
% Generated C++ header integrity preflight.  Do not invoke the compiler if
% source-line structure was damaged by MATLAB text construction.
% -------------------------------------------------------------------------
assert(token_count(src,'struct ResourceContract {')==1);
assert(token_count(src,'struct ResourceUsage {')==1);
assert(token_count(src,'struct SpeedCert {')==1);
assert(token_count(src,'struct PlanResult {')==1);

pRC=strfind(src,'struct ResourceContract {');
pSC=strfind(src,'struct SpeedCert {');
assert(isscalar(pRC)&&isscalar(pSC)&&pRC<pSC, ...
    '[AF7R57N] resource declaration ordering corrupted.');
assert(pSC==1 || src(pSC-1)==sprintf('\n'), ...
    '[AF7R57N] struct SpeedCert is not at a physical line boundary.');

% The first // resource comment must terminate before ResourceContract.
pComment=strfind(src,'// ==================== AF7R57N LIVE RESOURCE CONTRACT');
assert(isscalar(pComment)&&pComment<pRC);
nlAfterComment=find(src(pComment:pRC)==sprintf('\n'),1,'first');
assert(~isempty(nlAfterComment), ...
    '[AF7R57N] // resource comment swallows following declarations.');

% Exact resource-block brace balance.
assert(token_count(resourceBlock,'{')==token_count(resourceBlock,'}'), ...
    '[AF7R57N] resource block brace imbalance.');

fprintf(' generated header physical-newline check  = 1\n');
fprintf(' resource declaration ordering            = 1\n');
fprintf(' SpeedCert line-boundary check             = 1\n');
fprintf(' resource block brace balance              = 1\n');

headerChanged=write_if_changed(guardH,src);

% -------------------------------------------------------------------------
% Patch serializer/parser:
%   - include guarded header
%   - parse live contract fields from packet
%   - expose operation 9 = resource status query
% -------------------------------------------------------------------------
cpp=text_scalar(normalize_lf(fileread(parentCpp)));
cpp=text_scalar(strrep(text_scalar(cpp), ...
    '#include "af7r57k_trace_core.hpp"', ...
    '#include "af7r57n_v10_trace_core.hpp"'));

parseOld=['S.VQ=scalar(fld(P,"base_uav_speed"),"base_uav_speed");' newline ...
          '    return S;'];
parseOld=text_scalar(parseOld);
parseNew=[ ...
'S.VQ=scalar(fld(P,"base_uav_speed"),"base_uav_speed");' newline ...
'    ResourceContract rc;' newline ...
'    double ren=scalar(fld(P,"resource_guard_enabled"),"resource_guard_enabled");' newline ...
'    if(!(ren==0.0||ren==1.0))throw std::runtime_error("resource_guard_enabled must be 0 or 1");' newline ...
'    rc.enabled=(ren!=0.0);' newline ...
'    auto rcap=[&](const char*n)->uint64_t{' newline ...
'        double v=scalar(fld(P,n),n);' newline ...
'        if(!std::isfinite(v)||v<0.0||v>9007199254740992.0||std::floor(v)!=v)' newline ...
'            throw std::runtime_error(std::string("invalid integer resource cap: ")+n);' newline ...
'        return (uint64_t)v;' newline ...
'    };' newline ...
'    rc.max_obstacles=rcap("resource_max_obstacles");' newline ...
'    rc.max_episode_leaves=rcap("resource_max_episode_leaves");' newline ...
'    rc.max_episode_splits=rcap("resource_max_episode_splits");' newline ...
'    rc.max_episodes=rcap("resource_max_episodes");' newline ...
'    rc.max_components=rcap("resource_max_components");' newline ...
'    rc.max_sign_branches=rcap("resource_max_sign_branches");' newline ...
'    rc.max_qp_subset_tests=rcap("resource_max_qp_subset_tests");' newline ...
'    rc.max_candidates=rcap("resource_max_candidates");' newline ...
'    rc.max_stageA_leaves=rcap("resource_max_stageA_leaves");' newline ...
'    rc.max_stageA_splits=rcap("resource_max_stageA_splits");' newline ...
'    rc.max_stageB_leaves=rcap("resource_max_stageB_leaves");' newline ...
'    rc.max_stageB_splits=rcap("resource_max_stageB_splits");' newline ...
'    rc.max_geometry_leaves=rcap("resource_max_geometry_leaves");' newline ...
'    rc.max_geometry_splits=rcap("resource_max_geometry_splits");' newline ...
'    rc.max_geometry_depth=rcap("resource_max_geometry_depth");' newline ...
'    uint64_t nobs=(uint64_t)mxGetN(fld(P,"centers"));' newline ...
'    resource_configure(rc,nobs);' newline ...
'    return S;'];
parseNew=text_scalar(parseNew);
assert(token_count(cpp,parseOld)==1,'[AF7R57N] Live parser anchor mismatch.');
cpp=text_scalar(strrep(text_scalar(cpp), ...
    text_scalar(parseOld),text_scalar(parseNew)));

statusFunc = [ ...
'static mxArray* af7r57n_resource_status_mx(){' newline ...
'    const char*fn[]={"exceeded_code","obstacles","episode_leaves","episode_splits","episodes","components",' ...
'"sign_branch_upper","qp_subset_upper","candidates","stageA_leaves","stageA_splits","stageB_leaves","stageB_splits",' ...
'"geometry_leaves","geometry_splits","geometry_depth"};' newline ...
'    mxArray*s=mxCreateStructMatrix(1,1,16,fn);' newline ...
'    auto set=[&](const char*n,double v){mxSetField(s,0,n,mxCreateDoubleScalar(v));};' newline ...
'    set("exceeded_code",(double)G_RU.exceeded_code);set("obstacles",(double)G_RU.obstacles);' newline ...
'    set("episode_leaves",(double)G_RU.episode_leaves);set("episode_splits",(double)G_RU.episode_splits);' newline ...
'    set("episodes",(double)G_RU.episodes);set("components",(double)G_RU.components);' newline ...
'    set("sign_branch_upper",(double)G_RU.sign_branch_upper);set("qp_subset_upper",(double)G_RU.qp_subset_upper);' newline ...
'    set("candidates",(double)G_RU.candidates);set("stageA_leaves",(double)G_RU.stageA_leaves);' newline ...
'    set("stageA_splits",(double)G_RU.stageA_splits);set("stageB_leaves",(double)G_RU.stageB_leaves);' newline ...
'    set("stageB_splits",(double)G_RU.stageB_splits);set("geometry_leaves",(double)G_RU.geometry_leaves);' newline ...
'    set("geometry_splits",(double)G_RU.geometry_splits);set("geometry_depth",(double)G_RU.geometry_depth);return s;' newline ...
'}' newline newline];
statusFunc=text_scalar(statusFunc);

mexMarker='void mexFunction';
pos=strfind(cpp,mexMarker);assert(numel(pos)==1,'[AF7R57N] mexFunction marker mismatch.');
cpp=[cpp(1:pos-1) statusFunc cpp(pos:end)];

% Insert op 9 at very beginning of mexFunction body.
pos=strfind(cpp,mexMarker);pos=pos(1);
ob=find(cpp(pos:end)=='{',1,'first')+pos-1;
assert(~isempty(ob));
op9 = [newline ...
'    if(nrhs>=1 && mxIsDouble(prhs[0]) && !mxIsComplex(prhs[0]) && mxGetNumberOfElements(prhs[0])==1 && mxGetScalar(prhs[0])==9.0){' newline ...
'        if(nlhs>1)mexErrMsgIdAndTxt("AF7R57N:Output","op9 returns one status struct");' newline ...
'        plhs[0]=af7r57n_resource_status_mx();return;' newline ...
'    }' newline];
op9=text_scalar(op9);
cpp=text_scalar([cpp(1:ob) text_scalar(op9) cpp(ob+1:end)]);

[cpp,episodeTransportPatch]=patch_episode_resource_transport(cpp);
assert(episodeTransportPatch.closed, ...
    '[AF7R57N] episode resource transport patch did not close.');

cppChanged=write_if_changed(guardCpp,cpp);

fprintf(' guarded header content changed           = %d\n',headerChanged);
fprintf(' guarded MEX source content changed       = %d\n',cppChanged);
fprintf(' PlanResult early guards patched          = %d\n',nPlanEarly);
fprintf(' episode leaf/split patches              = %d / %d\n',nEpLeaf,nEpSplit);
fprintf(' post-episode admission patches           = %d\n',nAdmit);
fprintf(' QP subset pre-admission patches          = %d\n',nQP);
fprintf(' candidate admission patches              = %d\n',nCand);
fprintf(' Stage-A init/split patches               = %d / %d\n',nStageAInit,nStageASplit);
fprintf(' Stage-B init/split patches               = %d / %d\n',nSBTail,nSBSplit);
fprintf(' geometry leaf/split patches              = %d / %d\n',nGeomLeaf,nGeomSplit);
fprintf(' Stage-A sticky-abort patches             = %d\n',nStageASticky);
fprintf(' Stage-B sticky-abort patches             = %d\n',nStageBSticky);
fprintf(' fixed-episode exception token count      = %d\n',episodeTransportPatch.exception_token_count);
fprintf(' exact op4 anchor count                   = %d\n',episodeTransportPatch.op4_anchor_count);
fprintf(' exact op5 untouched anchor count         = %d\n',episodeTransportPatch.op5_anchor_count);
fprintf(' op4 result_struct serializer bound       = %d\n',episodeTransportPatch.result_struct_bound);
fprintf(' op4 202/203 return normalization         = %d\n',episodeTransportPatch.closed);

%% D. Build guarded MEX
fprintf('\n[D. BUILD GUARDED SHADOW MEX]\n');

needBuild=exist(guardName,'file')~=3 || headerChanged || cppChanged;
if needBuild
    inc=['-I' root];
    if ispc
        stdflag='COMPFLAGS=$COMPFLAGS /std:c++17';
    else
        stdflag='CXXFLAGS=$CXXFLAGS -std=c++17';
    end
    try
        mex('-R2018a',inc,stdflag,'-outdir',root,'-output',guardName,guardCpp);
    catch ME
        fprintf('\n[AF7R57N GENERATED-HEADER EXCERPT AROUND RESOURCE/SPEEDCERT]\n');
        dump_source_excerpt(guardH,'AF7R57N LIVE RESOURCE CONTRACT',8,55);
        dump_source_excerpt(guardH,'struct SpeedCert {',8,25);
        rethrow(ME);
    end
    rehash;
end
assert(exist(guardName,'file')==3);
fprintf(' guarded shadow MEX available             = 1\n');
fprintf(' production source modified               = 0\n');

%% E. Prime exact nominal AST
fprintf('\n[E. PRIME NOMINAL AST]\n');

assert(evalin('base','exist(''replanner_instance'',''var'')==1'));
replanner=evalin('base','replanner_instance');
[E0,MM,AUX,authoritySource]=AF7R52_LOAD_TRUE_TRIGGER_AUTHORITY(replanner); %#ok<ASGLU>
assert(isstruct(E0)&&logical(E0.valid)&&isstruct(MM)&&logical(MM.certified));

[expr,sig]=nominal_ast(replanner,MM);
ok=af7r57n_v10_guarded_core_mex(0,expr,sig);
assert(logical(ok));
fprintf(' nominal signature                       = %s\n',sig);

%% F. Current fixed-tU packet / discovery run
fprintf('\n[F. RESOURCE DISCOVERY RUN]\n');

rt1=record_for_candidate(C.trace_records,1);
pkt=rt1.pkt;
tU=double(K.tU);
assert(double(rt1.tU)==tU);

RCinf=unbounded_diagnostic_contract();
P=apply_contract(pkt,RCinf);
Rdisc=af7r57n_v10_guarded_core_mex(4,P,tU,tU);
Udisc=af7r57n_v10_guarded_core_mex(9);

assert(logical(Rdisc.success),'[AF7R57N] discovery run changed corrected theorem.');
assert(double(Udisc.exceeded_code)==0,'[AF7R57N] diagnostic caps unexpectedly exceeded.');

fprintf(' authority source                         = %s\n',char(string(authoritySource)));
print_usage(Udisc);

%% G. Exact-current-usage contract
fprintf('\n[G. EXACT-CURRENT-USAGE CONTRACT TEST]\n');

RCcur=contract_from_usage(Udisc);
Pcur=apply_contract(pkt,RCcur);
Rcur=af7r57n_v10_guarded_core_mex(4,Pcur,tU,tU);
Ucur=af7r57n_v10_guarded_core_mex(9);

same=theorem_equal(K.corrected,Rcur);
contractPass=logical(Rcur.success)&&double(Ucur.exceeded_code)==0&&same;

fprintf(' current-usage contract admits packet     = %d\n',contractPass);
fprintf(' theorem-relevant output exact identity   = %d\n',same);
fprintf(' resource reason                         = %.0f\n',double(Ucur.exceeded_code));
assert(contractPass,'[AF7R57N] exact-current-usage contract failed.');

%% H. One-at-a-time fail-closed fault injection
fprintf('\n[H. ONE-AT-A-TIME FAIL-CLOSED FAULT INJECTION]\n');

tests={
    'max_obstacles','obstacles',201
    'max_episode_leaves','episode_leaves',202
    'max_episode_splits','episode_splits',203
    'max_episodes','episodes',204
    'max_components','components',205
    'max_sign_branches','sign_branch_upper',206
    'max_qp_subset_tests','qp_subset_upper',207
    'max_candidates','candidates',208
    'max_stageA_leaves','stageA_leaves',209
    'max_stageA_splits','stageA_splits',210
    'max_stageB_leaves','stageB_leaves',211
    'max_stageB_splits','stageB_splits',212
    'max_geometry_leaves','geometry_leaves',213
    'max_geometry_splits','geometry_splits',214
    'max_geometry_depth','geometry_depth',215
    };

FI=struct('contract_field',{},'usage_field',{},'expected_reason',{}, ...
          'exercised',{},'closed',{},'transport',{}, ...
          'returned_success',{},'exception_identifier',{},'exception_message',{});
allClosed=true;
for i=1:size(tests,1)
    cf=tests{i,1}; uf=tests{i,2}; code=tests{i,3};
    use=double(Udisc.(uf));
    exercised=use>0;
    closed=true;
    got=NaN;
    transport='NOT_EXERCISED';
    returnedSuccess=false;
    exId='';
    exMsg='';
    if exercised
        RCi=RCcur;
        RCi.(cf)=use-1;
        Pi=apply_contract(pkt,RCi);

        [returned,Ri,MEi]=guarded_fault_call(Pi,tU);
        Ui=af7r57n_v10_guarded_core_mex(9);
        got=double(Ui.exceeded_code);

        if returned
            transport='RETURN';
            returnedSuccess=isstruct(Ri)&&isfield(Ri,'success')&&logical(Ri.success);
            closed=(~returnedSuccess)&&(got==code);
        else
            transport='EXCEPTION';
            returnedSuccess=false;
            exId=MEi.identifier;
            exMsg=MEi.message;
            closed=(got==code)&&~isempty(exMsg);
        end
        allClosed=allClosed&&closed;
    end

    fprintf([' %-24s | use=%-6g | exercised=%d | expected=%d | got=%g ' ...
             '| transport=%-9s | closed=%d\n'], ...
        cf,use,exercised,code,got,transport,closed);

    if exercised && strcmp(transport,'EXCEPTION')
        fprintf('   exception id                          = %s\n',exId);
        fprintf('   exception message                     = %s\n',exMsg);
    end

    FI(end+1)=struct('contract_field',cf,'usage_field',uf, ...
        'expected_reason',code,'exercised',exercised,'closed',closed, ...
        'transport',transport,'returned_success',returnedSuccess, ...
        'exception_identifier',exId,'exception_message',exMsg); %#ok<AGROW>
end

nReturn=sum(arrayfun(@(q)q.exercised&&strcmp(q.transport,'RETURN'),FI));
nException=sum(arrayfun(@(q)q.exercised&&strcmp(q.transport,'EXCEPTION'),FI));
nFaultClosed=sum(arrayfun(@(q)q.exercised&&q.closed,FI));
nFaultExercised=sum(arrayfun(@(q)q.exercised,FI));

fprintf('\n[H2. FAIL-CLOSED TRANSPORT SUMMARY]\n');
fprintf(' exercised resource caps                 = %d\n',nFaultExercised);
fprintf(' dedicated-return rejections             = %d\n',nReturn);
fprintf(' exception-transport rejections          = %d\n',nException);
fprintf(' all exercised resource reasons matched  = %d\n',nFaultClosed==nFaultExercised);
fprintf(' exception-free production transport      = %d\n',nException==0);
assert(nException==0, ...
    '[AF7R57N] V10 requires every exercised production op4 resource cap to use ordinary RETURN transport.');

focusCodes=[209 211 212];
focusClosed=true;
focusReturn=true;
for kk=1:numel(focusCodes)
    jj=find([FI.expected_reason]==focusCodes(kk),1,'first');
    if isempty(jj) || ~FI(jj).exercised
        continue;
    end
    focusClosed=focusClosed && FI(jj).closed;
    focusReturn=focusReturn && strcmp(FI(jj).transport,'RETURN');
end

fprintf('\n[H3. STICKY STAGE-A/B RESOURCE ABORT]\n');
fprintf(' Stage-A/B focused reasons closed         = %d\n',focusClosed);
fprintf(' Stage-A/B rejection transport RETURN     = %d\n',focusReturn);

%% I. Guarded-current QPC regression
fprintf('\n[I. GUARDED-CURRENT QPC REGRESSION]\n');

assert(exist('af7r57l_qpc_stamp_mex','file')==3, ...
    '[AF7R57N] R57L QPC stamp MEX unavailable.');

% Warmup outside hard population.
for k=1:50
    af7r57n_v10_guarded_core_mex(4,Pcur,tU,tU);
end

N=2000;
wall=zeros(N,1);valid=false(N,1);
for k=1:N
    a=af7r57l_qpc_stamp_mex();
    R=af7r57n_v10_guarded_core_mex(4,Pcur,tU,tU);
    b=af7r57l_qpc_stamp_mex();
    wall(k)=1000*(double(b.qpc_ticks)-double(a.qpc_ticks))/double(a.qpc_frequency);
    U=af7r57n_v10_guarded_core_mex(9);
    valid(k)=logical(R.success)&&double(U.exceeded_code)==0&&theorem_equal(K.corrected,R);
end
assert(all(valid));

hardPass=all(wall<4);
fprintf(' hard trials                              = %d\n',N);
fprintf(' guarded min / median / p95 / max [ms]    = %.6f / %.6f / %.6f / %.6f\n', ...
    min(wall),median(wall),prctile(wall,95),max(wall));
fprintf(' guarded >=4 ms spike count               = %d\n',nnz(wall>=4));
fprintf(' EVERY guarded-current trial <4 ms        = %d\n',hardPass);

%% I2. Episode-resource rejection QPC
fprintf('\n[I2. EPISODE 202/203 ORDINARY-REJECT QPC]\n');

episodeCodes=[202 203];
episodeFields={'max_episode_leaves','max_episode_splits'};
episodeUseFields={'episode_leaves','episode_splits'};
Nreject=1000;
rejectWorst=zeros(1,2);
rejectAllFast=true;
rejectAllClosed=true;

for jj=1:2
    RCj=RCcur;
    usej=double(Udisc.(episodeUseFields{jj}));
    assert(usej>0);
    RCj.(episodeFields{jj})=usej-1;
    Pj=apply_contract(pkt,RCj);

    for kk=1:20
        af7r57n_v10_guarded_core_mex(4,Pj,tU,tU);
    end

    tw=zeros(Nreject,1);
    okj=false(Nreject,1);
    for kk=1:Nreject
        a=af7r57l_qpc_stamp_mex();
        Rj=af7r57n_v10_guarded_core_mex(4,Pj,tU,tU);
        b=af7r57l_qpc_stamp_mex();
        tw(kk)=1000*(double(b.qpc_ticks)-double(a.qpc_ticks))/double(a.qpc_frequency);
        Uj=af7r57n_v10_guarded_core_mex(9);
        rcode=NaN;
        if isfield(Rj,'reason_code'),rcode=double(Rj.reason_code);end
        okj(kk)=~logical(Rj.success) && ...
            double(Uj.exceeded_code)==episodeCodes(jj) && ...
            (isnan(rcode)||rcode==episodeCodes(jj));
    end

    assert(all(okj));
    rejectWorst(jj)=max(tw);
    fastj=all(tw<4);
    rejectAllFast=rejectAllFast&&fastj;
    rejectAllClosed=rejectAllClosed&&all(okj);

    fprintf(' reason %d | trials=%d | min/med/p95/max [ms] = %.6f / %.6f / %.6f / %.6f | all<4=%d\n', ...
        episodeCodes(jj),Nreject,min(tw),median(tw),prctile(tw,95),max(tw),fastj);
end

fprintf(' episode reject transport closed          = %d\n',rejectAllClosed);
fprintf(' episode reject EVERY trial <4 ms         = %d\n',rejectAllFast);

%% J. Formal guard gate
fprintf('\n[J. FORMAL RESOURCE-GUARD GATE]\n');

mechanismClosed=contractPass&&allClosed&&focusClosed&&focusReturn&&all(valid);
exceptionFree=(nException==0);
transportClosed=mechanismClosed&&exceptionFree&&rejectAllClosed;
currentGuardedClosed=transportClosed&&hardPass&&rejectAllFast;

fprintf(' corrected parent theorem preserved       = %d\n',contractPass);
fprintf(' all exercised cap fault tests closed     = %d\n',allClosed);
fprintf(' guard mechanism closed                   = %d\n',mechanismClosed);
fprintf(' exception-free resource transport        = %d\n',exceptionFree);
fprintf(' complete resource transport gate         = %d\n',transportClosed);
fprintf(' current success + episode reject <4 ms   = %d\n',currentGuardedClosed);
fprintf(' production resource caps selected        = 0\n');
fprintf(' production-general <=4 ms proved         = 0\n');
fprintf(' production source modified               = 0\n');

if currentGuardedClosed
    next='BUILD_QPC_STRESS_RESOURCE_ENVELOPE_FOR_ADMISSIBLE_CAPS';
else
    next='FAIL_CLOSED__PROFILE_TRANSPORT_OR_REJECTION_TIMING';
end
fprintf(' next action                              = %s\n',next);
fprintf('====================================================================================================\n');

OUT.guard_mechanism_closed=mechanismClosed;
OUT.complete_resource_transport_gate=transportClosed;
OUT.closed=transportClosed;
OUT.current_guarded_hard_4ms=currentGuardedClosed;
OUT.production_resource_contract_selected=false;
OUT.production_general_hard_4ms=false;
OUT.discovery_usage=Udisc;
OUT.current_contract=RCcur;
OUT.fault_injection=FI;
OUT.exception_free_resource_transport=exceptionFree;
OUT.complete_resource_transport_gate=transportClosed;
OUT.exception_transport_count=nException;
OUT.episode_reject_worst_ms=rejectWorst;
OUT.episode_reject_all_fast=rejectAllFast;
OUT.wall_ms=wall;
OUT.worst_ms=max(wall);
OUT.production_modified=false;
OUT.next_action=next;

assignin('base','AF7R57N_V10_EXACT_OP4_EPISODE_RETURN_AUDIT_OUT',OUT);
save(fullfile(root,'AF7R57N_V10_EXACT_OP4_EPISODE_RETURN_AUDIT_OUT.mat'),'OUT','-v7.3');
end


function RC=unbounded_diagnostic_contract()
% Diagnostic discovery only. NOT a production resource contract.
u=flintmax;
RC=struct( ...
    'max_obstacles',u, ...
    'max_episode_leaves',u,'max_episode_splits',u, ...
    'max_episodes',u,'max_components',u,'max_sign_branches',u, ...
    'max_qp_subset_tests',u,'max_candidates',u, ...
    'max_stageA_leaves',u,'max_stageA_splits',u, ...
    'max_stageB_leaves',u,'max_stageB_splits',u, ...
    'max_geometry_leaves',u,'max_geometry_splits',u,'max_geometry_depth',u);
end


function RC=contract_from_usage(U)
RC=struct( ...
    'max_obstacles',double(U.obstacles), ...
    'max_episode_leaves',double(U.episode_leaves), ...
    'max_episode_splits',double(U.episode_splits), ...
    'max_episodes',double(U.episodes), ...
    'max_components',double(U.components), ...
    'max_sign_branches',double(U.sign_branch_upper), ...
    'max_qp_subset_tests',double(U.qp_subset_upper), ...
    'max_candidates',double(U.candidates), ...
    'max_stageA_leaves',double(U.stageA_leaves), ...
    'max_stageA_splits',double(U.stageA_splits), ...
    'max_stageB_leaves',double(U.stageB_leaves), ...
    'max_stageB_splits',double(U.stageB_splits), ...
    'max_geometry_leaves',double(U.geometry_leaves), ...
    'max_geometry_splits',double(U.geometry_splits), ...
    'max_geometry_depth',double(U.geometry_depth));
end


function P=apply_contract(P,RC)
P.resource_guard_enabled=1;
P.resource_max_obstacles=RC.max_obstacles;
P.resource_max_episode_leaves=RC.max_episode_leaves;
P.resource_max_episode_splits=RC.max_episode_splits;
P.resource_max_episodes=RC.max_episodes;
P.resource_max_components=RC.max_components;
P.resource_max_sign_branches=RC.max_sign_branches;
P.resource_max_qp_subset_tests=RC.max_qp_subset_tests;
P.resource_max_candidates=RC.max_candidates;
P.resource_max_stageA_leaves=RC.max_stageA_leaves;
P.resource_max_stageA_splits=RC.max_stageA_splits;
P.resource_max_stageB_leaves=RC.max_stageB_leaves;
P.resource_max_stageB_splits=RC.max_stageB_splits;
P.resource_max_geometry_leaves=RC.max_geometry_leaves;
P.resource_max_geometry_splits=RC.max_geometry_splits;
P.resource_max_geometry_depth=RC.max_geometry_depth;
end


function print_usage(U)
fprintf(' obstacles                               = %.0f\n',double(U.obstacles));
fprintf(' episode leaves / splits                 = %.0f / %.0f\n', ...
    double(U.episode_leaves),double(U.episode_splits));
fprintf(' episodes / components                   = %.0f / %.0f\n', ...
    double(U.episodes),double(U.components));
fprintf(' sign branch upper                       = %.0f\n',double(U.sign_branch_upper));
fprintf(' QP subset cumulative upper              = %.0f\n',double(U.qp_subset_upper));
fprintf(' candidates                              = %.0f\n',double(U.candidates));
fprintf(' Stage-A leaves / splits                 = %.0f / %.0f\n', ...
    double(U.stageA_leaves),double(U.stageA_splits));
fprintf(' Stage-B leaves / splits                 = %.0f / %.0f\n', ...
    double(U.stageB_leaves),double(U.stageB_splits));
fprintf(' geometry leaves / splits / depth        = %.0f / %.0f / %.0f\n', ...
    double(U.geometry_leaves),double(U.geometry_splits),double(U.geometry_depth));
fprintf(' resource exceeded code                  = %.0f\n',double(U.exceeded_code));
end


function [returned,R,ME]=guarded_fault_call(P,tU)
returned=false;
R=struct();
ME=struct('identifier','','message','');
try
    R=af7r57n_v10_guarded_core_mex(4,P,tU,tU);
    returned=true;
catch X
    ME.identifier=X.identifier;
    ME.message=X.message;
end
end


function tf=theorem_equal(A,B)
tf=logical(B.success)&&logical(B.geometry_pass)&&double(B.rejoin_pass_count)>=1;
fields={'reason_code','speed_uav','speed_payload','speed_accel','speed_jerk', ...
        'speed_sigma','speed_qdot','geometry_min_payload_margin', ...
        'geometry_min_uav_margin','geometry_min_cable_margin','geometry_min_margin'};
for i=1:numel(fields)
    f=fields{i};
    if isfield(A,f)&&isfield(B,f)
        tf=tf&&isequaln(A.(f),B.(f));
    end
end
if isfield(A,'control_points')&&isfield(B,'control_points')
    tf=tf&&exact_cp_equal(A.control_points,B.control_points);
end
end


function tf=exact_cp_equal(A,B)
if ~(iscell(A)&&iscell(B)&&isequal(size(A),size(B))),tf=false;return;end
tf=true;
for k=1:numel(A)
    tf=tf&&isequaln(A{k},B{k});
    if ~tf,return;end
end
end


function [expr,signature]=nominal_ast(replanner,M)
F=replanner.nominal_flat_derivative_fn(1:7);
assert(iscell(F)&&numel(F)>=7&&all(cellfun(@(f)isa(f,'function_handle'),F)));
S=cellfun(@func2str,F,'UniformOutput',false);
signature=sha256_text(strjoin(S,char(10)));
assert(strcmp(signature,char(M.nominal_signature)));
expr=cell(21,1);kk=0;
for r=1:7
    ss=strtrim(S{r});
    tok=regexp(ss,'^@\(([A-Za-z]\w*)\)(.*)$','tokens','once');
    assert(~isempty(tok)&&numel(tok)==2);
    body=strtrim(char(tok{2}));
    body=strrep(body,'.^','^');body=strrep(body,'.*','*');body=strrep(body,'./','/');
    z=str2sym(body);z=z(:);assert(numel(z)==3);
    t=sym('t','real');sv0=symvar(z);assert(numel(sv0)<=1);
    if ~isempty(sv0),z=subs(z,sv0(1),t);end
    sv=symvar(z);for j=1:numel(sv),assert(strcmp(char(sv(j)),'t'));end
    for j=1:3,kk=kk+1;expr{kk}=char(z(j));end
end
end


function [cpp,A]=patch_episode_resource_transport(cpp)
% EXACT op=4 normalization from AF7R57N V9 source forensic.
%
% V9 established the current corrected serializer source exactly:
%   op=4 line 439: EpisodeCert E=build_episode(...)
%   op=4 line 440: fixed-episode unavailable error
%   op=4 line 446: plhs[0]=result_struct(Q,t0,tm,t1,O);return;
% and a separate same-text error in op=5 trace mode.
%
% Therefore this patch is deliberately NOT proximity/regex inferred.
% It requires one exact op=4 source block and one exact op=5 source block.
% Only the op=4 error line is expanded. op=5 remains byte-for-byte semantic
% behavior as the diagnostic trace path.
%
% Resource reasons 202/203 only:
%   - return ordinary PlanResult schema,
%   - success=false,
%   - reason=resource_reason(),
%   - retain partial EpisodeCert E for diagnostics,
%   - use the existing op=4 result_struct(...,t0,tm,t1,O) serializer.
% Any non-resource fixed-episode failure still raises AF7R56:Episode.

cpp=text_scalar(normalize_lf(cpp));
A=struct('closed',false,'exception_token_count',0, ...
    'op4_anchor_count',0,'op5_anchor_count',0, ...
    'result_struct_bound',false);

A.exception_token_count=token_count(cpp,'fixed episode unavailable');
assert(A.exception_token_count==2, ...
    '[AF7R57N] V9 authority expected exactly two fixed-episode error tokens.');

op4Old=[ ...
'        if(op==4){' newline ...
'            if(nrhs!=4) mexErrMsgIdAndTxt("AF7R56:Arity","fixed continuation op: op, packet, tUpper, t1.");' newline ...
'            const mxArray*P=prhs[1];' newline ...
'            std::string sig=str(fld(P,"nominal_signature"),"nominal_signature");' newline ...
'            if(!G_compiled||sig!=G_nominal.signature) mexErrMsgIdAndTxt("AF7R56:Reference","compiled nominal signature mismatch.");' newline ...
'            ActiveReference R=parse_ref(P);Live L=parse_live(P);SpeedCert B=parse_speed(P);auto O=parse_obs(P);' newline ...
'            double t0=scalar(fld(P,"t0"),"t0"),tm=scalar(fld(P,"tm"),"tm");' newline ...
'            double tU=scalar(prhs[2],"tUpper"),t1=scalar(prhs[3],"t1");' newline ...
'            EpisodeCert E=build_episode(R,L,B,O,t0,tU);' newline ...
'            if(!E.ok||E.comp.empty()||E.ep.empty())mexErrMsgIdAndTxt("AF7R56:Episode","fixed episode unavailable");' newline ...
'            double tailStart=std::nextafter(E.comp.back().end,std::numeric_limits<double>::infinity());' newline ...
'            TailSafeCert T=certify_tail_safe(R,L,B,O,tailStart,tU);' newline ...
'            if(!(T.pass&&T.repr_fail==0&&t1>tailStart&&t1<=tU))' newline ...
'                mexErrMsgIdAndTxt("AF7R56:Episode","fixed-domain continuation theorem not closed for requested t1");' newline ...
'            PlanResult Q=plan_fixed_episode(R,L,B,O,t0,tm,t1,E);' newline ...
'            plhs[0]=result_struct(Q,t0,tm,t1,O);return;' newline ...
'        }'];
op4Old=text_scalar(op4Old);

op5Old=[ ...
'        if(op==5){' newline ...
'            if(nrhs!=5) mexErrMsgIdAndTxt("AF7R56:Arity","trace op: op, packet, tUpper, t1, candidateIndex.");' newline ...
'            const mxArray*P=prhs[1];' newline ...
'            std::string sig=str(fld(P,"nominal_signature"),"nominal_signature");' newline ...
'            if(!G_compiled||sig!=G_nominal.signature) mexErrMsgIdAndTxt("AF7R56:Reference","compiled nominal signature mismatch.");' newline ...
'            ActiveReference R=parse_ref(P);Live L=parse_live(P);SpeedCert B=parse_speed(P);auto O=parse_obs(P);' newline ...
'            double t0=scalar(fld(P,"t0"),"t0"),tm=scalar(fld(P,"tm"),"tm");' newline ...
'            double tU=scalar(prhs[2],"tUpper"),t1=scalar(prhs[3],"t1");' newline ...
'            int ci=(int)std::llround(scalar(prhs[4],"candidateIndex"))-1;' newline ...
'            EpisodeCert E=build_episode(R,L,B,O,t0,tU);' newline ...
'            if(!E.ok||E.comp.empty()||E.ep.empty())mexErrMsgIdAndTxt("AF7R56:Episode","fixed episode unavailable");' newline ...
'            double tailStart=std::nextafter(E.comp.back().end,std::numeric_limits<double>::infinity());' newline ...
'            TailSafeCert TS=certify_tail_safe(R,L,B,O,tailStart,tU);' newline ...
'            if(!(TS.pass&&TS.repr_fail==0&&t1>tailStart&&t1<=tU))' newline ...
'                mexErrMsgIdAndTxt("AF7R56:Episode","fixed-domain continuation theorem not closed");' newline ...
'            PlanResult Q=plan_fixed_episode(R,L,B,O,t0,tm,t1,E);' newline ...
'            if(ci<0||ci>=(int)Q.all_diag.size())mexErrMsgIdAndTxt("AF7R56:Candidate","candidate index invalid");' newline ...
'            double rhoBody=std::max({L.rQ,L.L+L.rL,L.L+L.rC});' newline ...
'            double lim=(L.sensor_range-rhoBody)/(L.horizon+L.dt);' newline ...
'            StageBTrace TT=stageB_trace(R,L,B,Q.all_diag[ci].cand,lim);' newline ...
'            plhs[0]=stageb_trace_struct(Q.all_diag[ci].cand,TT,lim);return;' newline ...
'        }'];
op5Old=text_scalar(op5Old);

A.op4_anchor_count=token_count(cpp,op4Old);
A.op5_anchor_count=token_count(cpp,op5Old);
assert(A.op4_anchor_count==1, ...
    '[AF7R57N] exact V9-bound op4 block mismatch.');
assert(A.op5_anchor_count==1, ...
    '[AF7R57N] exact V9-bound op5 trace block mismatch.');

op4New=[ ...
'        if(op==4){' newline ...
'            if(nrhs!=4) mexErrMsgIdAndTxt("AF7R56:Arity","fixed continuation op: op, packet, tUpper, t1.");' newline ...
'            const mxArray*P=prhs[1];' newline ...
'            std::string sig=str(fld(P,"nominal_signature"),"nominal_signature");' newline ...
'            if(!G_compiled||sig!=G_nominal.signature) mexErrMsgIdAndTxt("AF7R56:Reference","compiled nominal signature mismatch.");' newline ...
'            ActiveReference R=parse_ref(P);Live L=parse_live(P);SpeedCert B=parse_speed(P);auto O=parse_obs(P);' newline ...
'            double t0=scalar(fld(P,"t0"),"t0"),tm=scalar(fld(P,"tm"),"tm");' newline ...
'            double tU=scalar(prhs[2],"tUpper"),t1=scalar(prhs[3],"t1");' newline ...
'            EpisodeCert E=build_episode(R,L,B,O,t0,tU);' newline ...
'            if(!E.ok||E.comp.empty()||E.ep.empty()){' newline ...
'                if(resource_exceeded()&&(resource_reason()==202||resource_reason()==203)){' newline ...
'                    PlanResult Qr;Qr.success=false;Qr.reason=resource_reason();Qr.episode=E;' newline ...
'                    if(nlhs>0)plhs[0]=result_struct(Qr,t0,tm,t1,O);return;' newline ...
'                }' newline ...
'                mexErrMsgIdAndTxt("AF7R56:Episode","fixed episode unavailable");' newline ...
'            }' newline ...
'            double tailStart=std::nextafter(E.comp.back().end,std::numeric_limits<double>::infinity());' newline ...
'            TailSafeCert T=certify_tail_safe(R,L,B,O,tailStart,tU);' newline ...
'            if(!(T.pass&&T.repr_fail==0&&t1>tailStart&&t1<=tU))' newline ...
'                mexErrMsgIdAndTxt("AF7R56:Episode","fixed-domain continuation theorem not closed for requested t1");' newline ...
'            PlanResult Q=plan_fixed_episode(R,L,B,O,t0,tm,t1,E);' newline ...
'            plhs[0]=result_struct(Q,t0,tm,t1,O);return;' newline ...
'        }'];
op4New=text_scalar(op4New);

cpp=text_scalar(strrep(cpp,op4Old,op4New));

% Exact post-patch invariants.
assert(token_count(cpp,op4Old)==0, ...
    '[AF7R57N] old op4 block remained after patch.');
assert(token_count(cpp,op4New)==1, ...
    '[AF7R57N] patched op4 block not unique.');
assert(token_count(cpp,op5Old)==1, ...
    '[AF7R57N] op5 trace branch changed unexpectedly.');
assert(token_count(cpp,'fixed episode unavailable')==2, ...
    '[AF7R57N] fixed-episode fallback-token inventory changed unexpectedly.');
assert(token_count(cpp,'PlanResult Qr;Qr.success=false;Qr.reason=resource_reason();Qr.episode=E;')==1, ...
    '[AF7R57N] exact resource reject construction missing/duplicated.');
assert(token_count(cpp,'if(nlhs>0)plhs[0]=result_struct(Qr,t0,tm,t1,O);return;')==1, ...
    '[AF7R57N] exact op4 reject serializer missing/duplicated.');
A.result_struct_bound=true;
A.closed=true;
end


function c=text_scalar(x)
% Canonical source-text representation: exactly one row-char vector.
% If the input has multiple text rows/elements, physical newline characters
% are preserved/inserted between them. Never silently flatten source lines.
if ischar(x)
    if isempty(x)
        c='';
    elseif size(x,1)==1
        c=x;
    else
        rows=cellstr(x);
        c=strjoin(rows,newline);
    end
elseif isstring(x)
    q=cellstr(x(:));
    c=strjoin(q,newline);
elseif iscell(x)
    assert(all(cellfun(@(z)ischar(z)||(isstring(z)&&isscalar(z)),x(:))), ...
        '[AF7R57N] non-text cell in source generator.');
    q=cellfun(@text_scalar,x(:),'UniformOutput',false);
    c=strjoin(q,newline);
else
    error('[AF7R57N] source generator received non-text class %s.',class(x));
end
c=reshape(char(c),1,[]);
end


function n=token_count(s,a)
s=text_scalar(s);
a=text_scalar(a);
if isempty(a)
    n=0;
else
    n=numel(strfind(s,a)); %#ok<STREMP>
end
n=double(n);
assert(isscalar(n)&&isfinite(n)&&n>=0);
end


function [s,n]=replace_all_count(s,a,b)
s=text_scalar(s); a=text_scalar(a); b=text_scalar(b);
n=token_count(s,a);
if n>0
    s=strrep(s,a,b);
    s=text_scalar(s);
end
end


function [s,n]=replace_first_count(s,a,b)
s=text_scalar(s); a=text_scalar(a); b=text_scalar(b);
p=strfind(s,a);
n=double(~isempty(p));
if isempty(p),return;end
p=p(1);
s=[s(1:p-1) b s(p+length(a):end)];
s=text_scalar(s);
end


function changed=write_if_changed(fn,txt)
txt=text_scalar(txt);
changed=true;
if isfile(fn)
    old=normalize_lf(fileread(fn));
    if isequal(old,txt),changed=false;return;end
end
fid=fopen(fn,'w');assert(fid>=0);c=onCleanup(@()fclose(fid)); %#ok<NASGU>
fwrite(fid,txt,'char');
end


function s=normalize_lf(s)
s=text_scalar(s);
s=strrep(s,sprintf('\r\n'),sprintf('\n'));
s=strrep(s,sprintf('\r'),sprintf('\n'));
s=text_scalar(s);
end


function dump_source_excerpt(fn,token,beforeN,afterN)
try
    x=normalize_lf(fileread(fn));
    L=regexp(x,'\n','split');
    idx=find(contains(L,token),1,'first');
    if isempty(idx)
        fprintf(' token not found: %s\n',token);
        return;
    end
    lo=max(1,idx-beforeN); hi=min(numel(L),idx+afterN);
    for k=lo:hi
        fprintf(' L%-5d %s\n',k,L{k});
    end
catch
end
end


function r=record_for_candidate(R,c)
ids=cellfun(@(q)double(q.c),R);
k=find(ids==c,1,'last');assert(~isempty(k));r=R{k};
end


function A=get_authority(varName,fileName)
if evalin('base',sprintf('exist(''%s'',''var'')==1',varName))
    A=evalin('base',varName);return;
end
cand={which(fileName),fullfile(pwd,fileName),fullfile(fileparts(mfilename('fullpath')),fileName)};
u=getenv('USERPROFILE');
if ~isempty(u),cand{end+1}=fullfile(u,'Documents','GitHub','common_matlab','reference',fileName);end %#ok<AGROW>
p='';
for i=1:numel(cand)
    if ~isempty(cand{i})&&isfile(cand{i}),p=cand{i};break;end
end
assert(~isempty(p),'[AF7R57N] authority unavailable: %s',fileName);
q=load(p,'OUT');assert(isfield(q,'OUT'));A=q.OUT;
end


function fn=find_source(name)
p=which(name);
if ~isempty(p)&&isfile(p),fn=p;return;end
roots={pwd,fileparts(mfilename('fullpath'))};
u=getenv('USERPROFILE');
if ~isempty(u),roots{end+1}=fullfile(u,'Documents','GitHub','common_matlab','reference');end %#ok<AGROW>
for i=1:numel(roots)
    q=fullfile(roots{i},name);
    if isfile(q),fn=q;return;end
end
assert(false,'[AF7R57N] source unavailable: %s',name);
end


function h=sha256_text(s)
md=java.security.MessageDigest.getInstance('SHA-256');
md.update(uint8(unicode2native(char(s),'UTF-8')));
d=typecast(md.digest(),'uint8');
h=lower(reshape(dec2hex(d,2).',1,[]));
end


function h=sha256_file(fn)
md=java.security.MessageDigest.getInstance('SHA-256');
fid=fopen(fn,'rb');assert(fid>=0);c=onCleanup(@()fclose(fid));
while true
    x=fread(fid,1024*1024,'*uint8');if isempty(x),break;end;md.update(x);
end
d=typecast(md.digest(),'uint8');
h=lower(reshape(dec2hex(d,2).',1,[]));
end
