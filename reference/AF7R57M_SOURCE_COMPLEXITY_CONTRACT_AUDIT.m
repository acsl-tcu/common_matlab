function OUT = AF7R57M_SOURCE_COMPLEXITY_CONTRACT_AUDIT()
% AF7R57M_SOURCE_COMPLEXITY_CONTRACT_AUDIT
% =========================================================================
% Source-bound computational-complexity audit for the corrected AF7 native
% planner lineage.
%
% This audit does NOT infer a production-general 4 ms guarantee from one
% measured packet.  It establishes the exact structural growth terms that a
% future live resource/admission guard must bound:
%
%   sign branches                 <= 2^E
%   candidate segments            = 2*C + 1
%   QP active subsets / sign      <= sum_{k=0}^{min(n,m)} C(m,k), n=6*C
%   geometry primitive tests      >= 3*Nobs per geometry leaf
%
% and checks whether the current source has finite fail-closed resource caps
% for these loops.  If no such caps exist, a configuration-independent WCET
% theorem is impossible for the current algorithm.
%
% No production source is modified.
% =========================================================================

clc;
fprintf('\n====================================================================================================\n');
fprintf(' AF7R57M | SOURCE-BOUND LIVE COMPLEXITY CONTRACT AUDIT\n');
fprintf('====================================================================================================\n');
fprintf(['[AUTHORITY] sourceStructural=1 | noEmpiricalLinearExtrapolation=1 | ' ...
         'unboundedConfigNotClaimed=1 | productionModified=0\n']);

OUT=struct();
OUT.schema='AF7R57M_SOURCE_COMPLEXITY_CONTRACT_AUDIT_V1';
OUT.closed=false;
OUT.production_general_hard_4ms=false;
OUT.production_modified=false;

%% A. Closed foundations
fprintf('\n[A. CLOSED FOUNDATIONS]\n');

K=get_authority('AF7R57K_CORRECTED_NATIVE_CORE_AUDIT_OUT', ...
                'AF7R57K_CORRECTED_NATIVE_CORE_AUDIT_OUT.mat');
L=get_authority('AF7R57L_QPC_HARD4MS_CURRENT_PACKET_AUDIT_OUT', ...
                'AF7R57L_QPC_HARD4MS_CURRENT_PACKET_AUDIT_OUT.mat');
C=get_authority('AF7R57C_SNAPSHOT_BOUND_SCHEMA_CAPTURE_OUT', ...
                'AF7R57C_SNAPSHOT_BOUND_SCHEMA_CAPTURE_OUT.mat');

assert(logical(K.closed),'[AF7R57M] R57K theorem missing.');
assert(logical(L.current_packet_hard_4ms),'[AF7R57M] R57L current-packet hard gate missing.');
assert(logical(C.closed),'[AF7R57M] R57C authority missing.');

fprintf(' R57K corrected theorem                  = 1\n');
fprintf(' R57L current-packet hard 4 ms           = 1\n');
fprintf(' R57C source/schema authority            = 1\n');

%% B. Exact source binding
fprintf('\n[B. EXACT SOURCE BINDING]\n');

core=find_source('af7r56_trace_core.hpp');
rps=which('REPLANNING_SYSTEM');

expectedCore='9f1c7054bbac0dadc2e82fbb107183937143b39431d8996ee13a620cc62ef821';
expectedRps ='223d34c49d7148b8ea14ff4ab9c420d409560c0650ac19e9b0f99318887090c9';

sourceClosed=strcmpi(sha256_file(core),expectedCore)&&strcmpi(sha256_file(rps),expectedRps);
fprintf(' core SHA current                        = %d\n',strcmpi(sha256_file(core),expectedCore));
fprintf(' replanner SHA current                   = %d\n',strcmpi(sha256_file(rps),expectedRps));
assert(sourceClosed,'[AF7R57M] source authority changed.');

coreText=fileread(core);
rpsText=fileread(rps);

%% C. Source-structural complexity identities
fprintf('\n[C. SOURCE-STRUCTURAL COMPLEXITY IDENTITIES]\n');

% 1) Exact sign enumeration: recursive +1/-1 for each OR episode.
signPlus=contains(rpsText,'signs(idx)=1;');
signMinus=contains(rpsText,'signs(idx)=-1;');
signRec=contains(rpsText,'enumerate_piecewise_sign_qp_recursive');
signExact=signPlus&&signMinus&&signRec;

% 2) Candidate segments: start + component + transition + end = 2*C+1.
segSource=contains(coreText,'int nc=(int)comp.size(),nslot=2*nc') && ...
          contains(coreText,'if(j<nc-1)') && ...
          contains(coreText,'// start transition');

% 3) Active-set combinatorial enumeration.
activeComb=contains(coreText,'for(int k=1;k<=std::min(n,m);++k)rec(0,k,k)');

% 4) Whole-body every obstacle, three primitive margins.
geomObsLoop=contains(coreText,'for(auto&o:obs)');
geomThree=contains(coreText,'point_margin(pL') && ...
          contains(coreText,'point_margin(pQ') && ...
          contains(coreText,'segment_margin(pQ,pL');

fprintf(' exact +/- sign recursion present         = %d\n',signExact);
fprintf(' candidate 2*C+1 segment structure       = %d\n',segSource);
fprintf(' active-set subset enumeration present    = %d\n',activeComb);
fprintf(' geometry loops every obstacle            = %d\n',geomObsLoop);
fprintf(' L/Q/cable 3 primitive tests per obstacle = %d\n',geomThree);

structuralClosed=signExact&&segSource&&activeComb&&geomObsLoop&&geomThree;
assert(structuralClosed,'[AF7R57M] source structural identity changed.');

%% D. Current live workload
fprintf('\n[D. CURRENT LIVE WORKLOAD]\n');

R=K.corrected;

E=getnum(R,'episode_count',NaN);
Cc=getnum(R,'component_count',NaN);
if ~isfinite(Cc)
    rk=record_for_candidate(C.kkt_records,1);
    Cc=numel(rk.U.component_start);
end
Nobs=double(L.obstacle_count);

speedLeaves=getnum(R,'speed_leaf_count',NaN);
speedSplits=getnum(R,'speed_split_count',NaN);
geomLeaves=getnum(R,'geometry_leaf_count',NaN);
geomSplits=getnum(R,'geometry_split_count',NaN);
geomDepth=getnum(R,'geometry_max_depth',NaN);

rk=record_for_candidate(C.kkt_records,1);
nvar=size(double(rk.A),2);
mrow=size(double(rk.A),1);

if ~isfinite(E)
    if isfield(R,'corridor_signs')
        E=numel(R.corridor_signs);
    else
        E=NaN;
    end
end

if isfinite(E) && E>=0 && E<53
    signBranches=2^E;
else
    signBranches=Inf;
end

segments=2*Cc+1;
activeSubsetUpper=comb_sum_upper(mrow,min(nvar,mrow));
geomPrimitiveCurrent=3*Nobs*geomLeaves;

fprintf(' episodes E                              = %.0f\n',E);
fprintf(' components C                            = %.0f\n',Cc);
fprintf(' obstacles Nobs                          = %.0f\n',Nobs);
fprintf(' exact sign branch upper 2^E             = %.0f\n',signBranches);
fprintf(' candidate segments 2*C+1                = %.0f\n',segments);
fprintf(' QP variables / corridor rows            = %d / %d\n',nvar,mrow);
fprintf(' active-subset combinatorial upper        = %.0f\n',activeSubsetUpper);
fprintf(' speed leaves / splits                   = %.0f / %.0f\n',speedLeaves,speedSplits);
fprintf(' geometry leaves / splits / depth        = %.0f / %.0f / %.0f\n',geomLeaves,geomSplits,geomDepth);
fprintf(' selected-candidate geometry primitive count ~= 3*Nobs*Gleaf = %.0f\n', ...
    geomPrimitiveCurrent);
fprintf(' measured raw-QPC worst current [ms]      = %.6f\n',double(L.worst_ms));
fprintf(' measured hard slack current [ms]         = %.6f\n',double(L.hard_slack_ms));

%% E. Resource-cap audit
fprintf('\n[E. FINITE RESOURCE-CAP AUDIT]\n');

% Search only for explicit algorithmic stop caps in the bound source.  A
% binary64 unsplittable condition is fail-closed numerically but does NOT give
% a useful small WCET cap.
hasSignCap = contains(lower(rpsText),'max_sign') || contains(lower(coreText),'max_sign');
hasCandidateCap = contains(lower(coreText),'max_candidate') || contains(lower(rpsText),'max_candidate');
hasGeomLeafCap = contains(lower(coreText),'max_geometry_leaf') || contains(lower(coreText),'geometry_max_leaves');
hasGeomDepthCap = contains(lower(coreText),'max_geometry_depth') || contains(lower(coreText),'geometry_max_depth_cap');
hasSpeedSplitCap = contains(lower(coreText),'max_stageb_split') || contains(lower(coreText),'stageb_max_split');
hasQpSubsetCap = contains(lower(coreText),'max_active_set') || contains(lower(coreText),'max_qp_subset');

fprintf(' explicit sign-branch runtime cap         = %d\n',hasSignCap);
fprintf(' explicit candidate-count runtime cap     = %d\n',hasCandidateCap);
fprintf(' explicit geometry-leaf runtime cap       = %d\n',hasGeomLeafCap);
fprintf(' explicit geometry-depth runtime cap      = %d\n',hasGeomDepthCap);
fprintf(' explicit Stage-B split runtime cap       = %d\n',hasSpeedSplitCap);
fprintf(' explicit QP active-subset runtime cap    = %d\n',hasQpSubsetCap);

finiteResourceContract=hasSignCap&&hasCandidateCap&&hasGeomLeafCap&& ...
    hasGeomDepthCap&&hasSpeedSplitCap&&hasQpSubsetCap;

fprintf(' finite production resource contract      = %d\n',finiteResourceContract);

%% F. Formal complexity conclusion
fprintf('\n[F. FORMAL COMPLEXITY CONCLUSION]\n');

% Mathematical facts:
%   Bsign(E) = 2^E
%   S(C)     = 2C+1
%   A(n,m)  <= sum_{k=0}^{min(n,m)} C(m,k)
%   G        contains Nobs multiplicatively for each leaf.
%
% Therefore without finite live caps, no finite configuration-independent
% operation-count bound exists for this source.

unboundedImpossible = structuralClosed && ~finiteResourceContract;

fprintf(' sign complexity exponential in E         = 1\n');
fprintf(' geometry complexity multiplicative Nobs  = 1\n');
fprintf(' current source lacks full finite caps     = %d\n',~finiteResourceContract);
fprintf(' config-independent finite op bound exists = %d\n',finiteResourceContract);
fprintf(' current-packet hard 4 ms remains valid    = %d\n',logical(L.current_packet_hard_4ms));
fprintf(' production-general hard 4 ms proved       = 0\n');

if unboundedImpossible
    next='BUILD_LIVE_CONFIGURABLE_FAIL_CLOSED_RESOURCE_GUARD_IN_CORRECTED_SHADOW';
else
    next='CALIBRATE_RESOURCE_CONTRACT_WITH_QPC_AND_CLOSE_GENERAL_ADMISSION_GATE';
end
fprintf(' next action                              = %s\n',next);
fprintf('====================================================================================================\n');

OUT.closed=structuralClosed;
OUT.source_structural_closed=structuralClosed;
OUT.current_packet_hard_4ms=logical(L.current_packet_hard_4ms);
OUT.production_general_hard_4ms=false;
OUT.episodes=E;
OUT.components=Cc;
OUT.obstacles=Nobs;
OUT.sign_branch_upper=signBranches;
OUT.candidate_segments=segments;
OUT.qp_variables=nvar;
OUT.qp_rows=mrow;
OUT.active_subset_upper=activeSubsetUpper;
OUT.speed_leaves=speedLeaves;
OUT.speed_splits=speedSplits;
OUT.geometry_leaves=geomLeaves;
OUT.geometry_splits=geomSplits;
OUT.geometry_depth=geomDepth;
OUT.geometry_primitive_current=geomPrimitiveCurrent;
OUT.finite_resource_contract=finiteResourceContract;
OUT.resource_caps=struct( ...
    'sign',hasSignCap,'candidate',hasCandidateCap,'geometry_leaf',hasGeomLeafCap, ...
    'geometry_depth',hasGeomDepthCap,'stageB_split',hasSpeedSplitCap, ...
    'qp_subset',hasQpSubsetCap);
OUT.production_modified=false;
OUT.next_action=next;

assignin('base','AF7R57M_SOURCE_COMPLEXITY_CONTRACT_AUDIT_OUT',OUT);
save(fullfile(fileparts(mfilename('fullpath')), ...
    'AF7R57M_SOURCE_COMPLEXITY_CONTRACT_AUDIT_OUT.mat'),'OUT','-v7.3');
end


function v=getnum(S,f,d)
v=d;
if isstruct(S)&&isfield(S,f)
    q=double(S.(f));
    if isscalar(q)&&isfinite(q),v=q;end
end
end


function s=comb_sum_upper(m,kmax)
% Exact integer count in double until it exceeds flintmax, then Inf.
s=1;
term=1;
for k=1:kmax
    term=term*(m-k+1)/k;
    s=s+term;
    if ~isfinite(s)||s>flintmax
        s=Inf;return;
    end
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
assert(~isempty(p),'[AF7R57M] authority unavailable: %s',fileName);
q=load(p,'OUT');assert(isfield(q,'OUT'));A=q.OUT;
end


function fn=find_source(name)
p=which(name);
if ~isempty(p)&&isfile(p),fn=p;return;end
u=getenv('USERPROFILE');
q=fullfile(u,'Documents','GitHub','common_matlab','reference',name);
assert(isfile(q),'[AF7R57M] source unavailable: %s',name);
fn=q;
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
