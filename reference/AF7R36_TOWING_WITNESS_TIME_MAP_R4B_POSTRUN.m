function S = AF7R36_TOWING_WITNESS_TIME_MAP_R4B_POSTRUN()
% Read the diagnostic-only R4B fixed-t1 call ring.
% Finite witnesses are rejection-only evidence; they never grant acceptance.

M=af7r36_single_core_mex(6);
assert(isnumeric(M)&&size(M,2)==46, ...
    '[TOWMAP-R4B] Diagnostic ring missing/wrong schema. Install R4B first.');
assert(~isempty(M),'[TOWMAP-R4B] Diagnostic ring is empty.');

% Last t0 cluster corresponds to the last planning request family in the run.
t0last=M(end,2);
tol=128*eps(max(1,abs(t0last)));
A=M(abs(M(:,2)-t0last)<=tol,:);

fprintf('\n====================================================================================================\n');
fprintf(' AF7R36 TOWING WITNESS + t1 FAMILY MAP R4B\n');
fprintf(' diagnostic acceptance authority = 0 | finite witness = rejection only\n');
fprintf('====================================================================================================\n');
fprintf('all ring calls / last t0-cluster calls      = %d / %d\n',size(M,1),size(A,1));
fprintf('last-cluster t0 / tm                        = %.12g / %.12g s\n',A(1,2),A(1,3));
fprintf('finite-sensing UAV-speed limit              = %.12g m/s\n\n',A(end,5));

fprintf(' seq      trial-t1[s]  ok reason family                       env[m]    VQ/lim    qdot     P(c/s/r/g)  T(c/s/r/g)  F(c/s/r/g)\n');
fprintf('--------------------------------------------------------------------------------------------------------------------------------------------\n');
for i=1:size(A,1)
    fam=family_name(A(i,8));
    if A(i,6)>0.5
        env=A(i,9);util=A(i,10)/A(i,5);qd=A(i,15);
    else
        env=NaN;util=NaN;qd=NaN;
    end
    fprintf('%4.0f  %12.6f   %d  %3.0f  %-27s %8.3f  %8.5f %8.5f   %d/%d/%d/%d     %d/%d/%d/%d     %d/%d/%d/%d\n', ...
        A(i,1),A(i,4),A(i,6)>0.5,A(i,7),fam,env,util,qd, ...
        round(A(i,17)),round(A(i,18)),round(A(i,19)),round(A(i,20)), ...
        round(A(i,21)),round(A(i,22)),round(A(i,23)),round(A(i,24)), ...
        round(A(i,25)),round(A(i,26)),round(A(i,27)),round(A(i,28)));
end

fprintf('\n[PRIMARY FINITE SPEED WITNESSES]\n');
fprintf(' seq      t1[s]       count   minVQ/lim   min-time[s]   seg/class                 maxVQ/lim   max-time[s]   seg/class\n');
for i=1:size(A,1)
    if A(i,29)>0
        fprintf('%4.0f  %11.6f    %3.0f     %9.6f  %11.6f   %3.0f/%-20s %9.6f  %11.6f   %3.0f/%s\n', ...
            A(i,1),A(i,4),A(i,29),A(i,30)/A(i,5),A(i,31),A(i,32),class_name(A(i,33)), ...
            A(i,34)/A(i,5),A(i,35),A(i,36),class_name(A(i,37)));
    end
end

fprintf('\n[TOWING-CONSTANT BRANCH]\n');
fprintf(' seq      t1[s]       cand/speed/rejoin/geom   wit   minVQ/lim   time[s]       seg/class\n');
for i=1:size(A,1)
    if A(i,21)>0 || A(i,38)>0
        fprintf('%4.0f  %11.6f       %d/%d/%d/%d             %3.0f    %9.6f  %11.6f   %3.0f/%s\n', ...
            A(i,1),A(i,4),round(A(i,21)),round(A(i,22)),round(A(i,23)),round(A(i,24)), ...
            A(i,38),safe_ratio(A(i,39),A(i,5)),A(i,40),A(i,41),class_name(A(i,42)));
    end
end

% Classify actual primary witness locations over the real search calls.
classes=[];
for i=1:size(A,1)
    if A(i,29)>0
        if isfinite(A(i,33))&&A(i,33)>0, classes(end+1)=A(i,33); end %#ok<AGROW>
        if isfinite(A(i,37))&&A(i,37)>0, classes(end+1)=A(i,37); end %#ok<AGROW>
    end
end
pEntry=any(classes==1);
pUnsafe=any(classes==2);
pInter=any(classes==3);
pExit=any(classes==4);

primaryEverSpeed=any(A(:,18)>0);
towingGenerated=any(A(:,21)>0);
towingEverSpeed=any(A(:,22)>0);

fprintf('\n[SEARCH-LEVEL CLASSIFICATION]\n');
fprintf('primary speed pass observed in any trial    = %d\n',primaryEverSpeed);
fprintf('primary witness ENTRY/UNSAFE/INTER/EXIT     = %d / %d / %d / %d\n', ...
    pEntry,pUnsafe,pInter,pExit);
fprintf('towing candidate generated / speed pass     = %d / %d\n', ...
    towingGenerated,towingEverSpeed);

if primaryEverSpeed
    cls='PRIMARY_RECOVERS_WITHIN_ACTUAL_T1_SEARCH__TIME_ALLOCATION_IS_EFFECTIVE';
elseif pExit && ~pEntry && ~pUnsafe && ~pInter
    cls='PRIMARY_FAILURE_EXIT_REJOIN_ONLY__EXTEND_OR_RETIME_REJOIN_FIRST';
elseif pEntry && ~pUnsafe && ~pInter && ~pExit
    cls='PRIMARY_FAILURE_ENTRY_ONLY__REPLAN_ONSET_OR_ENTRY_GEOMETRY_FIRST';
elseif pUnsafe && ~pEntry && ~pInter && ~pExit
    cls='PRIMARY_FAILURE_UNSAFE_AFFINE_ONLY__SHAPE_VELOCITY_FIRST';
elseif pInter && ~pEntry && ~pUnsafe && ~pExit
    cls='PRIMARY_FAILURE_INTER_TRANSITION_ONLY__COMPONENT_TIMING_FIRST';
else
    cls='PRIMARY_FAILURE_MIXED_OR_T1_DEPENDENT__JOINT_SHAPE_TIME_DESIGN';
end

fprintf('diagnostic class                           = %s\n',cls);
fprintf('NOTE: R4B adds diagnostic writes; do not use its runtime as production timing.\n');
fprintf('====================================================================================================\n');

S=struct();
S.schema='AF7_TOWMAP_R4B';
S.all_ring=M;
S.last_cluster=A;
S.t0=A(1,2);
S.tm=A(1,3);
S.speed_limit=A(end,5);
S.primary_ever_speed_pass=primaryEverSpeed;
S.primary_entry_witness=pEntry;
S.primary_unsafe_witness=pUnsafe;
S.primary_inter_witness=pInter;
S.primary_exit_witness=pExit;
S.towing_candidate_generated=towingGenerated;
S.towing_ever_speed_pass=towingEverSpeed;
S.classification=cls;
end

function r=safe_ratio(a,b)
if isfinite(a)&&isfinite(b)&&b>0, r=a/b; else, r=NaN; end
end

function s=family_name(code)
switch round(code)
    case 1, s='MINIMUM_DEVIATION_AFFINE';
    case 2, s='MINIMUM_SNAP_FALLBACK';
    case 3, s='TOWING_CONSTANT_TRANSLATION';
    otherwise, s='NONE/FAILED';
end
end

function s=class_name(code)
switch round(code)
    case 1, s='ENTRY_C7';
    case 2, s='UNSAFE_AFFINE';
    case 3, s='INTER_C7';
    case 4, s='EXIT_REJOIN_C7';
    otherwise, s='NONE';
end
end
