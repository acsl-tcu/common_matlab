function S = AF7R36_TOWING_WITNESS_TIME_MAP_R4_POSTRUN()
% Reads diagnostic-only fixed-t1 call ring from af7r36_single_core_mex.
% No acceptance decision is made from these values.

M=af7r36_single_core_mex(6);
assert(isnumeric(M)&&size(M,2)==44, ...
    '[TOWMAP-R4] Diagnostic ring missing/wrong schema. Run the R4 installer first.');
assert(~isempty(M),'[TOWMAP-R4] Diagnostic ring is empty.');

% The last t0-cluster is the last planning event. Warmup/prime calls happen
% earlier, so this isolates the latest real/adaptive planning sequence without
% changing the production replanner.
t0last=M(end,2);
tol=128*eps(max(1,abs(t0last)));
idx=abs(M(:,2)-t0last)<=tol;
A=M(idx,:);

fprintf('\n====================================================================================================\n');
fprintf(' AF7R36 TOWING WITNESS + t1 FAMILY MAP R4\n');
fprintf(' diagnostic acceptance authority = 0 | rejection witness only = 1\n');
fprintf('====================================================================================================\n');
fprintf('all ring calls / last-cluster calls       = %d / %d\n',size(M,1),size(A,1));
fprintf('last-cluster t0 / tm                      = %.12g / %.12g s\n',A(1,2),A(1,3));
fprintf('finite-sensing UAV-speed limit            = %.12g m/s\n',A(end,5));
fprintf('\n');

fprintf(' seq      t1[s]       success family                         env[m]      VQ/lim    qdot      P(c/s/r/g)   T(c/s/r/g)   F(c/s/r/g)\n');
fprintf('------------------------------------------------------------------------------------------------------------------------------------------------\n');
for i=1:size(A,1)
    fam=family_name(A(i,7));
    if A(i,6)>0.5
        env=A(i,8); util=A(i,9)/A(i,5); qd=A(i,14);
    else
        env=NaN; util=NaN; qd=NaN;
    end
    fprintf('%4.0f  %11.6f     %d   %-28s %9.4f   %8.5f  %8.5f   %d/%d/%d/%d      %d/%d/%d/%d      %d/%d/%d/%d\n', ...
        A(i,1),A(i,4),A(i,6)>0.5,fam,env,util,qd, ...
        round(A(i,16)),round(A(i,17)),round(A(i,18)),round(A(i,19)), ...
        round(A(i,20)),round(A(i,21)),round(A(i,22)),round(A(i,23)), ...
        round(A(i,24)),round(A(i,25)),round(A(i,26)),round(A(i,27)));
end

fprintf('\n[PRIMARY FINITE-WITNESS MAP]\n');
fprintf(' seq      t1[s]       count   minVQ/lim   min-time[s]    min-seg/class          maxVQ/lim   max-time[s]    max-seg/class\n');
for i=1:size(A,1)
    if A(i,28)>0
        fprintf('%4.0f  %11.6f    %3.0f     %9.6f   %11.6f    %3.0f/%-18s  %9.6f   %11.6f    %3.0f/%s\n', ...
            A(i,1),A(i,4),A(i,28),A(i,29)/A(i,5),A(i,30),A(i,31),class_name(A(i,32)), ...
            A(i,33)/A(i,5),A(i,34),A(i,35),class_name(A(i,36)));
    end
end

fprintf('\n[TOWING-CONSTANT FINITE-WITNESS MAP]\n');
fprintf(' seq      t1[s]       candidates speed/rejoin/geom   count   minVQ/lim   min-time[s]    min-seg/class\n');
for i=1:size(A,1)
    if A(i,20)>0 || A(i,37)>0
        fprintf('%4.0f  %11.6f       %d      %d/%d/%d          %3.0f     %9.6f   %11.6f    %3.0f/%s\n', ...
            A(i,1),A(i,4),round(A(i,20)),round(A(i,21)),round(A(i,22)),round(A(i,23)), ...
            A(i,37),A(i,38)/A(i,5),A(i,39),A(i,40),class_name(A(i,41)));
    end
end

% Decision-relevant diagnostic classification.
pExit=false;pUnsafe=false;pEntry=false;pInter=false;
for i=1:size(A,1)
    if A(i,28)>0
        cls=[A(i,32),A(i,36)];
        pEntry=pEntry||any(cls==1);
        pUnsafe=pUnsafe||any(cls==2);
        pInter=pInter||any(cls==3);
        pExit=pExit||any(cls==4);
    end
end
towAnyCand=any(A(:,20)>0);
towSpeed=any(A(:,21)>0);

fprintf('\n[CLASSIFICATION]\n');
fprintf('primary witness observed: ENTRY / UNSAFE / INTER / EXIT = %d / %d / %d / %d\n', ...
    pEntry,pUnsafe,pInter,pExit);
fprintf('towing constant candidates generated / any speed pass = %d / %d\n',towAnyCand,towSpeed);

if pExit && ~pEntry && ~pUnsafe && ~pInter
    cls='PRIMARY_SPEED_FAILURE_LOCALIZED_TO_EXIT_REJOIN__TIME_ALLOCATION_FIRST';
elseif pEntry && ~pUnsafe && ~pInter && ~pExit
    cls='PRIMARY_SPEED_FAILURE_LOCALIZED_TO_ENTRY__GEOMETRY_OR_ENTRY_TRANSITION_FIRST';
elseif pUnsafe && ~pEntry && ~pInter && ~pExit
    cls='PRIMARY_SPEED_FAILURE_LOCALIZED_TO_UNSAFE_AFFINE__SHAPE_VELOCITY_FIRST';
elseif pInter && ~pEntry && ~pUnsafe && ~pExit
    cls='PRIMARY_SPEED_FAILURE_LOCALIZED_TO_INTER_COMPONENT_TRANSITION';
else
    cls='PRIMARY_SPEED_FAILURE_MIXED_OR_T1_DEPENDENT__JOINT_SHAPE_TIME_DESIGN';
end
fprintf('diagnostic class                           = %s\n',cls);
fprintf('NOTE: finite witnesses are rejection-only; no sampled value grants safety.\n');
fprintf('NOTE: this instrumentation perturbs runtime slightly; do not use it for production timing claims.\n');
fprintf('====================================================================================================\n');

S=struct();
S.schema='AF7_TOWMAP_R4';
S.all_ring=M;
S.last_cluster=A;
S.t0=A(1,2);
S.tm=A(1,3);
S.speed_limit=A(end,5);
S.primary_entry_witness=pEntry;
S.primary_unsafe_witness=pUnsafe;
S.primary_inter_witness=pInter;
S.primary_exit_witness=pExit;
S.towing_candidate_generated=towAnyCand;
S.towing_any_speed_pass=towSpeed;
S.classification=cls;
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
