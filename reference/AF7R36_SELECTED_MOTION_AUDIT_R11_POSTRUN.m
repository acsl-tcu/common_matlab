function S = AF7R36_SELECTED_MOTION_AUDIT_R11_POSTRUN()
M=af7r36_single_core_mex(18);
assert(isnumeric(M)&&size(M,2)==29,'[MOTION-R11] R11 ring unavailable/wrong schema.');
assert(~isempty(M),'[MOTION-R11] R11 ring empty.');

t0last=M(end,2);
tol=128*eps(max(1,abs(t0last)));
A=M(abs(M(:,2)-t0last)<=tol,:);

fprintf('\n====================================================================================================\n');
fprintf(' AF7R36 SELECTED MOTION AUDIT R11 RESULT\n');
fprintf(' sampled diagnostic only | production/safety/C7 unchanged\n');
fprintf('====================================================================================================\n');
fprintf('last-cluster t0 / tm = %.12g / %.12g s\n\n',A(1,2),A(1,3));

fprintf(' t1[s]       family                       fullExt[m]  baseExt[m]  corrExt[m]  endDisp[m]  vmax[m/s]  vxymax  moving/65  env[m]\n');
fprintf('---------------------------------------------------------------------------------------------------------------------------------\n');
for i=1:size(A,1)
    fprintf('%11.6f  %-27s  %10.4f  %10.4f  %10.4f  %10.4f  %10.4f  %8.4f    %2d/65    %8.4f\n', ...
        A(i,4),family_name(A(i,6)),A(i,7),A(i,9),A(i,10),A(i,11),A(i,12),A(i,13),round(A(i,16)),A(i,17));
end

fprintf('\n[SELECTED FULL REFERENCE POSITIONS]\n');
for i=1:size(A,1)
    fprintf('t1=%11.6f | p(t0)=[% .5f % .5f % .5f] | p(mid)=[% .5f % .5f % .5f] | p(t1)=[% .5f % .5f % .5f]\n', ...
        A(i,4),A(i,18),A(i,19),A(i,20),A(i,21),A(i,22),A(i,23),A(i,24),A(i,25),A(i,26));
    fprintf('             correction(mid)=[% .5f % .5f % .5f]\n',A(i,27),A(i,28),A(i,29));
end

moving=(A(:,7)>1e-6)&(A(:,12)>1e-6)&(A(:,16)>2);
corrActive=A(:,10)>1e-6;
if any(moving)&&any(corrActive)
    cls='PLANNED_SELECTED_REFERENCE_IS_MOVING_AND_CORRECTION_IS_ACTIVE';
elseif any(moving)&&~any(corrActive)
    cls='FULL_REFERENCE_MOVES_BUT_NEW_CORRECTION_IS_EFFECTIVELY_ZERO';
elseif ~any(moving)&&any(corrActive)
    cls='CORRECTION_NONZERO_BUT_FULL_REFERENCE_MOTION_NOT_OBSERVED__CHECK_CANCELLATION_OR_EVAL';
else
    cls='SELECTED_REFERENCE_APPEARS_STATIC_INSIDE_PLANNER__INVESTIGATE_GENERATION';
end

fprintf('\n[DECISION]\n');
fprintf('selected full reference moves internally    = %d\n',any(moving));
fprintf('new correction nonzero internally           = %d\n',any(corrActive));
fprintf('classification                              = %s\n',cls);
fprintf('====================================================================================================\n');

S=struct();
S.schema='AF7_SELECTED_MOTION_AUDIT_R11';
S.rows=A;
S.selected_reference_moves=any(moving);
S.correction_active=any(corrActive);
S.classification=cls;
end

function s=family_name(code)
switch round(code)
    case 1,s='MINIMUM_DEVIATION_AFFINE';
    case 2,s='MINIMUM_SNAP_FALLBACK';
    case 3,s='TOWING_C7_CONDENSED_C3';
    otherwise,s='NONE/FAILED';
end
end
