function S = AF7R36_PRIMARY_HEADING_AND_R7_MAP_R9_POSTRUN()
M=af7r36_single_core_mex(12);
assert(isnumeric(M)&&size(M,2)==30,'[PHR7-R9] R9 ring unavailable/wrong schema.');
assert(~isempty(M),'[PHR7-R9] R9 ring is empty.');

t0last=M(end,2);
tol=128*eps(max(1,abs(t0last)));
A=M(abs(M(:,2)-t0last)<=tol,:);

fprintf('\n====================================================================================================\n');
fprintf(' AF7R36 PRIMARY HEADING + R7 SEGMENT MAP R9 RESULT\n');
fprintf(' production selection changed = 0 | safety changed = 0 | translation C7 changed = 0\n');
fprintf('====================================================================================================\n');
fprintf('last-cluster t0 / tm = %.12g / %.12g s\n\n',A(1,2),A(1,3));

fprintf(' t1[s]       prod family                    heading  env[m]   VQ/lim   qdot   | R7 ok reason c/s/r/g/h\n');
fprintf('----------------------------------------------------------------------------------------------------------------\n');
for i=1:size(A,1)
    fprintf('%11.6f  %-28s    %d    %7.3f  %7.5f  %7.5f |  %d   %3.0f  %d/%d/%d/%d/%d\n', ...
        A(i,4),family_name(A(i,7)),round(A(i,8)),A(i,9),safe_ratio(A(i,10),A(i,5)),A(i,11), ...
        round(A(i,12)),A(i,13),round(A(i,14)),round(A(i,15)),round(A(i,16)),round(A(i,17)),round(A(i,18)));
end

fprintf('\n[CORRECTED R7 FINITE SPEED WITNESS MAP]\n');
for i=1:size(A,1)
    if A(i,22)>0
        fprintf(['t1=%11.6f | count=%d | min %.6fx @ %.6f s seg=%d %s' ...
                 ' | max %.6fx @ %.6f s seg=%d %s\n'], ...
            A(i,4),round(A(i,22)),safe_ratio(A(i,23),A(i,5)),A(i,24),round(A(i,25)),class_name(A(i,26)), ...
            safe_ratio(A(i,27),A(i,5)),A(i,28),round(A(i,29)),class_name(A(i,30)));
    end
end

prodPrimary=(A(:,6)>0.5)&(round(A(:,7))==1);
prodHeading=prodPrimary&(A(:,8)>0.5);
r7ok=A(:,12)>0.5;

fprintf('\n[DECISION]\n');
fprintf('compact primary selected                     = %d\n',any(prodPrimary));
fprintf('compact primary also heading-regular         = %d\n',any(prodHeading));
fprintf('R7 hard-certified + heading-regular          = %d\n',any(r7ok));

if any(prodHeading)
    cls='COMPACT_PRIMARY_C7_SAFE_HEADING_REGULAR__PRIMARY_IS_VIABLE_NORMAL_SOLUTION';
elseif any(prodPrimary)
    cls='COMPACT_PRIMARY_SAFE_C7_BUT_TANGENT_YAW_SINGULAR__NEED_PATH_TIME_OR_YAW_REDESIGN';
elseif any(r7ok)
    cls='R7_IS_FIRST_C7_SAFE_HEADING_REGULAR_SOLUTION';
else
    classes=unique(A(A(:,22)>0,26));
    if isempty(classes)
        cls='NO_HEADING_REGULAR_SOLUTION_AND_NO_R7_SPEED_WITNESS';
    elseif all(classes==1)
        cls='R7_TRUE_FAILURE_ENTRY_TRANSITION';
    elseif all(classes==2)
        cls='R7_TRUE_FAILURE_UNSAFE_AFFINE';
    elseif all(classes==3)
        cls='R7_TRUE_FAILURE_INTER_TRANSITION';
    elseif all(classes==4)
        cls='R7_TRUE_FAILURE_EXIT_TRANSITION';
    else
        cls='R7_TRUE_FAILURE_MIXED_SEGMENTS';
    end
end
fprintf('classification                              = %s\n',cls);
fprintf('====================================================================================================\n');

S=struct();
S.schema='AF7_PRIMARY_HEADING_R7_MAP_R9';
S.rows=A;
S.compact_primary_selected=any(prodPrimary);
S.compact_primary_heading_regular=any(prodHeading);
S.R7_certified_heading_regular=any(r7ok);
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

function r=safe_ratio(a,b)
if isfinite(a)&&isfinite(b)&&b>0,r=a/b;else,r=NaN;end
end

function s=class_name(code)
switch round(code)
    case 1,s='ENTRY_C7_SPLIT';
    case 2,s='UNSAFE_AFFINE';
    case 3,s='INTER_C7_SPLIT';
    case 4,s='EXIT_REJOIN_C7_SPLIT';
    otherwise,s='NONE';
end
end
