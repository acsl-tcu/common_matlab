function S = AF7R36_HEADING_TOPOLOGY_R10B_POSTRUN()
M=af7r36_single_core_mex(16);
assert(isnumeric(M)&&size(M,2)==20,'[HEADTOPO-R10B] R10B ring unavailable/wrong schema.');
assert(~isempty(M),'[HEADTOPO-R10B] R10B ring empty.');

t0last=M(end,2);
tol=128*eps(max(1,abs(t0last)));
A=M(abs(M(:,2)-t0last)<=tol,:);

fprintf('\n====================================================================================================\n');
fprintf(' AF7R36 HEADING TOPOLOGY R10B RESULT\n');
fprintf(' production selection/safety/C7 unchanged = 1 | exhaustive search = 0\n');
fprintf('====================================================================================================\n');
fprintf('last-cluster t0 / tm = %.12g / %.12g s\n\n',A(1,2),A(1,3));

fprintf(' t1[s]       family                       class                    v0xy       v1xy       a0xy       a1xy       J\n');
fprintf('--------------------------------------------------------------------------------------------------------------------------\n');
for i=1:size(A,1)
    fprintf('%11.6f  %-27s  %-24s %9.4g  %9.4g  %9.4g  %9.4g  %9.4g\n', ...
        A(i,4),family_name(A(i,6)),class_name(A(i,7)),A(i,12),A(i,13),A(i,14),A(i,15),A(i,16));
end

fprintf('\n[ENDPOINT THEOREM / CORE]\n');
fprintf(' t1[s]        left/right proven   deltaL[s]   deltaR[s]    core0       core1       core-pass\n');
for i=1:size(A,1)
    fprintf('%11.6f          %d/%d          %9.4g  %9.4g  %10.6f  %10.6f      %d\n', ...
        A(i,4),round(A(i,10)),round(A(i,11)),A(i,17),A(i,18),A(i,19),A(i,20),round(A(i,9)));
end

cls=round(A(:,7));
if any(cls==3)
    decision='INTERIOR_OR_CORE_HEADING_SINGULARITY__PATH_TIME_SEPARATED_REDESIGN_REQUIRED';
elseif any(cls==4)
    decision='ENDPOINT_DEPARTURE_NOT_PROVEN__NEED_HIGHER_ORDER_ENDPOINT_ANALYSIS';
elseif all(ismember(cls,[1 2]))
    if any(cls==2)
        decision='ENDPOINT_ONLY_PROVEN__BUILD_C7_PATH_PLUS_C6_STOP_HOLD_TIME_LAW';
    else
        decision='STRICT_TEMPORAL_TANGENT_ALREADY_PROVEN';
    end
else
    decision='HEADING_TOPOLOGY_R10B_INCONCLUSIVE';
end

fprintf('\n[DECISION]\n');
fprintf('any interior/core failure                   = %d\n',any(cls==3));
fprintf('any endpoint theorem unresolved             = %d\n',any(cls==4));
fprintf('endpoint-only proven present                = %d\n',any(cls==2));
fprintf('classification                              = %s\n',decision);
fprintf('====================================================================================================\n');

S=struct();
S.schema='AF7_HEADING_TOPOLOGY_R10B';
S.rows=A;
S.classification=decision;
S.any_interior_or_core_failure=any(cls==3);
S.any_endpoint_open=any(cls==4);
S.endpoint_only_proven=any(cls==2);
end

function s=family_name(code)
switch round(code)
    case 1,s='MINIMUM_DEVIATION_AFFINE';
    case 2,s='MINIMUM_SNAP_FALLBACK';
    case 3,s='TOWING_C7_CONDENSED_C3';
    otherwise,s='NONE/FAILED';
end
end

function s=class_name(code)
switch round(code)
    case 1,s='PASS_STRICT';
    case 2,s='ENDPOINT_ONLY_PROVEN';
    case 3,s='INTERIOR_OR_CORE_FAILURE';
    case 4,s='ENDPOINT_DEPARTURE_OPEN';
    otherwise,s='UNKNOWN';
end
end
