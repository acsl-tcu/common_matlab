function S = AF7R36_HEADING_TOPOLOGY_R10_POSTRUN()
M=af7r36_single_core_mex(14);
assert(isnumeric(M)&&size(M,2)==21,'[HEADTOPO-R10] R10 ring unavailable/wrong schema.');
assert(~isempty(M),'[HEADTOPO-R10] R10 ring empty.');

t0last=M(end,2);
tol=128*eps(max(1,abs(t0last)));
A=M(abs(M(:,2)-t0last)<=tol,:);

fprintf('\n====================================================================================================\n');
fprintf(' AF7R36 HEADING SINGULARITY TOPOLOGY R10 RESULT\n');
fprintf(' production selection/safety/C7 unchanged = 1\n');
fprintf('====================================================================================================\n');
fprintf('last-cluster t0 / tm = %.12g / %.12g s\n\n',A(1,2),A(1,3));

fprintf(' t1[s]       family                       strict  location        vxy(t0)     vxy(t1)     fail_t       width        fail_vxy\n');
fprintf('-------------------------------------------------------------------------------------------------------------------------------\n');
for i=1:size(A,1)
    fprintf('%11.6f  %-27s   %d    %-13s  %10.4g  %10.4g  %11.6f  %10.3g  %10.3g\n', ...
        A(i,4),family_name(A(i,6)),round(A(i,7)),loc_name(A(i,8)), ...
        A(i,12),A(i,13),A(i,9),A(i,10),A(i,11));
end

fprintf('\n[SMALLEST UNRESOLVED LEAF DERIVATIVE NORMS]\n');
fprintf(' t1[s]        |v|         |a|         |j|         |d4|        |d5|        |d6|       unresolved/depthhit\n');
for i=1:size(A,1)
    fprintf('%11.6f  %10.3g  %10.3g  %10.3g  %10.3g  %10.3g  %10.3g     %d/%d\n', ...
        A(i,4),A(i,14),A(i,15),A(i,16),A(i,17),A(i,18),A(i,19),round(A(i,20)),round(A(i,21)));
end

loc=A(:,8);
strict=A(:,7)>0.5;
if any(strict)
    cls='STRICT_NONZERO_TEMPORAL_HEADING_ALREADY_HOLDS';
elseif all(ismember(round(loc),[1 2 4])) && ~any(round(loc)==3)
    cls='HEADING_SINGULARITY_ENDPOINT_ONLY__USE_PATH_TIME_STOP_HOLD_THEOREM';
elseif any(round(loc)==3)
    cls='HEADING_SINGULARITY_INTERIOR__PATH_TIME_SEPARATED_REDESIGN_REQUIRED';
else
    cls='HEADING_TOPOLOGY_UNRESOLVED';
end

fprintf('\n[DECISION]\n');
fprintf('strict temporal tangent condition anywhere  = %d\n',any(strict));
fprintf('any interior unresolved singularity         = %d\n',any(round(loc)==3));
fprintf('endpoint-only unresolved topology            = %d\n',all(ismember(round(loc),[1 2 4])) && ~any(round(loc)==3));
fprintf('classification                              = %s\n',cls);
fprintf('====================================================================================================\n');

S=struct();
S.schema='AF7_HEADING_TOPOLOGY_R10';
S.rows=A;
S.classification=cls;
S.any_interior=any(round(loc)==3);
S.endpoint_only=all(ismember(round(loc),[1 2 4])) && ~any(round(loc)==3);
end

function s=family_name(code)
switch round(code)
    case 1,s='MINIMUM_DEVIATION_AFFINE';
    case 2,s='MINIMUM_SNAP_FALLBACK';
    case 3,s='TOWING_C7_CONDENSED_C3';
    otherwise,s='NONE/FAILED';
end
end

function s=loc_name(code)
switch round(code)
    case 0,s='PASS';
    case 1,s='T0_ENDPOINT';
    case 2,s='T1_ENDPOINT';
    case 3,s='INTERIOR';
    case 4,s='BOTH_ENDPOINTS';
    otherwise,s='UNKNOWN';
end
end
