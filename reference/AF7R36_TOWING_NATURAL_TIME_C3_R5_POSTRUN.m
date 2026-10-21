function S = AF7R36_TOWING_NATURAL_TIME_C3_R5_POSTRUN()
% Reads the existing R4C diagnostic ring after the R5 family replacement.

M=af7r36_single_core_mex(6);
assert(isnumeric(M)&&size(M,2)==46, ...
    '[TOWC3-R5] R4C diagnostic ring unavailable. Keep R4C installed.');
assert(~isempty(M),'[TOWC3-R5] Diagnostic ring is empty.');

t0last=M(end,2);
tol=128*eps(max(1,abs(t0last)));
A=M(abs(M(:,2)-t0last)<=tol,:);

fprintf('\n====================================================================================================\n');
fprintf(' AF7R36 TOWING NATURAL-TIME C3 R5 RESULT\n');
fprintf(' hard-certificate authority unchanged = 1\n');
fprintf('====================================================================================================\n');
fprintf('last-cluster t0 / tm                = %.12g / %.12g s\n',A(1,2),A(1,3));
fprintf('finite-sensing UAV-speed limit      = %.12g m/s\n\n',A(end,5));

fprintf(' seq      t1[s]       ok family                         env[m]     VQ/lim     qdot       A        J       sigma\n');
fprintf('------------------------------------------------------------------------------------------------------------------------\n');
for i=1:size(A,1)
    fam=family_name(A(i,8));
    if A(i,6)>0.5
        fprintf('%4.0f  %11.6f   1  %-28s %9.4f   %8.5f  %8.5f  %7.4f  %7.4f  %8.4f\n', ...
            A(i,1),A(i,4),fam,A(i,9),A(i,10)/A(i,5),A(i,15),A(i,12),A(i,13),A(i,14));
    else
        fprintf('%4.0f  %11.6f   0  %-28s      NaN        NaN       NaN      NaN      NaN       NaN\n', ...
            A(i,1),A(i,4),fam);
    end
end

fprintf('\n[BRANCH COUNTS]\n');
fprintf(' seq      t1[s]        primary c/s/r/g       towing-C3 c/s/r/g      fallback c/s/r/g\n');
for i=1:size(A,1)
    fprintf('%4.0f  %11.6f        %d/%d/%d/%d              %d/%d/%d/%d              %d/%d/%d/%d\n', ...
        A(i,1),A(i,4), ...
        round(A(i,17)),round(A(i,18)),round(A(i,19)),round(A(i,20)), ...
        round(A(i,21)),round(A(i,22)),round(A(i,23)),round(A(i,24)), ...
        round(A(i,25)),round(A(i,26)),round(A(i,27)),round(A(i,28)));
end

fprintf('\n[PRIMARY FINITE-WITNESS LOCATION]\n');
for i=1:size(A,1)
    if A(i,29)>0
        fprintf('t1=%11.6f | min %.5fx @ %.6f s %s | max %.5fx @ %.6f s %s\n', ...
            A(i,4),A(i,30)/A(i,5),A(i,31),class_name(A(i,33)), ...
            A(i,34)/A(i,5),A(i,35),class_name(A(i,37)));
    end
end

fprintf('\n[TOWING C3 FINITE-WITNESS LOCATION]\n');
for i=1:size(A,1)
    if A(i,38)>0
        fprintf('t1=%11.6f | min %.5fx @ %.6f s %s | max %.5fx @ %.6f s %s\n', ...
            A(i,4),safe_ratio(A(i,39),A(i,5)),A(i,40),class_name(A(i,42)), ...
            safe_ratio(A(i,43),A(i,5)),A(i,44),class_name(A(i,46)));
    elseif A(i,21)>0
        fprintf('t1=%11.6f | candidates=%d | no finite rejection witness recorded\n', ...
            A(i,4),round(A(i,21)));
    end
end

safeTow=(A(:,6)>0.5)&(round(A(:,8))==3);
safeFb=(A(:,6)>0.5)&(round(A(:,8))==2);

fprintf('\n[SUMMARY]\n');
fprintf('towing-C3 selected in any actual t1 trial = %d\n',any(safeTow));
fprintf('fallback selected in any actual t1 trial  = %d\n',any(safeFb));

if any(safeTow)
    I=find(safeTow);
    [~,k]=min(A(I,9));
    row=A(I(k),:);
    cls='TOWING_C3_CERTIFIED__COMPARE_QUALITY_AGAINST_FALLBACK';
    fprintf('best selected towing-C3 envelope           = %.9g m\n',row(9));
    fprintf('best selected towing-C3 VQ/limit           = %.9g\n',row(10)/row(5));
    fprintf('best selected towing-C3 qdot               = %.9g rad/s\n',row(15));
else
    if any(A(:,22)>0)
        cls='TOWING_C3_SPEED_PASSES_BUT_LATER_CERTIFICATE_FAILS';
    elseif any(A(:,21)>0)
        cls='TOWING_C3_STILL_FAILS_SPEED__NEXT_USE_CONSTRAINED_SHAPE_TIME_SEARCH';
    else
        cls='TOWING_C3_QP_GENERATION_FAILED';
    end
end
fprintf('classification                              = %s\n',cls);
fprintf('====================================================================================================\n');

S=struct();
S.schema='AF7_TOWING_NATURAL_TIME_C3_R5';
S.last_cluster=A;
S.speed_limit=A(end,5);
S.towing_selected=any(safeTow);
S.fallback_selected=any(safeFb);
S.classification=cls;
end

function r=safe_ratio(a,b)
if isfinite(a)&&isfinite(b)&&b>0, r=a/b; else, r=NaN; end
end

function s=family_name(code)
switch round(code)
    case 1, s='MINIMUM_DEVIATION_AFFINE';
    case 2, s='MINIMUM_SNAP_FALLBACK';
    case 3, s='TOWING_NATURAL_TIME_C3';
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
