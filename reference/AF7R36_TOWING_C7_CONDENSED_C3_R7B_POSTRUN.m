function S = AF7R36_TOWING_C7_CONDENSED_C3_R7B_POSTRUN()
% Evaluate actual adaptive t1 calls after R7.
% Family-3 selection implies ALL of:
%   structural translation C7,
%   speed certificate,
%   rejoin certificate,
%   continuous whole-body certificate,
%   continuous ||v_xy|| > 0 heading gate.

M=af7r36_single_core_mex(6);
assert(isnumeric(M)&&size(M,2)==46, ...
    '[TOWC7-R7B] R4C diagnostic ring unavailable.');
assert(~isempty(M),'[TOWC7-R7B] Diagnostic ring is empty.');

t0last=M(end,2);
tol=128*eps(max(1,abs(t0last)));
A=M(abs(M(:,2)-t0last)<=tol,:);

fprintf('\n====================================================================================================\n');
fprintf(' AF7R36 TOWING C7 CONDENSED C3 R7B RESULT\n');
fprintf(' translation C7 hard = 1 | safety hard = 1 | exact-heading regularity hard for family 3 = 1\n');
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
fprintf(' seq      t1[s]        primary c/s/r/g       R7 c/s/r/g             old-fallback c/s/r/g\n');
for i=1:size(A,1)
    fprintf('%4.0f  %11.6f        %d/%d/%d/%d              %d/%d/%d/%d              %d/%d/%d/%d\n', ...
        A(i,1),A(i,4), ...
        round(A(i,17)),round(A(i,18)),round(A(i,19)),round(A(i,20)), ...
        round(A(i,21)),round(A(i,22)),round(A(i,23)),round(A(i,24)), ...
        round(A(i,25)),round(A(i,26)),round(A(i,27)),round(A(i,28)));
end

fprintf('\n[R7 FINITE-WITNESS LOCATION WHEN SPEED FAILS]\n');
for i=1:size(A,1)
    if A(i,38)>0
        fprintf('t1=%11.6f | min %.5fx @ %.6f s %s | max %.5fx @ %.6f s %s\n', ...
            A(i,4),safe_ratio(A(i,39),A(i,5)),A(i,40),class_name(A(i,42)), ...
            safe_ratio(A(i,43),A(i,5)),A(i,44),class_name(A(i,46)));
    elseif A(i,21)>0
        fprintf('t1=%11.6f | candidates=%d | no finite speed rejection witness\n', ...
            A(i,4),round(A(i,21)));
    end
end

selR7=(A(:,6)>0.5)&(round(A(:,8))==3);
selOld=(A(:,6)>0.5)&(round(A(:,8))==2);

fprintf('\n[SUMMARY]\n');
fprintf('R7 selected in any actual t1 trial          = %d\n',any(selR7));
fprintf('old minimum-snap selected in any trial      = %d\n',any(selOld));

if any(selR7)
    I=find(selR7);
    [~,k]=min(A(I,9));
    row=A(I(k),:);
    fprintf('best R7 envelope                            = %.9g m\n',row(9));
    fprintf('best R7 VQ/limit                            = %.9g\n',row(10)/row(5));
    fprintf('best R7 qdot                                = %.9g rad/s\n',row(15));
    fprintf('best R7 accel / jerk                        = %.9g / %.9g\n',row(12),row(13));
    cls='R7_C7_SAFE_HEADING_REGULAR_CANDIDATE_EXISTS';
elseif any(A(:,22)>0)
    cls='R7_SPEED_PASSES_BUT_REJOIN_GEOMETRY_OR_HEADING_REJECTS';
elseif any(A(:,21)>0)
    cls='R7_STILL_SPEED_LIMITED';
else
    cls='R7_QP_GENERATION_FAILED';
end
fprintf('classification                              = %s\n',cls);
fprintf('====================================================================================================\n');

S=struct();
S.schema='AF7_TOWING_C7_CONDENSED_C3_R7B';
S.last_cluster=A;
S.speed_limit=A(end,5);
S.R7_selected=any(selR7);
S.old_fallback_selected=any(selOld);
S.classification=cls;
end

function r=safe_ratio(a,b)
if isfinite(a)&&isfinite(b)&&b>0,r=a/b;else,r=NaN;end
end

function s=family_name(code)
switch round(code)
    case 1, s='MINIMUM_DEVIATION_AFFINE';
    case 2, s='MINIMUM_SNAP_FALLBACK';
    case 3, s='TOWING_C7_CONDENSED_C3';
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
