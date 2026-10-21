function S = AF7R36_R7_SHADOW_AUTHORITY_R8_POSTRUN()
% Compare unchanged production result with independently forced R7 shadow.

P=af7r36_single_core_mex(6);
R=af7r36_single_core_mex(10);
assert(isnumeric(P)&&size(P,2)==46,'[R7SHADOW-R8] Production ring unavailable.');
assert(isnumeric(R)&&size(R,2)==29,'[R7SHADOW-R8] R7 shadow ring unavailable.');
assert(~isempty(P)&&~isempty(R),'[R7SHADOW-R8] Diagnostic rings are empty.');

t0last=R(end,2);
tol=128*eps(max(1,abs(t0last)));
A=R(abs(R(:,2)-t0last)<=tol,:);
B=P(abs(P(:,2)-t0last)<=tol,:);

fprintf('\n====================================================================================================\n');
fprintf(' AF7R36 R7 SHADOW AUTHORITY R8 RESULT\n');
fprintf(' production selection changed = 0 | R7 shadow forced independently = 1\n');
fprintf('====================================================================================================\n');
fprintf('last-cluster t0 / tm = %.12g / %.12g s\n\n',A(1,2),A(1,3));

fprintf(' t1[s]       production family              envP     VQP/lim | R7 ok reason c/s/r/g/h      envR     VQR/lim   qdotR\n');
fprintf('---------------------------------------------------------------------------------------------------------------------------------\n');
for i=1:size(A,1)
    t1=A(i,4);
    bp=find(abs(B(:,4)-t1)<=128*eps(max(1,abs(t1))),1,'last');
    if isempty(bp)
        pf='NO_MATCH';penv=NaN;putil=NaN;
    else
        pf=family_name(B(bp,8));
        if B(bp,6)>0.5,penv=B(bp,9);putil=B(bp,10)/B(bp,5);else,penv=NaN;putil=NaN;end
    end
    if A(i,6)>0.5
        renv=A(i,13);rutil=A(i,14)/A(i,5);rqd=A(i,19);
    else
        renv=NaN;rutil=NaN;rqd=NaN;
    end
    fprintf('%11.6f  %-28s %8.3f  %8.5f |  %d  %3.0f   %d/%d/%d/%d/%d      %8.3f  %8.5f  %8.5f\n', ...
        t1,pf,penv,putil,A(i,6)>0.5,A(i,7),round(A(i,8)),round(A(i,9)), ...
        round(A(i,10)),round(A(i,11)),round(A(i,12)),renv,rutil,rqd);
end

fprintf('\n[R7 FINITE SPEED WITNESS]\n');
for i=1:size(A,1)
    if A(i,21)>0
        fprintf('t1=%11.6f | count=%d | min %.6fx @ %.6f s %s | max %.6fx @ %.6f s %s\n', ...
            A(i,4),round(A(i,21)),safe_ratio(A(i,22),A(i,5)),A(i,23),class_name(A(i,25)), ...
            safe_ratio(A(i,26),A(i,5)),A(i,27),class_name(A(i,29)));
    end
end

ok=A(:,6)>0.5;
if any(ok)
    cls='R7_SHADOW_C7_SAFE_HEADING_REGULAR_EXISTS';
elseif any(A(:,8)>0 & A(:,9)>0 & A(:,10)>0 & A(:,11)>0 & A(:,12)==0)
    cls='R7_SHADOW_REACHES_WHOLEBODY_BUT_HEADING_REGULARITY_FAILS';
elseif any(A(:,8)>0 & A(:,9)>0 & A(:,10)>0 & A(:,11)==0)
    cls='R7_SHADOW_WHOLEBODY_FAILS';
elseif any(A(:,8)>0 & A(:,9)>0 & A(:,10)==0)
    cls='R7_SHADOW_REJOIN_FAILS';
elseif any(A(:,8)>0 & A(:,9)==0)
    cls='R7_SHADOW_SPEED_FAILS';
elseif any(A(:,8)==0)
    cls='R7_SHADOW_TRUE_QP_GENERATION_FAILURE';
else
    cls='R7_SHADOW_UNCLASSIFIED';
end

fprintf('\n[AUTHORITY]\n');
fprintf('R7 was actually evaluated in this run       = %d\n',size(A,1)>0);
fprintf('R7 hard-certified+heading candidate exists  = %d\n',any(ok));
fprintf('classification                              = %s\n',cls);

if any(ok)
    I=find(ok);
    [~,k]=min(A(I,13));r=A(I(k),:);
    fprintf('best R7 envelope                            = %.9g m\n',r(13));
    fprintf('best R7 VQ/limit                            = %.9g\n',r(14)/r(5));
    fprintf('best R7 qdot / accel / jerk                 = %.9g / %.9g / %.9g\n',r(19),r(16),r(17));
    fprintf('best R7 sigma                               = %.9g\n',r(18));
end

fprintf('NOTE: R7 shadow includes C7 structural construction and continuous heading regularity hard gate.\n');
fprintf('NOTE: runtime is diagnostic only and must not be used for the 4 ms claim.\n');
fprintf('====================================================================================================\n');

S=struct();
S.schema='AF7_R7_SHADOW_AUTHORITY_R8';
S.production=B;
S.r7_shadow=A;
S.R7_evaluated=true;
S.R7_certified_heading_regular=any(ok);
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
    case 1,s='ENTRY_C7';
    case 2,s='UNSAFE_AFFINE';
    case 3,s='INTER_C7';
    case 4,s='EXIT_REJOIN_C7';
    otherwise,s='NONE';
end
end
