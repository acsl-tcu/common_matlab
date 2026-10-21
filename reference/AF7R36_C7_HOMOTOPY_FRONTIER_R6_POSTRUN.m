function S = AF7R36_C7_HOMOTOPY_FRONTIER_R6_POSTRUN()
% Analyze R6 C7 homotopy diagnostic frontier.
% Production selection was not changed by R6.

H=af7r36_single_core_mex(8);
R=af7r36_single_core_mex(6);

assert(isnumeric(H)&&size(H,2)==21, ...
    '[C7FRONT-R6] Homotopy diagnostic ring missing/wrong schema.');
assert(~isempty(H),'[C7FRONT-R6] Homotopy diagnostic ring is empty.');
assert(isnumeric(R)&&size(R,2)==46, ...
    '[C7FRONT-R6] R4C reference ring missing/wrong schema.');

t0last=H(end,2);
tol=128*eps(max(1,abs(t0last)));
A=H(abs(H(:,2)-t0last)<=tol,:);
RR=R(abs(R(:,2)-t0last)<=tol,:);

fprintf('\n====================================================================================================\n');
fprintf(' AF7R36 C7 HOMOTOPY FRONTIER R6 RESULT\n');
fprintf(' production selection changed = 0 | hard safety changed = 0\n');
fprintf('====================================================================================================\n');

tvals=unique(A(:,4),'stable');
for it=1:numel(tvals)
    t1=tvals(it);
    B=A(abs(A(:,4)-t1)<=128*eps(max(1,abs(t1))),:);
    rr=RR(abs(RR(:,4)-t1)<=128*eps(max(1,abs(t1))),:);

    fprintf('\n[t1 = %.9f s]\n',t1);
    if ~isempty(rr) && rr(end,6)>0.5
        fprintf('production family/envelope/VQ/qdot = %s / %.6f m / %.6f / %.6f\n', ...
            family_name(rr(end,8)),rr(end,9),rr(end,10)/rr(end,5),rr(end,15));
        baseEnv=rr(end,9);
        baseVQ=rr(end,10);
        baseA=rr(end,12);
        baseJ=rr(end,13);
        baseSig=rr(end,14);
        baseQd=rr(end,15);
    else
        fprintf('production candidate at this t1 = none\n');
        baseEnv=NaN;baseVQ=NaN;baseA=NaN;baseJ=NaN;baseSig=NaN;baseQd=NaN;
    end

    eligible=(B(:,8)>0.5)&(B(:,9)>0.5)&(B(:,10)>0.5)&(B(:,11)>0.5)&(B(:,12)>0.5);
    fprintf('generated / safety+heading eligible = %d / %d\n',size(B,1),nnz(eligible));

    if any(eligible)
        E=B(eligible,:);
        [~,ie]=min(E(:,13));
        best=E(ie,:);
        fprintf('min-envelope eligible: alpha=%.4f sign=%d env=%.6f m VQ/lim=%.6f qdot=%.6f A=%.6f J=%.6f sigma=%.6f tangentLB=%.6g m/s\n', ...
            best(5),round(best(6)),best(13),best(19)/speed_limit_for_t1(RR,t1),best(18),best(15),best(16),best(17),best(20));

        if isfinite(baseEnv)
            dom=(best(19)<=baseVQ) && (best(18)<=baseQd) && ...
                (best(15)<=baseA) && (best(16)<=baseJ) && ...
                (best(17)>=baseSig) && (best(13)<baseEnv);
            fprintf('strict measured-quality dominance over production fallback = %d\n',dom);
        end

        % Print compact Pareto frontier over env, VQ, qdot, A, J with sigma reversed.
        keep=true(size(E,1),1);
        for i=1:size(E,1)
            for j=1:size(E,1)
                if i==j, continue; end
                noWorse = E(j,13)<=E(i,13) && E(j,19)<=E(i,19) && ...
                          E(j,18)<=E(i,18) && E(j,15)<=E(i,15) && ...
                          E(j,16)<=E(i,16) && E(j,17)>=E(i,17);
                strict = E(j,13)<E(i,13) || E(j,19)<E(i,19) || ...
                         E(j,18)<E(i,18) || E(j,15)<E(i,15) || ...
                         E(j,16)<E(i,16) || E(j,17)>E(i,17);
                if noWorse && strict
                    keep(i)=false;break;
                end
            end
        end
        P=E(keep,:);
        P=sortrows(P,[13 5 6]);
        fprintf('Pareto eligible rows: %d\n',size(P,1));
        fprintf(' alpha  sign    env[m]   VQ/lim    qdot       A         J       sigma    tangentLB\n');
        lim=speed_limit_for_t1(RR,t1);
        for k=1:size(P,1)
            fprintf(' %.4f   %2d   %8.4f  %8.5f  %8.5f  %8.5f  %8.5f  %8.5f  %.6g\n', ...
                P(k,5),round(P(k,6)),P(k,13),P(k,19)/lim,P(k,18),P(k,15),P(k,16),P(k,17),P(k,20));
        end
    end
end

% Overall decision.
eligible=(A(:,8)>0.5)&(A(:,9)>0.5)&(A(:,10)>0.5)&(A(:,11)>0.5)&(A(:,12)>0.5);
nonFallback=eligible & (A(:,5)<1-32*eps);
if any(nonFallback)
    cls='C7_SAFE_HEADING_REGULAR_HOMOTOPY_EXISTS__INTEGRATION_PROMISING';
else
    cls='NO_C7_SAFE_HEADING_REGULAR_INTERIOR_HOMOTOPY_ON_1_OVER_16_GRID';
end

fprintf('\n[OVERALL]\n');
fprintf('structural translation continuity              = C7\n');
fprintf('heading theorem condition checked continuously = ||v_xy|| > 0\n');
fprintf('interior alpha<1 hard-safe+heading candidate   = %d\n',any(nonFallback));
fprintf('classification                                = %s\n',cls);
fprintf('NOTE: alpha grid is diagnostic only; it never grants acceptance.\n');
fprintf('NOTE: runtime is diagnostic and must not be used for the 4 ms claim.\n');
fprintf('====================================================================================================\n');

S=struct();
S.schema='AF7_C7_HOMOTOPY_FRONTIER_R6';
S.rows=A;
S.reference_rows=RR;
S.translation_C7=true;
S.heading_condition='continuous ||v_xy|| > 0';
S.interior_eligible=any(nonFallback);
S.classification=cls;
end

function lim=speed_limit_for_t1(R,t1)
m=abs(R(:,4)-t1)<=128*eps(max(1,abs(t1)));
assert(any(m),'[C7FRONT-R6] Missing reference speed limit for t1.');
lim=R(find(m,1,'last'),5);
end

function s=family_name(code)
switch round(code)
    case 1, s='MINIMUM_DEVIATION_AFFINE';
    case 2, s='MINIMUM_SNAP_FALLBACK';
    case 3, s='TOWING_NATURAL_TIME_C3';
    otherwise, s='NONE/OTHER';
end
end
