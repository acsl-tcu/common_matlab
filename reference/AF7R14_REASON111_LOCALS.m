function OUT = AF7R14_REASON111_LOCALS()
% Run only while stopped at reason 111 inside
% build_static_continuous_wholebody_shadow_certificate.
%
% Purpose:
%   Distinguish the actual corrected geometry from the translation-only
%   geometry implicitly used when the current displacement w was selected.

vars={'ta','tb','dep','tmCell','halfWidth', ...
      'cellStaticL','cellStaticQ','cellStaticC', ...
      'pL','pQ','q','rhoL0','rhoQ0','rhoC0', ...
      'VL','VQ','VC','O','LIV','P','G'};
S=struct();
for k=1:numel(vars)
    try
        S.(vars{k})=evalin('caller',vars{k});
    catch
        S.(vars{k})=[];
    end
end

fprintf('\n');
fprintf('====================================================================================================\n');
fprintf(' AF7R14 | REASON-111 STATIC-GEOMETRY LOCAL DIAGNOSTIC\n');
fprintf('====================================================================================================\n');

ta=sc(S.ta); tb=sc(S.tb); tc=sc(S.tmCell); hw=sc(S.halfWidth);
fprintf(' failed leaf [ta,tb] [s]                = [%.12g, %.12g]\n',ta,tb);
fprintf(' failed midpoint / halfwidth [s]        = %.12g / %.12g\n',tc,hw);
fprintf(' depth                                  = %.0f\n',sc(S.dep));
fprintf(' actual static margins L/Q/C [m]        = %.12g / %.12g / %.12g\n', ...
    sc(S.cellStaticL),sc(S.cellStaticQ),sc(S.cellStaticC));

P=S.P;
LIV=S.LIV;
O=S.O;

w=double(P.displacement(:));
alpha=norm(w);
dir=w/max(alpha,realmin);
fprintf(' displacement alpha [m]                 = %.12g\n',alpha);
fprintf(' displacement direction                 = [% .12g % .12g % .12g]\n',dir);

% Actual corrected state already present in caller.
pLact=double(S.pL(:));
pQact=double(S.pQ(:));
qAct=double(S.q(:));

% Base flat reference at the same time.
xb=P.base_flat_eval_fn(double(tc));
xb=double(xb(:));
if numel(xb)<11
    error('[AF7R14] base flat reference too short.');
end
pLbase=xb(1:3);
aBase=xb(9:11);
g=double(LIV.gravity);
sfBase=aBase+[0;0;g];
qBase=-sfBase/norm(sfBase);
pQbase=pLbase-double(LIV.cable_length)*qBase;

% Position correction actually applied to the payload at this time.
dNow=pLact-pLbase;

% Translation-only geometry: this is what would happen if q did not change.
pLtrans=pLbase+dNow;
pQtrans=pQbase+dNow;

dq=qAct-qBase;
u=max(-1,min(1,dot(qAct,qBase)/(norm(qAct)*norm(qBase))));
qAngle=acos(u);

fprintf('\n[A. FLATNESS-COUPLING GAP]\n');
fprintf(' payload correction d(t) [m]            = [% .12g % .12g % .12g]\n',dNow);
fprintf(' q_base                                 = [% .12g % .12g % .12g]\n',qBase);
fprintf(' q_actual                               = [% .12g % .12g % .12g]\n',qAct);
fprintf(' q rotation angle [deg]                 = %.12g\n',qAngle*180/pi);
fprintf(' ||q_actual-q_base||                    = %.12g\n',norm(dq));
fprintf(' UAV translation-model error [m]        = %.12g\n',norm(pQact-pQtrans));
fprintf(' identity L*||dq|| [m]                  = %.12g\n',double(LIV.cable_length)*norm(dq));

% Recompute exact-support static margins for actual vs translation geometry.
[act,actObs]=wholebody_margins(pLact,pQact,O, ...
    sc(S.rhoL0),sc(S.rhoQ0),sc(S.rhoC0));
[trn,trnObs]=wholebody_margins(pLtrans,pQtrans,O, ...
    sc(S.rhoL0),sc(S.rhoQ0),sc(S.rhoC0));

fprintf('\n[B. ACTUAL vs TRANSLATION-ONLY STATIC SUPPORT]\n');
fprintf(' actual      L/Q/C margin [m]            = %.12g / %.12g / %.12g\n',act);
fprintf(' translation L/Q/C margin [m]            = %.12g / %.12g / %.12g\n',trn);
fprintf(' actual worst obstacles L/Q/C            = %.0f / %.0f / %.0f\n',actObs);
fprintf(' trans. worst obstacles L/Q/C            = %.0f / %.0f / %.0f\n',trnObs);
fprintf(' actual - translation L/Q/C [m]          = %.12g / %.12g / %.12g\n',act-trn);

[mAct,idxAct]=min(act);
[mTrn,idxTrn]=min(trn);
names={'PAYLOAD','UAV','CABLE'};
fprintf(' actual limiting component               = %s\n',names{idxAct});
fprintf(' actual limiting static margin [m]       = %.12g\n',mAct);
fprintf(' translation limiting component          = %s\n',names{idxTrn});
fprintf(' translation limiting static margin [m] = %.12g\n',mTrn);

fprintf('\n[C. FAILURE INTERPRETATION]\n');
if mTrn>0 && mAct<=0
    cls='TRANSLATION_MODEL_SAFE__ACTUAL_FLATNESS_GEOMETRY_UNSAFE';
elseif mTrn<=0 && mAct<=0
    cls='DISPLACEMENT_TOO_SMALL_EVEN_FOR_TRANSLATION_MODEL';
elseif mAct>0
    cls='LOCAL_RECOMPUTATION_POSITIVE__REVIEW_SOURCE_STATE';
else
    cls='MIXED_GEOMETRY_FAILURE';
end
fprintf(' classification                         = %s\n',cls);

fprintf('\n[D. MATHEMATICAL CONSEQUENCE]\n');
fprintf([' The next generator must solve displacement using the ACTUAL flatness map:\n' ...
         '   pL_alpha(t) = pL_0(t) + alpha*d*b(t)\n' ...
         '   q_alpha(t)  = -(a_0(t)+alpha*d*b_ddot(t)+g e3)/||...||\n' ...
         '   pQ_alpha(t) = pL_alpha(t)-L*q_alpha(t)\n' ...
         ' and certify whole-body support on that geometry.\n' ...
         ' A hand multiplier on alpha is not a proof.\n']);

OUT=S;
OUT.actual_margins=act;
OUT.translation_margins=trn;
OUT.actual_worst_obstacles=actObs;
OUT.translation_worst_obstacles=trnObs;
OUT.q_angle_deg=qAngle*180/pi;
OUT.uav_translation_model_error_m=norm(pQact-pQtrans);
OUT.classification=cls;
assignin('base','AF7R14_REASON111_LOCALS_OUT',OUT);

fprintf('====================================================================================================\n');
end

function [M,obs]=wholebody_margins(pL,pQ,O,rL,rQ,rC)
M=[inf inf inf];
obs=[NaN NaN NaN];
for ii=1:double(O.count)
    c=double(O.centers(:,ii));
    R=reshape(double(O.rotations_flat(:,ii)),3,3);
    a=double(O.base_axes(:,ii));

    mL=best_point_margin(pL,c,R,a,rL);
    mQ=best_point_margin(pQ,c,R,a,rQ);
    mC=best_segment_margin(pQ,pL,c,R,a,rC);

    if mL<M(1), M(1)=mL; obs(1)=ii; end
    if mQ<M(2), M(2)=mQ; obs(2)=ii; end
    if mC<M(3), M(3)=mC; obs(3)=ii; end
end
end

function m=best_point_margin(x,c,R,a,rho)
y=R.'*(x-c);
N=[R*(y./(a.^2)), x-c];
m=-inf;
for k=1:size(N,2)
    q=support_point(x,c,R,a,rho,N(:,k));
    if isfinite(q), m=max(m,q); end
end
end

function m=best_segment_margin(p0,p1,c,R,a,rho)
xstar=segment_closest_metric_point(p0,p1,c,R,a);
Y=[xstar,p0,p1];
N=zeros(3,0);
for k=1:size(Y,2)
    y=R.'*(Y(:,k)-c);
    N(:,end+1)=R*(y./(a.^2)); %#ok<AGROW>
    N(:,end+1)=Y(:,k)-c; %#ok<AGROW>
end
m=-inf;
for k=1:size(N,2)
    q=support_segment(p0,p1,c,R,a,rho,N(:,k));
    if isfinite(q), m=max(m,q); end
end
end

function m=support_point(x,c,R,a,rho,n)
n=n(:); nn=norm(n);
if ~(isfinite(nn)&&nn>0), m=-inf; return; end
h=dot(n,c)+norm(a.*(R.'*n));
m=(dot(n,x)-h-rho*nn)/nn;
end

function m=support_segment(p0,p1,c,R,a,rho,n)
n=n(:); nn=norm(n);
if ~(isfinite(nn)&&nn>0), m=-inf; return; end
h=dot(n,c)+norm(a.*(R.'*n));
m=(min(dot(n,p0),dot(n,p1))-h-rho*nn)/nn;
end

function xstar=segment_closest_metric_point(p0,p1,c,R,a)
d=p1-p0;
y0=R.'*(p0-c);
yd=R.'*d;
w=1./(a.^2);
A=sum(w.*(yd.^2));
B=sum(w.*y0.*yd);
if A<=0
    lam=0;
else
    lam=max(0,min(1,-B/A));
end
xstar=p0+lam*d;
end

function x=sc(v)
x=NaN;
try
    q=double(v);
    if isscalar(q)&&isfinite(q), x=q; end
catch
end
end
