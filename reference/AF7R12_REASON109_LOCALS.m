function OUT = AF7R12_REASON109_LOCALS()
% Run while stopped at G.reason_code=109 inside
% build_static_continuous_wholebody_shadow_certificate.

vars={'ta','tb','dep','tmCell','halfWidth','VL','VQ','VC', ...
      'trackingInfl','pL','pQ','O','LIV','P','G','cellPass', ...
      'leafCount','acceptCount','splitCount','maxDepth','maxLeaves', ...
      'resourceExhausted','obj'};
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
fprintf(' AF7R12 | REASON-109 FAILED-LEAF THEORY DIAGNOSTIC\n');
fprintf('====================================================================================================\n');

ta=sc(S.ta); tb=sc(S.tb); hw=sc(S.halfWidth); dep=sc(S.dep);
tm=sc(S.tmCell); VL=sc(S.VL); VQ=sc(S.VQ); VC=sc(S.VC);
fprintf(' failed leaf [ta,tb] [s]                = [%.12g, %.12g]\n',ta,tb);
fprintf(' failed leaf center / halfwidth [s]     = %.12g / %.12g\n',tm,hw);
fprintf(' depth / maxDepth                       = %.0f / %.0f\n',dep,sc(S.maxDepth));
fprintf(' leaf / accepted / splits               = %.0f / %.0f / %.0f\n', ...
    sc(S.leafCount),sc(S.acceptCount),sc(S.splitCount));

branch='UNKNOWN';
try
    if tm<=double(S.P.tm)
        branch='RISE';
    else
        branch='FALL';
    end
catch
end
fprintf(' branch                                  = %s\n',branch);

fprintf('\n[A. TEMPORAL TUBE USED BY CURRENT THEOREM]\n');
fprintf(' VL / VQ / VC [m/s]                     = %.12g / %.12g / %.12g\n',VL,VQ,VC);
fprintf(' temporal inflation L/Q/C [m]           = %.12g / %.12g / %.12g\n', ...
    VL*hw,VQ*hw,VC*hw);

track=sc(S.trackingInfl);
LIV=S.LIV;
rhoBaseL=double(LIV.r_load)+double(LIV.d_margin)+track;
rhoBaseQ=double(LIV.r_drone)+double(LIV.d_margin)+track;
rhoBaseC=double(LIV.r_cable)+double(LIV.d_margin)+track;

% Compute best support margins WITHOUT temporal inflation, using the same
% finite normal family as the production certificate.
O=S.O; pL=double(S.pL(:)); pQ=double(S.pQ(:));
bestL=inf; bestQ=inf; bestC=inf;
worstObsL=NaN; worstObsQ=NaN; worstObsC=NaN;

for ii=1:double(O.count)
    c=double(O.centers(:,ii));
    R=reshape(double(O.rotations_flat(:,ii)),3,3);
    a=double(O.base_axes(:,ii));

    mL=best_point_margin(pL,c,R,a,rhoBaseL);
    mQ=best_point_margin(pQ,c,R,a,rhoBaseQ);
    mC=best_segment_margin(pQ,pL,c,R,a,rhoBaseC);

    if mL<bestL, bestL=mL; worstObsL=ii; end
    if mQ<bestQ, bestQ=mQ; worstObsQ=ii; end
    if mC<bestC, bestC=mC; worstObsC=ii; end
end

fprintf('\n[B. MIDPOINT STATIC SUPPORT MARGIN BEFORE TIME INFLATION]\n');
fprintf(' payload margin / obstacle [m]          = %.12g / %.0f\n',bestL,worstObsL);
fprintf(' UAV margin / obstacle [m]              = %.12g / %.0f\n',bestQ,worstObsQ);
fprintf(' cable margin / obstacle [m]            = %.12g / %.0f\n',bestC,worstObsC);

certL=bestL-VL*hw;
certQ=bestQ-VQ*hw;
certC=bestC-VC*hw;
fprintf('\n[C. SAME SUPPORT MARGIN AFTER TIME INFLATION]\n');
fprintf(' payload / UAV / cable [m]              = %.12g / %.12g / %.12g\n', ...
    certL,certQ,certC);
fprintf(' predicted cell pass                    = %d\n',min([certL certQ certC])>0);

rL=safe_radius(bestL,VL);
rQ=safe_radius(bestQ,VQ);
rC=safe_radius(bestC,VC);
rAll=min([rL rQ rC]);
fprintf('\n[D. THEOREM-DERIVED LOCAL CERTIFIABLE TIME RADIUS]\n');
fprintf(' payload radius [s]                     = %.12g\n',rL);
fprintf(' UAV radius [s]                         = %.12g\n',rQ);
fprintf(' cable radius [s]                       = %.12g\n',rC);
fprintf(' common radius [s]                      = %.12g\n',rAll);
fprintf(' current halfwidth / common radius      = %.12g\n',hw/max(rAll,realmin));

extraDyadic=NaN;
if isfinite(rAll)&&rAll>0&&hw>rAll
    extraDyadic=ceil(log2(hw/rAll));
elseif isfinite(rAll)&&rAll>0
    extraDyadic=0;
end
fprintf(' additional dyadic levels mathematically sufficient for THIS midpoint normal = %.0f\n',extraDyadic);
fprintf([' NOTE: this is a diagnostic theorem consequence, NOT a recommendation\n' ...
         '       to hard-code a larger maxDepth.\n']);

fprintf('\n[E. NOMINAL-SWEEP vs REPAIRED-SWEEP SELF-CONSISTENCY]\n');
activeSweep=struct();
try
    activeSweep=S.obj.trigger_sweep_inflation;
catch
end
dt=double(LIV.dt);
repL=VL*dt/2;
repQ=VQ*dt/2;
repC=VC*dt/2;

nomL=fld(activeSweep,'payload_m');
nomQ=fld(activeSweep,'uav_m');
nomC=fld(activeSweep,'cable_m');

fprintf(' nominal proposal sweep L/Q/C [m]       = %.12g / %.12g / %.12g\n',nomL,nomQ,nomC);
fprintf(' repaired certificate sweep L/Q/C [m]  = %.12g / %.12g / %.12g\n',repL,repQ,repC);
fprintf(' repaired - nominal L/Q/C [m]           = %.12g / %.12g / %.12g\n', ...
    repL-nomL,repQ-nomQ,repC-nomC);

selfGap=max([repL-nomL,repQ-nomQ,repC-nomC]);
fprintf(' positive self-consistency gap          = %.12g\n',max(0,selfGap));

fprintf('\n[F. DIAGNOSIS]\n');
if min([bestL bestQ bestC])<=0
    CLASS='MIDPOINT_STATIC_SUPPORT_NOT_SEPARATED_WITH_CURRENT_NORMAL_FAMILY';
elseif min([certL certQ certC])<=0
    CLASS='STATIC_SEPARATION_POSITIVE__TEMPORAL_TUBE_CAUSES_INCONCLUSIVE_LEAF';
else
    CLASS='LOCAL_RECOMPUTATION_PASSES__SOURCE_STATE_MISMATCH_REVIEW';
end
fprintf(' failed-leaf classification             = %s\n',CLASS);

if selfGap>0
    fprintf([' generator/certificate consistency       = OPEN\n' ...
             ' reason                                  = proposal displacement is solved against nominal-speed sweep,\n' ...
             '                                           but certificate uses repaired-speed sweep.\n']);
else
    fprintf(' generator/certificate consistency       = no positive sweep gap detected\n');
end

fprintf('\n[G. THEORY, NOT TUNING]\n');
fprintf([' Current leaf theorem is:\n' ...
         '   m_mid(n) - V*h > 0  => entire leaf safe.\n\n' ...
         ' Therefore a positive static midpoint margin m_mid gives the\n' ...
         ' theorem-derived admissible halfwidth h < m_mid/V.\n' ...
         ' A fixed maxDepth is not part of this theorem and should not be\n' ...
         ' used as the design authority.\n\n' ...
         ' The proposal itself must also be self-consistent:\n' ...
         '   alpha >= Gamma(V_repaired(alpha,Tup,Tdown)).\n' ...
         ' This is the geometric fixed-point that replaces a hand-tuned\n' ...
         ' clearance multiplier.\n']);

OUT=S;
OUT.static_margin_payload=bestL;
OUT.static_margin_uav=bestQ;
OUT.static_margin_cable=bestC;
OUT.time_inflated_margin_payload=certL;
OUT.time_inflated_margin_uav=certQ;
OUT.time_inflated_margin_cable=certC;
OUT.common_certifiable_time_radius=rAll;
OUT.extra_dyadic_levels_diagnostic=extraDyadic;
OUT.nominal_sweep=[nomL;nomQ;nomC];
OUT.repaired_sweep=[repL;repQ;repC];
OUT.self_consistency_gap=max(0,selfGap);
OUT.classification=CLASS;
assignin('base','AF7R12_REASON109_LOCALS_OUT',OUT);

fprintf('====================================================================================================\n');
end

function x=sc(v)
x=NaN;
try
    q=double(v);
    if isscalar(q)&&isfinite(q), x=q; end
catch
end
end

function x=fld(S,n)
x=NaN;
try
    if isstruct(S)&&isfield(S,n)
        x=sc(S.(n));
    end
catch
end
end

function r=safe_radius(m,V)
if ~(isfinite(m)&&isfinite(V)) || m<=0
    r=0;
elseif V==0
    r=inf;
elseif V>0
    r=m/V;
else
    r=0;
end
end

function m=best_point_margin(x,c,R,a,rho)
y=R.'*(x-c);
N=[R*(y./(a.^2)), (x-c)];
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
% Minimize ellipsoid quadratic metric along segment analytically.
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
