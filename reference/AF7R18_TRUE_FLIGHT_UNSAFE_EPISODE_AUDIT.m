function OUT=AF7R18_TRUE_FLIGHT_UNSAFE_EPISODE_AUDIT()
% AF7R18_TRUE_FLIGHT_UNSAFE_EPISODE_AUDIT
% Run at the current K>> stop inside builtin_bezier_d15_c7_proposal
% (reason 20, true Flight request). Read-only diagnostic.
%
% Purpose:
%   Extract theorem-backed continuous nominal unsafe episodes PER obstacle.
%   This determines whether one constant translation w is structurally
%   over-constraining temporally separated obstacles.
%
% No sampled acceptance:
%   Every dt-cell is classified by midpoint support separation with
%   certified speed * halfwidth inflation.

obj=evalin('caller','obj');
P=evalin('caller','OUT');
CTX=evalin('caller','CTX');
holdEnd=evalin('caller','holdEnd');

assert(logical(obj.online_prime_done) && ...
       logical(obj.setup_exact_do_prime_done) && ...
       logical(obj.generator_integration_prime_done) && ...
       logical(obj.generator_backend_kernel_prime_done) && ...
       logical(obj.trigger_event_capture_prime_done) && ...
       double(obj.trigger_event_count)>=1, ...
       '[AF7R18] not a post-prime real Flight request.');

O=CTX.obstacles;
LIV=CTX.live;
SC=obj.active_speed_certificate;

dt=double(LIV.dt);
t0=double(P.t0);
te=double(holdEnd);
M=double(O.count);

VL=double(SC.payload_speed_UB_m_s);
VQ=double(SC.uav_speed_UB_m_s);
VC=max(VL,VQ);

fprintf('\n====================================================================================================\n');
fprintf(' AF7R18 | TRUE-FLIGHT CONTINUOUS UNSAFE-EPISODE AUDIT\n');
fprintf('====================================================================================================\n');
fprintf(' t0 / trigger collision / holdEnd [s]   = %.12g / %.12g / %.12g\n', ...
    t0,double(P.tm),te);
fprintf(' obstacle count                          = %d\n',M);
fprintf(' dt                                      = %.12g s\n',dt);
fprintf(' nominal VL / VQ [m/s]                  = %.12g / %.12g\n',VL,VQ);
fprintf('[AUTHORITY] continuous cell support=1 | sampled acceptance=0 | source modification=0\n');

EP=cell(M,1);
allBounds=[t0;te];

for ii=1:M
    unsafeStarts=[];
    unsafeEnds=[];
    inUnsafe=false;
    curStart=NaN;

    ta=t0;
    while ta<te-8*eps(max(1,te))
        tb=min(te,ta+dt);
        tf=cell_safe_one_obstacle(P,CTX,ii,ta,tb,VL,VQ,VC);
        if ~tf && ~inUnsafe
            inUnsafe=true;
            curStart=ta;
        elseif tf && inUnsafe
            unsafeStarts(end+1,1)=curStart; %#ok<AGROW>
            unsafeEnds(end+1,1)=ta; %#ok<AGROW>
            inUnsafe=false;
            curStart=NaN;
        end
        ta=tb;
    end
    if inUnsafe
        unsafeStarts(end+1,1)=curStart;
        unsafeEnds(end+1,1)=te;
    end

    EP{ii}=struct('start',unsafeStarts,'end',unsafeEnds);
    allBounds=[allBounds;unsafeStarts;unsafeEnds]; %#ok<AGROW>

    fprintf('\n[Obstacle %d | id=%g]\n',ii,double(O.ids(ii)));
    if isempty(unsafeStarts)
        fprintf(' continuous unsafe episodes              = NONE\n');
    else
        fprintf(' continuous unsafe episode count         = %d\n',numel(unsafeStarts));
        for ee=1:numel(unsafeStarts)
            fprintf('   episode %d                            = [%.12g, %.12g] s | duration %.12g s\n', ...
                ee,unsafeStarts(ee),unsafeEnds(ee),unsafeEnds(ee)-unsafeStarts(ee));
        end
    end
end

% Build event intervals from all episode boundaries.
tolT=64*eps(max(1,max(abs(allBounds))));
allBounds=sort(allBounds(:));
B=allBounds(1);
for k=2:numel(allBounds)
    if abs(allBounds(k)-B(end))>tolT
        B(end+1,1)=allBounds(k); %#ok<AGROW>
    end
end

fprintf('\n[A. EVENT INTERVALS / ACTIVE UNSAFE EPISODES]\n');
INT=struct('ta',{},'tb',{},'active_obstacles',{});
for k=1:numel(B)-1
    ta=B(k); tb=B(k+1);
    if ~(tb>ta), continue; end
    tc=0.5*(ta+tb);
    act=false(M,1);
    for ii=1:M
        E=EP{ii};
        for ee=1:numel(E.start)
            if tc>=E.start(ee)-tolT && tc<=E.end(ee)+tolT
                act(ii)=true;
                break;
            end
        end
    end
    INT(end+1)=struct('ta',ta,'tb',tb,'active_obstacles',find(act).'); %#ok<AGROW>
    fprintf(' %2d | [%.12g, %.12g] | active obs = [%s]\n', ...
        numel(INT),ta,tb,sprintf(' %d',find(act)));
end

% Connected overlap components of obstacle episodes.
% Two episodes belong to the same component when their time intervals overlap.
episodes=[];
for ii=1:M
    E=EP{ii};
    for ee=1:numel(E.start)
        episodes=[episodes; ii ee E.start(ee) E.end(ee)]; %#ok<AGROW>
    end
end

fprintf('\n[B. TEMPORAL OVERLAP COMPONENTS]\n');
COMP={};
if isempty(episodes)
    fprintf(' no unsafe episode component\n');
else
    N=size(episodes,1);
    seen=false(N,1);
    nc=0;
    for s=1:N
        if seen(s), continue; end
        nc=nc+1;
        queue=s;
        seen(s)=true;
        ids=[];
        while ~isempty(queue)
            q=queue(1); queue(1)=[];
            ids(end+1)=q; %#ok<AGROW>
            for j=1:N
                if seen(j), continue; end
                if intervals_overlap(episodes(q,3),episodes(q,4), ...
                                     episodes(j,3),episodes(j,4),tolT)
                    seen(j)=true;
                    queue(end+1)=j; %#ok<AGROW>
                end
            end
        end
        COMP{nc}=ids; %#ok<AGROW>
        tca=min(episodes(ids,3));
        tcb=max(episodes(ids,4));
        fprintf(' component %d | span [%.12g, %.12g] | episodes:',nc,tca,tcb);
        for q=ids
            fprintf(' (obs%d,e%d)',episodes(q,1),episodes(q,2));
        end
        fprintf('\n');
    end
end

% Single-translation flatness impossibility metric using the globally feasible
% R7 displacement already established by AF7R16R2.
try
    R16=evalin('base','AF7R16R2_REAL_FLIGHT_REASON20_OUT');
catch
    R16=[];
end

fprintf('\n[C. SINGLE-TRANSLATION FLATNESS CHECK]\n');
singleKnown=false;
if isstruct(R16) && isfield(R16,'global_qp_ok') && logical(R16.global_qp_ok)
    w=double(R16.global_w(:));
    nw=norm(w);
    Tup=double(P.tm)-t0;
    K2=210; % exact degree-15 endpoint-repeated C7 second-derivative Bernstein bound
    sigma0=double(SC.specific_force_norm_LB_m_s2);
    d2=nw*K2/Tup^2;
    wMaxStrict=sigma0*Tup^2/K2;
    fprintf(' global constant-translation ||w|| [m]  = %.12g\n',nw);
    fprintf(' rise duration Tup [s]                  = %.12g\n',Tup);
    fprintf(' exact K2                               = %.12g\n',K2);
    fprintf(' base specific-force LB [m/s^2]         = %.12g\n',sigma0);
    fprintf(' resulting correction accel UB [m/s^2] = %.12g\n',d2);
    fprintf(' strict max constant ||w|| [m]          = %.12g\n',wMaxStrict);
    fprintf(' single rise flatness feasible          = %d\n',d2<sigma0);
    singleKnown=true;
end

% Classification.
numEpisodes=size(episodes,1);
if numEpisodes==0
    cls='NO_NOMINAL_UNSAFE_EPISODE_FOUND__REVIEW_TRIGGER_GEOMETRY';
elseif numel(COMP)>1
    cls='TEMPORALLY_SEPARATED_UNSAFE_COMPONENTS__SINGLE_W_IS_STRUCTURALLY_OVERCONSTRAINED';
elseif singleKnown && isstruct(R16) && isfield(R16,'global_qp_ok') && ...
        logical(R16.global_qp_ok) && ...
        norm(double(R16.global_w(:)))*210/(double(P.tm)-t0)^2 >= ...
        double(SC.specific_force_norm_LB_m_s2)
    cls='ONE_OVERLAP_COMPONENT_BUT_SINGLE_C7_STOPPED_RISE_IS_FLATNESS_INFEASIBLE';
else
    cls='EPISODE_STRUCTURE_REQUIRES_PIECEWISE_C7_CORRIDOR_SYNTHESIS';
end

fprintf('\n[D. CLASSIFICATION]\n');
fprintf(' classification                         = %s\n',cls);
fprintf([' next generator structure               = piecewise degree-15 Bezier, C7 across knots,\n' ...
         '                                          episode-bound corridor constraints,\n' ...
         '                                          internal derivatives NOT forced to zero.\n']);
fprintf('====================================================================================================\n');

OUT=struct();
OUT.schema='AF7R18_TRUE_FLIGHT_UNSAFE_EPISODE_AUDIT_V1';
OUT.t0=t0;
OUT.tm=double(P.tm);
OUT.hold_end=te;
OUT.episodes=EP;
OUT.event_intervals=INT;
OUT.overlap_components=COMP;
OUT.classification=cls;
assignin('base','AF7R18_TRUE_FLIGHT_UNSAFE_EPISODE_OUT',OUT);
end

function tf=cell_safe_one_obstacle(P,CTX,ii,ta,tb,VL,VQ,VC)
LIV=CTX.live;
O=CTX.obstacles;
tc=0.5*(double(ta)+double(tb));
h=0.5*(double(tb)-double(ta));
tf=false;
if ~(isfinite(tc)&&isfinite(h)&&h>0), return; end

xd=double(P.base_flat_eval_fn(tc));
xd=xd(:);
if numel(xd)<11 || any(~isfinite(xd(1:11))), return; end

pL=xd(1:3);
sf=xd(9:11)+[0;0;double(LIV.gravity)];
ns=norm(sf);
if ~(isfinite(ns)&&ns>0), return; end
q=-sf/ns;
pQ=pL-double(LIV.cable_length)*q;

rhoL=double(LIV.r_load)+double(LIV.d_margin)+VL*h;
rhoQ=double(LIV.r_drone)+double(LIV.d_margin)+VQ*h;
rhoC=double(LIV.r_cable)+double(LIV.d_margin)+VC*h;

c=double(O.centers(:,ii));
R=reshape(double(O.rotations_flat(:,ii)),3,3);
a=double(O.base_axes(:,ii));

[~,okL]=best_point_margin(pL,c,R,a,rhoL);
[~,okQ]=best_point_margin(pQ,c,R,a,rhoQ);
[~,okC]=best_segment_margin(pQ,pL,c,R,a,rhoC);
tf=okL&&okQ&&okC;
end

function [m,ok]=best_point_margin(x,c,R,a,rho)
y=R.'*(x-c);
N=[R*(y./(a.^2)), x-c];
m=-inf; ok=false;
for k=1:size(N,2)
    [q,valid]=support_point(x,c,R,a,rho,N(:,k));
    if valid && q>m, m=q; end
end
ok=isfinite(m)&&m>0;
end

function [m,ok]=best_segment_margin(p0,p1,c,R,a,rho)
xstar=segment_closest_metric_point(p0,p1,c,R,a);
Y=[xstar,p0,p1];
N=zeros(3,0);
for k=1:size(Y,2)
    y=R.'*(Y(:,k)-c);
    N(:,end+1)=R*(y./(a.^2)); %#ok<AGROW>
    N(:,end+1)=Y(:,k)-c; %#ok<AGROW>
end
m=-inf; ok=false;
for k=1:size(N,2)
    [q,valid]=support_segment(p0,p1,c,R,a,rho,N(:,k));
    if valid && q>m, m=q; end
end
ok=isfinite(m)&&m>0;
end

function [m,ok]=support_point(x,c,R,a,rho,n)
nn=norm(n);
if ~(isfinite(nn)&&nn>0), m=-inf; ok=false; return; end
h=dot(n,c)+norm(a.*(R.'*n));
m=(dot(n,x)-h-rho*nn)/nn;
ok=isfinite(m);
end

function [m,ok]=support_segment(p0,p1,c,R,a,rho,n)
nn=norm(n);
if ~(isfinite(nn)&&nn>0), m=-inf; ok=false; return; end
h=dot(n,c)+norm(a.*(R.'*n));
m=(min(dot(n,p0),dot(n,p1))-h-rho*nn)/nn;
ok=isfinite(m);
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

function tf=intervals_overlap(a0,a1,b0,b1,tol)
tf=max(a0,b0) < min(a1,b1)-tol;
end
