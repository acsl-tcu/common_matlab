function [EPI,ok] = af7r33_episode_batch_allobs(replanner,P,CTX,searchEnd)
% Same R10A theorem, but all active obstacle intervals at each binary level
% share one active-reference batch evaluation and one paired-obstacle MEX call.

LIV=CTX.live; O=CTX.obstacles;
dt=double(LIV.dt); t0=double(P.t0); M=double(O.count);

EPI=struct();
EPI.certified=false; EPI.continuous_time=true; EPI.sampled_acceptance=false;
EPI.t0=t0; EPI.search_end_s=double(searchEnd); EPI.obstacle_count=M;
EPI.episodes=struct('obstacle_index',{},'obstacle_id',{},'start_s',{},'end_s',{},'component_index',{});
EPI.components=struct('start_s',{},'end_s',{},'duration_s',{},'episode_indices',{});
EPI.episode_count=0; EPI.component_count=0;
EPI.adaptive_leaf_count=0; EPI.adaptive_safe_leaf_count=0;
EPI.adaptive_unsafe_leaf_count=0; EPI.adaptive_split_count=0;
EPI.representability_fail_closed_count=0;
EPI.theorem=['ADAPTIVE_CONTINUOUS_SUPPORT_INTERVAL_COVER__' ...
    'SAFE_ONLY_BY_WHOLE_LEAF_CERTIFICATE__UNRESOLVED_LEQ_LIVE_DT_IS_UNSAFE'];
ok=false;

if ~(isfinite(t0)&&isfinite(searchEnd)&&searchEnd>t0&&isfinite(dt)&&dt>0&&M>=1), return; end
SC=replanner.active_speed_certificate;
replanner.validate_active_speed_certificate(SC);
VL=double(SC.payload_speed_UB_m_s); VQ=double(SC.uav_speed_UB_m_s);
track=0;
if isfield(P,'execution_tracking_inflation_m')&&isfinite(double(P.execution_tracking_inflation_m))
    track=max(0,double(P.execution_tracking_inflation_m));
end
rhoL0=double(LIV.r_load)+double(LIV.d_margin)+track;
rhoQ0=double(LIV.r_drone)+double(LIV.d_margin)+track;
rhoC0=double(LIV.r_cable)+double(LIV.d_margin)+track;
tolMerge=64*eps(max([1,abs(t0),abs(searchEnd),abs(dt)]));

A=repmat(t0,M,1);
B=repmat(double(searchEnd),M,1);
OI=(1:M).';
unsafeA=zeros(0,1); unsafeB=zeros(0,1); unsafeO=zeros(0,1);

while ~isempty(A)
    vAB=isfinite(A)&isfinite(B)&B>A;
    A=A(vAB); B=B(vAB); OI=OI(vAB);
    if isempty(A), break; end

    tc=0.5*(A+B); half=0.5*(B-A);
    splittable=isfinite(tc)&tc>A&tc<B;

    [Y,vflat]=af7r33_active_flat_jet_batch(replanner,tc.');
    pL=Y(1:3,:); aL=Y(7:9,:);
    sf=aL+[zeros(2,numel(tc));double(LIV.gravity)*ones(1,numel(tc))];
    ns=sqrt(sum(sf.^2,1));
    good=vflat & isfinite(ns) & ns>0;
    q=zeros(3,numel(tc)); q(:,good)=-sf(:,good)./ns(good);
    pQ=pL-double(LIV.cable_length)*q;

    safe=false(1,numel(tc));
    if any(good)
        safeGood=af7r33_episode_pair_mex( ...
            pL(:,good),pQ(:,good),half(good).', ...
            double(O.centers),double(O.rotations_flat),double(O.base_axes), ...
            double(OI(good)).',rhoL0,rhoQ0,rhoC0,VL,VQ);
        safe(good)=logical(safeGood);
    end

    EPI.adaptive_leaf_count=EPI.adaptive_leaf_count+numel(tc);
    EPI.adaptive_safe_leaf_count=EPI.adaptive_safe_leaf_count+nnz(safe);

    unresolved=~safe;
    if ~any(unresolved), A=[]; B=[]; OI=[]; continue; end

    Au=A(unresolved); Bu=B(unresolved); Ou=OI(unresolved);
    tmu=tc(unresolved); splitu=splittable(unresolved);
    width=Bu-Au;
    terminal=(width<=dt+tolMerge)|~splitu;

    if any(terminal)
        unsafeA=[unsafeA;Au(terminal)]; %#ok<AGROW>
        unsafeB=[unsafeB;Bu(terminal)]; %#ok<AGROW>
        unsafeO=[unsafeO;Ou(terminal)]; %#ok<AGROW>
        EPI.adaptive_unsafe_leaf_count=EPI.adaptive_unsafe_leaf_count+nnz(terminal);
        EPI.representability_fail_closed_count= ...
            EPI.representability_fail_closed_count+nnz(terminal&~splitu);
    end

    keep=~terminal;
    if any(keep)
        ak=Au(keep); bk=Bu(keep); mk=tmu(keep); okobs=Ou(keep);
        A=[ak;mk]; B=[mk;bk]; OI=[okobs;okobs];
        EPI.adaptive_split_count=EPI.adaptive_split_count+numel(ak);
    else
        A=[]; B=[]; OI=[];
    end
end

episodes=EPI.episodes;
for ii=1:M
    idx=find(unsafeO==ii);
    if isempty(idx), continue; end
    ua=unsafeA(idx); ub=unsafeB(idx);
    [ua,ord]=sort(ua); ub=ub(ord);
    es=ua(1); ee=ub(1);
    for k=2:numel(ua)
        if ua(k)<=ee+tolMerge
            ee=max(ee,ub(k));
        else
            episodes(end+1)=struct('obstacle_index',ii, ... %#ok<AGROW>
                'obstacle_id',double(O.ids(ii)),'start_s',double(es), ...
                'end_s',double(ee),'component_index',0);
            es=ua(k); ee=ub(k);
        end
    end
    episodes(end+1)=struct('obstacle_index',ii, ... %#ok<AGROW>
        'obstacle_id',double(O.ids(ii)),'start_s',double(es), ...
        'end_s',double(ee),'component_index',0);
end

if isempty(episodes)
    EPI.episodes=episodes; EPI.certified=true; ok=true; return;
end

[~,ord]=sort([episodes.start_s]); episodes=episodes(ord);
tolT=64*eps(max([1,abs(t0),abs(searchEnd)]));
comps=EPI.components; cc=0;
for ie=1:numel(episodes)
    es=double(episodes(ie).start_s); ee=double(episodes(ie).end_s);
    if cc==0||es>double(comps(cc).end_s)+tolT
        cc=cc+1;
        comps(cc)=struct('start_s',es,'end_s',ee, ... %#ok<AGROW>
            'duration_s',ee-es,'episode_indices',ie);
    else
        comps(cc).end_s=max(double(comps(cc).end_s),ee);
        comps(cc).duration_s=double(comps(cc).end_s)-double(comps(cc).start_s);
        comps(cc).episode_indices(end+1)=ie;
    end
    episodes(ie).component_index=cc;
end

EPI.episodes=episodes; EPI.components=comps;
EPI.episode_count=numel(episodes); EPI.component_count=numel(comps);
EPI.certified=true; ok=true;
end
