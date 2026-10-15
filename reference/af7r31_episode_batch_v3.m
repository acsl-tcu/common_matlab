function [EPI,ok] = af7r31_episode_batch_v3(replanner,P,CTX,searchEnd)
% Same R10A interval predicate, evaluated level-wise in batches.
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

episodes=EPI.episodes;
tolMerge=64*eps(max([1,abs(t0),abs(searchEnd),abs(dt)]));

for ii=1:M
    A=t0; B=double(searchEnd);
    unsafeA=zeros(0,1); unsafeB=zeros(0,1);

    while ~isempty(A)
        A=double(A(:)); B=double(B(:));
        vAB=isfinite(A)&isfinite(B)&B>A;
        A=A(vAB); B=B(vAB);
        if isempty(A), break; end

        tc=0.5*(A+B); half=0.5*(B-A);
        splittable=isfinite(tc)&tc>A&tc<B;

        [Y,vflat]=af7r31_flat_jet_batch_v3(replanner,tc.');
        pL=Y(1:3,:); aL=Y(7:9,:);
        sf=aL+[zeros(2,numel(tc));double(LIV.gravity)*ones(1,numel(tc))];
        ns=sqrt(sum(sf.^2,1));
        good=vflat & isfinite(ns) & ns>0;

        q=zeros(3,numel(tc));
        q(:,good)=-sf(:,good)./ns(good);
        pQ=pL-double(LIV.cable_length)*q;

        c=double(O.centers(:,ii));
        R=reshape(double(O.rotations_flat(:,ii)),3,3);
        ax=double(O.base_axes(:,ii));

        safe=false(1,numel(tc));
        if any(good)
            sg=af7r31_geometry_batch_mex(0,pL(:,good),pQ(:,good), ...
                half(good).',c,R,ax,rhoL0,rhoQ0,rhoC0,VL,VQ);
            safe(good)=logical(sg);
        end

        EPI.adaptive_leaf_count=EPI.adaptive_leaf_count+numel(tc);
        EPI.adaptive_safe_leaf_count=EPI.adaptive_safe_leaf_count+nnz(safe);

        unresolved=~safe;
        if ~any(unresolved), A=[]; B=[]; continue; end

        Au=A(unresolved); Bu=B(unresolved); tmu=tc(unresolved);
        splitu=splittable(unresolved); width=Bu-Au;
        terminal=(width<=dt+tolMerge)|~splitu;

        if any(terminal)
            unsafeA=[unsafeA;Au(terminal)]; %#ok<AGROW>
            unsafeB=[unsafeB;Bu(terminal)]; %#ok<AGROW>
            EPI.adaptive_unsafe_leaf_count=EPI.adaptive_unsafe_leaf_count+nnz(terminal);
            EPI.representability_fail_closed_count= ...
                EPI.representability_fail_closed_count+nnz(terminal&~splitu);
        end

        keep=~terminal;
        if any(keep)
            ak=Au(keep); bk=Bu(keep); mk=tmu(keep);
            A=[ak;mk]; B=[mk;bk];
            EPI.adaptive_split_count=EPI.adaptive_split_count+numel(ak);
        else
            A=[]; B=[];
        end
    end

    if isempty(unsafeA), continue; end
    [unsafeA,ord]=sort(unsafeA); unsafeB=unsafeB(ord);
    es=unsafeA(1); ee=unsafeB(1);
    for ku=2:numel(unsafeA)
        if unsafeA(ku)<=ee+tolMerge
            ee=max(ee,unsafeB(ku));
        else
            episodes(end+1)=struct('obstacle_index',ii, ... %#ok<AGROW>
                'obstacle_id',double(O.ids(ii)),'start_s',double(es), ...
                'end_s',double(ee),'component_index',0);
            es=unsafeA(ku); ee=unsafeB(ku);
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
