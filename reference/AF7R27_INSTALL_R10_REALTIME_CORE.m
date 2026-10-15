function OUT = AF7R27_INSTALL_R10_REALTIME_CORE()
% AF7R27_INSTALL_R10_REALTIME_CORE
% =========================================================================
% R10 algorithmic real-time reduction for the R9 piecewise-C7 suspended-load
% replanner. Targets the measured 9.85--10.40 s retry burst.
%
% R10-A  Adaptive continuous unsafe-episode cover
% R10-B  Projected corridor upper-envelope row compression
% R10-C  Small-QP active-set/codegen solver path
%
% Safety policy is unchanged:
%   continuous-time acceptance only;
%   no fixed depth acceptance;
%   unresolved numerical cases fail closed;
%   independent continuous whole-body certificate remains final authority.
% =========================================================================

clc;
fprintf('\n');
fprintf('====================================================================================================\n');
fprintf(' AF7R27 | INSTALL R10 REAL-TIME CORE\n');
fprintf('====================================================================================================\n');

src=which('REPLANNING_SYSTEM');
assert(~isempty(src)&&isfile(src),'[AF7R27] REPLANNING_SYSTEM.m not found.');
raw=fileread(src);

assert(contains(raw,'AF7R21-R9'),'[AF7R27] R9 marker missing.');
assert(~contains(raw,'AF7R27-R10'),'[AF7R27] R10 already appears installed.');

stamp=char(datetime('now','Format','yyyyMMdd_HHmmss'));
backup=fullfile(fileparts(src),['REPLANNING_SYSTEM_pre_R10_' stamp '.m']);
copyfile(src,backup);

% =========================================================================
% R10-A
% =========================================================================
a0='        function [EPI,ok] = build_bezier_nominal_unsafe_episode_components( ...';
a1='        function tf = nominal_reference_cell_continuously_safe_one_obstacle( ...';
[i0,i1]=unique_bounds(raw,a0,a1,'R10-A episode function');

newEpisode = r'''
        function [EPI,ok] = build_bezier_nominal_unsafe_episode_components( ...
                obj,P,CTX,searchEnd)
            % =============================================================
            % AF7R27-R10A ADAPTIVE CONTINUOUS SUPPORT COVER
            %
            % A leaf is accepted SAFE only when the same continuous
            % midpoint+speed support theorem certifies payload/UAV/cable
            % throughout the whole leaf. Otherwise it is subdivided.
            %
            % There is no fixed depth authority. If an unresolved leaf
            % reaches width <= live dt, it is conservatively UNSAFE.
            % Binary64 non-splittability is also fail-closed UNSAFE.
            % =============================================================
            LIV=CTX.live;
            O=CTX.obstacles;
            dt=double(LIV.dt);
            t0=double(P.t0);
            M=double(O.count);

            EPI=struct();
            EPI.certified=false;
            EPI.continuous_time=true;
            EPI.sampled_acceptance=false;
            EPI.t0=t0;
            EPI.search_end_s=double(searchEnd);
            EPI.obstacle_count=M;
            EPI.episodes=struct('obstacle_index',{},'obstacle_id',{}, ...
                'start_s',{},'end_s',{},'component_index',{});
            EPI.components=struct('start_s',{},'end_s',{}, ...
                'duration_s',{},'episode_indices',{});
            EPI.episode_count=0;
            EPI.component_count=0;
            EPI.adaptive_leaf_count=0;
            EPI.adaptive_safe_leaf_count=0;
            EPI.adaptive_unsafe_leaf_count=0;
            EPI.adaptive_split_count=0;
            EPI.representability_fail_closed_count=0;
            EPI.theorem=[ ...
                'ADAPTIVE_CONTINUOUS_SUPPORT_INTERVAL_COVER__' ...
                'SAFE_ONLY_BY_WHOLE_LEAF_CERTIFICATE__' ...
                'UNRESOLVED_LEQ_LIVE_DT_IS_UNSAFE'];
            ok=false;

            if ~(isfinite(t0)&&isfinite(searchEnd)&&searchEnd>t0&& ...
                    isfinite(dt)&&dt>0&&M>=1)
                return;
            end

            SC=obj.active_speed_certificate;
            obj.validate_active_speed_certificate(SC);

            episodes=EPI.episodes;
            tolMerge=64*eps(max([1,abs(t0),abs(searchEnd),abs(dt)]));

            for ii=1:M
                stackA=t0;
                stackB=double(searchEnd);
                unsafeA=zeros(0,1);
                unsafeB=zeros(0,1);

                while ~isempty(stackA)
                    ta=stackA(end); stackA(end)=[];
                    tb=stackB(end); stackB(end)=[];
                    if ~(isfinite(ta)&&isfinite(tb)&&tb>ta)
                        continue;
                    end

                    EPI.adaptive_leaf_count=EPI.adaptive_leaf_count+1;
                    safe=obj.r10_nominal_interval_continuously_safe_one_obstacle( ...
                        P,ta,tb,CTX,ii,SC);

                    if safe
                        EPI.adaptive_safe_leaf_count= ...
                            EPI.adaptive_safe_leaf_count+1;
                        continue;
                    end

                    width=tb-ta;
                    if width<=dt+tolMerge
                        unsafeA(end+1,1)=ta; %#ok<AGROW>
                        unsafeB(end+1,1)=tb; %#ok<AGROW>
                        EPI.adaptive_unsafe_leaf_count= ...
                            EPI.adaptive_unsafe_leaf_count+1;
                        continue;
                    end

                    tm=0.5*(ta+tb);
                    if ~(isfinite(tm)&&tm>ta&&tm<tb)
                        unsafeA(end+1,1)=ta; %#ok<AGROW>
                        unsafeB(end+1,1)=tb; %#ok<AGROW>
                        EPI.adaptive_unsafe_leaf_count= ...
                            EPI.adaptive_unsafe_leaf_count+1;
                        EPI.representability_fail_closed_count= ...
                            EPI.representability_fail_closed_count+1;
                        continue;
                    end

                    % LIFO: right first, left second -> left processed first.
                    stackA(end+1)=tm; %#ok<AGROW>
                    stackB(end+1)=tb; %#ok<AGROW>
                    stackA(end+1)=ta; %#ok<AGROW>
                    stackB(end+1)=tm; %#ok<AGROW>
                    EPI.adaptive_split_count=EPI.adaptive_split_count+1;
                end

                if isempty(unsafeA)
                    continue;
                end

                [unsafeA,ord]=sort(unsafeA);
                unsafeB=unsafeB(ord);
                es=unsafeA(1);
                ee=unsafeB(1);
                for ku=2:numel(unsafeA)
                    if unsafeA(ku)<=ee+tolMerge
                        ee=max(ee,unsafeB(ku));
                    else
                        episodes(end+1)=struct( ... %#ok<AGROW>
                            'obstacle_index',ii, ...
                            'obstacle_id',double(O.ids(ii)), ...
                            'start_s',double(es), ...
                            'end_s',double(ee), ...
                            'component_index',0);
                        es=unsafeA(ku);
                        ee=unsafeB(ku);
                    end
                end
                episodes(end+1)=struct( ... %#ok<AGROW>
                    'obstacle_index',ii, ...
                    'obstacle_id',double(O.ids(ii)), ...
                    'start_s',double(es), ...
                    'end_s',double(ee), ...
                    'component_index',0);
            end

            if isempty(episodes)
                EPI.episodes=episodes;
                EPI.certified=true;
                ok=true;
                return;
            end

            [~,ord]=sort([episodes.start_s]);
            episodes=episodes(ord);
            tolT=64*eps(max([1,abs(t0),abs(searchEnd)]));
            comps=EPI.components;
            cc=0;
            for ie=1:numel(episodes)
                es=double(episodes(ie).start_s);
                ee=double(episodes(ie).end_s);
                if cc==0 || es>double(comps(cc).end_s)+tolT
                    cc=cc+1;
                    comps(cc)=struct('start_s',es,'end_s',ee, ...
                        'duration_s',ee-es,'episode_indices',ie); %#ok<AGROW>
                else
                    comps(cc).end_s=max(double(comps(cc).end_s),ee);
                    comps(cc).duration_s=double(comps(cc).end_s)- ...
                        double(comps(cc).start_s);
                    comps(cc).episode_indices(end+1)=ie;
                end
                episodes(ie).component_index=cc;
            end

            EPI.episodes=episodes;
            EPI.components=comps;
            EPI.episode_count=numel(episodes);
            EPI.component_count=numel(comps);
            EPI.certified=true;
            ok=true;
        end


        function tf = r10_nominal_interval_continuously_safe_one_obstacle( ...
                obj,P,ta,tb,CTX,ii,SC)
            % Same continuous theorem as the old cell routine. SC was
            % validated once by the caller, avoiding repeated validation.
            LIV=CTX.live;
            O=CTX.obstacles;
            tf=false;

            if ii<1 || ii>double(O.count)
                return;
            end

            tc=0.5*(double(ta)+double(tb));
            h=0.5*(double(tb)-double(ta));
            if ~(isfinite(tc)&&isfinite(h)&&h>0)
                return;
            end

            xd=double(P.base_flat_eval_fn(tc));
            xd=xd(:);
            if numel(xd)<11 || any(~isfinite(xd(1:11)))
                return;
            end

            pL=xd(1:3);
            sf=xd(9:11)+[0;0;double(LIV.gravity)];
            ns=norm(sf);
            if ~(isfinite(ns)&&ns>0)
                return;
            end
            q=-sf/ns;
            pQ=pL-double(LIV.cable_length)*q;

            VL=double(SC.payload_speed_UB_m_s);
            VQ=double(SC.uav_speed_UB_m_s);
            VC=max(VL,VQ);
            track=0;
            if isfield(P,'execution_tracking_inflation_m') && ...
                    isfinite(double(P.execution_tracking_inflation_m))
                track=max(0,double(P.execution_tracking_inflation_m));
            end

            rhoL=double(LIV.r_load)+double(LIV.d_margin)+track+VL*h;
            rhoQ=double(LIV.r_drone)+double(LIV.d_margin)+track+VQ*h;
            rhoC=double(LIV.r_cable)+double(LIV.d_margin)+track+VC*h;

            c=double(O.centers(:,ii));
            R=reshape(double(O.rotations_flat(:,ii)),3,3);
            a=double(O.base_axes(:,ii));

            [~,okL]=obj.best_point_support_margin(pL,c,R,a,rhoL);
            if ~okL, return; end
            [~,okQ]=obj.best_point_support_margin(pQ,c,R,a,rhoQ);
            if ~okQ, return; end
            [~,okC]=obj.best_segment_support_margin(pQ,pL,c,R,a,rhoC);
            tf=okC;
        end


''';

raw=[raw(1:i0-1) newEpisode raw(i1:end)];

% =========================================================================
% R10-B
% =========================================================================
b0='        function [A,b,ok] = bezier_piecewise_episode_corridor_rows( ...';
b1='        function CANDS = enumerate_piecewise_sign_qp_recursive( ...';
[i0,i1]=unique_bounds(raw,b0,b1,'R10-B corridor function');

newCorridor = r'''
        function [A,b,ok] = bezier_piecewise_episode_corridor_rows( ...
                obj,P,CTX,EPI,episodeIndex,n,nvar)
            % =============================================================
            % AF7R27-R10B CERTIFIED PROJECTED CORRIDOR COMPRESSION
            %
            % For fixed n, every raw row is
            %   n'*a_j + s_i n'*v_j >= r_i.
            % With u=n'*a_j, w=n'*v_j:
            %   u >= r_i - s_i*w.
            %
            % Duplicate s values are merged by max(r). A middle row is
            % removed only when it is strictly below the upper hull by more
            % than a floating arithmetic guard. Ambiguous rows are retained.
            % =============================================================
            ep=EPI.episodes(episodeIndex);
            jj=double(ep.component_index);
            tComp=double(EPI.components(jj).start_s);
            LIV=CTX.live;
            O=CTX.obstacles;
            SC=obj.active_speed_certificate;
            obj.validate_active_speed_certificate(SC);

            A=zeros(0,nvar);
            b=zeros(0,1);
            ok=false;
            n=double(n(:));
            nn=norm(n);
            if ~(numel(n)==3&&all(isfinite(n))&&isfinite(nn)&&nn>0)
                return;
            end
            n=n/nn;

            ii=double(ep.obstacle_index);
            c=double(O.centers(:,ii));
            R=reshape(double(O.rotations_flat(:,ii)),3,3);
            a=double(O.base_axes(:,ii));
            hObs=dot(n,c)+norm(a.*(R.'*n));

            VL=double(SC.payload_speed_UB_m_s);
            VQ=double(SC.uav_speed_UB_m_s);
            VC=max(VL,VQ);
            track=0;
            if isfield(P,'execution_tracking_inflation_m') && ...
                    isfinite(double(P.execution_tracking_inflation_m))
                track=max(0,double(P.execution_tracking_inflation_m));
            end

            pidx=(2*jj-2)*3+(1:3);
            vidx=(2*jj-1)*3+(1:3);
            dt=double(LIV.dt);
            ta=double(ep.start_s);
            te=double(ep.end_s);

            ncell=max(1,ceil((te-ta)/dt));
            S=zeros(2*ncell+2,1);
            B=zeros(2*ncell+2,1);
            nr=0;

            while ta<te-8*eps(max(1,abs(te)))
                tb=min(te,ta+dt);
                tc=0.5*(ta+tb);
                half=0.5*(tb-ta);
                xd=double(P.base_flat_eval_fn(tc)); xd=xd(:);
                if numel(xd)<11 || any(~isfinite(xd(1:11)))
                    return;
                end

                pL=xd(1:3);
                sf=xd(9:11)+[0;0;double(LIV.gravity)];
                ns=norm(sf);
                if ~(isfinite(ns)&&ns>0)
                    return;
                end
                q=-sf/ns;
                pQ=pL-double(LIV.cable_length)*q;

                rhoL=double(LIV.r_load)+double(LIV.d_margin)+track+VL*half;
                rhoQ=double(LIV.r_drone)+double(LIV.d_margin)+track+VQ*half;
                rhoC=double(LIV.r_cable)+double(LIV.d_margin)+track+VC*half;

                req=max([ ...
                    hObs+rhoL-dot(n,pL), ...
                    hObs+rhoQ-dot(n,pQ), ...
                    hObs+rhoC-min(dot(n,pQ),dot(n,pL))]);
                if ~isfinite(req)
                    return;
                end
                req=req+64*eps(max(1,abs(req)));

                nr=nr+1;
                S(nr)=ta-tComp;
                B(nr)=req;
                nr=nr+1;
                S(nr)=tb-tComp;
                B(nr)=req;
                ta=tb;
            end

            S=S(1:nr);
            B=B(1:nr);
            if isempty(S)
                return;
            end

            keep=obj.r10_upper_envelope_row_indices(S,B);
            if isempty(keep)
                return;
            end

            S=S(keep);
            B=B(keep);
            nk=numel(S);
            A=zeros(nk,nvar);
            b=B(:);
            A(:,pidx)=repmat(n.',nk,1);
            A(:,vidx)=S(:)*n.';
            ok=all(isfinite(A(:)))&&all(isfinite(b));
        end


        function keep = r10_upper_envelope_row_indices(~,S,B)
            S=double(S(:));
            B=double(B(:));
            keep=zeros(0,1);
            if numel(S)~=numel(B)||isempty(S)|| ...
                    any(~isfinite(S))||any(~isfinite(B))
                return;
            end

            [S,ord]=sort(S);
            B=B(ord);

            % Exact duplicate abscissae -> keep strongest requirement.
            Su=zeros(numel(S),1);
            Bu=zeros(numel(S),1);
            Iu=zeros(numel(S),1);
            nu=0;
            k=1;
            while k<=numel(S)
                nu=nu+1;
                ss=S(k);
                j=k;
                bb=B(k);
                src=k;
                while j<numel(S)&&S(j+1)==ss
                    j=j+1;
                    if B(j)>bb
                        bb=B(j);
                        src=j;
                    end
                end
                Su(nu)=ss;
                Bu(nu)=bb;
                Iu(nu)=ord(src);
                k=j+1;
            end
            Su=Su(1:nu);
            Bu=Bu(1:nu);
            Iu=Iu(1:nu);

            H=zeros(nu,1);
            nh=0;
            for k=1:nu
                while nh>=2
                    i1=H(nh-1);
                    i2=H(nh);
                    i3=k;
                    dx1=Su(i2)-Su(i1);
                    dy1=Bu(i2)-Bu(i1);
                    dx2=Su(i3)-Su(i2);
                    dy2=Bu(i3)-Bu(i2);
                    t1=dx1*dy2;
                    t2=dy1*dx2;
                    cr=t1-t2;
                    scale=max([1,abs(t1),abs(t2),abs(dx1),abs(dx2),abs(dy1),abs(dy2)]);
                    guard=128*eps(scale);

                    % Upper hull requires clockwise/non-positive turn.
                    if cr>guard
                        nh=nh-1;
                    elseif cr==0
                        nh=nh-1;
                    else
                        break;
                    end
                end
                nh=nh+1;
                H(nh)=k;
            end

            H=H(1:nh);
            keep=sort(Iu(H));
        end


''';

raw=[raw(1:i0-1) newCorridor raw(i1:end)];

% =========================================================================
% R10-C
% =========================================================================
c0='        function [z,cost,minSlack,ok] = solve_piecewise_fixed_sign_qp( ...';
c1='        function H = piecewise_minimum_snap_hessian( ...';
[i0,i1]=unique_bounds(raw,c0,c1,'R10-C QP solver');

newQP = r'''
        function [z,cost,minSlack,ok] = solve_piecewise_fixed_sign_qp( ...
                ~,H,A,b)
            % =============================================================
            % AF7R27-R10C SMALL CONVEX QP
            %
            % Same QP as R8/R9. R10-B reduces only provably redundant
            % projected corridor rows. Active-set is used because the online
            % problem is small and dense; it is also quadprog's supported
            % code-generation algorithm.
            % =============================================================
            z=nan(size(H,1),1);
            cost=NaN;
            minSlack=NaN;
            ok=false;

            if exist('quadprog','file')~=2 || isempty(A) || ...
                    size(H,1)~=size(H,2) || size(A,2)~=size(H,1) || ...
                    numel(b)~=size(A,1) || any(~isfinite([H(:);A(:);b(:)]))
                return;
            end

            H=0.5*(double(H)+double(H).');
            hs=max(abs(H(:)));
            if ~(isfinite(hs)&&hs>0)
                return;
            end

            Hsolve=H/hs;
            f=zeros(size(H,1),1);
            x0=zeros(size(H,1),1);

            persistent QPOPT QPOPT_READY
            if isempty(QPOPT_READY)
                QPOPT_READY=true;
                try
                    QPOPT=optimoptions('quadprog', ...
                        'Display','off', ...
                        'Algorithm','active-set', ...
                        'UseCodegenSolver',true);
                catch
                    QPOPT=optimoptions('quadprog', ...
                        'Display','off', ...
                        'Algorithm','active-set');
                end
            end

            try
                [zc,~,exitflag]=quadprog( ...
                    Hsolve,f,-double(A),-double(b(:)), ...
                    [],[],[],[],x0,QPOPT);
            catch
                return;
            end

            if ~(exitflag>0&&numel(zc)==size(H,1)&&all(isfinite(zc)))
                return;
            end

            Az=double(A)*double(zc(:));
            slack=Az-double(b(:));
            tol=512*eps(max([ones(size(slack)),abs(double(b(:))),abs(Az)],[],2));
            if any(slack < -tol)
                return;
            end

            z=double(zc(:));
            cost=0.5*z.'*H*z;
            minSlack=min(slack);
            ok=isfinite(cost);
        end


''';

raw=[raw(1:i0-1) newQP raw(i1:end)];

% Header marker.
hdr='classdef REPLANNING_SYSTEM < handle';
ih=strfind(raw,hdr);
assert(numel(ih)==1,'[AF7R27] class header not unique.');
pos=ih+length(hdr);
raw=[raw(1:pos) newline ...
    '% AF7R27-R10: adaptive interval cover + projected corridor hull + active-set QP.' ...
    raw(pos+1:end)];

% Invariants.
assert(~contains(raw,'''sampled_acceptance'',true'), ...
    '[AF7R27] sampled_acceptance=true detected.');
assert(contains(raw,'AF7R27-R10A ADAPTIVE CONTINUOUS SUPPORT COVER'), ...
    '[AF7R27] R10-A marker missing.');
assert(contains(raw,'AF7R27-R10B CERTIFIED PROJECTED CORRIDOR COMPRESSION'), ...
    '[AF7R27] R10-B marker missing.');
assert(contains(raw,'AF7R27-R10C SMALL CONVEX QP'), ...
    '[AF7R27] R10-C marker missing.');

fid=fopen(src,'w');
assert(fid>=0,'[AF7R27] cannot open production source.');
fwrite(fid,raw,'char');
fclose(fid);

OUT=struct();
OUT.schema='AF7R27_INSTALL_R10_REALTIME_CORE_V1';
OUT.source=src;
OUT.backup=backup;
OUT.installed=true;
OUT.sampled_acceptance=false;
OUT.fixed_depth_acceptance=false;
OUT.current_case_constants_hardcoded=false;
OUT.deadline_modified=false;

fprintf(' source                                  = %s\n',src);
fprintf(' backup                                  = %s\n',backup);
fprintf(' R10-A adaptive continuous cover         = 1\n');
fprintf(' R10-B projected corridor compression    = 1\n');
fprintf(' R10-C active-set/codegen QP path        = 1\n');
fprintf(' sampled acceptance introduced           = 0\n');
fprintf(' fixed-depth acceptance introduced       = 0\n');
fprintf(' hard deadline modified                  = 0\n');
fprintf(' current-case constants hardcoded        = 0\n');
fprintf('====================================================================================================\n');
end


function [i0,i1]=unique_bounds(raw,a0,a1,label)
p0=strfind(raw,a0);
p1=strfind(raw,a1);
assert(numel(p0)==1,['[AF7R27] nonunique start: ' label]);
assert(numel(p1)==1&&p1>p0,['[AF7R27] nonunique end: ' label]);
i0=p0;
i1=p1;
end
