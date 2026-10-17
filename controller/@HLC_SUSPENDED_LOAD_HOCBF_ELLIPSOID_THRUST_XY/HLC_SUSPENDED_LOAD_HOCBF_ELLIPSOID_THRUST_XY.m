classdef HLC_SUSPENDED_LOAD_HOCBF_ELLIPSOID_THRUST_XY < handle
    % ZOH sampled-data cable safety shield + altitude-4th-order nominal control.
    % IMPORTANT: Continuous 21-state HOCBF is used for DIAGNOSTICS ONLY.
    % The certified safety constraint is the swept-tube halfspace certificate
    % supplied by ZOH_Cable_Ellipsoid_TubeQP, not the 21-state HOCBF inequality.
    % 高度: Altitude4_ThrustExtension_ZOH が生成する4階モデルを利用。
    % 旧4階HOCBF: CBF_Constraints_HOCBF_ELLIPSOID_THRUST_XY はログ専用。
    % Hard safety QP variables: ACTUALLY APPLIED [u1,u2,u3] this ZOH period.
    % nu=ddot(u1) controls the nominal thrust planner ONLY, not the plant.
    % 推力範囲0～20 N、全トルク-1～1を厳守。CBFはハード制約、slackなし。
    % Conditional theoretical certificate: exact 19-state model/estimation,
    % static ellipsoids, unit q and pT, input bound and EVERY cycle feasible.
    % No recursive feasibility / model uncertainty guarantee is claimed.
    % 障害物パラメータ・XY専用版・Sim・DRAWは一切変更しない。

    properties
        self
        result
        param
        u1_state
        du1_state
        obs_cache
        cache_L
        shape_cache
        history
    end

    methods
        function obj = HLC_SUSPENDED_LOAD_HOCBF_ELLIPSOID_THRUST_XY(self,param)
            obj.self = self;
            obj.param = param;
            obj.u1_state = [];
            obj.du1_state = 0;
            obj.obs_cache = [];
            obj.cache_L = [];
            obj.shape_cache = {};
            obj.result = struct();
            obj.result.min_clearance = Inf;
            obj.result.rl = 0.04;
            obj.history = struct( ...
                'k',0,'dt',param.dt, ...
                'pL',zeros(3,0),'pT',zeros(3,0),'pQuad',zeros(3,0), ...
                'refXYZ',zeros(3,0),'input',zeros(4,0),'inputNom',zeros(4,0), ...
                'deltaU23',zeros(2,0),'deltaNu',zeros(1,0), ...
                'nu',zeros(1,0),'nuNom',zeros(1,0),'nuNomRaw',zeros(1,0), ...
                'nuLB',zeros(1,0),'nuUB',zeros(1,0), ...
                'u1dot',zeros(1,0),'u1dotNext',zeros(1,0), ...
                'u1Next',zeros(1,0),'u1Ref',zeros(1,0), ...
                'u1Error',zeros(1,0),'nuClipError',zeros(1,0), ...
                'nuLimited',false(1,0),'rateLimited',false(1,0), ...
                'nominalThrustOutsideLimits',false(1,0), ...
                'zError',zeros(1,0), ...
                'zError4',zeros(4,0),'zStateDerivatives',zeros(4,0), ...
                'zSnapTarget',zeros(1,0),'zSnapActualModel',zeros(1,0), ...
                'altitudeDecoupling',zeros(1,0), ...
                'minCBFH',zeros(1,0),'clearanceProxy',zeros(1,0), ...
                'qpExitflag',zeros(1,0),'cbfFeasible',false(1,0), ...
                'qpActive',false(1,0),'numViolatedNom',zeros(1,0), ...
                'maxViolationNom',zeros(1,0),'minSlackSafe',zeros(1,0), ...
                'controllerTime',zeros(1,0), ...
                'h1',zeros(0,0),'h2',zeros(0,0), ...
                'h3',zeros(0,0),'h4',zeros(0,0), ...
                'tubeMinSlack',zeros(1,0),'tubeStartGap',zeros(1,0), ...
                'guardDeltaU1',zeros(1,0),'guidanceActive',false(1,0));
        end

        function result = do(obj,varargin) %#ok<INUSD>
            Param=obj.param;
            model=obj.self.estimator.result;
            ref=obj.self.reference.result;
            if isprop(ref.state,'xd')
                xd=ref.state.xd;
            else
                xd=ref.state.get();
            end

            pL=model.state.pL;
            if isprop(model.state,'pT')
                pT=model.state.pT;
            else
                dp=pL-model.state.p;
                if norm(dp)>1e-9
                    pT=dp/norm(dp);
                else
                    pT=[0;0;-1];
                end
            end
            assert(abs(norm(pT)-1)<1e-3, ...
                'pT が単位ベクトルではありません。推定・プラントを確認してください。');
            P=[obj.self.parameter.get(["mass","jx","jy","jz", ...
                "gravity","loadmass","cableL"]),0,0];
            x=[model.state.getq('compact');model.state.w; ...
                pL;model.state.vL;pT;model.state.wL];
            yaw=wrapToPi(model.state.q(3));
            yawd=xd(4);
            yawUnit=[cos(yaw);sin(yaw);0];
            yawdUnit=[cos(yawd);sin(yawd);0];
            deltaYaw=sign(cross(yawdUnit,yawUnit))*acos(yawdUnit'*yawUnit);
            xd(4)=-deltaYaw(3)+yaw;
            xd=[xd;zeros(28-size(xd,1),1)];
            tic_start=tic;

            %% 1. ノミナルコントローラ（高度以外の既存HLCは同じ）
            F1=Param.F1;F2=Param.F2;F3=Param.F3;F4=Param.F4;
            vf=Vfd_SuspendedLoadxyDst(obj,Param.dt,x,xd',F1);
            vs=Vs_SuspendedLoadxyDst(obj,x,xd',vf,P,F2,F3,F4);
            uf=Uf_SuspendedLoadxyDst(obj,x,xd',vf,P);
            beta2=Beta2_SuspendedLoadxyDst(obj,x,xd',vf,P);
            vs_alpha2=V2_alpha2_SuspendedLoadxyDst(obj,x,xd',vf,vs',P);
            us=beta2\vs_alpha2;
            uNom=[uf(1);us];
            assert(numel(uNom)==4 && all(isfinite(uNom)), ...
                'ノミナル制御入力に非有限値があります。');

            % =============================================================
            % 2. 4階の高度誤差を推力拡張状態 [u1;du1] から直接制御
            % =============================================================
            assert(exist('Altitude4_ThrustExtension_ZOH','file')==2, ...
                ['generateAltitude4_ThrustExtension_ZOH を一度実行し、' ...
                 'Altitude4_ThrustExtension_ZOH.m を生成してください。']);
            dt=Param.dt;
            assert(isscalar(dt)&&isfinite(dt)&&dt>0,'dtが不正です。');

            if isempty(obj.u1_state)
                obj.u1_state=max(0,min(20,uNom(1)));
                obj.du1_state=0;
            end
            u1_val=obj.u1_state;
            du1_val=obj.du1_state;
            assert(all(isfinite([u1_val;du1_val])) && ...
                u1_val>=-1e-8 && u1_val<=20+1e-8, ...
                '推力拡張の内部状態に異常があります。');
            u1_ref=max(0,min(20,uNom(1))); % 比較用のみ。制御則には使用しない。

            u234=max(-1,min(1,uNom(2:4)));
            [z0,z1,z2,z3,z4drift,z4gain]= ...
                Altitude4_ThrustExtension_ZOH([x;u1_val;du1_val],P);
            z4gain=z4gain(:).';
            assert(numel(z4gain)==4 && ...
                all(isfinite([z0;z1;z2;z3;z4drift;z4gain(:)])), ...
                '高度4階モデルの出力が非有限です。');
            bNu=z4gain(1);
            if abs(bNu) <= 1e-8
                error(['高度4階制御の推力デカップリング係数がゼロに近いです。' ...
                    ' 姿勢・紐方向を確認してください: bNu=%.4e'],bNu);
            end

            % xd=[位置4;速度4;加速度4;jerk4;snap4;...] の既存順序
            zDesired=[xd(3);xd(7);xd(11);xd(15);xd(19)];
            e4=[z0-zDesired(1);z1-zDesired(2); ...
                z2-zDesired(3);z3-zDesired(4)];

            % 元の高度サブシステム e'' + F1(2)e' + F1(1)e=0 を基準に
            % 4階用目標極を2組設定（追加の恣意的なゲインは置かない）。
            F1=Param.F1(:).';
            assert(numel(F1)==2 && all(isfinite(F1)), ...
                '元HLCの高度ゲインF1が不正です。');
            contA=[0 1;-F1(1) -F1(2)];
            basePoles=eig(contA);
            assert(all(real(basePoles)<0), ...
                '元の高度制御サブシステムの連続時間極が安定ではありません。');
            desiredPoles=exp(dt*basePoles);
            polynomial4=real(conv(poly(desiredPoles),poly(desiredPoles)));

            % ZOH中はu1が保持され、nuは次周期のu1に効く。
            % よって高度4階の理想ZOHチェーンとはB行列が異なる。
            % 状態e4=[e,e_dot,e_ddot,e_jerk]の近傍線形化（snap=v）:
            % e(k+1)≈Ad*e(k)+Bd*v(k)
            Ad=[1 dt dt^2/2 0;
                0 1 dt 0;
                0 0 1 dt;
                0 0 0 1];
            Bd=[0;0;dt^2/2;dt];
            ctrb=[Bd,Ad*Bd,Ad^2*Bd,Ad^3*Bd];
            assert(rcond(ctrb)>1e-14,'高度の離散拡張系が数値的に不可制御です。');
            polyAd=Ad^4+polynomial4(2)*Ad^3+ ...
                polynomial4(3)*Ad^2+polynomial4(4)*Ad+ ...
                polynomial4(5)*eye(4);
            Kd=real([0 0 0 1]*(ctrb\polyAd));
            assert(max(abs(eig(Ad-Bd*Kd)))<1, ...
                '高度の離散閉ループ極が単位円外です。');
            % ドリフト（姿勢・紐の非線形項）と各トルクの寄与を相殺する。
            zSnapTarget=zDesired(5)-Kd*e4;
            nuNomRaw=(zSnapTarget-z4drift-z4gain(2:4)*u234)/bNu;
            assert(isfinite(nuNomRaw),'計算したnuが非有限です。');

            %% 3. 障害物・ケーブルの外接楕円体（初回だけ計算）
            assert(exist('CBF_Constraints_HOCBF_ELLIPSOID_THRUST_XY','file')==2, ...
                '先に generateCBF_EllipsoidLink_ThrustXY を実行してください。');
            L_cable=P(7);
            cable_radius=0.04;
            beta_shape=1.0;
            if isempty(obj.cache_L)||obj.cache_L~=L_cable
                obj.obs_cache=ENVIRONMENT_OBSTACLE_ELLIPSOID_STATIC();
                obj.shape_cache=cell(1,numel(obj.obs_cache));
                for j=1:numel(obj.obs_cache)
                    obstacle=obj.obs_cache(j);
                    R=obstacle.R_obs;
                    radii=obstacle.ellipsoid_radii(:);
                    assert(norm(R.'*R-eye(3),'fro')<1e-8 && det(R)>0 ...
                        && all(radii>0) && obstacle.d_margin>=0, ...
                        '障害物 %d の楕円体パラメータが不正です。',j);
                    rho=cable_radius+obstacle.d_margin;
                    a=L_cable/2+rho;
                    b2=rho*a;
                    kappa=(1+beta_shape)*(a*a-b2);
                    Sobs=R*diag(radii.^2)*R.';
                    C=(1+1/beta_shape)*Sobs+(1+beta_shape)*b2*eye(3);
                    W=C\eye(3);
                    W=(W+W.')/2;
                    sp.vec=[W(1,1);W(2,2);W(3,3); ...
                        W(1,2);W(1,3);W(2,3);kappa];
                    sp.W=W;
                    sp.C=C; % DRAW needs actual outer ellipsoid shape
                    sp.kappa=kappa;
                    sp.minEigC=min(eig(C));
                    obj.shape_cache{j}=sp;
                end
                obj.cache_L=L_cable;
            end
            obs_env=obj.obs_cache;
            num_obs=numel(obs_env);
            p_mid=pL-0.5*L_cable*pT;
            gamma_params_xy=[2;8;16;32];
            xAug=[x;u1_val;du1_val];

            %% 4. 各障害物の21状態4階HOCBF、[nu,u2,u3]の線形制約
            A_xy_qp_list=zeros(num_obs,3);
            b_xy_qp_list=zeros(num_obs,1);
            log_h1=zeros(num_obs,1);
            log_h2=zeros(num_obs,1);
            log_h3=zeros(num_obs,1);
            log_h4=zeros(num_obs,1);
            log_p_obs=cell(1,num_obs);
            log_r_obs_margin=cell(1,num_obs);
            log_r_obs=cell(1,num_obs);
            log_d_margin=cell(1,num_obs);
            log_r_minimal=cell(1,num_obs);
            log_r_minimal_no_margin=cell(1,num_obs);
            min_surf_dist=Inf;
            min_cbf_h=Inf;
            u4_val=u234(3);
            for i=1:num_obs
                obs=obs_env(i);
                sp=obj.shape_cache{i};
                obs_params=[obs.p_obs(:);sp.vec];
                [A0,b0,h1,h2,h3,h4]= ...
                    CBF_Constraints_HOCBF_ELLIPSOID_THRUST_XY( ...
                        obj,xAug,xd,u4_val,obs_params,gamma_params_xy,cable_radius,P);
                assert(all(isfinite([A0(:);b0;h1;h2;h3;h4])), ...
                    '障害物 %d のHOCBFに非有限値があります。',i);
                A_xy_qp_list(i,:)=reshape(A0,1,3);
                b_xy_qp_list(i)=b0;
                log_h1(i)=h1;
                log_h2(i)=h2;
                log_h3(i)=h3;
                log_h4(i)=h4;
                min_cbf_h=min(min_cbf_h,h1);
                d=p_mid-obs.p_obs(:);
                Wd=sp.W*d;
                hgeom=d.'*Wd-sp.kappa*(pT.'*Wd)^2/ ...
                    (1+sp.kappa*(pT.'*sp.W*pT))-1;
                assert(abs(hgeom-h1)<1e-7*max(1,abs(hgeom)), ...
                    '障害物 %d の生成CBFと幾何が不一致です。',i);
                dproxy=(sqrt(max(0,1+hgeom))-1)*sqrt(sp.minEigC);
                min_surf_dist=min(min_surf_dist,dproxy);
                log_p_obs{i}=obs.p_obs;
                log_r_obs{i}=obs.r_obs;
                log_r_obs_margin{i}=obs.r_obs_margin;
                log_d_margin{i}=obs.d_margin;
                log_r_minimal{i}=dproxy;
                log_r_minimal_no_margin{i}=NaN;
            end

            %% 5. Nominal XY anticipatory guidance (NOT the safety proof).
            % Original XY 4th-order CBF was effective for early lateral steering.
            % It supplies a PREFERRED torque, never a hard safety condition.
            u23Nom=u234(1:2);
            u23Guide=u23Nom;
            guidanceActive=false;
            if exist('CBF_Constraints_HOCBF_ELLIPSOID_LINK_XY','file')==2
                Aguide=zeros(num_obs,2);bguide=zeros(num_obs,1);
                for i=1:num_obs
                    ob=obj.obs_cache(i);sp=obj.shape_cache{i};
                    [Ag,bg]=CBF_Constraints_HOCBF_ELLIPSOID_LINK_XY( ...
                        obj,x,xd,u1_val,u234(3),[ob.p_obs(:);sp.vec], ...
                        gamma_params_xy,cable_radius,P);
                    Aguide(i,:)=reshape(Ag,1,2);
                    bguide(i)=bg;
                end
                if any(Aguide*u23Nom>bguide+1e-7)
                    optsGuide=optimoptions('quadprog','Display','off');
                    [uTry,~,flagGuide]=quadprog(eye(2),-u23Nom, ...
                        Aguide,bguide,[],[],[-1;-1],[1;1],u23Nom,optsGuide);
                    if flagGuide>0 && ~isempty(uTry) && ...
                            all(Aguide*uTry<=bguide+1e-5)
                        u23Guide=uTry(:);
                        guidanceActive=norm(u23Guide-u23Nom,inf)>1e-8;
                    end
                end
            end

            %% 6. PROVABLY-SUFFICIENT ZOH swept-tube safety constraints.
            % This is applied to u1 NOW, not nu for the next time step.
            % The bound spans ALL times between samples, not just endpoints.
            [Atube,btube,tubeInfo]=ZOH_Cable_Ellipsoid_TubeQP( ...
                x,P,dt,obj.obs_cache,cable_radius,0);
            qNom=[u1_val;u23Guide];
            lb_qp=[0;-1;-1]; ub_qp=[20;1;1];
            tubeNomSlack=btube-Atube*qNom;
            if all(tubeNomSlack>=-1e-10)
                qsafe=qNom;
                exitflag=1;
            else
                % There is NO slack in the geometrical safety constraints.
                % The physical input is computed by a convex QP.
                Hqp=diag([1/20^2,1,1]);
                fqp=-Hqp*qNom;
                opts=optimoptions('quadprog','Display','off', ...
                    'ConstraintTolerance',1e-10);
                [qsafe,~,exitflag]=quadprog(Hqp,fqp,Atube,btube, ...
                    [],[],lb_qp,ub_qp,qNom,opts);
                if exitflag<=0 || isempty(qsafe)
                    error(['ZOH swept-volume safety constraint infeasible at t=%.3f. ' ...
                         'No safe command is certified under the input bounds. ' ...
                         'Never fall back to the nominal control.'],obj.history.k*dt);
                end
            end
            qsafe=qsafe(:);
            tubeSlack=btube-Atube*qsafe;
            assert(all(isfinite(qsafe)) && ...
                all(qsafe>=lb_qp-1e-9) && all(qsafe<=ub_qp+1e-9) && ...
                all(tubeSlack>=-1e-9), ...
                'ZOH physical cable safety certificate violated.');
            u1Safe=qsafe(1);
            u23Safe=qsafe(2:3);
            u4_val=u234(3);
            u234safe=[u23Safe;u4_val];
            input=[u1Safe;u234safe]; % ACTUAL APPLIED, held this entire period.
            % The auxiliary 21-state derivatives are NOT the physical
            % derivatives of the discontinuous ZOH plant. Diagnostic only.
            halfdt2=0.5*dt^2;
            nuLB=(0-u1_val-dt*du1_val)/halfdt2;
            nuUB=(20-u1_val-dt*du1_val)/halfdt2;
            nuNom=max(nuLB,min(nuUB,nuNomRaw));
            u1NextNom=u1_val+dt*du1_val+halfdt2*nuNom;
            nu=nuNom;
            du1Next=du1_val+dt*nu;
            u1Next=max(0,min(20, ...
                u1NextNom+(u1Safe-u1_val))); % reconcile shield action
            bz=z4gain(:).';
            zSnapActualModel=z4drift+bz*[nu;u234safe];
            nuClipError=nuNom-nuNomRaw;
            nuLimited=abs(nuClipError)>1e-6;
            num_violated=sum(tubeNomSlack < -1e-9);
            tubeMinSlack=min([Inf;tubeSlack]);
            tubeStartGap=min([Inf;tubeInfo.startGap]);
            if num_violated>0
                fprintf(['[ZOH TUBE] t=%.3f, min plane gap=%.4g m, ' ...
                    'applied u1=%.4g N, guide=%.d, min slack=%.3e\n'], ...
                    obj.history.k*dt,tubeStartGap,u1Safe,guidanceActive,tubeMinSlack);
            end

            %% 6. 既存LOGGER/DRAWとの互換フィールド
            obj.result.tmp=uNom;
            obj.result.tmp_fix=input;
            obj.result.input=input;
            obj.result.x=x;
            obj.result.xd=xd;
            obj.result.p_mid=p_mid;
            obj.result.rl=cable_radius;
            obj.result.u1_nom=uNom(1);
            obj.result.u1_ref=u1_ref;
            obj.result.u1dot=du1_val;
            obj.result.u1_rate_next=du1Next;
            obj.result.u1_next=u1Next;
            obj.result.nu=nu;
            obj.result.nu_nom=nuNom;
            obj.result.nu_nom_raw=nuNomRaw;
            obj.result.altitude_error_4=e4;
            obj.result.altitude_decoupling=bNu;
            obj.result.z_snap_target=zSnapTarget;
            obj.result.z_snap_model=zSnapActualModel;
            obj.result.altitude_discrete_gain=Kd;
            obj.result.delta_nu=0; % no hard CBF-QP on nu
            obj.result.nu_clip_error=nuClipError;
            obj.result.u1_nu_lb=nuLB;
            obj.result.u1_nu_ub=nuUB;
            obj.result.force_rate_max=Inf;
            obj.result.force_accel_max=Inf;
            obj.result.u1_tracking_error=u1_ref-u1_val;
            obj.result.rate_limited=false;
            obj.result.nu_limited=nuLimited;
            obj.result.nominal_thrust_outside_limits= ...
                (uNom(1)<0 || uNom(1)>20);
            obj.result.delta_u23=u23Safe-uNom(2:3);
            obj.result.qp_exitflag=exitflag;
            obj.result.cbf_feasible=(exitflag>0);
            obj.result.qp_active=norm(qsafe-[u1_val;u23Nom],inf)>1e-8;
            obj.result.qp_input=[nu;u23Safe]; % diagnostic continuous model only
            obj.result.safety_type='ZOH swept-volume tube (not 4th-order HOCBF)';
            obj.result.zoh_tube_A=Atube;
            obj.result.zoh_tube_b=btube;
            obj.result.zoh_tube_min_slack=tubeMinSlack;
            obj.result.zoh_tube_start_gap=tubeStartGap;
            obj.result.zoh_guard_delta_u1=u1Safe-u1_val;
            obj.result.guidance_active=guidanceActive;
            obj.result.A_xy_qp_list=A_xy_qp_list;
            obj.result.b_xy_qp_list=b_xy_qp_list;
            obj.result.slack_check=-tubeNomSlack; % positive => nominal violates tube
            obj.result.slack_nom=-tubeNomSlack; % positive => ZOH nominal unsafe
            obj.result.slack_safe=tubeSlack; % VALID physical ZOH safety certificate
            obj.result.num_violated=num_violated;
            obj.result.max_nominal_violation= ...
                max([0;-tubeNomSlack]);
            obj.result.min_qp_margin=tubeMinSlack;
            obj.result.min_cbf_h=min_cbf_h;
            obj.result.min_clearance=min_surf_dist;
            obj.result.margin_m=[obs_env.d_margin];
            obj.result.ellipsoid_radii={obs_env.ellipsoid_radii};
            obj.result.R_obs={obs_env.R_obs};
            obj.result.p_obs=log_p_obs;
            obj.result.r_obs_margin=log_r_obs_margin;
            obj.result.r_obs=log_r_obs;
            obj.result.d_margin=log_d_margin;
            obj.result.r_minimal=log_r_minimal;
            obj.result.r_minimal_no_margin=log_r_minimal_no_margin;
            obj.result.log_h1=log_h1;
            obj.result.log_h2=log_h2;
            obj.result.log_h3=log_h3;
            obj.result.log_h4=log_h4;
            obj.result.controllertime=toc(tic_start);

            %% 7. 既存と同じ履歴フィールド（追加save処理なし）
            k=obj.history.k+1;
            obj.history.k=k;
            obj.history.pL(:,k)=pL(:);
            obj.history.pT(:,k)=pT(:);
            obj.history.pQuad(:,k)=pL(:)-L_cable*pT(:);
            obj.history.refXYZ(:,k)=xd(1:3);
            obj.history.input(:,k)=input;
            obj.history.inputNom(:,k)=uNom;
            obj.history.deltaU23(:,k)=obj.result.delta_u23;
            obj.history.deltaNu(1,k)=obj.result.delta_nu;
            obj.history.nu(1,k)=nu;
            obj.history.nuNom(1,k)=nuNom;
            obj.history.nuNomRaw(1,k)=nuNomRaw;
            obj.history.nuLB(1,k)=nuLB;
            obj.history.nuUB(1,k)=nuUB;
            obj.history.u1dot(1,k)=du1_val;
            obj.history.u1dotNext(1,k)=du1Next;
            obj.history.u1Next(1,k)=u1Next;
            obj.history.u1Ref(1,k)=u1_ref;
            obj.history.u1Error(1,k)=u1_ref-u1_val;
            obj.history.nuClipError(1,k)=nuClipError;
            obj.history.zError4(:,k)=e4;
            obj.history.zStateDerivatives(:,k)=[z0;z1;z2;z3];
            obj.history.zSnapTarget(1,k)=zSnapTarget;
            obj.history.zSnapActualModel(1,k)=zSnapActualModel;
            obj.history.altitudeDecoupling(1,k)=bNu;
            obj.history.nuLimited(1,k)=nuLimited;
            obj.history.rateLimited(1,k)=false;
            obj.history.nominalThrustOutsideLimits(1,k)= ...
                obj.result.nominal_thrust_outside_limits;
            obj.history.zError(1,k)=xd(3)-pL(3);
            obj.history.minCBFH(1,k)=min_cbf_h;
            obj.history.clearanceProxy(1,k)=min_surf_dist;
            obj.history.qpExitflag(1,k)=exitflag;
            obj.history.cbfFeasible(1,k)=obj.result.cbf_feasible;
            obj.history.qpActive(1,k)=obj.result.qp_active;
            obj.history.numViolatedNom(1,k)=num_violated;
            obj.history.maxViolationNom(1,k)=obj.result.max_nominal_violation;
            obj.history.minSlackSafe(1,k)=obj.result.min_qp_margin;
            obj.history.tubeMinSlack(1,k)=tubeMinSlack;
            obj.history.tubeStartGap(1,k)=tubeStartGap;
            obj.history.guardDeltaU1(1,k)=u1Safe-u1_val;
            obj.history.guidanceActive(1,k)=guidanceActive;
            obj.history.controllerTime(1,k)=obj.result.controllertime;
            obj.history.h1(:,k)=log_h1(:);
            obj.history.h2(:,k)=log_h2(:);
            obj.history.h3(:,k)=log_h3(:);
            obj.history.h4(:,k)=log_h4(:);

            obj.u1_state=u1Next; % 次回do()から使う
            obj.du1_state=du1Next;
            result=obj.result;
        end

        function show(obj)
            obj.result
        end
    end
end
