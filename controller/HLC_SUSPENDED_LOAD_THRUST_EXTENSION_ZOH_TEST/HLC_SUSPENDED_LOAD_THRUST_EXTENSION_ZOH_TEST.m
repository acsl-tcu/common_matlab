classdef HLC_SUSPENDED_LOAD_THRUST_EXTENSION_ZOH_TEST < handle
    % 推力の動的拡張に対応する高度4階フィードバック・単独検証用HLC。
    % 元のxy/yawトルク生成を保持し、高度制御だけを4階の拡張状態で設計。
    % CBF/QPなし。u2,u3,u4 は入力上下限(-1～1)を適用。
    % 推力は各周期の左端値をZOHでプラントへ与え、次周期値は
    % 4階高度フィードバックから得るnu=ddot(u1)で離散更新する。
    % 推力0～20 Nのみを物理的上下限として課す。
    % 旧試験にあった根拠のない +/-20 N/s、+/-400 N/s^2 は適用しない。
    % 4階Lie微分は別途生成した Altitude4_ThrustExtension_ZOH.m を使用。
    %
    % 連続モデル: d(u1)/dt=w, dw/dt=nu。
    %   nu_k を区間中一定として内部状態を厳密更新:
    %       u1(k+1) = u1(k) + dt*w(k) + 0.5*dt^2*nu(k)
    %       w(k+1)  = w(k)  + dt*nu(k)
    %   実プラントに与える u1 は u1(k) を次周期までZOH保持。
    %
    % 重要: これはCBFなしの高度単独検証で、障害物回避保証はない。
    % Lie微分は連続時間拡張モデル、実プラントはZOHであり完全一致しない。
    % F1の2階閉ループ特性多項式を二重化した4階目標を基に、
    % 入力の1周期遅れを含むリフト型高度チェーンで離散極を配置する。
    % 制約非活性の局所設計であり、入力飽和時の安定性は要検証。
    % 新たなファイル保存や独自のSim実行機構は追加しない。

    properties
        self
        result
        param
        u1_state
        du1_state
        history
    end

    methods
        function obj = HLC_SUSPENDED_LOAD_THRUST_EXTENSION_ZOH_TEST(self,param)
            obj.self = self;
            obj.param = param;
            obj.u1_state = [];
            obj.du1_state = 0;
            obj.result = struct();
            obj.result.min_clearance = NaN; % CBFは実装しない
            obj.result.rl = 0.04;           % 従来DRAWとの互換
            obj.history = struct( ...
                'k',0, 'dt',param.dt, ...
                'pL',zeros(3,0), 'pT',zeros(3,0), 'pQuad',zeros(3,0), ...
                'refXYZ',zeros(3,0), 'input',zeros(4,0), 'inputNom',zeros(4,0), ...
                'deltaU23',zeros(2,0), 'deltaNu',zeros(1,0), ...
                'nu',zeros(1,0), 'nuNom',zeros(1,0), 'nuNomRaw',zeros(1,0), ...
                'nuLB',zeros(1,0), 'nuUB',zeros(1,0), ...
                'u1dot',zeros(1,0), 'u1dotNext',zeros(1,0), ...
                'u1Next',zeros(1,0), 'u1Ref',zeros(1,0), ...
                'u1Error',zeros(1,0), 'nuClipError',zeros(1,0), ...
                'nuLimited',false(1,0), 'rateLimited',false(1,0), ...
                'nominalThrustOutsideLimits',false(1,0), ...
                'zError',zeros(1,0), ...
                'zError4',zeros(4,0), 'zStateDerivatives',zeros(4,0), ...
                'zSnapTarget',zeros(1,0), 'zSnapActualModel',zeros(1,0), ...
                'altitudeDecoupling',zeros(1,0), ...
                'minCBFH',zeros(1,0), 'clearanceProxy',zeros(1,0), ...
                'qpExitflag',zeros(1,0), 'cbfFeasible',false(1,0), ...
                'qpActive',false(1,0), 'numViolatedNom',zeros(1,0), ...
                'maxViolationNom',zeros(1,0), 'minSlackSafe',zeros(1,0), ...
                'controllerTime',zeros(1,0), ...
                'h1',zeros(0,0), 'h2',zeros(0,0), ...
                'h3',zeros(0,0), 'h4',zeros(0,0));
        end

        function result = do(obj,varargin) %#ok<INUSD>
            Param = obj.param;
            model = obj.self.estimator.result;
            ref = obj.self.reference.result;

            if isprop(ref.state,'xd')
                xd = ref.state.xd;
            else
                xd = ref.state.get();
            end

            pL = model.state.pL;
            if isprop(model.state,"pT")
                pT = model.state.pT;
            else
                delta = pL - model.state.p;
                if norm(delta)>1e-9
                    pT = delta/norm(delta);
                else
                    pT = [0;0;-1];
                end
            end

            assert(abs(norm(pT)-1)<1e-3, ...
                'pT が単位ベクトルではありません。推定・プラント状態を確認してください。');

            P = [obj.self.parameter.get(["mass","jx","jy","jz", ...
                "gravity","loadmass","cableL"]),0,0];
            x = [model.state.getq('compact');model.state.w; ...
                pL;model.state.vL;pT;model.state.wL];

            yaw = wrapToPi(model.state.q(3));
            yawd = xd(4);
            yawUnit = [cos(yaw);sin(yaw);0];
            yawdUnit = [cos(yawd);sin(yawd);0];
            deltaYaw = sign(cross(yawdUnit,yawUnit))*acos(yawdUnit'*yawUnit);
            xd(4) = -deltaYaw(3)+yaw;
            xd = [xd;zeros(28-size(xd,1),1)];

            tic_start=tic;
            % =============================================================
            % 1. 元HLCと同一のノミナル推力・トルク生成（改変しない）
            % =============================================================
            F1=Param.F1;F2=Param.F2;F3=Param.F3;F4=Param.F4;
            % これら5関数は別ファイルの通常関数。第一引数のobjは生成時の互換引数。
            vf = Vfd_SuspendedLoadxyDst(obj,Param.dt,x,xd',F1);
            vs = Vs_SuspendedLoadxyDst(obj,x,xd',vf,P,F2,F3,F4);
            uf = Uf_SuspendedLoadxyDst(obj,x,xd',vf,P);
            beta2 = Beta2_SuspendedLoadxyDst(obj,x,xd',vf,P);
            vs_alpha2 = V2_alpha2_SuspendedLoadxyDst(obj,x,xd',vf,vs',P);
            us=beta2\vs_alpha2;
            uNom = [uf(1);us];
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

            % ユーザーが指定した物理推力制限0～20 Nのみを適用する。
            % プラントへは左端u1をZOH入力するので、次周期のu1を制約。
            % 推力変化率・2階微分の上限値は指定されていないため創作しない。
            nuLB=2*(0-u1_val-dt*du1_val)/(dt^2);
            nuUB=2*(20-u1_val-dt*du1_val)/(dt^2);
            nu=max(nuLB,min(nuUB,nuNomRaw));
            u1Next=u1_val+dt*du1_val+0.5*dt^2*nu;
            du1Next=du1_val+dt*nu;
            if ~isfinite(u1Next)||~isfinite(du1Next) || ...
                    u1Next<-1e-7 || u1Next>20+1e-7
                error('次周期のZOH推力が0～20 Nの制限を超えました。');
            end
            u1Next=max(0,min(20,u1Next)); % 浮動小数の微小誤差のみ
            zSnapActualModel=z4drift+z4gain*[nu;u234];
            nuClipError=nu-nuNomRaw;
            nuLimited=abs(nuClipError)>1e-6;

            % =============================================================
            % 3. プラントへの実入力: 左端値u1(k)をZOH保持。
            %    XYトルクは元のノミナル値を上下限に射影するだけ。
            %    CBF/QPは一切呼び出さない。
            % =============================================================
            input=[u1_val;u234];
            obj.result.tmp=uNom;
            obj.result.tmp_fix=input;
            obj.result.input=input;
            obj.result.x=x;
            obj.result.xd=xd;
            obj.result.p_mid=pL-0.5*P(7)*pT;
            obj.result.u1_nom=uNom(1);
            obj.result.u1_ref=u1_ref;
            obj.result.u1dot=du1_val;
            obj.result.u1_rate_next=du1Next;
            obj.result.u1_next=u1Next;
            obj.result.nu=nu;
            obj.result.nu_nom=nu;  % CBF-QPなし。限界反映後の指令。
            obj.result.nu_nom_raw=nuNomRaw;
            obj.result.altitude_error_4=e4;
            obj.result.altitude_decoupling=bNu;
            obj.result.z_snap_target=zSnapTarget;
            obj.result.z_snap_model=zSnapActualModel;
            obj.result.altitude_discrete_gain=Kd;
            obj.result.delta_nu=0;
            obj.result.nu_clip_error=nuClipError;
            obj.result.u1_nu_lb=nuLB;
            obj.result.u1_nu_ub=nuUB;
            obj.result.force_rate_max=Inf; % 物理的な速度制限は指定されていない
            obj.result.force_accel_max=Inf;
            obj.result.u1_tracking_error=u1_ref-u1_val;
            obj.result.rate_limited=false;
            obj.result.nu_limited=nuLimited;
            obj.result.nominal_thrust_outside_limits= ...
                (uNom(1)<0 || uNom(1)>20);
            obj.result.delta_u23=input(2:3)-uNom(2:3);
            obj.result.qp_active=false; % QP自体が存在しない
            obj.result.qp_exitflag=NaN;
            obj.result.cbf_feasible=false; % CBF判定を実施していない
            obj.result.min_cbf_h=NaN;
            obj.result.min_clearance=NaN;
            obj.result.max_nominal_violation=NaN;
            obj.result.min_qp_margin=NaN;
            obj.result.num_violated=0;
            obj.result.A_xy_qp_list=zeros(0,3);
            obj.result.b_xy_qp_list=zeros(0,1);
            obj.result.slack_check=[];
            obj.result.log_h1=NaN;obj.result.log_h2=NaN;
            obj.result.log_h3=NaN;obj.result.log_h4=NaN;
            obj.result.controllertime=toc(tic_start);

            % =============================================================
            % 4. 既存HLC同様のメモリ上の時系列履歴（独自saveなし）
            % =============================================================
            k=obj.history.k+1;
            obj.history.k=k;
            obj.history.pL(:,k)=pL(:);
            obj.history.pT(:,k)=pT(:);
            obj.history.pQuad(:,k)=pL(:)-P(7)*pT(:);
            obj.history.refXYZ(:,k)=xd(1:3);
            obj.history.input(:,k)=input;
            obj.history.inputNom(:,k)=uNom;
            obj.history.deltaU23(:,k)=obj.result.delta_u23;
            obj.history.deltaNu(1,k)=0;
            obj.history.nu(1,k)=nu;
            obj.history.nuNom(1,k)=nu;
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
            obj.history.nuLimited(1,k)=obj.result.nu_limited;
            obj.history.rateLimited(1,k)=obj.result.rate_limited;
            obj.history.nominalThrustOutsideLimits(1,k)= ...
                obj.result.nominal_thrust_outside_limits;
            obj.history.zError(1,k)=xd(3)-pL(3);
            obj.history.minCBFH(1,k)=NaN;
            obj.history.clearanceProxy(1,k)=NaN;
            obj.history.qpExitflag(1,k)=NaN;
            obj.history.cbfFeasible(1,k)=false;
            obj.history.qpActive(1,k)=false;
            obj.history.numViolatedNom(1,k)=0;
            obj.history.maxViolationNom(1,k)=NaN;
            obj.history.minSlackSafe(1,k)=NaN;
            obj.history.controllerTime(1,k)=obj.result.controllertime;

            % 更新後の状態は「次回のdo」で使用する。今周期の出力には使わない。
            obj.u1_state=u1Next;
            obj.du1_state=du1Next;
            result=obj.result;
        end

        function show(obj)
            obj.result
        end
    end
end
