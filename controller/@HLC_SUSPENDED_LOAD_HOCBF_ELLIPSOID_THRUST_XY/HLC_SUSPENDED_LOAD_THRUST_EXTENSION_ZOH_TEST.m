classdef HLC_SUSPENDED_LOAD_THRUST_EXTENSION_ZOH_TEST < handle
    % 推力の動的拡張だけを切り出した閉ループ診断用HLC。
    %
    % 元の階層型線形化による u1_nom, u2_nom, u3_nom, u4_nom の計算は維持。
    % u2,u3,u4 は入力上下限だけ適用（CBF-QPなし）。
    % u1 は元の HLC_SUSPENDED_LOAD_HOCBF_ELLIPSOID_THRUST_XY と同じ
    % 2階追従則・同じ制限値・同じ離散更新を使う。
    %
    % 連続モデル: d(u1)/dt=w, dw/dt=nu。
    %   nu_k を区間中一定として内部状態を厳密更新:
    %       u1(k+1) = u1(k) + dt*w(k) + 0.5*dt^2*nu(k)
    %       w(k+1)  = w(k)  + dt*nu(k)
    %   実プラントに与える u1 は u1(k) を次周期までZOH保持。
    %
    % 重要: これは「CBFなし」の診断用であり、障害物回避の安全保証はない。
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
            vf = obj.Vfd_SuspendedLoadxyDst(Param.dt,x,xd',F1);
            vs = obj.Vs_SuspendedLoadxyDst(x,xd',vf,P,F2,F3,F4);
            uf = obj.Uf_SuspendedLoadxyDst(x,xd',vf,P);
            beta2 = obj.Beta2_SuspendedLoadxyDst(x,xd',vf,P);
            vs_alpha2 = obj.V2_alpha2_SuspendedLoadxyDst(x,xd',vf,vs',P);
            us=beta2\vs_alpha2;
            uNom = [uf(1);us];
            assert(numel(uNom)==4 && all(isfinite(uNom)), ...
                'ノミナル制御入力に非有限値があります。');

            % =============================================================
            % 2. 旧推力併用HLCの動的拡張ロジックを変更せずに抽出
            % =============================================================
            dt = Param.dt;
            assert(isscalar(dt) && isfinite(dt) && dt>0,'制御周期dtが不正です。');
            if isempty(obj.u1_state)
                obj.u1_state=max(0,min(20,uNom(1)));
                obj.du1_state=0;
            end
            u1_val=obj.u1_state;
            du1_val=obj.du1_state;
            assert(all(isfinite([u1_val;du1_val])), ...
                '推力拡張の内部状態が非有限です。');
            u1_ref=max(0,min(20,uNom(1)));

            % 原因切り分けが目的なので、現行版と同じ値・同じ制限処理を使う。
            forceRateMax=20;         % N/s
            forceAccelMax=400;       % N/s^2
            wn=20;zeta=1;
            nuNomRaw=wn^2*(u1_ref-u1_val)-2*zeta*wn*du1_val;

            tau=dt*[0.25;0.50;0.75;1.00];
            nuLB=max([-forceAccelMax; ...
                2*(0-u1_val-tau.*du1_val)./(tau.^2); ...
                (-forceRateMax-du1_val)/dt]);
            nuUB=min([ forceAccelMax; ...
                2*(20-u1_val-tau.*du1_val)./(tau.^2); ...
                ( forceRateMax-du1_val)/dt]);
            if nuLB>nuUB+1e-9
                error('推力制約が実行不能: u1=%.6g, w=%.6g, [nuLB,nuUB]=[%.6g,%.6g]', ...
                    u1_val,du1_val,nuLB,nuUB);
            end
            nuLB=min(nuLB,nuUB);
            nu=max(nuLB,min(nuUB,nuNomRaw));

            u1Next=u1_val+dt*du1_val+0.5*dt^2*nu;
            du1Next=du1_val+dt*nu;
            if u1Next < -1e-8 || u1Next > 20+1e-8 || ...
                    abs(du1Next)>forceRateMax+1e-8
                error('推力内部状態の更新で上下限を超えました。');
            end
            % 区間内で内部u1が上下限を越えないことも確認する。
            if abs(nu)>1e-12
                tvertex=-du1_val/nu;
                if tvertex>0 && tvertex<dt
                    uvertex=u1_val+du1_val*tvertex+0.5*nu*tvertex^2;
                    if uvertex<-1e-8 || uvertex>20+1e-8
                        error('推力の内部モデルが制御周期内で上下限を超えました。');
                    end
                end
            end

            % =============================================================
            % 3. プラントへの実入力: 左端値u1(k)をZOH保持。
            %    XYトルクは元のノミナル値を上下限に射影するだけ。
            %    CBF/QPは一切呼び出さない。
            % =============================================================
            input=[u1_val;max(-1,min(1,uNom(2:4)))];
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
            obj.result.delta_nu=0;
            obj.result.nu_clip_error=nu-nuNomRaw;
            obj.result.u1_nu_lb=nuLB;
            obj.result.u1_nu_ub=nuUB;
            obj.result.force_rate_max=forceRateMax;
            obj.result.force_accel_max=forceAccelMax;
            obj.result.u1_tracking_error=u1_ref-u1_val;
            obj.result.rate_limited=abs(du1_val)>=forceRateMax-1e-5;
            obj.result.nu_limited=abs(nu-nuNomRaw)>1e-6;
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
            obj.history.nuClipError(1,k)=nu-nuNomRaw;
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
