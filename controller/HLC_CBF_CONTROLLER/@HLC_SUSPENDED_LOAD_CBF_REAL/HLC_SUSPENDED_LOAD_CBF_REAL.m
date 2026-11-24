classdef HLC_SUSPENDED_LOAD_CBF_REAL < handle
% クアッドコプター用階層型線形化を使った入力算出
properties
    self
    result
    param
end
properties (SetAccess = private)
    cbfConfig
end

methods

    function obj = HLC_SUSPENDED_LOAD_CBF_REAL(self, param, cbfId)
        obj.self = self;
        obj.param = param;
    
        if nargin < 3 || isempty(cbfId)
            cbfId = 0;
        end
    
        obj.cbfConfig = CBF_REAL_INIT(cbfId, param);
        obj.result = struct();
        if obj.cbfConfig.cbfId==3
            % Input-side adapter only. MODEL_CLASS.m is NOT modified.
            % ode15s sees true T(s), while the held control is ddot(T).
            CBF_REAL_PAPER_INPUT_ADAPTER('install',self);
        end
    end

    function result = do(obj, varargin)
        Param = obj.param; % param (optional) : 構造体：ゲインF1-F4
        model = obj.self.estimator.result; % 推定した状態
        ref = obj.self.reference.result; % 目標値

        % 目標値を取得
        if isprop(ref.state, 'xd')
            xd = ref.state.xd; % 20次元の目標値に対応する用
        else
            xd = ref.state.get();
        end

        pL = model.state.pL;
        if isprop(model.state, "pT")
            pT = model.state.pT;
        else
            delta = pL - model.state.p;
            if norm(delta) > 1e-9
                pT = delta / norm(delta);
            else
                pT = [0; 0; -1];
            end
        end
        P = [obj.self.parameter.get(["mass", "jx", "jy", "jz", "gravity", "loadmass", "cableL"]), 0, 0];

        x = [model.state.getq('compact'); model.state.w; pL; model.state.vL; pT; model.state.wL]; % [q, w ,pL, vL, pT, wL]に並べ替え
        % [model.state.p, x(8:10), xd(1:3), x(8:10) - xd(1:3)]
        % yaw角の定義域の問題を回避,h4 = yaw - yawd(誤差)だがyawd = -(誤差)+yawの値を入れる．x,y,yawの仮想入力はVs_SuspendedLoadはクオータニオンで計算するため
        % yawサブシステムの入力を設計するときにyaw角を打ち消して定義域修正した誤差を反映
        yaw = wrapToPi(model.state.q(3)); % 機体yaw角[-pi,pi]にする特にyaw
        yawd = xd(4); % 目標yaw角
        yawUnit = [cos(yaw); sin(yaw); 0]; % yawの方向ベクトル
        yawdUnit = [cos(yawd); sin(yawd); 0]; % yawdの方向ベクトル
        deltaYaw = sign(cross(yawdUnit, yawUnit)) * acos(yawdUnit' * yawUnit); % 目標角度からみた機体角度との誤差
        xd(4) = -deltaYaw(3) + yaw; % yaw打ち消しと誤差をyawの目標角に入れる．
        %目標値の格納
        xd = [xd; zeros(28 - size(xd, 1), 1)];
        tic
        % 階層型線形化による入力計算
        % 仮想入力のゲイン
        F1 = Param.F1; % z方向サブシステムのゲイン
        F2 = Param.F2; % x方向サブシステムのゲイン
        F3 = Param.F3; % y方向サブシステムのゲイン
        F4 = Param.F4; % yaw方向サブシステムのゲイン

        vf = obj.Vfd_SuspendedLoadxyDst(Param.dt, x, xd', F1); % 実験で刻み時間が変わったときに対応
        vs = obj.Vs_SuspendedLoadxyDst(x, xd', vf, P, F2, F3, F4); % 第二層x,y,yawサブシステムの仮想入力の計算
        uf = obj.Uf_SuspendedLoadxyDst(x, xd', vf, P); % 第一層の仮想入力の実入力(推力)への変換
        % h234  = obj.H234_SuspendedLoadxyDst(x,xd',vf,P);  % ただの単位行列なのでなくてもいい
        %tic
        beta2 = obj.Beta2_SuspendedLoadxyDst(x, xd', vf, P); % 第二層のbetaの逆行列
        vs_alpha2 = obj.V2_alpha2_SuspendedLoadxyDst(x, xd', vf, vs', P); % 第二層のvs - alpha
        us = beta2 \ vs_alpha2; % 第二層の実入力（roll,pitch,yawのトルク）への変換：bate^(-1)*(vs - alpha) %h234*invbeta2*a2;
        %obj.result.aa=toc;
        tmp = [uf(1); us]; % 実入力へ変換
        obj.result.tmp = tmp; % 入力に制限を付けてない値を格納
        obj.result.controllertime=toc;
        % 安全のため入力値に制限を付ける．推定した牽引物質量や紐の長さ，外乱などを表示．
        %disp("time: "+ num2str(t,2)+" z position of drone:
        %"+num2str(model.state.p(3),3)+" estimated load mass:
        %"+num2str(P(6),4)+" dst:(x,y) "+num2str(P(end-1:end),4))
        % PAPER ID3 is activated only for the flight ('f') phase.
        % During takeoff/landing keep original HLC + physical box exactly:
        % obstacle CBF is not called and there is no fictitious thrust rate.
        if obj.cbfConfig.cbfId==3 && ~strcmp(char(obj.self.cha),'f')
            loBase=[0;-1;-1;-1]; hiBase=[20;1;1;1];
            applied=min(max(double(tmp(:)),loBase),hiBase);
            obj.cbfConfig.paperThrustState=struct( ...
                'initialized',false,'T',NaN,'q',NaN);
            obj.result.cbf=struct('cbfId',3,'success',true, ...
                'status','PAPER_BYPASS_NON_FLIGHT', ...
                'nominal',double(tmp(:)),'nominalRaw',double(tmp(:)), ...
                'input',applied,'modified',any(applied~=tmp(:)), ...
                'nominalFeasible',all(tmp(:)>=loBase & tmp(:)<=hiBase), ...
                'observedParts',0,'elapsedMs',0,'maxViolation',0, ...
                'stageValues',[],'barrierResiduals',[], ...
                'paperHold',[],'paperStateCommitted',false, ...
                'sampledDataObstacleCertified',false);
            obj.result.input=applied;
            obj.result.xd=xd;
            obj.result.x=x;
            result=obj.result;
            return;
        end
        % At first flight cycle, seed T from the actual previously applied
        % held thrust (last takeoff sample). q=0 is an explicit hybrid
        % initialization; we never fabricate q from the current HLC raw T.
        if obj.cbfConfig.cbfId==3 && ...
                ~obj.cbfConfig.paperThrustState.initialized
            if ~isfield(obj.result,'input') || ...
                    ~isnumeric(obj.result.input) || ...
                    numel(obj.result.input)~=4 || ...
                    any(~isfinite(obj.result.input(:)))
                error('CBFPaper:MissingTakeoffHandoff', ...
                    'Cannot initialize paper thrust: last applied HLC input missing.');
            end
            lastThrust=double(obj.result.input(1));
            if lastThrust<0 || lastThrust>20
                error('CBFPaper:InvalidTakeoffHandoff', ...
                    'Last applied HLC thrust outside [0,20] N.');
            end
            obj.cbfConfig.paperThrustState=struct( ...
                'initialized',true,'T',lastThrust,'q',0);
        end
        cbfOut = CBF_REAL_DISPATCH( ...
            obj.self, obj.cbfConfig, ...
            x, tmp, P, Param.dt, varargin{:});
        
        % CBF診断情報をLOGGERへ渡す
        obj.result.cbf = cbfOut.info;
        obj.result.cbfId = obj.cbfConfig.cbfId;
        
        % CBF方式に依存しない失敗時ポリシー
        policy = 'stop';
        
        if isfield(obj.cbfConfig,'failurePolicy')
            policy = char(obj.cbfConfig.failurePolicy);
        end
        % ID3 research simulation must CONTINUE on an infeasible CBF.
        % This deliberately overrides older ID3 configs with policy='stop'.
        % An uncertified fallback is NOT a safety certificate.
        if obj.cbfConfig.cbfId==3
            policy = 'continueBounded';
        end

        candidate = cbfOut.input;
        if obj.cbfConfig.cbfId==3 && ~cbfOut.success
            % Unlike physical-thrust ZOH, paper-style ID3 MUST carry a
            % coherent [T,q] and ddot(T)-hold through an unsuccessful QP.
            % The local actuator-only backup uses the same scalar bounds
            % and viability functions as the paper ID3 scheme. No CBF
            % constraint is claimed satisfied by this continuation.
            [candidate,fb] = CBF_REAL_SCHEME_03_PAPER_EXTENSION( ...
                'fallback',obj.cbfConfig,tmp,Param.dt);
            obj.result.cbf.paperHold = fb.paperHold;
            obj.result.cbf.paperNu = fb.paperNu;
            obj.result.cbf.paperThrust0 = fb.paperThrust0;
            obj.result.cbf.paperRate0 = fb.paperRate0;
            obj.result.cbf.paperThrustEnd = fb.paperThrustEnd;
            obj.result.cbf.paperRateEnd = fb.paperRateEnd;
            obj.result.cbf.paperStateCommitted = false;
            obj.result.cbf.paperActuatorFallbackStatus = fb.status;
            obj.result.cbf.paperActuatorFallbackRateReset = fb.rateReset;
            obj.result.cbf.paperActuatorFallbackRateResetReason = fb.rateResetReason;
            obj.result.cbf.paperUncertifiedFailureStatus = cbfOut.info.status;
            obj.result.cbf.input = candidate;
            obj.result.cbf.modified = any(abs(candidate(:)-tmp(:))>1e-9);
        end

        low  = [0;-1;-1;-1];
        high = [20;1;1;1];
        
        isFinite4 = @(u) isnumeric(u) && isreal(u) && ...
            numel(u)==4 && all(isfinite(u(:)));
        
        if cbfOut.success
        
            % CBF成功時：従来の入力を採用
            if ~isFinite4(candidate)
                error('CBFReal:InvalidInput', ...
                    'CBF returned invalid input.');
            end
        
            % 大きな入力範囲違反は異常として扱う
            if any(candidate(:)<low-1e-9) || ...
               any(candidate(:)>high+1e-9)
                error('CBFReal:InputOutOfBounds', ...
                    'CBF returned input outside physical bounds.');
            end
        
            applied = min(max(double(candidate(:)),low),high);
            usedCandidate = true;
            continuedUncertified = false;
        
        else
        
            % CBF失敗時
            if ~strcmp(policy,'continueBounded')
                obj.result.input = [];
                error('CBFReal:NoFeasibleInput', ...
                    'CBF failed: %s',cbfOut.info.status);
            end
        
            % 研究用：入力上下限を維持して継続
            if isFinite4(candidate)
        
                applied = min(max(double(candidate(:)),low),high);
                usedCandidate = true;
        
            elseif isFinite4(tmp)
        
                % CBF出力が不正な場合のみHLC入力にフォールバック
                applied = min(max(double(tmp(:)),low),high);
                usedCandidate = false;
        
            else
                error('CBFReal:NoFiniteInput', ...
                    'Neither CBF nor HLC returned a finite input.');
            end
        
            continuedUncertified = true;
        
        end
        
        obj.result.cbf.inputApplied = usedCandidate;
        obj.result.cbf.appliedWithoutCertificate = continuedUncertified;
        
        % 最終物理入力
        obj.result.input = applied;
        % ID3 ONLY, SIMULATION: the held control is nuT, NOT physical T.
        % MODEl_CLASS.do will use paperHold to integrate a quadratic thrust
        % profile. Advance extended state only when plant is advanced.
        if obj.cbfConfig.cbfId==3
            if ~isfield(obj.result.cbf,'paperHold') || ...
                    isempty(obj.result.cbf.paperHold) || ...
                    ~obj.result.cbf.paperHold.active
                error('CBFPaper:MissingHold', ...
                    'ID3 output has no valid paper-style thrust hold.');
            end
            hh=obj.result.cbf.paperHold;
            if abs(applied(1)-hh.T0)>1e-9
                error('CBFPaper:ThrustClipChanged', ...
                    'The post-CBF clipping changed initial thrust state.');
            end
            % In arming/stop/quit MODEL_CLASS does not advance the plant.
            % Do not advance [T,qT] in such a phase either.
            phase=char(obj.self.cha);
            if strcmp(phase,'f')
                obj.cbfConfig.paperThrustState=struct( ...
                    'initialized',true,'T',hh.Tend,'q',hh.qEnd);
                obj.result.cbf.paperStateCommitted=true;
            else
                obj.result.cbf.paperStateCommitted=false;
            end
        end
        % obj.result.input = [0.0,0,0,(0.5236+0.04)*9.81]';%
        obj.result.xd = xd;
        obj.result.x = x;
        result = obj.result;

    end

    function show(obj)
        obj.result
    end
end

end
