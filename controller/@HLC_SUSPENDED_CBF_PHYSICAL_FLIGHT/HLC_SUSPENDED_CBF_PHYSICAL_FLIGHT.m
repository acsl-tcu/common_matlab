classdef HLC_SUSPENDED_CBF_PHYSICAL_FLIGHT < handle
% Original HLC algebra is preserved; flight-only experimental CBF wrapper.
% Preflight is performed off-line during arming. Flight must be initialized
% before the first f-phase controller call. Never claims 25ms hard realtime.
properties
    self
    result
    param
    % ===== CBF configuration =====
    cbfEnabled = false
    cbfMode = 'physical'
    cbfBackend = 'legacy' % 'legacy' (old HOCBF) or 'zoh' (physical ZOH predictor)
    cbfMethod = 'S3'
    cbfAux = []
    cbfAuxMode = ''
    cbfTime = 0
    cbfShapes = struct()
    cbfKappa = 2*ones(6,1)
    cbfOpts = struct()
    cbfSupervisor = struct()   % persistent monitor/engagement state
    cbfCaptureNominal = true
    cbfWarmupContext = []
    cbfPreflightCompleted = false
    cbfPreflightReport = struct()
end

methods

    function obj = HLC_SUSPENDED_CBF_PHYSICAL_FLIGHT(self, param)
        obj.self = self;
        obj.param = param;

    end

    function result = do(obj, varargin)
        if obj.cbfEnabled
            % Never reuse previous-step commands/CBF fields after a failure.
            % These changes affect diagnostics only, before new computation.
            obj.result.flightHlcAudit=struct();
            obj.result.tmp=nan(4,1);
            obj.result.cbf=[];
        end
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
        % The state manager has updated the shared mass parameter before
        % this controller call. Check that the sample-k EKF mass, rather
        % than a stale number from a previous prediction, is used here.
        % The 3.5-s preview holds THIS sample's estimate constant and
        % reinitializes its model with a fresh estimate next sample.
        if obj.cbfEnabled && isprop(model.state,'mL')
            currentHat=double(model.state.mL);
            if ~isscalar(currentHat) || ~isfinite(currentHat) || ...
                    currentHat<=0 || ...
                    abs(double(P(6))-currentHat)>64*eps(max(1,abs(currentHat)))
                error('CBF:MassEstimateNotCurrent', ...
                    'Sample-k mass mismatch: parameter=%.12g, EKF=%.12g.', ...
                    double(P(6)),currentHat);
            end
        end

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
        % [FLIGHT-ONLY READ-ONLY HLC AUDIT, 2026-10-16]
        % The original hierarchical inverse remains EXACTLY unchanged.
        % Measure beta2 conditioning and inversion numerator BEFORE solving,
        % including cases where the CBF subsequently stops the controller.
        % Outside 'f', do not compute/store this extra diagnostic.
        if obj.cbfEnabled
            if ~isequal(size(beta2),[3,3]) || any(~isfinite(beta2(:))) || ...
                    any(~isfinite(vs_alpha2(:)))
                error('CBF:HLCNonfiniteLinearization', ...
                    'Flight HLC beta2 or inversion right-hand side is nonfinite.');
            end
            betaSing = svd(beta2);
            hAudit=struct('beta2Rcond',rcond(beta2), ...
                'beta2SigmaMin',min(betaSing),'beta2SigmaMax',max(betaSing),...
                'beta2RHSNorm',norm(vs_alpha2(:)), ...
                'beta2RHSInf',norm(vs_alpha2(:),inf), ...
                'loadCableDirectionZ',double(x(16)), ...
                'zRef0',double(xd(3)), 'zRef1',double(xd(7)), ...
                'zRef2',double(xd(11)), 'zRef3',double(xd(15)), ...
                'zRef4',double(xd(19)), 'zRef5',double(xd(23)), ...
                'zRef6',double(xd(27)));
            obj.result.flightHlcAudit=hAudit;
        end
        us = beta2 \ vs_alpha2; % ORIGINAL HLC inverse, not regularized
        if obj.cbfEnabled
            obj.result.flightHlcAudit.rawThrust=double(uf(1));
            obj.result.flightHlcAudit.rawMomentInf=norm(us(:),inf);
            obj.result.flightHlcAudit.rawMoment2=norm(us(:),2);
        end
        %obj.result.aa=toc;
        tmp = [uf(1); us]; % 実入力へ変換
        obj.result.tmp = tmp; % 入力に制限を付けてない値を格納
        % READ-ONLY hierarchy/saturation audit. The original HLC, CBF,
        % estimator and actuator command are NOT altered. In particular,
        % clipping f cannot be interpreted as a change to v1 alone while
        % leaving its derivative jet and the second-layer map untouched.
        if obj.cbfEnabled
            try
                limH=[0;20;1;1;1];
                if isfield(obj.cbfOpts,'uLimits') && ...
                        numel(obj.cbfOpts.uLimits)==5
                    limH=double(obj.cbfOpts.uLimits(:));
                end
                lowH=[limH(1);-limH(3:5)];
                highH=[limH(2);limH(3:5)];
                clippedH=min(max(tmp(:),lowH),highH);
                vfZeroH=vf(:);vfZeroH(1)=0;
                vfOneH=vfZeroH;vfOneH(1)=1;
                fZeroH=obj.Uf_SuspendedLoadxyDst(x,xd',vfZeroH,P);
                fOneH=obj.Uf_SuspendedLoadxyDst(x,xd',vfOneH,P);
                slopeH=fOneH(1)-fZeroH(1);
                v1ClipH=NaN;dv1H=NaN;
                if isfinite(slopeH) && abs(slopeH)>1e-12
                    v1ClipH=(clippedH(1)-fZeroH(1))/slopeH;
                    dv1H=v1ClipH-vf(1);
                end
                clipFDeltaH=clippedH(1)-tmp(1);
                clipMDeltaH=norm(clippedH(2:4)-tmp(2:4),inf);
                reconstructionH=norm(beta2*us(:)-vs_alpha2(:),inf)/ ...
                    max(1,norm(vs_alpha2(:),inf));
                torqueProxyH=NaN;
                if abs(clipFDeltaH)<=1e-9
                    % When f is unchanged, the same first-layer derivative
                    % jet is valid for this instantaneous torque-only audit.
                    torqueProxyH=norm(beta2*(clippedH(2:4)-tmp(2:4)),inf)/ ...
                        max(1,norm(vs_alpha2(:),inf));
                end
                obj.result.flightHlcAudit.nominalV1=double(vf(1));
                obj.result.flightHlcAudit.clipEquivalentV1=double(v1ClipH);
                obj.result.flightHlcAudit.clipEquivalentDeltaV1=double(dv1H);
                obj.result.flightHlcAudit.thrustClippingDelta_N=double(clipFDeltaH);
                obj.result.flightHlcAudit.torqueClippingDeltaInf_Nm=double(clipMDeltaH);
                obj.result.flightHlcAudit.rawSecondLayerResidualRel=double(reconstructionH);
                obj.result.flightHlcAudit.clippedTorqueVirtualProxyRel=double(torqueProxyH);
                obj.result.flightHlcAudit.nominalHierarchyUnchangedAfterClip= ...
                    double(norm(clippedH(:)-tmp(:),inf)<=1e-9);
            catch
                % Audit errors must NEVER change the flight/controller.
                % Missing diagnostics are represented as NaNs by the RUN.
            end
        end
        % Capture only a small arming-step context, not a trajectory.
        % CBF preflight uses a DRY RUN without advancing any live state.
        if obj.cbfCaptureNominal
            obj.cbfWarmupContext = struct('x',x(:),'xd',xd(:), ...
                'vf',vf(:),'F1',F1(:),'P',P(:),'uRaw',tmp(:), ...
                'dt',double(Param.dt));
        end

        % ===== ACTUATOR-AWARE SUPERVISOR =====
        % Only the safety layer is changed. The nominal HLC above is intact.
        if obj.cbfEnabled && ~obj.cbfPreflightCompleted
            error('CBF:PreflightMissing', ...
                'Flight-only CBF enabled without successful preflight warmup.');
        end
        if obj.cbfEnabled
            % A/B research comparison: only the requested architecture is used.
            % Virtual->physical fallback is forbidden by run configuration.
            if ~any(strcmpi(obj.cbfMode,{'physical','virtual'}))
                error('CBF:UnknownInputMode','CBF input mode must be physical or virtual.');
            end
            % No stale previous-step CBF diagnostics on a failed call.
            obj.result.cbf=[];
            obs = ENVIRONMENT_OBSTACLE_ELLIPSE_MOVE(obj.cbfTime);
            % The cable direction/angular velocity live on T(S^2).
            % The EKF is Euclidean and may exhibit small constraint drift.
            % Retract the CBF-ONLY estimated representation, not the
            % nominal HLC state, EKF or physical plant. Refuse large drift.
            [xCBF,geometryAudit] = obj.localCBFStateRetraction(x);
            obj.result.cbfGeometryAudit = geometryAudit;
            if strcmpi(obj.cbfBackend,'zoh')
                % Common actual-input ZOH plant, DIFFERENT physical/virtual
                % objective coordinates. No high-order thrust extension here.
                % Carry the previous INPUT plan only as a warm start;
                % the filter independently rechecks current-state safety.
                zohOpts=obj.cbfOpts;
                zohOpts.zohWarmStart=obj.cbfSupervisor;
                zohOpts.zohTime_s=obj.cbfTime;
                cbf = CBF_ZOH_FILTER('filter',obj.cbfMode,obj, ...
                    xCBF,xd,vf,P,tmp,obs,obj.cbfMethod,obj.cbfShapes, ...
                    zohOpts,Param.dt);
            elseif strcmpi(obj.cbfBackend,'legacy')
                cbf = Generate_CBF_ActuatorAware_6Methods( ...
                    'supervise',obj.cbfMode,obj,xCBF,xd',vf,F1,P,...
                    tmp,obs,obj.cbfMethod,obj.cbfShapes,obj.cbfKappa,...
                    obj.cbfOpts,obj.cbfSupervisor,Param.dt);
            else
                error('CBF:UnknownBackend','Unknown CBF backend: %s',char(obj.cbfBackend));
            end
            cbf.cbfGeometryAudit = geometryAudit;
            obj.result.cbf = cbf;
            if ~cbf.success
                % Keep the run's controlled-stop/logging path for a CBF failure.
                % This does not modify the nominal HLC or CBF-QP constraints.
                error('CBF:SupervisorFailed', ...
                    'Actuator-aware CBF failed: %s',cbf.reason);
            end
            obj.cbfSupervisor = cbf.supervisor;
            tmp = cbf.uSafe(:);
        else
            % This path reproduces the ORIGINAL saturated HLC exactly.
            obj.result.cbf = [];
        end

        obj.result.controllertime=toc;
        % 安全のため入力値に制限を付ける．推定した牽引物質量や紐の長さ，外乱などを表示．
        %disp("time: "+ num2str(t,2)+" z position of drone:
        %"+num2str(model.state.p(3),3)+" estimated load mass:
        %"+num2str(P(6),4)+" dst:(x,y) "+num2str(P(end-1:end),4))
        if obj.cbfEnabled
            % QP input limits already applied; do NOT clip after solving.
            obj.result.input = tmp;
        else
            obj.result.input = [max(0, min(20, tmp(1))); max(-1, min(1, tmp(2))); max(-1, min(1, tmp(3))); max(-1, min(1, tmp(4)))];
        end
        % obj.result.input = [0.0,0,0,(0.5236+0.04)*9.81]';%
        obj.result.xd = xd;
        obj.result.x = x;
        result = obj.result;

    end

    function report = prepareFlightCBF(obj)
        % Must be called during arming, NEVER on first flight step.
        if strcmpi(obj.cbfBackend,'zoh')
            % A ZOH rollout uses NO high-order generated derivatives. Do
            % not run the unrelated legacy QP as a mandatory preflight.
            s=CBF_ZOH_FILTER('selftest');
            report=struct('passed',logical(s.passed), ...
                'mode',obj.cbfMode,'monitorRows',0, ...
                'note','ZOH analytical sanity check; not a flight safety proof.');
        else
            report=CBF_PREFLIGHT_WARMUP(obj);
        end
        obj.cbfPreflightReport=report;
        obj.cbfPreflightCompleted=report.passed;
        % No warmup-only state is needed by the live supervisor.
        obj.cbfWarmupContext=[];
        obj.cbfCaptureNominal=false;
    end

    function [xSafe,audit] = localCBFStateRetraction(~,x)
        % A limited model-manifold correction, NOT a safety relaxation.
        % This does not alter physical state, estimator state or HLC input.
        % These numerical thresholds are for SIMULATION model consistency;
        % they are not bounds on physical estimation error.
        xSafe=x(:);
        if numel(xSafe)~=19 || any(~isfinite(xSafe))
            error('CBF:EstimatorManifoldInvalid', ...
                'CBF state must be a finite 19-vector.');
        end
        q=xSafe(1:4); p=xSafe(14:16); w=xSafe(17:19);
        nq=norm(q); np=norm(p);
        if nq<1e-8 || np<1e-8
            error('CBF:EstimatorManifoldInvalid', ...
                'Estimated quaternion or cable direction has zero norm.');
        end
        qError=abs(nq-1); pError=abs(np-1);
        pUnit=p/np;
        tangentError=abs(dot(pUnit,w));
        audit=struct('qNormError',qError, ...
            'cableNormError',pError,'tangentError',tangentError, ...
            'maxCorrection',NaN,'retracted',false, ...
            'tangentVelocityProjected',true);
        % Unit direction and quaternion still need to be reasonably close
        % to the manifold. The angular velocity's radial component,
        % however, is IDENTICALLY ZERO in the rigid-cable state space.
        % For fixed pUnit the least-squares consistent angular velocity is
        % w_T=(I-pUnit*pUnit')*w. Its radial correction is measured/audited,
        % not arbitrarily rejected at 0.003 rad/s. This is a state-model
        % projection, NOT a bound on EKF error or a safety certificate.
        if qError>1e-3 || pError>1e-3
            error('CBF:EstimatorManifoldInvalid', ...
                ['Estimator unit states far from model manifold: ' ...
                 '|q|-1=%.8g, |pT|-1=%.8g, radial wL=%.8g.'], ...
                qError,pError,tangentError);
        end
        xSafe(1:4)=q/nq;
        xSafe(14:16)=pUnit;
        xSafe(17:19)=w-pUnit*dot(pUnit,w);
        audit.maxCorrection=norm(xSafe-x(:),inf);
        audit.retracted=audit.maxCorrection>1e-12;
    end

    function show(obj)
        obj.result
    end
end

end
