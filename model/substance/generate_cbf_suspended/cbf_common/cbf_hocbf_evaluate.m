function [qp, timing]=cbf_hocbf_evaluate(mode,shape,kind,x,settings,obstacle,h,info,section,jetCache)
%CBF_HOCBF_EVALUATE  Taylor係数からHOCBF-QP用のA*u>=bを作る。
%
% ・実入力: 状態29、決定入力 [T^(4); tau_x'';tau_y'';tau_z'']。
%   幾何学的安全関数は6階で評価し、入力上限は別の積分器HOCBFで追加。
% ・仮想入力: 理想状態20、決定入力[pLx^(6);pLy^(6);pLz^(6);yaw'']。
%   機体・紐は4階、吊り荷は6階（固定球・支持関数の候補）。
%
% 【重要】これは安全条件の「候補行」です。低次入力依存は数値検査する。
% 入力係数の退化、補助状態psi<0、QP実行不能は別途検査。
% 静止障害物のみ。移動障害物は障害物側の高階微分が必要。

profileOn=(nargout>=2);
if profileOn, tAll=tic; timing=struct(); tStage=tic; else, timing=[]; end
if nargin<9 || isempty(section), section='all'; end
if nargin<10, jetCache=[]; end
if ~any(strcmp(section,{'all','obstacles'}))
    error('CBF:Section','section は all または obstacles です。');
end
if ~isfield(settings,'cbf') || ~isfield(settings.cbf,'hocbfGain') || ...
   ~isnumeric(settings.cbf.hocbfGain) || isempty(settings.cbf.hocbfGain)
    error('CBF:MissingGain','settings.cbf.hocbfGainに正のゲインを実行時に指定してください。');
end
kappa=settings.cbf.hocbfGain;
if ~isscalar(kappa) || ~isfinite(kappa) || kappa<=0
    error('CBF:InvalidGain','hocbfGainは正のスカラーです。');
end
if isfield(settings.cbf,'degeneracyTolerance') && ~isempty(settings.cbf.degeneracyTolerance)
    degTol=settings.cbf.degeneracyTolerance;
else
    degTol=0; % 追加の判定閾値を勝手に設定しない
end
if ~isscalar(degTol) || ~isfinite(degTol) || degTol<0
    error('CBF:InvalidTolerance','degeneracyToleranceは0以上です。');
end
% HOCBFの地面用ゲインは独立に指定可能。省略時は従来のゲイン。
if isfield(settings.cbf,'groundGain') && ~isempty(settings.cbf.groundGain)
    kGround=settings.cbf.groundGain;
    if ~isscalar(kGround) || ~isfinite(kGround) || kGround<=0
        error('CBF:GroundGain','groundGain は正のスカラーです。');
    end
else
    kGround=kappa;
end
P=cbf_parameter_vector(settings);
O=cbf_obstacle_vector(obstacle,kind);
plan=cbf_choose_cable_spheres(settings);
if strcmp(section,'obstacles')
    keep=~strncmp(info.labels,'ground_',7);
    h=h(keep);
    info.labels=info.labels(keep);
    info.orderHint=info.orderHint(keep);
end
if strcmp(mode,'real')
    order=6*ones(numel(h),1);K=6;
else
    order=info.orderHint;K=6;
end

if profileOn, timing.parametersMs=1000*toc(tStage); tStage=tic; end

% 名目決定入力0および標準基底の各方向についてTaylor係数を計算。
if isempty(jetCache)
    H0=makeJets(zeros(4,1));
else
    if ~strcmp(jetCache.mode,mode)
        error('CBF:JetCacheMode','Taylorキャッシュのモードが一致しません。');
    end
    J=jetCache.jets{1};
    H0=cbf_geometry_taylor(shape,kind,J.pLoad,J.qCable,J.pDrone,P,O,plan,section);
end
assert(numel(h)==size(H0,1),'CBF:RowMismatch','安全関数の行数が一致しません。');
if norm(H0(:,1)-h(:),Inf)>1e-7*(1+norm(h(:),Inf))
    error('CBF:GeometryMismatch','高階Taylorの0階値と元のh(x)が一致しません。');
end
if profileOn, timing.zeroTaylorMs=1000*toc(tStage); tStage=tic; end
Hbasis=zeros(size(H0,1),K+1,4);
for i=1:4
    if isempty(jetCache)
        Hbasis(:,:,i)=makeJets(double((1:4)'==i));
    else
        J=jetCache.jets{i+1};
        Hbasis(:,:,i)=cbf_geometry_taylor(shape,kind,J.pLoad,J.qCable,J.pDrone,P,O,plan,section);
    end
end

if profileOn, timing.fourBasisTaylorMs=1000*toc(tStage); tStage=tic; end
n=size(H0,1);A=zeros(n,4);b=zeros(n,1);
psi=cell(n,1); earlyViolation=zeros(n,1);
for i=1:n
    r=order(i);
    if ~isfinite(r) || r<1 || r>6 || floor(r)~=r
        error('CBF:InvalidOrder','安全関数の設計階数が不正です。');
    end
    % 地面専用HOCBFゲイン。両者の選択は実行時に外部から指定する。
    gain=kappa;
    if strncmp(info.labels{i},'ground_',7), gain=kGround; end
    coeff=H0(i,1:r+1);
    derivatives=coeff.*factorial(0:r);
    % h^(r)+sum_{j<r} nchoosek(r,j)*kappa^(r-j)*h^(j) >=0
    lower=0;
    for j=0:r-1
        lower=lower+nchoosek(r,j)*gain^(r-j)*derivatives(j+1);
    end
    drift=factorial(r)*coeff(r+1)+lower;
    b(i)=-drift;
    for v=1:4
        diffCoeff=Hbasis(i,:,v)-H0(i,:);
        % 制御入力が設計次数より低い階で出現しないか診断。
        low=max(abs(diffCoeff(1:r).*factorial(0:r-1)));
        earlyViolation(i)=max(earlyViolation(i),low);
        A(i,v)=factorial(r)*diffCoeff(r+1);
    end
    % psi_j=(D+kappa)^j h, j=0,...,r-1。初期条件チェック用。
    p=zeros(r,1);
    for j=0:r-1
        for m=0:j
            p(j+1)=p(j+1)+nchoosek(j,m)*gain^(j-m)*derivatives(m+1);
        end
    end
    psi{i}=p;
end
% 非常に高い係数倍率による見かけ上の丸め誤差を考慮した診断閾値は
% 実行時設定で利用者が指定できる。低次作用を勝手に無視しない。
if any(earlyViolation>1e-7*max(1,max(abs(H0(:)))))
    error('CBF:EarlyInput', ...
        '指定した階数より低次で入力が現れました。固定次数HOCBFを生成できません。');
end

if profileOn, timing.hocbfAssemblyMs=1000*toc(tStage); tStage=tic; end
labels=info.labels;
if strcmp(mode,'real') && strcmp(section,'all')
    [Aa,bb,labInput,psiInput]=cbf_real_input_hocbf(x,settings);
    A=[A;Aa];b=[b;bb];labels=[labels;labInput];
    psi=[psi;psiInput];order=[order;4;4;2*ones(6,1)];
end
if profileOn, timing.realInputBoundsMs=1000*toc(tStage); tStage=tic; end
if any(~isfinite(A(:))) || any(~isfinite(b))
    error('CBF:NonfiniteQP','CBF-QP係数が非有限です。');
end
qp=struct();
qp.A=A;qp.b=b;qp.labels=labels;qp.order=order;
qp.psi=psi;qp.psiInitiallyNonnegative=cellfun(@(z)all(z>=0),psi);
qp.effectNorm=sqrt(sum(A.^2,2));
qp.degenerate=(qp.effectNorm<=degTol);
qp.uncontrollableViolation=qp.degenerate & b>0;
qp.earlyInputResidual=earlyViolation;
qp.relativeDegreeAtState=order;
qp.relativeDegreeAtState(qp.degenerate)=NaN; % 退化時はそれ以上の階数を未確定とする
qp.modelDiagnostics=cbf_model_diagnostics(mode,x,settings);
qp.mode=mode;qp.section=section;qp.decisionOrder={'nuT','nuTauX','nuTauY','nuTauZ'};
qp.inputLimitIncluded=strcmp(mode,'real') && strcmp(section,'all');
if strcmp(mode,'virtual')
    qp.decisionOrder={'pLx6','pLy6','pLz6','yaw2'};
    qp.note=['理想HLCの幾何CBF。実推力・トルク制約はここには含まれない。' ...
             '実HLCの逆変換からQP行を追加するまで安全保証とは呼ばない。'];
else
    qp.note=['非線形実入力拡張モデル。実推力/トルクの範囲を高次状態条件として追加。' ...
             '各補助条件の初期成立・QP可行性・モデル成立領域が必要。'];
end

if profileOn
    timing.diagnosticsMs=1000*toc(tStage);
    timing.totalMs=1000*toc(tAll);
end

    function H=makeJets(u)
        if strcmp(mode,'real')
            Z=cbf_real_state_taylor(x,u,P,K);
            lp=Z(8:10,:);qt=Z(14:16,:);dp=lp-P(3)*qt;
        else
            [lp,qt,dp]=cbf_virtual_state_taylor(x,u,P,K);
        end
        H=cbf_geometry_taylor(shape,kind,lp,qt,dp,P,O,plan,section);
    end
end
