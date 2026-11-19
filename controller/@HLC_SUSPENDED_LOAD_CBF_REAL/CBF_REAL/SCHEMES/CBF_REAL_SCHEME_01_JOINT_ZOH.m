function [u,info] = CBF_REAL_SCHEME_01_JOINT_ZOH(agent,cfg,x,uNom,Praw,dt,varargin)
% CBF_REAL_SCHEME_01_JOINT_ZOH
% 【CBF方式01：この1ファイルで方式と近似を一括管理】
%
%   CBF方式: Stage16の実入力4変数同時非線形最適化（試験版）
%   近似形状: 機体=球、牽引物=球、紐=楕円体
%   障害物:   センサが検知した楕円体等（Sim側の障害物近似を使用）
%   地面:     センサから受け取った水平半空間
%   入力:     u=[T;tauX;tauY;tauZ], 1周期内ZOH
%
%   今後、この方式の「近似とCBFをセットで」変更するときは
%   この1ファイルの設定と処理を変更する。
%   別方式を追加するときはSCHEMESに新しい番号の1ファイルを作り、
%   INITとDISPATCHへ番号を登録する。HLCの計算部は変更しない。
%
%   警告: この方式は安全性が証明された標準HOCBFではない。
%   求解不可・検知欠落などのときはsuccess=falseまたはエラーで停止。

start=tic;
u=[];
if cfg.cbfId~=1
    error('CBFReal:SchemeId','Scheme 01 expects cbfId=1.');
end

% =======================================================================
% 方式01の設定（近似・ソルバ・ログ方針を、このファイルだけに集約）
% =======================================================================
geometryOptions=struct(); % CBF_COMMON_GEOMETRY既定値:
%   droneRadius=0.25, loadRadius=0.16, cableRadius=0.015,
%   safetyMargin=0.05, cableEllipsoidEta=0.25 など。
% 変更例: geometryOptions=struct('droneRadius',0.25,'loadRadius',0.16);
%
solverOptions=struct();   % Stage16の既定値を利用。下記dtは毎周期正しく設定。
%   inputLower=[0;-1;-1;-1], inputUpper=[20;1;1;1],
%   sampleFractions=[0 .25 .5 .75 1], lambda=[1 1 1],
%   useRK4Check=true, allowUnsafeStart=false など。
% 変更例: solverOptions=struct('maxIterations',65,'lambda',[1 1 1]);
%
% =======================================================================

validateattributes(x,{'numeric'}, ...
    {'vector','numel',19,'real','finite'});
validateattributes(uNom,{'numeric'}, ...
    {'vector','numel',4,'real','finite'});
validateattributes(Praw,{'numeric'}, ...
    {'vector','numel',9,'real','finite'});

% ── 1. 実際に観測された障害物と地面だけを取得 ───────────────
if ~isprop(agent,'sensor') || isempty(agent.sensor) || ...
        ~isprop(agent.sensor,'result')
    error('CBFReal:NoSensor','agent.sensor.result is not accessible.');
end
sensorResult=agent.sensor.result;
if ~isstruct(sensorResult) || ...
        ~isfield(sensorResult,'cbfObservedWorld') || ...
        ~isfield(sensorResult,'cbfSensorTime')
    error('CBFReal:NoCBFSensor', ...
        'cbfObservedWorld/cbfSensorTime not found. Register cbf_spherical.');
end
observed=sensorResult.cbfObservedWorld;
if ~isstruct(observed) || ~isscalar(observed) || ...
        ~isfield(observed,'parts') || ~isstruct(observed.parts)
    error('CBFReal:Observation', ...
        'observed.parts missing. Never substitute unseen world obstacles.');
end
sensorTime=sensorResult.cbfSensorTime;
if ~isnumeric(sensorTime) || ~isscalar(sensorTime) || ...
        ~isfinite(sensorTime)
    error('CBFReal:ObservationTime','Invalid CBF sensor timestamp.');
end
if ~isempty(varargin)
    v=varargin{1};
    if isobject(v) && isprop(v,'t')
        tNow=double(v.t);
    elseif isnumeric(v) && isscalar(v)
        tNow=double(v);
    else
        tNow=NaN;
    end
    if isfinite(tNow) && abs(tNow-sensorTime)>1.5*dt
        error('CBFReal:StaleSensor', ...
            'Sensor timestamp %.6g and current %.6g differ > 1.5 dt.', ...
            sensorTime,tNow);
    end
end

% ── 2. 公称HLCと同じ推定状態・推定パラメータを使用 ─────────────
p=double(Praw(:));
P=struct('m',p(1),'J',p(2:4),'gravity',p(5), ...
    'mL',p(6),'cableL',p(7),'dstx',p(8),'dsty',p(9));

% ── 3. 方式01の機体・紐・牽引物近似および地面定義 ─────────────
G=make_geometry_01(P,observed,geometryOptions);

% ── 4. 推力とトルク3軸を同時に最適化 ───────────────────────
solverOptions.dt=dt;
result=CBF_REAL_ZOH_JOINT_FILTER(double(x(:)),double(uNom(:)), ...
    observed.parts,P,G,solverOptions);
if result.success
    u=result.input(:);  % 成功後に再クリップしない（制約を変えてしまうため）
end

% ── 5. 記録用の要約 ────────────────────────────────────
info=struct('success',logical(result.success), ...
    'status',char(result.status),'cbfId',cfg.cbfId, ...
    'observedParts',numel(observed.parts),'sensorTime',sensorTime, ...
    'nominal',uNom(:),'input',u,'modified',false, ...
    'maxViolation',result.maxViolation, ...
    'nominalFeasible',result.nominalFeasible, ...
    'elapsedMs',1000*toc(start),'solverExitflag',result.exitflag, ...
    'solverIterations',NaN,'solverStatus',char(result.status));
if isstruct(result.solverOutput) && isfield(result.solverOutput,'iterations')
    info.solverIterations=result.solverOutput.iterations;
end
if result.success
    info.modified=norm(u-uNom(:),inf)>1e-10;
end
end

function G = make_geometry_01(P,observed,geometryOptions)
% 方式01に固有の地面の扱いと幾何構成（別ファイルへ分割しない）。
G=CBF_COMMON_GEOMETRY(P,geometryOptions);
if ~isfield(observed,'ground')
    error('CBFReal:GroundMissing', ...
        'Sensor observation must contain ground (possibly []).');
end
if isempty(observed.ground)
    G.includeGround=false;
    return
end
ground=observed.ground;
if ~isstruct(ground) || ~isscalar(ground) || ...
        ~isfield(ground,'kind') || ~strcmp(char(ground.kind),'halfspace') || ...
        ~isfield(ground,'normal') || ~isfield(ground,'point')
    error('CBFReal:GroundUnsupported', ...
        'Scheme 01 supports horizontal upward halfspace ground only.');
end
n=ground.normal(:);
q=ground.point(:);
if numel(n)~=3 || any(~isfinite(n)) || norm(n-[0;0;1])>1e-9 || ...
        numel(q)~=3 || any(~isfinite(q))
    error('CBFReal:GroundUnsupported', ...
        'Ground must have normal=[0;0;1] and finite point.');
end
if isfield(ground,'forbiddenSide') && ...
        ~strcmp(char(ground.forbiddenSide),'negative')
    error('CBFReal:GroundUnsupported', ...
        'Expected forbiddenSide=negative.');
end
groundMargin=0;
if isfield(ground,'safetyMargin')
    groundMargin=ground.safetyMargin;
end
validateattributes(groundMargin,{'numeric'}, ...
    {'scalar','real','finite','nonnegative'});
G.includeGround=true;
G.groundZ=q(3)+groundMargin;
end
