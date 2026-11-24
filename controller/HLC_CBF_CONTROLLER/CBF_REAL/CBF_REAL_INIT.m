function cfg = CBF_REAL_INIT(cbfId,hlcParam)
% CBF_REAL_INIT  飛行開始時にCBF方式を1つだけ確定する。
%
%   cbfId = 0 : CBFを使用しない（従来HLCと同じ入力制限）
%   cbfId = 1 : 機体球・吊り荷球・紐楕円体 + 実入力4変数同時最適化
%   cbfId = 2 : 3D attitude-dependent cable ellipsoid only, tauX/tauY adjusted
%   cbfId = 3 : Liu-2026 CT-4 HOCBF, dynamic thrust extension + XY torques (Sim only)
%
% CBF方式・機体等の近似・オプションはCBF_REAL/SCHEMES/*.mで管理する。
% 01 uses analytic ZOH jets only; prior experimental HOCBF and joint filter
% are NOT called. No dynamic thrust extension is used.
% 飛行中にこの関数を呼び直さない。初期化時以外はパス変更しない。
if nargin < 1 || isempty(cbfId)
    cbfId=0;
end
validateattributes(cbfId,{'numeric'}, ...
    {'scalar','integer','nonnegative','finite'});
validateattributes(hlcParam.dt,{'numeric'}, ...
    {'scalar','real','finite','positive'});

cfg=struct('cbfId',double(cbfId),'dt',double(hlcParam.dt), ...
    'failurePolicy','stop'); % Generic HLC policy; each scheme may configure it
if cbfId==0
    cfg.name='OFF_NOMINAL_HLC';
    fprintf('[CBF REAL INIT] id=0 OFF (nominal HLC + original clipping)\n');
    return
end

% 増設時はここ（登録）・DISPATCH（分岐）・SCHEMES内の1ファイルを追加。
switch cbfId
    case 1
        cfg.name='JOINT_ZOH_SPHERE_CABLE_ELLIPSOID';
        schemeFile='CBF_REAL_SCHEME_01_JOINT_ZOH.m';
        generatedSubfolder=fullfile('HOCBF_ZOH_THRUST','GENERATED');
        requiredGenerated={'CBF_REAL_ZOH_DIAG_EVAL.m'};
    case 2
        cfg.name='XY';
        schemeFile='CBF_REAL_SCHEME_02_XY.m';
        generatedSubfolder=fullfile('XY','GENERATED');
        requiredGenerated={'CBF_REAL_XY_JET.m', ...
            'CBF_REAL_XY_EVAL.m'};
        % ORIGINAL 3-D obstacle, rotating central cable ellipsoid ONLY.
        % NO drone sphere, NO payload sphere, NO ground safety conditions.
        % No thrust/yaw adjustment and NO dynamic extension.
        cfg.xyLambda=[1;2;8;16]; % latest user setting; adjust as needed
        cfg.xyTorqueWeights=[1;1];
        % HARD tilt limits in degrees from the world vertical axis.
        % Body b3 relative +Z: order-2 HOCBF, independent 2 gains.
        cfg.xyBodyTiltEnabled=true;
        cfg.xyBodyTiltMaxDeg=45;
        cfg.xyBodyTiltLambda=[2;2];
        % Cable pT relative -Z: order-4 HOCBF, independent 4 gains.
        cfg.xyCableTiltEnabled=true;
        cfg.xyCableTiltMaxDeg=60;
        cfg.xyCableTiltLambda=[1;2;8;16]; % latest user setting
        % Optional smooth multi-obstacle guidance / one-ZOH-step guard.
        % All obstacle/tilt hard inequalities stay in original QP.
        % This is NOT guaranteed sampled-data or Backup CBF certification.
        cfg.xyPreviewEnabled=true;
        cfg.xyGuideStartH=3.5;          % smooth activation, zero at 3m
        cfg.xyGuideMaxShiftNm=0.36;      % MAX adaptive guide shift, not fixed input
        cfg.xyGuideMaxExtraNm=0.36;      % maximum extra torque vs hard QP
        cfg.xyGuideSmoothWeight=0.16;   % preference for prior applied torque
        cfg.xyGuideVelocityDamping=0.35;
        cfg.xyGuideVelocityScale=0.45;
        cfg.xyGuideMaxReserveLoss=0.012; % allowable 1-step predicted reserve loss
        cfg.xyGuideMaxLowerDrop=0.025;   % relative tolerance for EACH lower stage
        % Approximate one-ZOH lower-stage guard: ALL observed obstacles,
        % body tilt and cable tilt, two time nodes, zero extra CBF EVALs.
        % THIS IS A MODEL-BASED TAYLOR GUARD, NOT AN INTERSAMPLE PROOF.
        cfg.xyZohGuardEnabled=true;
        cfg.xyZohGuardMargin=0.0;
        cfg.xyPreviewMaxPredictedParts=4; % cost cap; beyond => original hard QP
        cfg.xyStateProjectionMaxError=0.005;
        cfg.xyMaxObstacles=Inf;
        cfg.xyCableRadiusM=0.20;      % transverse SEMI-axis [m]
        cfg.xyCableAxialMarginM=0.02; % a=L/2+axialMargin [m]
        cfg.failurePolicy='continueBounded'; % Research Sim: bounded but uncertified on CBF failure
    case 3
        % PAPER-STYLE augmentation, not original physical thrust ZOH.
        % Physical thrust T(s) varies quadratically while nuT/tau are ZOH.
        cfg.name='PAPER_DYNAMIC_THRUST_IHOCBF_INPUT_ADAPTER_SIM_ONLY';
        schemeFile='CBF_REAL_SCHEME_03_PAPER_EXTENSION.m';
        generatedSubfolder=fullfile('HOCBF_D2T_OBSTACLE_3D','GENERATED');
        requiredGenerated={'CBF_REAL_D2T_JET.m', ...
                           'CBF_REAL_D2T_OBSTACLE_EVAL.m'};
        cfg.paperThrustState=struct('initialized',false,'T',NaN,'q',NaN);
        cfg.paperThrustBounds=[0;20];
        cfg.paperAccelMaxNps2=100; % TEST assumption, not actuator proof
        cfg.paperRateMaxNps=40; % TEST assumption, not actuator proof
        cfg.paperControlScale=[100;1;1]; % non-paper normalization study choice
        cfg.paperLambda=[2;4;8;16];
        cfg.paperCableRadiusM=0.20;
        cfg.paperCableAxialMarginM=0.02;
        cfg.paperPhysicalCableRadiusM=0.015;
        cfg.paperStateProjectionLimit=0.005;
        cfg.paperMaxParts=36;
        % Never apply an uncertified fallback on a QP failure.
        cfg.failurePolicy='continueBounded';
    otherwise
        error('CBFReal:UnknownCBF', ...
            'Unknown cbfId=%d; register a scheme in INIT and DISPATCH.',cbfId);
end

classDir = fileparts(fileparts(mfilename('fullpath')));
cbfDir = fullfile(classDir,'CBF_REAL');
schemeDir = fullfile(cbfDir,'SCHEMES');
if ~isfile(fullfile(schemeDir,schemeFile))
    error('CBFReal:SchemeMissing', ...
        'CBF scheme is missing: %s',fullfile(schemeDir,schemeFile));
end

% CBF_GENERATORはこのclassと別場所にあるので、初期化時に1度探す。
ancestor = cbfDir;
generatorRoot='';
for depth=1:9
    locations={fullfile(ancestor,'CBF_GENERATOR'), ...
        fullfile(ancestor,'model','substance','CBF_GENERATOR'), ...
        fullfile(ancestor,'substance','CBF_GENERATOR')};
    for j=1:numel(locations)
        if isfile(fullfile(locations{j},'COMMON','CBF_COMMON_MODEL.m'))
            generatorRoot=locations{j};
            break
        end
    end
    if ~isempty(generatorRoot),break;end
    parent=fileparts(ancestor);
    if strcmp(parent,ancestor),break;end
    ancestor=parent;
end
if isempty(generatorRoot)
    error('CBFReal:GeneratorMissing', ...
        'Cannot find CBF_GENERATOR under common_matlab from %s',cbfDir);
end

commonDir=fullfile(generatorRoot,'COMMON');
generatedDir=fullfile(generatorRoot,'REAL',generatedSubfolder);
for k=1:numel(requiredGenerated)
    file=fullfile(generatedDir,requiredGenerated{k});
    if ~isfile(file)
        error('CBFReal:NotGenerated', ...
            ['Generated function is missing: %s\n' ...
             'Run the generator matching cbfId in MATLAB.'],file);
    end
end

% MATLABは@class下の任意のサブフォルダを自動でpathに入れない。
if cbfId==3
    sourceDir=fileparts(generatedDir);
    addpath(cbfDir,schemeDir,sourceDir,commonDir,generatedDir,'-begin');
else
    addpath(schemeDir,commonDir,generatedDir,'-begin');
end
% 古い同名生成関数がpathの別位置から呼ばれないよう検査する。
files=[{schemeFile},requiredGenerated];
dirs=[{schemeDir},repmat({generatedDir},1,numel(requiredGenerated))];
for k=1:numel(files)
    [~,fn]=fileparts(files{k});
    wanted=fullfile(dirs{k},files{k});
    found=which(fn);
    if ~same_path(found,wanted)
        error('CBFReal:PathShadowed', ...
            'Function %s resolves to %s, expected %s',fn,found,wanted);
    end
end
cfg.generatorRoot=generatorRoot;
fprintf('[CBF REAL INIT] id=%d %s / dt=%.6g s\n', ...
    cfg.cbfId,cfg.name,cfg.dt);
fprintf('[CBF REAL INIT] scheme includes geometry and solver configuration.\n');
end

function tf=same_path(a,b)
a=strrep(char(a),'/',filesep);
b=strrep(char(b),'/',filesep);
if ispc, tf=strcmpi(a,b);else,tf=strcmp(a,b);end
end
