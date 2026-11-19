function cfg = CBF_REAL_INIT(cbfId,classDir,hlcParam)
% CBF_REAL_INIT  飛行開始時にCBF方式を1つだけ確定する。
%
%   cbfId = 0 : CBFを使用しない（従来HLCと同じ入力制限）
%   cbfId = 1 : 機体球・吊り荷球・紐楕円体 + 実入力4変数同時最適化
%
% CBF方式・機体等の近似・オプションは1つのSCHEMES/*.mで管理する。
% 飛行中にこの関数を呼び直さない。初期化時以外はパス変更しない。
if nargin < 1 || isempty(cbfId)
    cbfId=0;
end
validateattributes(cbfId,{'numeric'}, ...
    {'scalar','integer','nonnegative','finite'});
validateattributes(hlcParam.dt,{'numeric'}, ...
    {'scalar','real','finite','positive'});

cfg=struct('cbfId',double(cbfId),'dt',double(hlcParam.dt));
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
    otherwise
        error('CBFReal:UnknownCBF', ...
            'Unknown cbfId=%d; register a scheme in INIT and DISPATCH.',cbfId);
end

cbfDir=fullfile(classDir,'CBF');
schemeDir=fullfile(cbfDir,'SCHEMES');
if ~isfile(fullfile(schemeDir,schemeFile))
    error('CBFReal:SchemeMissing', ...
        'CBF scheme is missing: %s',fullfile(schemeDir,schemeFile));
end

% CBF_GENERATORはこのclassと別場所にあるので、初期化時に1度探す。
ancestor=classDir;
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
        'Cannot find CBF_GENERATOR under common_matlab from %s',classDir);
end

commonDir=fullfile(generatorRoot,'COMMON');
generatedDir=fullfile(generatorRoot,'REAL','HOCBF_ZOH_THRUST','GENERATED');
requiredGenerated={'CBF_REAL_ZOH_DIAG_EVAL.m', ...
    'CBF_REAL_ZOH_HOCBF_INEQUALITY.m', ...
    'CBF_REAL_ZOH_JOINT_FILTER.m'};
for k=1:numel(requiredGenerated)
    file=fullfile(generatedDir,requiredGenerated{k});
    if ~isfile(file)
        error('CBFReal:NotGenerated', ...
            ['Generated function is missing: %s\n' ...
             'Run generate_CBF_REAL_HOCBF_ZOH in MATLAB.'],file);
    end
end

% MATLABは@class下の任意のサブフォルダを自動でpathに入れない。
addpath(schemeDir,commonDir,generatedDir,'-begin');
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
