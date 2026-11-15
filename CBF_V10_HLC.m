function output=CBF_V10_HLC(action,varargin)
%CBF_V10_HLC 既存HLC_SUSPENDED_LOADの生成済みメソッドを実際に呼ぶ。
% コントローラ本体の .do() に存在する物理入力クリップは呼ばない。
% ここでは必ずクリップ前の raw = [T;tau] を記録し、範囲外は失敗とする。
switch lower(char(action))
 case 'init'
  s=varargin{1}; opts=varargin{2};
  if isfield(opts,'hlcFolder') && ~isempty(opts.hlcFolder)
    folder=char(opts.hlcFolder);
    [parent,last]=fileparts(folder);
    if strcmp(last,'@HLC_SUSPENDED_LOAD'),folder=parent;end
    if ~isfolder(folder),error('CBF:V10HLCFolder','HLC親フォルダが存在しません: %s',folder);end
    addpath(folder);
  end
  % 実HLC親フォルダがパスにない場合だけ、通常のプロジェクト構造を探す。
  if exist('HLC_SUSPENDED_LOAD','class')~=8
    here=fileparts(mfilename('fullpath'));
    candidateFolders={fullfile(here,'controller'), ...
      fullfile(here,'..','controller'), ...
      fullfile(here,'..','..','controller'), ...
      fullfile(pwd,'controller'),fullfile(pwd,'..','controller')};
    for iCandidate=1:numel(candidateFolders)
      if isfolder(fullfile(candidateFolders{iCandidate},'@HLC_SUSPENDED_LOAD'))
         addpath(candidateFolders{iCandidate});break;
      end
    end
  end
  if exist('HLC_SUSPENDED_LOAD','class')~=8
    error('CBF:V10HLCMissing', ...
     ['本物のHLC_SUSPENDED_LOADが見つかりません。' ...
      'controller/@HLC_SUSPENDED_LOAD の親フォルダを options.hlcFolder に指定してください。']);
  end
  handle=HLC_SUSPENDED_LOAD([],struct());
  dt=localOption(opts,'dt',0.01);
  A2=[0 1;0 0]; B2=[0;1];
  A6=diag(ones(1,5),1);B6=[0;0;0;0;0;1];
  F1=lqrd(A2,B2,diag([150 10]),0.01,dt);
  F2=lqrd(A6,B6,diag([100 150 100 10 1 0.1]),0.001,dt);
  F3=F2;F4=lqrd(A2,B2,diag([10 1]),0.01,dt);
  if isfield(opts,'F1'),F1=opts.F1;end
  if isfield(opts,'F2'),F2=opts.F2;end
  if isfield(opts,'F3'),F3=opts.F3;end
  if isfield(opts,'F4'),F4=opts.F4;end
  assert(numel(F1)==2 && numel(F2)==6 && numel(F3)==6 && numel(F4)==2, ...
    'CBF:V10HLCCtrlGain','ゲインの次元が不一致です');
  output=struct('object',handle,'F1',F1(:).','F2',F2(:).','F3',F3(:).', ...
     'F4',F4(:).','dt',dt,'massDrone',s.physical.massDrone, ...
     'massLoadEstimated',s.physical.massLoad);
 case 'command'
  adapter=varargin{1};x=varargin{2};s=varargin{3};
  route=varargin{4};t=varargin{5};opts=varargin{6};
  if numel(x)<19,error('CBF:V10HLCState','19以上の実状態が必要');end
  y=x(1:19);
  XD=zeros(1,28);
  for k=0:6
     r=CBF_V10_Reference(route,t,k,opts);
     XD((4*k+1):(4*k+3))=r(:).';
  end
  % 機体yaw目標はゼロ。この試験ではゼロyawから出発し、wrap処理は
  % 状態依存のyaw誤差を保持する。生成HLCの .do は入力をクリップするため使わない。
  P=[s.physical.massDrone;s.physical.inertia(:);s.physical.gravity; ...
     s.physical.massLoad;s.physical.cableLength;s.physical.disturbanceXY(:)].';
  if numel(P)~=9,error('CBF:V10HLCParam','HLCのPは9要素');end
  obj=adapter.object;
  vf=obj.Vfd_SuspendedLoadxyDst(adapter.dt,y,XD,adapter.F1);
  vs=obj.Vs_SuspendedLoadxyDst(y,XD,vf,P,adapter.F2,adapter.F3,adapter.F4);
  uf=obj.Uf_SuspendedLoadxyDst(y,XD,vf,P);
  beta=obj.Beta2_SuspendedLoadxyDst(y,XD,vf,P);
  rhs=obj.V2_alpha2_SuspendedLoadxyDst(y,XD,vf,vs(:).',P);
  if ~isequal(size(beta),[3 3]) || numel(rhs)~=3 || numel(uf)<1
      error('CBF:V10HLCInterface','HLC生成関数の出力次元が不正');
  end
  condBeta=rcond(beta);
  if ~isfinite(condBeta) || condBeta<1e-10
    error('CBF:V10HLCBetaSingular','Beta2の条件が悪い: rcond=%g',condBeta);
  end
  raw=[uf(1);beta\rhs(:)];
  if any(~isfinite(raw))
    error('CBF:V10HLCNonfinite','HLCの実入力に非有限値');
  end
  within=raw(1)>=s.input.thrustMin && raw(1)<=s.input.thrustMax && ...
   all(raw(2:4)>=s.input.torqueMin(:)) && all(raw(2:4)<=s.input.torqueMax(:));
  output=struct('rawPhysicalInput',raw,'withinInputLimits',within,...
     'rcondBeta',condBeta,'vf',vf,'vs',vs,'XD',XD,'P',P, ...
     'nominalThrust',raw(1),'nominalTorque',raw(2:4));
 otherwise
  error('CBF:V10HLCAction','init/commandを指定');
end
end
function v=localOption(s,key,default)
if isfield(s,key) && ~isempty(s.(key)),v=s.(key);else,v=default;end
end
