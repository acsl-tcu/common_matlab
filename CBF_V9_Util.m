function varargout=CBF_V9_Util(action,varargin)
% CBF_V9_UTIL V9独立監査用の共通処理。生成器・CBF-QPには一切変更しない。
switch lower(action)
 case 'options'
  opts=varargin{1}; key=varargin{2}; default=varargin{3};
  if isfield(opts,key) && ~isempty(opts.(key)), varargout{1}=opts.(key);
  else, varargout{1}=default; end
 case 'folder'
  paths=test_CBF_SetupPaths();
  out=fullfile(paths.generator,'CBF','test_results');
  if exist(out,'dir')~=7,mkdir(out);end
  varargout{1}=out;
 case 'masks'
  qp=varargin{1}; labels=string(qp.labels(:));
  gm=startsWith(labels,'obs1_ground_') | startsWith(labels,'ground_');
  im=contains(labels,'thrust_')|contains(labels,'tau_');
  varargout={gm(:),im(:),~(gm(:)|im(:))};
 case 'groundqp'
  mode=varargin{1};shape=varargin{2};x=varargin{3};s=varargin{4};
  % ダミーの障害物を一度だけ設定し、地面・入力行以外を厳格に除いて返す。
  % 距離だけを理由に障害物CBFを安全と判定する方法ではない。
  obs=struct('type','sphere','center',[1.234;2.345;100.567], 'radius',.1);
  base=cbf_hocbf_scene(mode,shape,x,s,{obs});
  [gm,im]=CBF_V9_Util('masks',base);
  keep=gm|im;
  q=base;
  q.A=base.A(keep,:);q.b=base.b(keep);q.labels=base.labels(keep);
  q.order=base.order(keep);q.psi=base.psi(keep);
  q.psiInitiallyNonnegative=base.psiInitiallyNonnegative(keep);
  q.effectNorm=base.effectNorm(keep);q.degenerate=base.degenerate(keep);
  q.uncontrollableViolation=base.uncontrollableViolation(keep);
  q.relativeDegreeAtState=base.relativeDegreeAtState(keep);
  % 注意: earlyInputResidual は幾何CBF行のみ、推力/トルク制限行の要素はない。
  % これまで keep（全QP行）で参照していたため、logical添字範囲エラーが起きた。
  geometryRow=~(contains(string(base.labels),'_thrust_') | ...
                contains(string(base.labels),'_tau_'));
  assert(numel(base.earlyInputResidual)==sum(geometryRow), ...
     'CBF:V10EarlyRowSize','低次入力作用の行数が不整合です');
  q.earlyInputResidual=base.earlyInputResidual(keep(geometryRow));
  assert(size(q.A,1)==numel(q.labels) && ...
         size(q.A,1)==numel(q.psi) && ...
         numel(q.earlyInputResidual)==sum(keep & geometryRow), ...
     'CBF:V10GroundQPRows','抽出後のQP行数が不整合です');
  varargout{1}=q;
 case 'lp'
  q=varargin{1};subset=varargin{2};
  out=CBF_Audit_LP(q,subset);
  varargout{1}=out;
 case 'groundmargin'
  x=varargin{1};s=varargin{2};
  pL=x(10);pD=pL-s.physical.cableLength*x(16);
  floor=s.ground.height+s.cbf.safetyMargin;
  margins=[pL-s.shape.loadRadius-floor;pD-s.shape.droneRadius-floor;...
           min(pL,pD)-s.shape.cableRadius-floor];
  varargout{1}=margins;
 case 'feasiblezero'
  q=varargin{1};
  resid=-q.b;
  % 行ごとにスケールした数値残差。無制御ゼロ入力が可行かを判定する。
  varargout{1}=all(resid>=-1e-9*(1+abs(q.b)));
 otherwise
  error('CBF:V9Action','未知の動作: %s',action);
end
end
