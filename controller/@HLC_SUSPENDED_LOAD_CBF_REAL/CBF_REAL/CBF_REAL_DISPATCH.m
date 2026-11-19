function out = CBF_REAL_DISPATCH(agent,cfg,x,uNom,P,dt,varargin)
% CBF_REAL_DISPATCH  公称物理入力が決まった直後の唯一のCBF分岐。
% cfg.cbfIdはSim初期化時に固定（飛行中に変更しない）。
% この関数に幾何・安全制約・最適化の実装を書かない。
validateattributes(uNom,{'numeric'}, ...
    {'vector','numel',4,'real','finite'});
if abs(double(dt)-cfg.dt)>1e-12
    error('CBFReal:DtMismatch','HLC dt and CBF init dt differ.');
end
switch cfg.cbfId
    case 0
        u=min(max(uNom(:),[0;-1;-1;-1]),[20;1;1;1]);
        info=struct('success',true,'status','CBF_OFF','cbfId',0, ...
            'nominal',uNom(:),'modified',false,'elapsedMs',0);
    case 1
        % 機体・紐・吊り荷の近似とCBF設定はこの1ファイルが所有する。
        [u,info]=CBF_REAL_SCHEME_01_JOINT_ZOH( ...
            agent,cfg,x,uNom,P,dt,varargin{:});
    otherwise
        error('CBFReal:UnknownCBF','Unknown cbfId=%d.',cfg.cbfId);
end
out=struct('success',logical(info.success),'input',u,'info',info);
end
