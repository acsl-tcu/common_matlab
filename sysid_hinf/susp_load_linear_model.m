function [P, info] = susp_load_linear_model(agent, opt)
% susp_load_linear_model  トルク(τy)→ 牽引物の揺れ角(θ) の線形モデルを構築する.
%
%   [P, info] = susp_load_linear_model(agent, opt)
%
%   単機牽引（吊り下げ負荷 = suspended load）ドローンの，ピッチ面内における
%   「ピッチトルク τy → 負荷の揺れ角 θ」の連続時間線形モデルを作る．
%   この解析モデルは，実験データによる同定（identify_sway_tf.m）の
%   初期値・妥当性チェック，および H∞ 設計（design_hinf_sway.m）の
%   ノミナルモデルとして用いる．
%
%   === 物理（ピッチ x-z 平面，微小角） ==================================
%     状態         x = [beta; beta_dot; theta; theta_dot]
%       beta   : 機体ピッチ角  [rad]   （y軸まわり）
%       theta  : 負荷の揺れ角  [rad]   （鉛直からの傾き．pT より算出）
%     入力         u = tau_y            機体ピッチトルク [N*m]
%     出力         y = theta            揺れ角 [rad]
%
%     機体姿勢 :   Jy*beta'' = tau_y - c_att*beta'          (ロータ抗力 c_att)
%     負荷振り子 : M*L*theta'' + (M+mp)*g*theta = -f0*beta   (微小角の連成)
%       → theta'' = -wn^2*(theta + beta) - 2*zeta_p*wn*theta'
%       wn^2 = (M+mp)*g/(M*L)      （機体質量 M と ケーブル長 L．負荷質量 mp ではない）
%       f0   = (M+mp)*g            （ホバリング推力）
%
%     伝達関数（c_att, zeta_p を無視した理想形）:
%       P(s) = theta/tau = -wn^2 / [ Jy*s^2 * (s^2 + wn^2) ]
%     → 姿勢の二重積分 × 軽減衰の振り子共振（ω_n）を直列にした 4 次系．
%       この共振ピークが「牽引物がふらふらする」正体．
%   =====================================================================
%
%   入力:
%     agent : DRONE インスタンス（省略/空なら opt の既定パラメータを使用）.
%             agent.parameter が DRONE_PARAM_SUSPENDED_LOAD の場合，
%             mass/loadmass/cableL/gravity/jy をそこから取得する.
%     opt   : name-value（すべて任意）
%             "zeta_p" : 振り子の減衰比（既定 0.03．実測で更新推奨）
%             "c_att"  : 機体姿勢のロータ抗力係数（既定 0.02）
%             "mass","loadmass","cableL","gravity","jy" : 手動上書き
%
%   出力:
%     P    : ss  トルク→揺れ角 の連続時間状態空間モデル（1入力1出力）
%     info : struct  wn, f0, 使用パラメータ, poles など

arguments
  agent = []
  opt.zeta_p (1,1) double = 0.03
  opt.c_att  (1,1) double = 0.02
  opt.mass     double = []
  opt.loadmass double = []
  opt.cableL   double = []
  opt.gravity  double = []
  opt.jy       double = []
end

% --- パラメータ取得（agent 優先 → opt 上書き → 既定 DIATONE 値） ---
% 既定は acsl-tcu/common_matlab の DRONE_PARAM_SUSPENDED_LOAD("DIATONE")
def = struct("mass",0.762,"loadmass",0.0556,"cableL",0.46, ...
             "gravity",9.81,"jy",0.02985236);
if ~isempty(agent) && isprop(agent,"parameter") && ~isempty(agent.parameter)
  p = agent.parameter;
  for f = ["mass","loadmass","cableL","gravity"]
    if isprop(p,f), def.(f) = p.(f); end
  end
  if isprop(p,"jy"), def.jy = p.jy; elseif isprop(p,"Jy"), def.jy = p.Jy; end
end
fn = fieldnames(def);
for i = 1:numel(fn)
  if ~isempty(opt.(fn{i})), def.(fn{i}) = opt.(fn{i}); end
end

M  = def.mass;  mp = def.loadmass;  L = def.cableL;
g  = def.gravity;  Jy = def.jy;
zeta_p = opt.zeta_p;  c_att = opt.c_att;

wn2 = (M+mp)*g/(M*L);          % 振り子固有角周波数^2
wn  = sqrt(wn2);
f0  = (M+mp)*g;                % ホバリング推力

% --- 状態空間  x=[beta;beta_dot;theta;theta_dot], u=tau_y, y=theta ---
A = [ 0        1          0            0        ;
      0    -c_att/Jy      0            0        ;
      0        0          0            1        ;
    -wn2       0        -wn2     -2*zeta_p*wn  ];
B = [0; 1/Jy; 0; 0];
C = [0 0 1 0];
D = 0;

P = ss(A,B,C,D);
P.InputName  = "tau_y";
P.OutputName = "theta";
P.StateName  = ["beta","beta_dot","theta","theta_dot"];

info = struct();
info.wn      = wn;            % [rad/s]
info.fn      = wn/2/pi;       % [Hz]
info.zeta_p  = zeta_p;
info.f0      = f0;
info.params  = def;
info.poles   = pole(P);
info.tf_ideal = tf(-wn2, conv([Jy 0 0],[1 0 wn2]));  % 理想（減衰無視）伝達関数

if nargout == 0
  fprintf("wn = %.4f rad/s (%.4f Hz),  hover thrust f0 = %.4f N\n", wn, info.fn, f0);
  disp("poles:"); disp(info.poles);
end
end
