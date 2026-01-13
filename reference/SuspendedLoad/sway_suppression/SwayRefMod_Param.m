function p = SwayRefMod_Param()
% 揺れ判定（ヒステリシス）
p.S_on  = 0.25;
p.S_off = 0.18;

% 揺れ指標 S = ||vr_xy|| + sr*||r_xy||
p.sr = 0.3;

% 目標修正ゲイン（位置目標 xd の水平成分を微修正）
%   xd_xy := xd_xy - kv*vr_xy - kr*r_xy
p.kv = 0.6;     % 速度ベース（主）
p.kr = 0.05;    % 位置ベース（補助：小さめ推奨）

% 速度目標(5:7)がある場合の微修正（必要なければ0でもOK）
p.kdv = 0.8;    % 速度目標への速度ベース補正
p.kdr = 0.02;   % 速度目標への位置ベース補正（小さめ）

% CBFゲート（揺れ角制約の代理：||r_xy|| <= L*sin(theta_max)）
p.theta_max = deg2rad(15);  % 許容揺れ角
p.h_gate    = 0.02;         % 危険域のしきい値（m^2）：小さいほど境界に近い

% 危険域(danger)のときの挙動
p.gain_boost = 2.0;             % 危険域での補正強化倍率
p.force_on_when_danger = true;  % 危険域なら揺れ判定に関係なくONにして良い
p.hold_on_when_danger  = true;  % 危険域ならOFFに戻さない
end

