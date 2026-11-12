function [h,info] = demo_CBF_Usage(xReal,settings,obstacle)
%DEMO_CBF_USAGE 外部から受け取ったパラメータで実入力CBFを評価します。
%
% 数値定数はこの関数に埋め込みません。
%   xReal    : 29要素の実入力拡張状態
%   settings : CBF_RuntimeSettings_Template をもとに全数値を指定
%   obstacle : .center, .radius などを実行時に指定
%
%% 1. 生成器の場所を取得
CBF_initialize();

%% 2. 初回のみCBF/realに生成
if exist('CBF_real_C_sphere','file')~=2
    generateCBF_RealInput();
end

%% 3. 設定を実行時に渡して安全関数を評価
[h,info] = CBF_real_C_sphere(xReal,settings,obstacle);
end
