function setting = mec_dataset_settings()
% mainGUIで使うMECシミュレーション設定をまとめる。
% シミュレーション時の切替・振幅・外力は、基本的にこのファイルだけを編集する。

% true：ランダムスプライン軌道と学習用ログでデータ収集、false：通常の軌道。
setting.enabled = false;

% true：自動試行、false：GUIのStartとキー操作で手動実行。
setting.auto_trials = true;

% 手動実行時に使うシード。自動実行時は下のseedsを使う。
setting.manual_seed = 1;

% 自動試行のシード一覧。シードごとに軌道と励振の周波数・位相が変わる。
setting.seeds = 1:10;

% true：学習用4入力励振ON、false：4入力励振OFF。
% 励振はenabled=trueのデータ収集モードで、飛行段階fの間だけ加わる。
% setting.excitation_enabled = true;
setting.excitation_enabled = false;

% MEC補償の切替。
% falseにすると補償入力だけOFF（HLC制御と誤差計算は継続）。
setting.use_compensation = true;

% 4入力励振の振幅上限 [推力[N]; rollトルク[N m]; pitchトルク[N m]; yawトルク[N m]]。
% 値を大きくすると外乱も大きくなる。成分を0にするとその入力軸だけ励振しない。
setting.amplitude = [0.5;0.008;0.008;0.006];

% 4入力励振の周波数範囲 [Hz]。
setting.frequency_range = [0.1,2.0];

% 各入力に重ねる正弦波の本数。
setting.component_count = 5;

% 4入力励振の立ち上げ・立ち下げ時間 [s]。
setting.ramp_time = 2;

% データ収集軌道・4入力励振の長さ [s]。
setting.flight_time = 40;

% 実機相当プラントだけに加える世界座標の外力。
% mode：'none'（OFF）、'sweep'（スイープコサイン）、'step'（一定外力）。
setting.wind_mode = 'none';
setting.wind_amplitude = 0.2; % 外力振幅 [N]
setting.wind_axis = 3; % 世界座標の方向。1=x、2=y、3=z
setting.wind_step_start = 10; % step外力の開始時刻 [s]
setting.wind_step_end = 20; % step外力の終了時刻 [s]
setting.wind_sweep_start = 7; % sweep外力の開始時刻 [s]
setting.wind_sweep_duration = 40; % sweep外力の継続時間 [s]
setting.wind_frequency_range = [0.1,2.0]; % sweep周波数 [Hz]
setting.wind_ramp_time = 2; % sweep外力の立ち上げ・立ち下げ [s]

% true：実行中の数値ログを表示、false：表示を抑制。
setting.display_on = true;

% 学習用MATファイルの保存先。
repo_root = fileparts(fileparts(fileparts(mfilename('fullpath'))));
setting.output_dir = fullfile(repo_root,'datasets','MEC_4input');
end
