function data = Remake_Normalization(Data)
% 誤差系の正規化
% e[k+1] = A*e[k] + B*delta_u[k]
%
% E      : e[k]
% E_next : e[k+1]
% dU     : delta_u[k]
%
% 平均値は引かず、標準偏差でスケーリングする。

%% データの確認
assert(isequal(size(Data.E), size(Data.E_next)), ...
    'EとE_nextのサイズが一致しません');

assert(size(Data.E,2) == size(Data.dU,2), ...
    'EとdUのデータ数が一致しません');

assert(all(isfinite([Data.E(:); Data.E_next(:); Data.dU(:)])), ...
    'データにNaNまたはInfが含まれています');

%% 標準偏差の計算

% 状態の標準偏差
stdE = std(Data.E, 0, 2);

% 入力の標準偏差
stdU = std(Data.dU, 0, 2);

%% ゼロ除算の防止

scaleE = stdE;
scaleU = stdU;

% 標準偏差が極端に小さい状態はスケーリングしない
tol = 1e-12;

scaleE(scaleE < tol) = 1;
scaleU(scaleU < tol) = 1;

%% 正規化

% EとE_nextには同じスケールを使用する
data.E = Data.E ./ scaleE;
data.E_next = Data.E_next ./ scaleE;

% 入力の正規化
data.dU = Data.dU ./ scaleU;

%% 正規化係数を保存

data.scaleE = scaleE;
data.scaleU = scaleU;

data.stdValue.e = scaleE;
data.stdValue.u = scaleU;

data.Se = diag(scaleE);
data.Su = diag(scaleU);

fprintf('誤差状態の正規化が完了しました\n');

end