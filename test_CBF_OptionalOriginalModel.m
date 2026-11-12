function test_CBF_OptionalOriginalModel(physicalParam)
%TEST_CBF_OPTIONALORIGINALMODEL  ユーザの既存FLxyDst/GLxyDstとの一致を調べる任意試験。
% 準備: ご自身の環境でFLxyDst, GLxyDstを生成しMATLAB pathに配置。
% 注意: generateModel内に同名関数の生成箇所が複数あるため、
%       19状態を取る版を使ってください。
% 引数: physicalParam は既存関数用の22要素（元の並びのまま）。
if nargin<1,error('CBF:OriginalParameters','既存の22要素 physicalParam を渡してください。');end
if exist('FLxyDst','file')~=2 || exist('GLxyDst','file')~=2
    error('CBF:OriginalMissing','FLxyDst/GLxyDstがMATLAB pathにありません。');
end
test_CBF_SetupPaths();
s=test_CBF_Fixture();
M=cbf_symbolic_model('real');
fn=matlabFunction(M.f(1:19),'Vars',{M.x,M.p});
x=zeros(29,1);x(1)=1;x(8:10)=[0;0;1];x(14:16)=[0;0;-1];
x(20)=(s.physical.massDrone+s.physical.massLoad)*s.physical.gravity;
P=cbf_parameter_vector(s);
F0=FLxyDst(x(1:19),physicalParam);
G0=GLxyDst(x(1:19),physicalParam);
if ~isequal(size(F0),[19,1]) || ~isequal(size(G0),[19,4])
    error('CBF:OriginalMismatch','現在のFLxyDst/GLxyDstは19状態の版ではありません。');
end
v0=F0+G0*[x(20);x(24:26)];
v1=fn(x,P);
assert(norm(v0-v1,Inf)<1e-8, ...
    'モデル不一致: 外乱、物理パラメータ、生成したFL/GLの版を確認してください。');
fprintf('[PASS] 元モデルと実入力拡張モデルの19状態の運動方程式が一致します。\n');
end
