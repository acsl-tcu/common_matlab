function test_CBF_RuntimeParameters()
%TEST_CBF_RUNTIMEPARAMETERS 生成後に寸法を変えても再生成不要なことを確認。
test_CBF_SetupPaths();
if exist('CBF_real_C_sphere','file')~=2
    generateCBF_RealInput();
end

%% 1. 外部設定の数値を別々に用意
s1=test_CBF_Fixture();
s2=s1;
s2.physical.cableLength=2*s1.physical.cableLength;
s2.shape.droneRadius=1.3*s1.shape.droneRadius;
obs=struct('center',[2;0;1.2],'radius',0.25);

%% 2. 生成された同じ1つの関数へ別々の設定を渡す
x=zeros(29,1);x(1)=1;x(8:10)=[0;0;1];x(14:16)=[0;0;-1];
x(20)=(s1.physical.massDrone+s1.physical.massLoad)*s1.physical.gravity;
[h1,meta1]=CBF_real_C_sphere(x,s1,obs);
[h2,meta2]=CBF_real_C_sphere(x,s2,obs);
assert(meta2.numberOfCableSpheres>meta1.numberOfCableSpheres, ...
    'ケーブル長が変わっても球数が変わりませんでした。');
assert(numel(h2)>numel(h1),'球数の変化が安全関数に反映されていません。');

%% 3. 未入力のテンプレートはエラーになること
failed=false;
try
    CBF_real_C_sphere(x,CBF_RuntimeSettings_Template(),obs);
catch
    failed=true;
end
assert(failed,'設定の空欄を拒否しませんでした。');
fprintf('[PASS] 数値は生成時に固定されず、実行時に変更可能です。\n');
end
