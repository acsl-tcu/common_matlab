function test_CBF_InitialSet()
%TEST_CBF_INITIALSET h>=0だけでは高次安全集合に入らないことを検証。
% 障害物へ高速接近している状態。CBF-QPが解けたとしても
% 補助条件psiの初期非負性を満たさないので安全の証明ではない。
test_CBF_SetupPaths();s=test_CBF_Fixture();
obs=struct('center',[0.8;0;1.40],'radius',0.14);
xV=zeros(20,1);xV(1:3)=[0;0;1.40];xV(4:6)=[4;0;0];
[h,~,qp]=CBF_virtual_B_sphere(xV,s,obs);
assert(all(h>0),'高速接近でも初期幾何条件が正のテスト設定が必要です。');
assert(~all(qp.psiInitiallyNonnegative), ...
       'h>0なのに補助条件psi<0を検出できません。');
assert(any(cellfun(@(p)any(p<0),qp.psi)), ...
       '補助安全条件の陰性が報告されていません。');
fprintf(' [PASS] h>0でも高階補助条件が負になるケースを検出\n');
end
