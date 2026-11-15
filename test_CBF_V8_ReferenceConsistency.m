function ok=test_CBF_V8_ReferenceConsistency(varargin)
% 高次参照の初期0～6階整合と導関数の近傍一致を確認。
names={'hover','line','circle','ascent','figure8'};
p0=[0;0;1.5];T=3;
for nameCell=names
 name=nameCell{1};
 for j=1:6
   val=CBF_V8_Reference(name,0,j,p0,T);
   assert(norm(val,Inf)<1e-7,'CBF:V8InitialDerivativeNonzero',...
       '参照の初期高階微分が非ゼロ: %s, d=%d',name,j);
 end
 assert(norm(CBF_V8_Reference(name,0,0,p0,T)-p0,Inf)<1e-12);
 for t=[.7 1.5 2.2]
  for j=0:4
    d=.0001;
    v1=CBF_V8_Reference(name,t+d,j,p0,T);
    v2=CBF_V8_Reference(name,t-d,j,p0,T);
    expected=CBF_V8_Reference(name,t,j+1,p0,T);
    err=norm((v1-v2)/(2*d)-expected,Inf);
    assert(err<2e-3*(1+norm(expected,Inf)),...
     'CBF:V8RefDerivativeMismatch','参照微分の数値比較が不一致: %s d%d',name,j);
  end
 end
end
fprintf('[PASS] 5参照: 0～6階初期整合と近傍導関数。\n');
ok=true;
end
