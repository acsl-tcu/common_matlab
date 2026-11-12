function report=test_CBF_FastCoefficientEquivalence()
%TEST_CBF_FASTCOEFFICIENTEQUIVALENCE 旧計算と解析勾配高速計算の一致。
% テストのみ数値設定を使用し、生成コードには寸法を埋め込まない。
% 球/任意姿勢楕円体、実入力/仮想入力、C/B/Dと非平衡状態を比較。
test_CBF_SetupPaths();s=test_CBF_Fixture();
s.cbf.pruneRedundantGround=false;
obs={struct('type','sphere','center',[.8;.2;1.5],'radius',.19), ...
     struct('type','ellipsoid','center',[1.5;-.35;1.6], ...
     'axes',[.13;.21;.33],'R',localRotation(.31,-.22,.19))};
maxAbsA=0;maxRelA=0;maxAbsB=0;cases=0;
for modeCell={'real','virtual'}
    mode=modeCell{1};
    for shapeCell={'C','B','D'}
        shape=shapeCell{1};
        for stateCase=1:3
            x=localState(mode,s,stateCase);
            s.cbf.fastGeometrySensitivity=false;
            old=cbf_hocbf_scene(mode,shape,x,s,obs);
            s.cbf.fastGeometrySensitivity=true;
            now=cbf_hocbf_scene(mode,shape,x,s,obs);
            assert(isequal(old.labels,now.labels),'CBF:FastLabels','制約の名前が不一致');
            assert(isequal(size(old.A),size(now.A)),'CBF:FastSize','係数の寸法が不一致');
            dA=old.A-now.A; dB=old.b-now.b;
            a=max(abs(dA(:)));
            b=max(abs(dB(:)));
            scaleA=max(1,max(abs(old.A(:))));
            scaleB=max(1,max(abs(old.b(:))));
            maxAbsA=max(maxAbsA,a);maxRelA=max(maxRelA,a/scaleA);
            maxAbsB=max(maxAbsB,b);
            assert(a<=2e-7*scaleA && b<=2e-7*scaleB, ...
                'CBF:FastMismatch','%s %s case%d: A誤差%.6g b誤差%.6g', ...
                mode,shape,stateCase,a,b);
            assert(isequal(old.psiInitiallyNonnegative,now.psiInitiallyNonnegative), ...
                'CBF:FastPsi','補助条件の初期判定が不一致');
            assert(isequal(old.order,now.order),'CBF:FastOrder','設計次数が不一致');
            assert(isequal(old.degenerate,now.degenerate), ...
                'CBF:FastDegenerate','入力係数の退化判定が不一致');
            assert(isequal(old.uncontrollableViolation,now.uncontrollableViolation), ...
                'CBF:FastUncontrollable','非制御可能制約の判定が不一致');
            cases=cases+1;
        end
        fprintf('[PASS] %-7s %s: 球/任意姿勢楕円体 x 3状態\n',mode,shape);
    end
end
report=struct('cases',cases,'maxAbsA',maxAbsA, ...
    'maxRelativeA',maxRelA,'maxAbsB',maxAbsB);
fprintf('[PASS] 計%dケース  A最大相対誤差 %.3g, b最大絶対誤差 %.3g\n', ...
    cases,maxRelA,maxAbsB);
fprintf('※ MATLAB上の一致試験。これだけで閉ループ安全性は保証しません。\n');
end

function x=localState(mode,s,variant)
if strcmp(mode,'real')
    x=zeros(29,1);
    a=.11*(variant-1);b=-.07*(variant-1);
    qq=[cos(a/2)*cos(b/2);sin(a/2)*cos(b/2); ...
        cos(a/2)*sin(b/2);-sin(a/2)*sin(b/2)];
    x(1:4)=qq/norm(qq);
    x(5:7)=[.08;-.04;.02]*(variant-1);
    x(8:10)=[.06*(variant-1);-.03*(variant-1);1.4];
    x(11:13)=[.04;-.03;.02]*(variant-1);
    pt=[.12*(variant-1);.06*(variant-1);-1];
    x(14:16)=pt/norm(pt);
    x(17:19)=[.08;.05;0]*(variant-1);
    x(20)=(s.physical.massDrone+s.physical.massLoad)*s.physical.gravity;
    x(21:23)=[.03;-.02;.01]*(variant-1);
    x(24:26)=[.04;-.01;.01]*(variant-1);
    x(27:29)=[.02;.03;-.01]*(variant-1);
else
    x=zeros(20,1);x(1:3)=[.06*(variant-1);-.02*(variant-1);1.4];
    x(4:6)=[.05;-.02;.01]*(variant-1);
    x(7:9)=[.11;-.15;.07]*(variant-1);
    x(10:12)=[.15;.04;-.06]*(variant-1);
    x(13:15)=[.05;-.08;.03]*(variant-1);
    x(16:18)=[.11;-.07;.06]*(variant-1);
end
end
function R=localRotation(a,b,c)
Rz=[cos(a) -sin(a) 0;sin(a) cos(a) 0;0 0 1];
Ry=[cos(b) 0 sin(b);0 1 0;-sin(b) 0 cos(b)];
Rx=[1 0 0;0 cos(c) -sin(c);0 sin(c) cos(c)];
R=Rz*Ry*Rx;
end
