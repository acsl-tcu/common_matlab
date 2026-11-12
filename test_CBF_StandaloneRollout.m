function test_CBF_StandaloneRollout()
%TEST_CBF_STANDALONEROLLOUT HLCなしの小規模な数値閉ループ試験。
% 目的: QP解→入力を固定→拡張状態積分→幾何/高次条件を再評価する。
% 仮想: pLの6重積分器を厳密な多項式解で進める。
% 実入力: 元generateModelの数式から独立記述したRHSをRK4で積分。
% 単機、張り紐、静止球障害物、短時間。任意初期状態保証ではない。
paths=test_CBF_SetupPaths();s=test_CBF_Fixture();
obs=struct('center',[0.8;0;1.40],'radius',0.14);
W=eye(4);

%% 1. 仮想入力モデル: HLCを模擬せず、あえて大きな6階名目入力を与える
x=zeros(20,1);x(1:3)=[0;0;1.40];
dt=0.025;steps=24;logV=zeros(steps,8);changed=0;
for k=1:steps
    [h,~,qp]=CBF_virtual_B_sphere(x,s,obs);
    [mnPsi,ok]=checkInitial(qp);
    assert(ok,'仮想系: ステップ%dの初期高次安全条件が破れています。',k);
    nuNom=[100;0;0;0]; % ストレス入力。HLCではない。
    sol=cbf_solve_hocbf_qp(qp,nuNom,W);
    assert(sol.exitflag>0 && sol.minResidual>=-1e-7, ...
      '仮想系: ステップ%dでQP実行不能、または残差が負。',k);
    changed=changed+(norm(sol.nu-nuNom)>1e-6);
    logV(k,:)=[(k-1)*dt,min(h),mnPsi,sol.minResidual, ...
               norm(sol.nu-nuNom),nan,nan,double(any(qp.degenerate))];
    x=virtualStep(x,sol.nu,dt);
    hNext=CBF_virtual_B_sphere(x,s,obs);
    assert(all(hNext>=-1e-7),'仮想系: ステップ%dで幾何安全関数が負。',k);
end
assert(changed>0,'仮想系: QPがストレス名目入力を一度も修正していません。');
fprintf(' [PASS] 仮想系: %d回のQPと時間積分、修正 %d回\n',steps,changed);

%% 2. 実入力拡張系: 推力・トルクとモデル制約を同時に監視
x=zeros(29,1);x(1)=1;x(8:10)=[0;0;1.40];x(14:16)=[0;0;-1];
x(20)=(s.physical.massDrone+s.physical.massLoad)*s.physical.gravity;
dt=0.01;steps=25;logR=zeros(steps,8);changed=0;
for k=1:steps
    [h,~,qp]=CBF_real_B_sphere(x,s,obs);
    [mnPsi,ok]=checkInitial(qp);
    assert(ok,'実入力系: ステップ%dの初期高次安全条件が破れています。',k);
    nuNom=[0;20;0;0]; % torque_x二階微分のストレス入力。HLCではない。
    sol=cbf_solve_hocbf_qp(qp,nuNom,W);
    assert(sol.exitflag>0 && sol.minResidual>=-1e-7, ...
      '実入力系: ステップ%dでQP実行不能、または残差が負。',k);
    changed=changed+(norm(sol.nu-nuNom)>1e-6);
    logR(k,:)=[(k-1)*dt,min(h),mnPsi,sol.minResidual, ...
               norm(sol.nu-nuNom),x(20),max(abs(x(24:26))),double(any(qp.degenerate))];
    x=realRK4Step(x,sol.nu,dt,s);
    assert(abs(norm(x(1:4))-1)<1e-6 && abs(norm(x(14:16))-1)<1e-6, ...
       '実入力系: 単位クォータニオン/ケーブル拘束が積分で崩れました。');
    assert(x(20)>=s.input.thrustMin-1e-7 && x(20)<=s.input.thrustMax+1e-7, ...
       '実入力系: 推力の制限に違反。');
    assert(all(x(24:26)>=s.input.torqueMin-1e-7) && ...
           all(x(24:26)<=s.input.torqueMax+1e-7), ...
       '実入力系: トルク制限に違反。');
    physicalInfo=cbf_model_diagnostics('real',x,s);
    assert(physicalInfo.tension>0, ...
       '実入力系: ケーブルの張力が正でなくなりました。');
    hNext=CBF_real_B_sphere(x,s,obs);
    assert(all(hNext>=-1e-7),'実入力系: ステップ%dで幾何安全関数が負。',k);
end
assert(changed>0,'実入力系: QPがストレス名目入力を一度も修正していません。');
fprintf(' [PASS] 実入力系: %d回のQPと時間積分、修正 %d回\n',steps,changed);

%% 3. 自動ログ（生成器のCBF/test_resultsへ保存）
folder=fullfile(paths.generator,'CBF','test_results');
if exist(folder,'dir')~=7,mkdir(folder);end
headers={'time_s','min_h','min_psi','min_QP_residual', ...
         'correction_norm','thrust_N','max_abs_torque_Nm','has_degenerate_row'};
writeCSV(fullfile(folder,'standalone_virtual_B_sphere.csv'),headers,logV);
writeCSV(fullfile(folder,'standalone_real_B_sphere.csv'),headers,logR);
fprintf(' [INFO] 結果CSV: %s\n',folder);
end

function [v,ok]=checkInitial(qp)
% .psiにはそれぞれ h,psi_1,...,psi_(r-1) を保持している。
allPsi=vertcat(qp.psi{:});
if isempty(allPsi),v=inf;else,v=min(allPsi);end
ok=all(qp.psiInitiallyNonnegative) || v>=-1e-8;
end

function xn=virtualStep(x,nu,dt)
% 6階積分器の正確な時間解。2次yaw系も同様。
xn=x;
for j=0:5
    value=zeros(3,1);
    for l=j:5
        value=value+x(3*l+1:3*l+3)*dt^(l-j)/factorial(l-j);
    end
    value=value+nu(1:3)*dt^(6-j)/factorial(6-j);
    xn(3*j+1:3*j+3)=value;
end
xn(19)=x(19)+dt*x(20)+0.5*dt^2*nu(4);
xn(20)=x(20)+dt*nu(4);
end

function xn=realRK4Step(x,nu,dt,s)
% 独立運動方程式。毎積分ステップの終了時に状態を再評価する。
k1=test_CBF_IndependentPlantRHS(x,nu,s);
k2=test_CBF_IndependentPlantRHS(x+dt*k1/2,nu,s);
k3=test_CBF_IndependentPlantRHS(x+dt*k2/2,nu,s);
k4=test_CBF_IndependentPlantRHS(x+dt*k3,nu,s);
xn=x+dt*(k1+2*k2+2*k3+k4)/6;
end

function writeCSV(file,headers,data)
fp=fopen(file,'w');
if fp<0,error('CBF:LogFile','ログに書き込めません: %s',file);end
cleanup=onCleanup(@()fclose(fp));
fprintf(fp,'%s\n',strjoin(headers,','));
for k=1:size(data,1)
    fprintf(fp,'%.15g,%.15g,%.15g,%.15g,%.15g,%.15g,%.15g,%.15g\n',data(k,:));
end
end
