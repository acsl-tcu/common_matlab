function report=test_CBF_V11_VirtualHLC(opts)
%TEST_CBF_V11_VIRTUALHLC  既存HLCの生成済み関数へ高階仮想入力を接続する前の整合性監査。
% 20状態(pLの0～5階,yaw0～1階)、nuV=[pL6;yaw2]、v1^(0..4)を別途復元。
% 仮想CBFの最終的な物理入力制約を保証する完成版ではありません。
% V11.1 修正: HLC実行各段階の例外識別子・本文・発生行をCSVへ保存。
% V11.2 修正: Generated V2_alpha2 の入力 in5 は 3x1 必須。
%   obj.Vs は 1x3 を返すため、vs(:).' ではなく vs(:) で呼ぶ。
%   根拠: 既存 HLC_SUSPENDED_LOAD.do も vs' を渡す。
%   本体HLC関数は無変更。
% 追跡用 opts.failFast=true を指定すると最初のエラーをMATLAB画面に再送出。
if nargin<1,opts=struct();end
test_CBF_SetupPaths();s=test_CBF_Fixture();
common=fileparts(mfilename('fullpath'));
root=fullfile(common,'controller');
if isfield(opts,'hlcFolder') && ~isempty(opts.hlcFolder)
 root=char(opts.hlcFolder);
end
[~,last]=fileparts(root);
if strcmp(last,'@HLC_SUSPENDED_LOAD'),root=fileparts(root);end
rows=cell(0,19);n=0;
if isfolder(fullfile(root,'@HLC_SUSPENDED_LOAD'))
 addpath(root,'-begin');
end
if exist('HLC_SUSPENDED_LOAD','class')~=8
 n=n+1;
 rows(n,:)={0,'HLC_CLASS_MISSING',NaN,NaN,NaN,NaN,NaN,NaN,NaN, ...
    NaN,NaN,NaN,NaN,NaN,NaN,root, ...
    'preflight','CBF:HLCClassMissing','common_matlab/controller/@HLC_SUSPENDED_LOAD が見つかりません'};
else
 dt=0.01;if isfield(opts,'dt'),dt=opts.dt;end
 A2=[0 1;0 0];B2=[0;1];
 F1=lqrd(A2,B2,diag([150,10]),0.01,dt);
 if isfield(opts,'F1') && ~isempty(opts.F1),F1=opts.F1(:).';end
 F1=F1(:).';
 obj=HLC_SUSPENDED_LOAD([],struct());
 P=[s.physical.massDrone;s.physical.inertia(:);s.physical.gravity; ...
    s.physical.massLoad;s.physical.cableLength;s.physical.disturbanceXY(:)].';
 for cid=1:4
  stage='START';
  [x19,u,~]=CBF_V11_Examples(cid,s,'sphere',1);
  try
   stage='physical_taylor_and_virtual_state';
   % 元物理モデルの連続時間Taylorから仮想状態を生成する。
   Z=CBF_V11_RealTaylor('R1',x19,u,s,6);
   xv=zeros(20,1);
   for j=0:5
       xv(3*j+1:3*j+3)=factorial(j)*Z(8:10,j+1);
   end
   xv(19:20)=0; % 試験はyaw=0。幾何比較ではyawを使用しない。
   nuV=[factorial(6)*Z(8:10,7);0];
   ref=zeros(3,7);ref(:,1)=x19(8:10);
   if cid>1,ref(3,1)=ref(3,1)+0.2;end
   v1=zeros(1,5);
   for j=0:3
       v1(j+1)=xv(3*(j+2)+3)-ref(3,j+3);
   end
   v1(5)=nuV(3)-ref(3,7);
   % 第一層の名目Vfdは比較対象。CBF補正後の時間導関数と同一とは仮定しない。
   XD=zeros(1,28);
   for j=0:6
       XD(4*j+1:4*j+3)=ref(:,j+1).';
   end
   stage='HLC_Vfd_SuspendedLoadxyDst';
   vfNom=obj.Vfd_SuspendedLoadxyDst(dt,x19,XD,F1);
   assert(numel(vfNom)>=5 && all(isfinite(vfNom(1:5))), ...
       'CBF:V11VfdInvalid','Vfdの出力が不足または非有限です。');
   e=[x19(10)-ref(3,1);x19(13)-ref(3,2)];
   Acl=[0 1;-F1(1) -F1(2)];
   trueNom=zeros(1,5);
   for j=0:4,trueNom(j+1)=-F1*(Acl^j)*e;end
   vfd0error=abs(vfNom(1)-trueNom(1));
   vfdHigherError=norm(vfNom(2:5)-trueNom(2:5),Inf);
   % C/B/Dの実/仮想幾何を、同一物理状態で6階までのpLから復元。
   minGeometryError=0;
   stage='physical_virtual_flat_geometry';
   for shape={'C','B','D'}
    [~,~,pD]=cbf_virtual_state_taylor(xv,nuV,cbf_parameter_vector(s),4);
    ZD=Z(8:10,1:5)-s.physical.cableLength*Z(14:16,1:5);
    minGeometryError=max(minGeometryError,norm(pD-ZD,Inf));
   end
   % HLCの反転（Ufはv1、Beta2/alpha2はv1^(4)まで）。
   vfActual=[v1,0];
   stage='HLC_Uf_SuspendedLoadxyDst';
   thrust=obj.Uf_SuspendedLoadxyDst(x19,XD,vfActual,P);
   stage='HLC_Beta2_SuspendedLoadxyDst';
   beta=obj.Beta2_SuspendedLoadxyDst(x19,XD,vfActual,P);
   assert(all(size(beta)==[3 3]),'CBF:V11BetaShape', ...
       'Beta2は3行3列でなければなりません。');
   stage='HLC_gain_F2F3F4';
   v2gain=lqrd(diag(ones(1,5),1),[zeros(5,1);1], ...
       diag([100,150,100,10,1,0.1]),0.001,dt);
   f4=lqrd(A2,B2,diag([10,1]),0.01,dt);
   stage='HLC_Vs_SuspendedLoadxyDst';
   vs=obj.Vs_SuspendedLoadxyDst(x19,XD,vfActual,P,v2gain,v2gain,f4);
   assert(isequal(size(vs),[1 3]) && all(isfinite(vs(:))), ...
       'CBF:V11VsShape', ...
       '生成Vsは有限な1x3の行ベクトルでなければなりません。');
   % 生成済みV2_alpha2は in5(1,:),in5(2,:),in5(3,:) を参照する。
   % 元のHLC_SUSPENDED_LOAD.doと同じ列ベクトル形で渡す。
   vsColumn=vs(:);
   assert(isequal(size(vsColumn),[3 1]), ...
       'CBF:V11Alpha2InputShape','V2_alpha2の仮想入力は3x1が必要です。');
   stage='HLC_V2_alpha2_SuspendedLoadxyDst';
   rhs=obj.V2_alpha2_SuspendedLoadxyDst(x19,XD,vfActual,vsColumn,P);
   assert(numel(rhs)==3 && all(isfinite(rhs(:))), ...
       'CBF:V11AlphaInvalid','V2_alpha2は有限な3要素でなければなりません。');
   stage='HLC_inverse_and_checks';
   betaCond=rcond(beta);
   physicalTorque=[NaN;NaN;NaN];
   if betaCond>1e-12
       physicalTorque=beta\rhs(:);
   end
   % v1^(4) の上流変化が第二層トルクに反映されるかを数値診断。
   stage='HLC_v1fourth_sensitivity';
   delta=1e-4;vfP=vfActual;vfM=vfActual;
   vfP(5)=vfP(5)+delta;vfM(5)=vfM(5)-delta;
   rhsP=obj.V2_alpha2_SuspendedLoadxyDst(x19,XD,vfP,vsColumn,P);
   rhsM=obj.V2_alpha2_SuspendedLoadxyDst(x19,XD,vfM,vsColumn,P);
   sensitivity=norm((rhsP(:)-rhsM(:))/(2*delta),2);
   rawT=thrust(1);
   inputOK=rawT>=s.input.thrustMin && rawT<=s.input.thrustMax && ...
      all(physicalTorque>=s.input.torqueMin(:)) && ...
      all(physicalTorque<=s.input.torqueMax(:));
   if cid==1 && (minGeometryError>1e-6 || vfd0error>1e-6)
       status='FAILED_HOVER_IDENTITY';
   else
       status='EVALUATED_NOT_SAFETY_PROOF';
   end
   n=n+1;
   rows(n,:)={cid,status,rawT,u(1),norm(physicalTorque,Inf), ...
       double(inputOK),betaCond,minGeometryError, ...
       vfd0error,vfdHigherError,v1(1),v1(5), ...
       sensitivity,norm(vfActual(1:5)-vfNom(1:5),Inf), ...
       norm(nuV(1:3),Inf),root,'','',''};
  catch ME
   % エラーを握りつぶさない。段階・識別子・メッセージ・位置をCSVへ残す。
   loc='';
   if ~isempty(ME.stack)
       loc=sprintf(' [%s:%d]',ME.stack(1).name,ME.stack(1).line);
   end
   detail=sprintf('%s%s',ME.message,loc);
   detail=regexprep(detail,'[\r\n]+',' ');
   fprintf(2,'[V11 HLC ケース%d / %s] %s: %s\n',cid,stage,ME.identifier,detail);
   n=n+1;
   rows(n,:)={cid,['TEST_ERROR_' ME.identifier],NaN,NaN,NaN, ...
       NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN,root, ...
       stage,ME.identifier,detail};
   if isfield(opts,'failFast') && opts.failFast
       % この設定時は意図的にMATLABの完全な例外スタックを表示する。
       rethrow(ME);
   end
  end
 end
end
report=cell2table(rows,'VariableNames', ...
 {'stateId','status','recoveredThrust','physicalThrust','recoveredTorqueInf', ...
 'withinHardPhysicalLimits','betaRcond','physicalVirtualGeometryError', ...
 'VfdZeroOrderError','VfdHigherVsContinuousError', ...
 'v1Value','v1FourthDerivative','torqueRhsSensitivityToV1Fourth', ...
 'actualVsNominalVfdDifference','physicalLoadSixthInf','hlcFolder', ...
 'errorStage','errorIdentifier','errorText'});
writetable(report,fullfile(CBF_V11_OutputFolder(),'v11_virtual_hlc_chain.csv'));
fprintf('[V11] 仮想HLC微分鎖：%d条件。HLCが無ければ明示的MISSING。\n',height(report));
end
