function diagnostic=CBF_V11_R2CandidateQP(shape,x23,s,obs)
%CBF_V11_R2CANDIDATEQP テスト専用：R2=[T^(4),tau]の硬制約候補。
% 機体/ケーブル r=4, 吊り荷 r=6 の候補を作り、
% 低次作用/入力非線形/退化を必ず同時診断する。
% 全状態でCBF保証を得た関数ではない。実制御器で使用禁止。
mode='R2';u0=zeros(4,1);
[H0,labels]=CBF_V11_JetRows(mode,shape,x23,u0,s,obs,6);
n=size(H0,1);A=zeros(n,4);b=zeros(n,1);psimin=zeros(n,1);
early=zeros(n,1);nonlinear=zeros(n,1);orders=zeros(n,1);
for idx=1:n
 lab=labels{idx};
 if strcmp(lab,'obstacle_load')||strcmp(lab,'ground_load')|| ...
    strcmp(lab,'ground_cable_load_end')
     orders(idx)=6;
 else
     orders(idx)=4;
 end
end
HP=zeros(n,7,4);HM=HP;
for j=1:4
 d=zeros(4,1);d(j)=1;
 HP(:,:,j)=CBF_V11_JetRows(mode,shape,x23,u0+d,s,obs,6);
 HM(:,:,j)=CBF_V11_JetRows(mode,shape,x23,u0-d,s,obs,6);
end
for i=1:n
 r=orders(i);
 gain=s.cbf.hocbfGain;
 if strncmp(labels{i},'ground_',7) && isfield(s.cbf,'groundGain') && ~isempty(s.cbf.groundGain)
    gain=s.cbf.groundGain;
 end
 deriv=H0(i,1:r+1).*factorial(0:r);
 lower=0;
 for k=0:r-1
     lower=lower+nchoosek(r,k)*gain^(r-k)*deriv(k+1);
 end
 % 低次の入力作用がない時だけ、補助条件の再帰が正しい。
 for j=1:4
   A(i,j)=factorial(r)*(HP(i,r+1,j)-HM(i,r+1,j))/2;
   early(i)=max(early(i),max(abs( ...
       (HP(i,1:r,j)-HM(i,1:r,j)).*factorial(0:r-1)/2)));
   nonlinear(i)=max(nonlinear(i),abs(factorial(r)* ...
       (HP(i,r+1,j)+HM(i,r+1,j)-2*H0(i,r+1))));
 end
 b(i)=-(deriv(r+1)+lower);
 psi=zeros(r,1);
 for k=0:r-1
   for m=0:k
     psi(k+1)=psi(k+1)+nchoosek(k,m)*gain^(k-m)*deriv(m+1);
   end
 end
 psimin(i)=min(psi);
end
% 推力Tは4段拡張、tauは物理入力なので直接箱制約にする。
x29=[x23;zeros(6,1)];
[Ain,bin]=cbf_real_input_hocbf(x29,s);
Abox=[zeros(3,1),eye(3);zeros(3,1),-eye(3)];
bbox=[s.input.torqueMin(:);-s.input.torqueMax(:)];
Aall=[A;Ain(1:2,:);Abox];ball=[b;bin(1:2);bbox];
maxEarly=max(early);maxNL=max(nonlinear);
% 診断上の数値しきい値。保証の基準とは呼ばない。
validAffine=maxEarly<1e-7 && maxNL<1e-6;
status='EARLY_OR_NONAFFINE';exitflag=NaN;residual=NaN;unctrl=any(sum(abs(A),2)<1e-10 & b>1e-8);
if validAffine
 try
  opt=optimoptions('linprog','Display','off');
  [sol,~,exitflag]=linprog(zeros(4,1),-Aall,-ball,[],[],[],[],opt);
  if ~isempty(sol),residual=min(Aall*sol-ball);end
  if exitflag>0 && isfinite(residual) && residual>=-1e-6
      status='FEASIBLE_AT_THIS_STATE';
  else
      status='INFEASIBLE_OR_SOLVER_FAILURE';
  end
 catch ME
  status=['SOLVER_ERROR_' ME.identifier];
 end
end
diagnostic=struct('A',A,'b',b,'labels',{labels},'order',orders, ...
  'minPsi',psimin,'maxEarlyInput',maxEarly,'maxNonaffine',maxNL, ...
  'validAffineAtPoint',validAffine,'hasUncontrollableViolation',unctrl, ...
  'solverStatus',status,'solverExitflag',exitflag, 'minResidual',residual, ...
  'fullA',Aall,'fullB',ball);
end
