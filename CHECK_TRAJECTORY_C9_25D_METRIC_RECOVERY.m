function R=CHECK_TRAJECTORY_C9_25D_METRIC_RECOVERY()
% CHECK ONLY; standalone fast verification of the 25D metric repair.
% Results live at common_matlab root, independently of pwd.
root=fileparts(mfilename('fullpath'));
R=struct('status','FAIL_C9_25D_METRIC_RECOVERY','error_message','', ...
 'tensor_size',[],'nan_count',NaN,'cross_axis_nonzero_count',NaN, ...
 'metric_rank',NaN,'metric_min_diagonal',NaN, ...
 'metric_reconstruction_relative_error',NaN, ...
 'metric_gram_relative_error',NaN,'fd_max_relative_error',NaN, ...
 'elapsed_ms',NaN,'safe_to_commit',false);
clock=tic;
try
 T=27;J0=zeros(3,10);J1=zeros(3,10);
 J0(:,1)=[0;0;3];J1(:,1)=[0;T;3];
 J0(:,2)=[0;1;0];J1(:,2)=[0;1;0];
 X=[zeros(1,8);10:17;3*ones(1,8)];
 S=TRAJECTORY_C9_25D_FULLBODY_TANGENT( ...
    0,T,J0,J1,X,0.37,[0;0;-9.80665],2);
 assert(strcmp(S.status, ...
  'VALID_LOCAL_C9_25D_TANGENT__NOT_A_CONTINUOUS_TIME_CERTIFICATE'),S.status);
 C=S.acceleration_control_J;
 R.tensor_size=size(C);
 assert(isequal(size(C),[3,26,25]),'Invalid 25D acceleration tensor dimensions.');
 R.nan_count=sum(~isfinite(C(:)));
 assert(R.nan_count==0,'25D acceleration tensor contains NaNs or Inf.');
 bad=0;
 for k=1:24
   ax=mod(k-1,3)+1;other=setdiff(1:3,ax);
   tmp=C(other,:,k);
   bad=bad+sum(tmp(:)~=0);
 end
 R.cross_axis_nonzero_count=bad;
 assert(bad==0,'Cross-axis derivatives should be exact zeros.');
 % The *actual* Jacobian is checked against independent perturbations of
 % the 26 acceleration controls, including the physical time column.
 [P,~]=TRAJECTORY_C9_27_SINGLE_PIECE_BASIS(J0,J1,X,T);
 a0=27*26/T^2*diff(P,2,2);
 err=0;
 for k=1:25
    Fp=X;Fm=X;Tp=T;Tm=T;
    if k==25
        h=1e-4;Tp=T+h;Tm=T-h;
    else
        h=1e-5;ax=mod(k-1,3)+1;idx=ceil(k/3);
        Fp(ax,idx)=Fp(ax,idx)+h;
        Fm(ax,idx)=Fm(ax,idx)-h;
    end
    Pp=TRAJECTORY_C9_27_SINGLE_PIECE_BASIS(J0,J1,Fp,Tp);
    Pm=TRAJECTORY_C9_27_SINGLE_PIECE_BASIS(J0,J1,Fm,Tm);
    ap=27*26/Tp^2*diff(Pp,2,2);
    am=27*26/Tm^2*diff(Pm,2,2);
    dfd=(ap-am)/(2*h);
    dd=norm(dfd-C(:,:,k),'fro')/(1+norm(dfd,'fro'));
    err=max(err,dd);
 end
 R.fd_max_relative_error=err;
 assert(err<1e-6,'Acceleration controls do not match 25D finite differences.');
 M=TRAJECTORY_C9_25D_EXACT_ACCELERATION_QR(C,T);
 assert(M.valid,M.status);
 R.metric_rank=M.rank;
 R.metric_min_diagonal=min(abs(diag(M.U)));
 R.metric_reconstruction_relative_error=M.factorization_residual;
 G=TRAJECTORY_C9_13D_STATIC_GRAM(25,true);
 H=zeros(25);
 for a=1:3
    Ca=reshape(C(a,:,:),26,25);
    H=H+T*Ca.'*G*Ca;
 end
 H=0.5*(H+H.');
 R.metric_gram_relative_error=norm(M.H-H,'fro')/max(1,norm(H,'fro'));
 assert(R.metric_rank==25&&R.metric_min_diagonal>0&& ...
        R.metric_reconstruction_relative_error<1e-11&& ...
        R.metric_gram_relative_error<1e-9, ...
        'Degree-27 Gauss/QR metric parity not established.');
 assert(norm(a0,'fro')<1e-8, ...
        'The straight-line fixture itself is not acceleration-free.');
 R.status='PASS_C9_25D_ACCELERATION_JACOBIAN_AND_QR__NOT_FLIGHT_CERTIFIED';
catch ME
 R.error_message=getReport(ME,'extended','hyperlinks','off');
end
R.elapsed_ms=1000*toc(clock);
save(fullfile(root,'TRAJECTORY_C9_25D_METRIC_RECOVERY_RESULT.mat'),'R');
fid=fopen(fullfile(root,'TRAJECTORY_C9_25D_METRIC_RECOVERY_RESULT.txt'),'w');
if fid>0
 fprintf(fid,'status=%s\nerror=%s\n',R.status,R.error_message);
 fprintf(fid,'tensor_finite_failures=%g cross_axis_nonzeros=%g\n', ...
     R.nan_count,R.cross_axis_nonzero_count);
 fprintf(fid,'metric_rank=%g min_R_diag=%.17g QR_err=%.17g gram_err=%.17g FD_err=%.17g\n', ...
     R.metric_rank,R.metric_min_diagonal, ...
     R.metric_reconstruction_relative_error, ...
     R.metric_gram_relative_error,R.fd_max_relative_error);
 fclose(fid);
end
end
