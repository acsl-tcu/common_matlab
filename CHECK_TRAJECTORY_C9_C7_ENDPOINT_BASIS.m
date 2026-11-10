function R = CHECK_TRAJECTORY_C9_C7_ENDPOINT_BASIS()
%CHECK_TRAJECTORY_C9_C7_ENDPOINT_BASIS Offline research-only check.
% Lives in common_matlab root. No Flight HLC/plant/sensor calls.
% Output files are stored next to this CHECK, never in current pwd.
root = fileparts(mfilename('fullpath'));
src = fullfile(root,'reference','REPLANNING_SYSTEM','TRAJECTORY_GENERATION');
if exist(fullfile(src,'TRAJECTORY_C9_C7_ENDPOINT_BASIS.m'),'file') ~= 2
    error('C9C7:MissingSource','The trajectory basis source is absent.');
end
addpath(src);
old_rng = rng;
cleanup = onCleanup(@() rng(old_rng)); %#ok<NASGU>
rng(7617,'twister');
R = struct('status','UNTESTED','error_message','','safe_to_commit',false, ...
    'runtime_or_hlc_certified',false,'tests',0,'max_endpoint_error',0, ...
    'max_duration_jac_error',0,'max_free_jac_error',0, ...
    'nested_C9_in_C7_error',0,'degree',23,'C7_free_3D',8,'C9_free_3D',4);
try
    for ord=[7,9]
        q = 24-2*(ord+1);
        for it=1:100
            T=2+3*rand;
            z=1./factorial(0:ord);
            J0=bsxfun(@times,randn(3,ord+1),z);
            J1=bsxfun(@times,randn(3,ord+1),z);
            X=randn(3,q);
            [P,dT,B]=TRAJECTORY_C9_C7_ENDPOINT_BASIS(J0,J1,X,T,ord);
            assert(isequal(size(P),[3,24]) && isequal(size(dT),[3,24]));
            assert(isequal(size(B),[72,3*q]));
            Q=P;
            for k=0:ord
                e=max([abs(Q(:,1)-J0(:,k+1));abs(Q(:,end)-J1(:,k+1))]);
                R.max_endpoint_error=max(R.max_endpoint_error,e);
                if k<ord
                    Q=(23-k)/T*diff(Q,1,2);
                end
            end
            % All derivatives are fixed at the endpoints, including order 7.
            h=1e-6*T;
            Pp=TRAJECTORY_C9_C7_ENDPOINT_BASIS(J0,J1,X,T+h,ord);
            Pm=TRAJECTORY_C9_C7_ENDPOINT_BASIS(J0,J1,X,T-h,ord);
            R.max_duration_jac_error=max(R.max_duration_jac_error, ...
                max(abs((Pp(:)-Pm(:))/(2*h)-dT(:))));
            dX=randn(size(X));
            hX=1e-6;
            Pp=TRAJECTORY_C9_C7_ENDPOINT_BASIS(J0,J1,X+hX*dX,T,ord);
            Pm=TRAJECTORY_C9_C7_ENDPOINT_BASIS(J0,J1,X-hX*dX,T,ord);
            R.max_free_jac_error=max(R.max_free_jac_error, ...
                max(abs((Pp(:)-Pm(:))/(2*hX)-B*dX(:))));
            R.tests=R.tests+1;
            if ord==9
                X7=P(:,9:16);  % 8 interior controls in C7 basis
                P7=TRAJECTORY_C9_C7_ENDPOINT_BASIS(J0(:,1:8),J1(:,1:8),X7,T,7);
                R.nested_C9_in_C7_error=max(R.nested_C9_in_C7_error,max(abs(P(:)-P7(:))));
            end
        end
    end
    % Numerical tolerances are FOR CHECKS ONLY, never flight clearance.
    assert(R.max_endpoint_error<2e-4,'High-order endpoint derivative mismatch.');
    assert(R.max_duration_jac_error<1e-7,'Duration sensitivity mismatch.');
    assert(R.max_free_jac_error<1e-7,'Free-control sensitivity mismatch.');
    assert(R.nested_C9_in_C7_error<1e-10,'Old C9 class must be included in the new C7 class.');
    R.status='PASS_BASIS_IDENTITY__PLANNER_HLC_AND_SAFETY_NOT_PROVEN';
catch ME
    R.status='FAIL_BASIS_CHECK';
    R.error_message=ME.getReport('basic','hyperlinks','off');
end
base='TRAJECTORY_C9_C7_ENDPOINT_BASIS_RESULT';
matpath=fullfile(root,[base,'.mat']);
txtpath=fullfile(root,[base,'.txt']);
zipath=fullfile(root,[base,'_FOR_CHATGPT.zip']);
save(matpath,'R');
fid=fopen(txtpath,'w');
if fid < 0, error('C9C7:File','Could not open results text.'); end
fprintf(fid,'status=%s\nerror=%s\ntests=%d\nmax_endpoint_error=%.17g\nmax_duration_jac_error=%.17g\nmax_free_jac_error=%.17g\nnested_C9_in_C7_error=%.17g\nsafe_to_commit=0\nHLC_proven=0\n', ...
    R.status,R.error_message,R.tests,R.max_endpoint_error,R.max_duration_jac_error, ...
    R.max_free_jac_error,R.nested_C9_in_C7_error);
fclose(fid);
if exist(zipath,'file')==2,delete(zipath);end
zip(zipath,{matpath,txtpath});
end
