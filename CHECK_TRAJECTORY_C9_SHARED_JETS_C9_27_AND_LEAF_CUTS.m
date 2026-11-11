function R=CHECK_TRAJECTORY_C9_SHARED_JETS_C9_27_AND_LEAF_CUTS()
% Offline mathematical identities, NOT full Flight/certification A/B.
% Must live in common_matlab/ root. Result files are created there.
root=fileparts(mfilename('fullpath'));
production=fullfile(root,'reference','REPLANNING_SYSTEM','TRAJECTORY_GENERATION');
addpath(production,'-begin');
old=rng;clean=onCleanup(@()rng(old)); %#ok<NASGU>
rng(271113,'twister');
R=struct('status','NOT_RUN','error_message','','safe_to_commit',false, ...
    'actual_HLC_proven',false,'fulltime_collision_proven',false, ...
    'shared_jet_max_scaled_error',0,'shared_jet_max_absolute_error',0, ...
    'degree27_endpoint_control_scaled_error',0, ...
    'degree27_endpoint_physical_abs_error',0, ...
    'degree27_duration_scaled_error',0, ...
    'degree23_embedding_scaled_error',0, ...
    'leaf_cut_gradient_max_scaled_error',0, ...
    'tests',0,'shared_jet_checks',0);
try
    for it=1:100
        T=2+4*rand;
        w=1./factorial(0:9);
        j0=bsxfun(@times,randn(3,10),w);
        j1=bsxfun(@times,randn(3,10),w);
        free=randn(3,4);
        phases=[0,sort(rand(1,11)),1];
        S=TRAJECTORY_C9_DURATION_SENSITIVITY(8,T,j0,j1,free,phases,true);
        U=TRAJECTORY_C9_13D_SHARED_JET_KERNEL(S,T,phases);
        Z=zeros(42,24);Z(1:3,:)=S.piece.control_points_m;
        for j=1:12
            ax=mod(j-1,3)+1;
            k=floor((j-1)/3)+11;
            Z(3*j+ax,k)=1;
        end
        for k=0:6
            Z(40:42,:)=S.dphysical_derivative_controls_dT{k+1};
            B=local_bernstein(23-k,phases);
            legacy=Z*B;
            delta=abs(legacy-U{k+1});
            R.shared_jet_max_absolute_error=max( ...
                R.shared_jet_max_absolute_error,max(delta(:)));
            R.shared_jet_max_scaled_error=max( ...
                R.shared_jet_max_scaled_error,max(delta(:))/max(1,max(abs(legacy(:)))));
            R.shared_jet_checks=R.shared_jet_checks+1;
            if k<6
                Z=(23-k)/T*diff(Z,1,2);
            end
        end
        % 13D local Bernstein-leaf plane analytic differential, the
        % mathematical core of the additional certificate-guided cut.
        u0=0.1+0.3*rand;u1=0.6+0.3*rand;
        [~,C]=TRAJECTORY_C9_BERNSTEIN_SUBINTERVAL_CONTROLS( ...
            S.piece.control_points_m,u0,u1);
        k=randi(24);normal=randn(3,1);normal=normal/norm(normal);
        h=T*1e-6;direction=randn(13,1);
        dX=reshape(direction(1:12),3,4);
        dt=direction(13);
        Pplus=TRAJECTORY_C9_FLAT_BERNSTEIN.build( ...
            8,T+h*dt,j0,j1,free+h*dX);
        Pminus=TRAJECTORY_C9_FLAT_BERNSTEIN.build( ...
            8,T-h*dt,j0,j1,free-h*dX);
        fd=normal.'*(Pplus.control_points_m-Pminus.control_points_m)* ...
            C(:,k)/(2*h);
        g=zeros(13,1);
        for j=1:4,g(3*j-2:3*j)=normal*C(10+j,k);end
        g(13)=normal.'*(S.dcontrol_points_dT_mps*C(:,k));
        model=g.'*direction;
        R.leaf_cut_gradient_max_scaled_error=max( ...
            R.leaf_cut_gradient_max_scaled_error, ...
            abs(fd-model)/max([1,abs(fd),abs(model)]));
        free27=randn(3,8);
        [P27,Dt27,Jx]=TRAJECTORY_C9_27_SINGLE_PIECE_BASIS(j0,j1,free27,T);
        assert(isequal(size(P27),[3 28])&&isequal(size(Jx),[84 24]));
        D=P27;
        for k=0:9
            raw=max([abs(D(:,1)-j0(:,k+1)); ...
                     abs(D(:,end)-j1(:,k+1))]);
            R.degree27_endpoint_physical_abs_error=max( ...
                R.degree27_endpoint_physical_abs_error,raw);
            a=T^k/prod(28-(1:k)); % (27)_k denominator; prod(empty)=1
            R.degree27_endpoint_control_scaled_error=max( ...
                R.degree27_endpoint_control_scaled_error, ...
                raw*a/max(1,max(abs(P27(:)))));
            if k<9,D=(27-k)/T*diff(D,1,2);end
        end
        h=T*1e-6;
        Pplus=TRAJECTORY_C9_27_SINGLE_PIECE_BASIS(j0,j1,free27,T+h);
        Pminus=TRAJECTORY_C9_27_SINGLE_PIECE_BASIS(j0,j1,free27,T-h);
        R.degree27_duration_scaled_error=max( ...
            R.degree27_duration_scaled_error, ...
            max(abs((Pplus(:)-Pminus(:))/(2*h)-Dt27(:)))/ ...
            max(1,max(abs(Dt27(:)))));
        % Exact degree elevation embeds ALL existing C9 degree-23 curves.
        P23=S.piece.control_points_m;
        for n=23:26
            next=zeros(3,n+2);next(:,1)=P23(:,1);
            next(:,end)=P23(:,end);
            for j=1:n
                ratio=j/(n+1);
                next(:,j+1)=ratio*P23(:,j)+(1-ratio)*P23(:,j+1);
            end
            P23=next;
        end
        nested=TRAJECTORY_C9_27_SINGLE_PIECE_BASIS( ...
            j0,j1,P23(:,11:18),T);
        R.degree23_embedding_scaled_error=max( ...
            R.degree23_embedding_scaled_error, ...
            max(abs(nested(:)-P23(:)))/max(1,max(abs(P23(:)))));
        R.tests=R.tests+1;
    end
    % CHECK numerical thresholds, not arbitrary physical safety margins.
    assert(R.shared_jet_max_scaled_error<1e-10,'Shared Jet numerical parity.');
    assert(R.degree27_endpoint_control_scaled_error<1e-10, ...
        'Degree27 endpoint in Bernstein control units.');
    assert(R.degree27_duration_scaled_error<1e-7, ...
        'Degree27 physical duration differential.');
    assert(R.leaf_cut_gradient_max_scaled_error<1e-7, ...
        'Failed-leaf hull analytic local QP gradient.');
    assert(R.degree23_embedding_scaled_error<1e-10, ...
        'Degree23 must embed in degree27 with same C9 jets.');
    R.status='PASS_LOCAL_POLYNOMIAL_IDENTITIES__FULL_FLIGHT_HLC_AND_15_CASE_PENDING';
catch ME
    R.status='FAIL_LOCAL_POLYNOMIAL_IDENTITY';
    R.error_message=ME.getReport('basic','hyperlinks','off');
end
base='TRAJECTORY_C9_SHARED_JETS_C9_27_AND_LEAF_CUTS_RESULT';
matpath=fullfile(root,[base,'.mat']);
textpath=fullfile(root,[base,'.txt']);
zipath=fullfile(root,[base,'_FOR_CHATGPT.zip']);
save(matpath,'R');
fid=fopen(textpath,'w');
if fid<0,error('C9_JET_CHECK:File','Cannot open audit log.');end
fprintf(fid,'status=%s\nerror=%s\ntests=%d\nshared_jet_checks=%d\n', ...
    R.status,R.error_message,R.tests,R.shared_jet_checks);
fprintf(fid,'jet_scaled_error=%.17g\njet_abs_error=%.17g\n', ...
    R.shared_jet_max_scaled_error,R.shared_jet_max_absolute_error);
fprintf(fid,'degree27_endpoint_control_scaled=%.17g\n', ...
    R.degree27_endpoint_control_scaled_error);
fprintf(fid,'degree27_endpoint_physical_absolute=%.17g\n', ...
    R.degree27_endpoint_physical_abs_error);
fprintf(fid,'leaf_cut_gradient_scaled=%.17g\n', ...
    R.leaf_cut_gradient_max_scaled_error);
fprintf(fid,'degree27_duration_scaled=%.17g\nembedded23_scaled=%.17g\n', ...
    R.degree27_duration_scaled_error,R.degree23_embedding_scaled_error);
fprintf(fid,'safe_to_commit=0\nHLC_proven=0\n');
fclose(fid);
if isfile(zipath),delete(zipath);end
zip(zipath,{matpath,textpath});
end

function B=local_bernstein(n,u)
B=zeros(n+1,numel(u));
for i=1:numel(u)
    v=u(i);
    if v==0,B(1,i)=1;continue;end
    if v==1,B(end,i)=1;continue;end
    for k=0:n
        B(k+1,i)=nchoosek(n,k)*v^k*(1-v)^(n-k);
    end
end
end
