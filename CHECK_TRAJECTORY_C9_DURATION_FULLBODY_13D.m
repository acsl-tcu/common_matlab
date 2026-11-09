function R=CHECK_TRAJECTORY_C9_DURATION_FULLBODY_13D()
% Temporary diagnostic only. At fixed phase and fixed physical time verify
% ALL 13 coordinate derivatives for the three ideal taut-body centers.
% Does not authorize a Flight commit or make a continuous-time claim.
root=fileparts(mfilename('fullpath'));
name='TRAJECTORY_C9_DURATION_FULLBODY_13D_RESULT';
R=struct('status','FAIL_NOT_RUN','stage','INIT', ...
 'error_identifier','','error_message','','error_stack',{{}}, ...
 'max_jacobian_error_rel',NaN,'max_fixed_time_error_rel',NaN, ...
 'max_cable_error_rel',NaN,'max_direction_orthogonality',NaN, ...
 'max_fullbody_affine_residual',NaN,'max_so3_covariance_error',NaN, ...
 'batch_default_enabled_on_multiple_times',false, ...
 'batch_explicit_off_parity',false, ...
 'safe_to_commit',false,'actual_hlc_tracking_proven',false, ...
 'zip_path',fullfile(root,[name '_FOR_CHATGPT.zip']));
try
    addpath(fullfile(root,'reference','REPLANNING_SYSTEM', ...
        'TRAJECTORY_GENERATION'),'-begin');
    R.stage='13D_FULLBODY_DIFFERENTIAL_VS_FINITE_DIFFERENCE';
    rng(609,'twister');
    maxphase=0;maxtime=0;maxcable=0;maxortho=0;maxaff=0;maxrot=0;
    for trial=1:8
        t0=4+rand;T=2+4*rand;L=1+2*rand;
        left=zeros(3,10);right=zeros(3,10);
        left(:,1)=[-2;0;3]+0.3*randn(3,1);
        right(:,1)=[2;0;3]+0.3*randn(3,1);
        for k=2:10
            left(:,k)=0.01^(k-2)*0.03*randn(3,1);
            right(:,k)=0.01^(k-2)*0.03*randn(3,1);
        end
        free=repmat([0;0;3],1,4)+0.2*randn(3,4);
        g=[0;0;-9.80665];
        phases=[0.13 0.41 0.79];
        S=TRAJECTORY_C9_DURATION_FULLBODY_JACOBIAN( ...
            t0,T,left,right,free,phases,g,L);
        assert(startsWith(S.status,'VALID_LOCAL_13D_TANGENT'), ...
            'C9_DURATION_13D:Base','Taut reference did not produce a 13D tangent.');
        for i=1:numel(phases)
            q=S.cable_direction(:,i);
            Jq=S.cable_q_J_fixed_phase(:,:,i);
            maxortho=max(maxortho,max(abs(q.'*Jq),[],'all'));
            for lam=[0 0.37 1]
                point=S.payload_m(:,i)-lam*L*q;
                Jphase=S.payload_J_fixed_phase(:,:,i)-lam*L*Jq;
                Jtime=S.payload_J_fixed_time(:,:,i)- ...
                    lam*L*S.cable_q_J_fixed_time(:,:,i);
                % The same lambda represents a fixed physical cable fraction.
                for j=1:13
                    h=2e-5;
                    lp=left;rp=right;fp=free;Tp=T;
                    lm=left;rm=right;fm=free;Tm=T;
                    if j<13
                        fp(j)=fp(j)+h;fm(j)=fm(j)-h;
                    else
                        Tp=T+h;Tm=T-h;
                    end
                    pp=TRAJECTORY_C9_FLAT_BERNSTEIN.build(t0,Tp,lp,rp,fp);
                    pm=TRAJECTORY_C9_FLAT_BERNSTEIN.build(t0,Tm,lm,rm,fm);
                    up=phases(i);um=phases(i);
                    tp=t0+phases(i)*T;
                    pplus=local_body(pp,up,g,L,lam);
                    pminus=local_body(pm,um,g,L,lam);
                    fd=(pplus-pminus)/(2*h);
                    err=norm(fd-Jphase(:,j))/max(1,norm(fd));
                    maxphase=max(maxphase,err);
                    if j==13
                        up=(tp-t0)/Tp;um=(tp-t0)/Tm;
                        fdtime=(local_body(pp,up,g,L,lam)- ...
                            local_body(pm,um,g,L,lam))/(2*h);
                        maxtime=max(maxtime, ...
                            norm(fdtime-Jtime(:,j))/max(1,norm(fdtime)));
                    end
                end
                % Linearizing a separation plane at a fixed normal uses
                % normal'*Jacobian without changing the obstacle support.
                n=randn(3,1);n=n/norm(n);
                maxaff=max(maxaff,abs((n.'*Jphase)*(1:13).'/13- ...
                    sum(Jphase*(1:13).'/13.*n)));
                maxcable=max(maxcable,norm(point- ...
                    ((1-lam)*S.payload_m(:,i)+lam*S.uav_m(:,i))));
            end
        end
        % SO(3) covariance of the full body tangent (no fixed direction).
        [Q,~]=qr(randn(3));Q(:,end)=Q(:,end)*det(Q);
        leftR=Q*left;rightR=Q*right;freeR=Q*free;
        A=TRAJECTORY_C9_DURATION_FULLBODY_JACOBIAN( ...
            t0,T,leftR,rightR,freeR,phases,Q*g,L);
        for i=1:numel(phases)
            % Free-column rotation mixes coordinate columns: compare the
            % physical duration column, and the free 3x3 column blocks.
            maxrot=max(maxrot,norm(A.uav_J_fixed_phase(:,13,i)- ...
                Q*S.uav_J_fixed_phase(:,13,i)));
            for k=1:4
                b=(k-1)*3+(1:3);
                maxrot=max(maxrot,norm(A.uav_J_fixed_phase(:,b,i)- ...
                    Q*S.uav_J_fixed_phase(:,b,i)*Q.','fro'));
            end
        end
    end
    R.max_jacobian_error_rel=maxphase;
    R.max_fixed_time_error_rel=maxtime;
    R.max_cable_error_rel=maxcable;
    R.max_direction_orthogonality=maxortho;
    R.max_fullbody_affine_residual=maxaff;
    R.max_so3_covariance_error=maxrot;
    assert(maxphase<2e-5&&maxtime<2e-5&&maxcable<1e-10&& ...
        maxortho<1e-9&&maxaff<1e-10&&maxrot<1e-8, ...
        'C9_DURATION_13D:Jacobian','13D body tangent failed numerical checks.');
    R.stage='BATCH_DEFAULT_SEARCH_ONLY';
    fn=which('TRAJECTORY_C9_TORQUE_POINT_EVALUATE');
    src=fileread(fn);
    R.batch_default_enabled_on_multiple_times= ...
        contains(src,'batch_bernstein=(numel(ts)>1)');
    R.batch_explicit_off_parity= ...
        contains(src,"if isfield(options,'use_vectorized_bernstein_point_jets')");
    assert(R.batch_default_enabled_on_multiple_times&&R.batch_explicit_off_parity, ...
        'C9_DURATION_13D:PointBatch','Point probe default or override missing.');
    R.status='PASS_13D_FULLBODY_TANGENT__NUMERICAL_PARITY_ONLY';
catch ME
    R.status='FAIL_C9_DURATION_13D';
    R.error_identifier=ME.identifier;R.error_message=ME.message;
    R.error_stack=arrayfun(@(x)sprintf('%s:%d',x.name,x.line), ...
        ME.stack,'UniformOutput',false);
end
save(fullfile(root,[name '.mat']),'R');
fid=fopen(fullfile(root,[name '.txt']),'w');
if fid>=0
    fprintf(fid,'status=%s stage=%s error=%s %s\n', ...
        R.status,R.stage,R.error_identifier,R.error_message);
    fprintf(fid,'phase_jac_rel=%.12g physical_time_jac_rel=%.12g cable=%.12g ortho=%.12g plane=%.12g SO3=%.12g\n', ...
        R.max_jacobian_error_rel,R.max_fixed_time_error_rel, ...
        R.max_cable_error_rel,R.max_direction_orthogonality, ...
        R.max_fullbody_affine_residual,R.max_so3_covariance_error);
    fprintf(fid,'batch_default_multi=%d explicit_override=%d safe_to_commit=%d hlc=%d\n', ...
        R.batch_default_enabled_on_multiple_times,R.batch_explicit_off_parity, ...
        R.safe_to_commit,R.actual_hlc_tracking_proven);
    for k=1:numel(R.error_stack),fprintf(fid,'stack=%s\n',R.error_stack{k});end
    fclose(fid);
end
zip(R.zip_path,{[name '.mat'],[name '.txt']},root);
fprintf('[C9 13D DURATION & FULLBODY] %s\n',R.status);
end

function b=local_body(piece,u,g,L,lambda)
D=TRAJECTORY_C9_FLAT_BERNSTEIN.actual_derivative_controls(piece);
p=local_point(D{1},u);a=local_point(D{3},u);
w=g-a;q=w/norm(w);
b=p-lambda*L*q;
end

function v=local_point(P,u)
for k=size(P,2):-1:2
    P=(1-u)*P(:,1:k-1)+u*P(:,2:k);
end
v=P(:,1);
end
