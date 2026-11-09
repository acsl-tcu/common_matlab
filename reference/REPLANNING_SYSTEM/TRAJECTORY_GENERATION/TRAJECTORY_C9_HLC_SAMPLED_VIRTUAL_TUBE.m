function R=TRAJECTORY_C9_HLC_SAMPLED_VIRTUAL_TUBE(hlc_gains,dt_s,horizon_s,initial_error_norm_bounds,inter_sample_residual_bounds)
%TRAJECTORY_C9_HLC_SAMPLED_VIRTUAL_TUBE  SAMPLED-DATA virtual chain LEMMA.
% Applies to EXACTLY the held virtual-error law
%   e(t_k+s)=[exp(A*s)-integral_0^s exp(A*tau)B d tau * F]e(t_k)+r_k(s)
%   ||r_k(s)||_2 <= d, for every s in [0,dt] (including endpoint).
% Gains and dt MUST come from the actual Flight HLC. Initial errors and
% residual bounds MUST be proven over the REAL plant+EKF+HLC trajectory;
% measured maxima, guessed errors, and presumed exact cancellation are
% not certificates. No attempt is made to infer these bounds here.
%
% For each n=2/6 virtual integrator chain, compute an exact-degree-n
% Bernstein matrix polynomial G(s) on [0,h]. Convexity of the spectral
% norm gives sup_s ||G(s)|| <= max_j ||G_BernsteinControl_j||.
% A discrete Lyapunov P for M=G(dt) propagates the errors at HLC ticks;
% the Bernstein bound controls ALL intersample times. Thus this accounts
% for the sample/hold which a continuous-time e'=(A-BF)e model omits.
%
% IMPORTANT: MATLAB double arithmetic / dlyap are NOT directed-rounding
% interval computations. The result is a CONDITIONAL REAL-ARITHMETIC
% mathematical estimate only. It neither proves the real HLC implements
% this exact virtual law nor authorizes physical safety or COMMIT.
R=struct('status','UNKNOWN_INVALID_VIRTUAL_TUBE_INPUT', ...
    'safe_to_commit',false,'actual_HLC_equivalence_proven',false, ...
    'physical_tracking_certified',false,'directed_rounding_certified',false, ...
    'subsystem_names',{{'z','x','y','yaw'}}, ...
    'max_intersample_error_2norm',NaN(4,1), ...
    'terminal_error_2norm',NaN(4,1), ...
    'max_virtual_feedback_2norm',NaN(4,1), ...
    'sample_step_transition',{{}}, 'sample_step_gamma',NaN(4,1), ...
    'max_Bernstein_transition_norm',NaN(4,1), ...
    'sample_count',NaN,'elapsed_ms',NaN, ...
    'scope','CONDITIONAL_SAMPLED_VIRTUAL_CHAIN_NOT_ACTUAL_HLC');
clock=tic;
if ~isstruct(hlc_gains)||~isscalar(hlc_gains)|| ...
        ~all(isfield(hlc_gains,{'F1','F2','F3','F4'}))|| ...
        ~finite_scalar(dt_s,true)||~finite_scalar(horizon_s,false)|| ...
        ~valid_nonnegative_4(initial_error_norm_bounds)|| ...
        ~valid_nonnegative_4(inter_sample_residual_bounds)
    R.elapsed_ms=1000*toc(clock);return
end
names={'F1','F2','F3','F4'};ns=[2 6 6 2];
Fcell=cell(1,4);
for k=1:4
    F=hlc_gains.(names{k});
    if ~(isnumeric(F)&&isreal(F)&&isvector(F)&& ...
            numel(F)==ns(k)&&all(isfinite(F(:))))
        R.status='UNKNOWN_LIVE_HLC_GAIN_DIMENSIONS';
        R.elapsed_ms=1000*toc(clock);return
    end
    Fcell{k}=double(F(:).');
end
N=floor(horizon_s/dt_s);
rem_s=horizon_s-N*dt_s;
if rem_s<0 || rem_s>=dt_s || ~isfinite(N) || N>double(intmax('int32'))
    R.status='UNKNOWN_SAMPLE_GRID_NUMERIC_DOMAIN';
    R.elapsed_ms=1000*toc(clock);return
end
R.sample_count=N+double(rem_s>0);
e0=double(initial_error_norm_bounds(:));
d=double(inter_sample_residual_bounds(:));
for k=1:4
    n=ns(k);F=Fcell{k};
    A=diag(ones(n-1,1),1);B=zeros(n,1);B(n)=1;
    [M,Gmax]=transition_hull(A,B,F,dt_s);
    R.sample_step_transition{k}=M;
    R.max_Bernstein_transition_norm(k)=Gmax;
    if ~all(isfinite(M(:)))||~isfinite(Gmax)
        R.status='UNKNOWN_TRANSITION_NONFINITE';
        R.elapsed_ms=1000*toc(clock);return
    end
    if max(abs(eig(M)))>=1
        R.status=['UNKNOWN_SAMPLED_' names{k} '_NOT_SCHUR'];
        R.elapsed_ms=1000*toc(clock);return
    end
    try
        P=dlyap(M.',eye(n));
        P=0.5*(P+P.');
        ev=eig(P);
        if any(~isfinite(ev))||min(ev)<=0
            R.status='UNKNOWN_SAMPLED_LYAPUNOV_NOT_POSITIVE';
            R.elapsed_ms=1000*toc(clock);return
        end
        L=chol(P,'lower');
        gamma=norm((L.')*M/(L.'),2);
    catch
        R.status='UNKNOWN_DISCRETE_LYAPUNOV_UNAVAILABLE';
        R.elapsed_ms=1000*toc(clock);return
    end
    if ~(isfinite(gamma)&&gamma<1&&gamma>=0)
        R.status='UNKNOWN_DISCRETE_LYAPUNOV_NOT_CONTRACTIVE';
        R.elapsed_ms=1000*toc(clock);return
    end
    R.sample_step_gamma(k)=gamma;
    a=sqrt(max(ev))*e0(k);
    max_a=a;
    % Exact real-arithmetic closed form of the nonnegative scalar
    % recurrence a_(j+1)=gamma*a_j+sqrt(lmax(P))*d.  It is monotone,
    % therefore its largest value over all N ticks is max(a0,aN).
    % Unlike a loop, computation is O(1) in horizon/sample_dt.
    if N>0
        if gamma==0
            geometric_sum=1;
        else
            % -expm1(N*log(gamma)) avoids cancellation when gamma~=1.
            geometric_sum=-expm1(N*log(gamma))/(1-gamma);
        end
        a=(gamma^N)*a+sqrt(max(ev))*d(k)*geometric_sum;
        max_a=max(max_a,a);
    end
    if N>0
        err_max=Gmax*max_a/sqrt(min(ev))+d(k);
    else
        err_max=e0(k);
    end
    if rem_s>0
        [Mp,Gpartial]=transition_hull(A,B,F,rem_s);
        err_max=max(err_max,Gpartial*a/sqrt(min(ev))+d(k));
        err_end=norm(Mp,2)*a/sqrt(min(ev))+d(k);
    else
        err_end=a/sqrt(min(ev));
    end
    R.max_intersample_error_2norm(k)=max([e0(k),err_max]);
    R.terminal_error_2norm(k)=err_end;
    R.max_virtual_feedback_2norm(k)=norm(F,2)*max_a/sqrt(min(ev));
end
if all(isfinite([R.max_intersample_error_2norm; ...
        R.terminal_error_2norm;R.max_virtual_feedback_2norm]))
    R.status='CONDITIONAL_SAMPLED_VIRTUAL_TUBE__REAL_HLC_MODEL_PENDING';
else
    R.status='UNKNOWN_NONFINITE_VIRTUAL_TUBE_RESULT';
end
R.elapsed_ms=1000*toc(clock);
end

function [M,gmax]=transition_hull(A,B,F,h)
% G(h*u)=sum_{j=0}^n G_j B_j^n(u), including ALL s in [0,h].
n=size(A,1);
C=zeros(n,n,n+1);
for order=0:n
    if order<n
        C(:,:,order+1)=A^order/factorial(order);
    end
    if order>=1
        C(:,:,order+1)=C(:,:,order+1)- ...
            A^(order-1)*B*F/factorial(order);
    end
    C(:,:,order+1)=C(:,:,order+1)*h^order;
end
gmax=0;
M=zeros(n);
for j=0:n
    Bj=zeros(n);
    for k=0:j
        Bj=Bj+(nchoosek(j,k)/nchoosek(n,k))*C(:,:,k+1);
    end
    gmax=max(gmax,norm(Bj,2));
    if j==n,M=Bj;end
end
end

function yes=finite_scalar(x,positive)
yes=isnumeric(x)&&isreal(x)&&isscalar(x)&& ...
    isfinite(x)&&(x>=0)&&(~positive || x>0);
end
function yes=valid_nonnegative_4(x)
yes=isnumeric(x)&&isreal(x)&&isequal(size(x),[4 1])&& ...
    all(isfinite(x))&&all(x>=0);
end
