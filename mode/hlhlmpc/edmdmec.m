%% ====================================================================
%  EDMD-MEC 一键训练脚本 (logger 版)  train_edmd_mec_oneclick.m
%  直接读你的 LOGGER 结构，裁切 phase==102 的 lemniscate 段，拟合 (A_r,B_r)，
%  打印体检(rank/cond)，离线回放看 Δu，保存 mec_model_<标签>.mat。
%  改下面【CONFIG】，按 F5 运行。
%
%  HL / HLMPC：现在 CONTROLLER='HL'；以后存了 hlmpc 改 'QPHLMPC' + 换 DATA_FILES，
%  再跑一次，另存 mec_model_QPHLMPC.mat，不覆盖 HL。算法完全一样。
%% ====================================================================
clear; clc; close all;

%% ====================== 【CONFIG：只改这一段】 ======================
CONTROLLER = 'HLMPC2';                 % 'HL' / 以后 'QPHLMPC'

DATA_FILES = { ...                 % 你那几条 lemniscate(错惯量、关补偿)的 .mat
    'exphlmpcfigure8jx0.6jyo.6jz0.6_Log(18-Jun-2026_13_31_20).mat', ...
    'exphlmpcfigure8jx0.6jyo.6jz0.6_Log(18-Jun-2026_13_32_56).mat', ...
    'exphlmpcfigure8jx0.6jyo.6jz0.6_Log(18-Jun-2026_13_34_30).mat', ...
    'exphlmpcfigure8jx0.6jyo.6jz0.6_Log(18-Jun-2026_13_35_27).mat', ...
    'exphlmpcfigure8jx0.6jyo.6jz0.6_Log(18-Jun-2026_13_37_05).mat' };

PHASE_VAL = 102;                   % 你的 lemniscate 段用 102 标记

UNOM_IS_MOTOR = false;             % agent.input 若是电机[T1..T4]→true(用Bu反算回[T,τ])
Bu = [ 1  1  1  1;                 % 仅 UNOM_IS_MOTOR=true 用；换成你真实分配矩阵(eq2.7)
       1 -1  1 -1;
      -1  1  1 -1;
       1 -1 -1  1 ];

Q_IS_YPR = false;                  % state.q 若是[yaw,pitch,roll]设true(默认[roll,pitch,yaw])
SMOOTH_OMEGA = false;              % estimator 的 w 很抖才设 true

% LAMBDA = 1e-2; BETA = 0.15; DUMAX = 0.1*ones(4,1);   % EDMD-MEC 参数 HL(保守)
LAMBDA = 1e-2; BETA = 0.10; DUMAX = [0.03;0.006;0.006;0.006];
J_WRONG   = [0.06   0.06   0.06 ];                  % 惯量(仅记录,不进训练)
J_CORRECT = [1.62e-3 3.6e-3 4.5e-3];
%% ================== 【CONFIG 结束，下面不用改】 ==================


%% ----------------------- 主流程 -----------------------
fprintf('==== EDMD-MEC 训练 : %s ====\n', CONTROLLER);
Z0=[]; Z1=[]; U=[]; runsStd={}; dts=[];
for i = 1:numel(DATA_FILES)
    r = load_and_crop(DATA_FILES{i}, PHASE_VAL, UNOM_IS_MOTOR, Bu, Q_IS_YPR, SMOOTH_OMEGA);
    runsStd{i}=r; dts(end+1)=median(diff(r.t)); %#ok<SAGROW>
    Phi = zeros(26, numel(r.t));
    for k=1:numel(r.t)
        Phi(:,k)=build_phi(r.p(k,:).', r.eul(k,:).', r.v(k,:).', r.om(k,:).', r.Re3(k,:).');
    end
    Z0=[Z0, Phi(:,1:end-1)]; Z1=[Z1, Phi(:,2:end)]; U=[U, r.unom(1:end-1,:).']; %#ok<AGROW>
    fprintf('  %s : 102段 %d 步 -> %d 对 (dt~=%.4gs)\n', DATA_FILES{i}, numel(r.t), numel(r.t)-1, dts(end));
end
DT = median(dts);
if max(dts)-min(dts) > 0.2*DT
    warning('各条 dt 不一致(%.4g..%.4g)，模型混了采样率；建议统一控制周期。', min(dts), max(dts));
end

% ---- 拟合(单模型,纯最小二乘) ----
W  = Z1 * pinv([Z0; U]);
Ar = W(:,1:26); Br = W(:,27:30);
M  = (Br.'*Br + LAMBDA*eye(4)) \ Br.';
model = struct('Ar',Ar,'Br',Br,'M',M,'dt',DT,'lambda',LAMBDA,'beta',BETA, ...
               'dumax',DUMAX(:),'controller',CONTROLLER, ...
               'J_wrong',J_WRONG,'J_correct',J_CORRECT,'q_is_ypr',Q_IS_YPR);

% ---- 体检 ----
s = svd(Br); fitres = Z1-(Ar*Z0+Br*U); relfit = norm(fitres,'fro')/max(norm(Z1,'fro'),eps);
fprintf('\n---- 诊断 ----\n');
fprintf('总相邻对 N      = %d\n', size(Z0,2));
fprintf('采样周期 dt     = %.4g s (自动)\n', DT);
fprintf('rank(Br)        = %d   (应为4)\n', rank(Br));
fprintf('cond(Br)        = %.3e (>1e8偏病态:调大LAMBDA或加dither)\n', s(1)/max(s(end),eps));
fprintf('Br 奇异值       = [%.3e %.3e %.3e %.3e]\n', s(1),s(2),s(3),s(4));
fprintf('一步拟合相对残差 = %.4f (越小越好)\n', relfit);
if rank(Br)<4, warning('Br秩<4:力矩通道激励不足,换更转弯多的轨迹或加dither。'); end

% ---- 离线回放(第1条):看 Δu 量级/饱和 ----
r=runsStd{1}; du=zeros(numel(r.t)-1,4); dup=zeros(4,1);
for k=1:numel(r.t)-1
    zk =build_phi(r.p(k,:).',  r.eul(k,:).',  r.v(k,:).',  r.om(k,:).',  r.Re3(k,:).');
    zr =build_phi(r.p(k+1,:).',r.eul(k+1,:).',r.v(k+1,:).',r.om(k+1,:).',r.Re3(k+1,:).');
    rk =zr - Ar*zk - Br*r.unom(k,:).';
    dup=(1-BETA)*dup + BETA*max(min(M*rk,DUMAX),-DUMAX);
    du(k,:)=dup.';
end
dn=vecnorm(du,2,2); satf=mean(any(abs(du)>=0.999*DUMAX.',2));
fprintf('\n---- 离线回放(第1条,Δu用下一步实测近似仅看量级) ----\n');
fprintf('||Δu|| 均值/最大 = %.4f / %.4f\n', mean(dn), max(dn));
fprintf('饱和占比         = %.1f%% (常饱和→Δu_max偏小或方向不对)\n', 100*satf);
figure('Name',['Delta u ' CONTROLLER]); plot((1:size(du,1))*DT, du); grid on;
xlabel('t [s]'); ylabel('\Deltau'); title(['EDMD-MEC \Deltau (' CONTROLLER ', run1)']);
legend('\DeltaT','\Delta\tau_x','\Delta\tau_y','\Delta\tau_z');

% ---- 保存 ----
outname=sprintf('mec_model_%s.mat',CONTROLLER); save(outname,'model');
fprintf('\n已保存 -> %s\n==== 完成 ====\n', outname);


%% =====================================================================
%% ======================  局部函数(不用改)  ==========================
%% =====================================================================
function r = load_and_crop(fname, phaseVal, unomIsMotor, Bu, qYPR, smoothW)
    log = LOGGER(fname);                       % 你的 logger 打开方式
    D   = log.Data;

    % --- 裁切 phase==102 段: a=前数第2个, b=后数第1个(=最后一个) ---
    ph  = D.phase(:);
    idx = find(ph==phaseVal);
    if numel(idx)<2, error('%s: phase==%g 少于2个样本', fname, phaseVal); end
    a = idx(2); b = idx(end);
    sel = a:b;
    t = D.t(:); t = t(sel);
    fprintf('  %s : 102共%d个, 起a=%d 终b=%d, t=[%.3f..%.3f]s\n', ...
            fname, numel(idx), a, b, t(1), t(end));

    % --- 逐时刻从 estimator cell 取 state ---
    res = D.agent.estimator.result;            % cell {1,K}
    n=numel(sel); p=zeros(n,3); eul=zeros(n,3); v=zeros(n,3); om=zeros(n,3);
    for ii=1:n
        st = res{1, sel(ii)}.state;
        p(ii,:)   = st.p(:).';
        qv        = st.q(:).';
        if qYPR, qv = qv([3 2 1]); end          % [yaw,pitch,roll]->[roll,pitch,yaw]
        eul(ii,:) = qv;
        v(ii,:)   = st.v(:).';
        om(ii,:)  = st.w(:).';
    end
    if smoothW, for j=1:3, om(:,j)=smoothdata(om(:,j),'sgolay',9); end, end

    % --- u_nom = controller 实际输出 agent.input ---
    U = norm_input(D.agent.input, numel(ph));  % -> K×4
    unom = U(sel,:);
    if unomIsMotor, unom = (Bu*unom.').'; end
    if size(unom,2)~=4, error('%s: u_nom 应为4列[T,τx,τy,τz],现%d列', fname, size(unom,2)); end

    R = eul2R_seq(eul); Re3 = squeeze(R(:,3,:)).';
    r.t=t; r.p=p; r.eul=eul; r.v=v; r.om=om; r.Re3=Re3; r.unom=unom;
end

function U = norm_input(inp, K)
% 把 agent.input 规整成 K×4 (兼容 K×4 / 4×K / cell{1,k})
    if iscell(inp)
        U=zeros(numel(inp),4); for k=1:numel(inp), U(k,:)=inp{k}(:).'; end
    elseif isnumeric(inp)
        if size(inp,2)==4,      U=inp;
        elseif size(inp,1)==4,  U=inp.';
        else, error('agent.input 形状不是4路,实为 %dx%d', size(inp,1),size(inp,2)); end
    else, error('agent.input 类型无法识别'); end
    if size(U,1)>K, U=U(1:K,:); end
end

function z = build_phi(p, eul, v, om, Re3)
% 26维字典(论文eq3.17)。和在线 build_phi.m 必须一致。
    phi=eul(1); th=eul(2); o1=om(1); o2=om(2); o3=om(3);
    x=[p(:);eul(:);v(:);om(:)];
    cphi=cos(phi); sphi=sin(phi); cth=cos(th); sth=sin(th);
    sg=@(x)(x>=0)*2-1; cphi=sg(cphi)*max(abs(cphi),1e-3); cth=sg(cth)*max(abs(cth),1e-3);
    z=[ x; Re3(:); 1; o1*o2; o2*o3; o3*o1; o2*cphi; o3*sphi; ...
        o1*cth/cphi; o2*sphi/cphi; o3*cphi/cth; o2*sphi*sth/cphi; o3*cphi*sth/cth ];
end

function R=eul2R_seq(eul)            % ZYX: R=Rz(psi)Ry(theta)Rx(phi)
    T=size(eul,1); R=zeros(3,3,T);
    for k=1:T
        cp=cos(eul(k,1));sp=sin(eul(k,1));ct=cos(eul(k,2));st=sin(eul(k,2));
        cy=cos(eul(k,3));sy=sin(eul(k,3));
        R(:,:,k)=[cy -sy 0;sy cy 0;0 0 1]*[ct 0 st;0 1 0;-st 0 ct]*[1 0 0;0 cp -sp;0 sp cp];
    end
end

%% =====================================================================
%% 【在线施加:复制进你的控制器循环】(训练脚本不执行,仅参考)
%% =====================================================================
% L=load('mec_model_HL.mat'); model=L.model; du_prev=zeros(4,1);   % 启动加载一次
% % 每步算完 unom_k([T,τ]) 后:
% zk  = build_phi(p_k,      q_k,      v_k,      w_k,      Re3_k);
% zr  = build_phi(pref_kp1, qref_kp1, vref_kp1, wref_kp1, Re3ref_kp1);  % 参考下一步(解析)
% rk  = zr - model.Ar*zk - model.Br*unom_k;
% du  = (1-model.beta)*du_prev + model.beta*max(min(model.M*rk,model.dumax),-model.dumax);
% u   = unom_k + du;     % 加在[T,τ]层, 再走你原本的 Bu 分配+电机饱和
% du_prev = du;