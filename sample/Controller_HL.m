function Controller= Controller_HL(dt)
% 階層型線形化コントローラの設定
%% dt = 0.025 くらいの時に有効（これより粗いdtの時はZOH誤差を無視しているためもっと穏やかなゲインの方が良い）
% 機体名：足柄 2025/06/09 小関チューニング %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
Controller.F1=lqrd([0 1;0 0],[0;1],diag([100,1]),[1],dt);                                % z 
% Controller.F2=lqrd(diag([1,1,1],1),[0;0;0;1],diag([1500,1000,200,1]),[0.025],dt); % xdiag([100,10,10,1])
% Controller.F3=lqrd(diag([1,1,1],1),[0;0;0;1],diag([1500,1000,200,1]),[0.025],dt); % ydiag([100,10,10,1])
Controller.F2=lqrd(diag([1,1,1],1),[0;0;0;1],diag([2200,1000,200,1]),[0.025],dt); % xdiag([100,10,10,1])
Controller.F3=lqrd(diag([1,1,1],1),[0;0;0;1],diag([2200,1000,200,1]),[0.025],dt); % ydiag([100,10,10,1])
Controller.F4=lqrd([0 1;0 0],[0;1],diag([100,10]),[0.1],dt);                       % ヨー角 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Controller.F1= [28.4299    8.0587];%lqrd([0 1;0 0],[0;1],diag([100,1]),[0.1],dt);                                % z 
% Controller.F2=[ 82.3694  132.1277   64.7875   13.9398];%lqrd(diag([1,1,1],1),[0;0;0;1],diag([100,100,10,1]),[0.01],dt); % xdiag([100,10,10,1])
% Controller.F3=[ 82.3694  132.1277   64.7875   13.9398];%lqrd(diag([1,1,1],1),[0;0;0;1],diag([100,100,10,1]),[0.01],dt); % ydiag([100,10,10,1])
% Controller.F4= [28.4299    8.0587];%lqrd([0 1;0 0],[0;1],diag([100,10]),[0.1],dt);                       % ヨー角 

% % dt = 0.2 くらいの時用
% Controller.F1=lqrd([0 1;0 0],[0;1],diag([100,1]),[0.1],dt);                                % z 
% Controller.F2=lqrd(diag([1,1,1],1),[0;0;0;1],diag([1,1,1,1]),[1],dt); % xdiag([100,10,10,1])
% Controller.F3=lqrd(diag([1,1,1],1),[0;0;0;1],diag([1,1,1,1]),[1],dt); % ydiag([100,10,10,1])
% Controller.F4=lqrd([0 1;0 0],[0;1],diag([100,10]),[0.1],dt);                       % ヨー角 


% 極配置
% Controller.F2=place(diag([1,1,1],1),[0;0;0;1],Eig);
% Controller.F3=place(diag([1,1,1],1),[0;0;0;1],Eig);

% 設定確認
Controller.dt = dt;
Controller.enable_show = 0;
Controller.explore.enable = 1;
Controller.explore.flight_only = 1;
Controller.explore.mode = 'structured_multisine';
Controller.explore.start_time = 0.0;
Controller.explore.ramp_time = 1.0;
Controller.explore.u1_amp = [0.35; 0.15];
Controller.explore.u1_freq = [0.35; 0.90];
Controller.explore.u1_phase = [0.20; 1.10];
Controller.explore.u2_amp = [0.010; 0.005];
Controller.explore.u2_freq = [0.55; 1.25];
Controller.explore.u2_phase = [0.40; 1.30];
Controller.explore.u3_amp = [0.010; 0.005];
Controller.explore.u3_freq = [0.70; 1.55];
Controller.explore.u3_phase = [0.90; 0.30];
Controller.explore.u4_amp = [0.0010; 0.0005];
Controller.explore.u4_freq = [0.45; 1.05];
Controller.explore.u4_phase = [0.50; 1.40];
Controller.explore.du_cap = [0.55; 0.018; 0.018; 0.003];
eig(diag([1,1,1],1)-[0;0;0;1]*Controller.F2)
end
