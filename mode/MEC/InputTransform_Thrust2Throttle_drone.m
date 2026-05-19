function u_trans_param = InputTransform_Thrust2Throttle_drone(varargin)
    % input transformation from thrust force to throttle level for
    % drone Prop. input

    %% transmitter system
    % u_trans_param.gain_tl =[600;600;600;40]; % gain : [roll pitch yaw throttle]' %不明[850;850;600;600] 4s[700;700;600;400] 複数機[700;700;600;200] 発掘[800;800;800;400]
    % u_trans_param.th_offset_tl = 335;         % offset 3s[1021] 4s[900]　発掘[926]
    % u_trans_param.gain = [400;400;400;40];
    % u_trans_param.th_offset = 335;

    %単機牽引用↓
    u_trans_param.th_offset     = 335;%機体質量と釣り合うスロットルオフセット
    u_trans_param.th_offset_tl  = 100;  %テークオフとランディング初期オフセット。
    u_trans_param.gain          = [600;600;600;30];%　
    u_trans_param.gain_tl       = [600;600;600;30];%
    u_trans_param.fGroundEffect = 0; % 1: enable altitude-based ground-effect offset
    u_trans_param.ge_z_low = 0.0;
    u_trans_param.ge_z_high = 1.0;
    u_trans_param.ge_th_offset_low = u_trans_param.th_offset_tl;
    u_trans_param.ge_th_offset_high = u_trans_param.th_offset;

    %droneにあった単機牽引の値↓
    % u_trans_param.th_offset     = 331;%機体質量と釣り合うスロットルオフセット
    % u_trans_param.th_offset_tl  = 260;  %テークオフとランディング初期オフセット。
    % u_trans_param.gain          = [300;300;300;20];%　
    % u_trans_param.gain_tl       = [300;300;300;20];%
    
    % u_trans_param.gain_SuspendedLoad =[500;500;500;100]; % gain : [roll pitch yaw throttle]' %不明[850;850;600;600] 4s[700;700;600;400] 複数機[700;700;600;200] 発掘[800;800;800;400]
    % u_trans_param.th_offset_SuspendedLoad = 450;         % offset 3s[1021] 4s[900]　発掘[926]
end
