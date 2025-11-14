clear; clc; close all;

u_HL   = [7.3578; 0; 0; 0];
u_kmpc = [7.3562; -0.0138; 0.0001; 0.0000];

dt    = 0.025;  
Te = 2.0;
Tm = 1;
Nstep = round(Te/dt);

sw.Nsw    = round(Tm/dt); 
drf.beta  = 0.8;           
drf.du_max = 0.35;         


t          = (0:Nstep-1)' * dt; 
ak_hist    = zeros(Nstep,1);    
ak_raw_hist= zeros(Nstep,1);    
w_hist     = zeros(Nstep,1);   
u_mix_hist = zeros(Nstep,4);     
u_cmd_hist = zeros(Nstep,4);    


for k = 1:Nstep
  
    ak     = (min(1, k / sw.Nsw))^2;             
    w      = 3*ak^2 - 2*ak^3;     
    u_mix = (1 - w)*u_HL + w*u_kmpc; 

 
    if k == 1

        u_prev = u_HL;
    else

        u_prev = u_cmd_hist(k-1,:).';
    end


    du_raw = u_mix - u_prev;


    du_sat = max(min(du_raw,  drf.du_max), -drf.du_max);


    u_cmd = u_prev + (1 - drf.beta) * du_sat;

  
    ak_hist(k)     = ak;
    w_hist(k)      = w;
    u_mix_hist(k,:)= u_mix.';
     u_cmd_hist(k,:)= u_cmd.';
end

figure;

plot(t, w_hist, 'LineWidth', 1.8);
grid on;
xlabel('Time [s]');
ylabel('w(k)');
title('Smoothstep weight w(k) = 3\alpha^2 - 2\alpha^3');


figure;
names = {'Thrust','Roll','Pitch','Yaw'};

for i = 1:4
    subplot(4,1,i); hold on; grid on;
   
    yline(u_HL(i),   ':', 'LineWidth', 1.0);
    yline(u_kmpc(i), ':', 'LineWidth', 1.0);
   
    plot(t, u_mix_hist(:,i), 'LineWidth', 1.5);
     plot(t, u_cmd_hist(:,i), '--', 'LineWidth', 1.5);
    
    ylabel(names{i});
    if i == 1
        title('All inputs: HL / KMPC / mixed / DRF output');
    end
    if i == 4
        xlabel('Time [s]');
    end
    
    legend('u_{HL}', 'u_{KMPC}', 'u_{mix}', 'u_{cmd}', ...
           'Location', 'best');
end