function ShipLandingDemo()
   
    Sim.dt = 0.03;          
    Sim.T_max = 25;         
    Env.WaveParams = [
        0.9, 0.15, 0.05, 1.0, 0;    
        0.2, 0.40, 0.30, 2.5, 1.0;  
    ];
    Env.WindBase = [2.5; 1.5; 0]; 
    Env.WindGust = [0;0;0];

    Geo.ShipH = 2.0;       
    Geo.DroneLegH = 0.6;   
  
    State.Pos = [-10; -10; 12];
    State.Vel = [0;0;0];
    
    Ship.Pos = [0;0;0];
    Ship.Vel = [0.6; 0.2; 0]; 

 
    fig = figure('Name', 'Wireframe Landing', 'Color', 'w', ...
                 'Units', 'pixels', 'Position', [50, 50, 1600, 900]);
    ax = axes('Parent', fig, 'Position', [0 0 1 1]);
    axis(ax, 'equal'); hold(ax, 'on'); axis(ax, 'off');
    
 
    set(ax, 'Color', [0.95, 0.98, 1.0]); 
    view(ax, 3); camproj(ax, 'perspective'); camva(ax, 40);
    
 
    light('Position', [50, -50, 100], 'Style', 'local', 'Color', [1 1 1]);
    
    t_ship   = hgtransform('Parent', ax);
    t_drone  = hgtransform('Parent', ax);
    t_rotors = gobjects(4,1);
    for i=1:4, t_rotors(i) = hgtransform('Parent',t_drone); end

  
    L_sea = 50;
    [Sea.X, Sea.Y] = meshgrid(-L_sea:2.0:L_sea, -L_sea:2.0:L_sea); 
    Sea.h = surf(Sea.X, Sea.Y, zeros(size(Sea.X)), ...
        'FaceColor', [0.8 0.9 1.0], 'FaceAlpha', 0.4, ... 
        'EdgeColor', [0.4 0.6 0.8], 'EdgeAlpha', 0.3);    

    
    [s_v, s_f] = GetShipModel(Geo.ShipH);
    patch('Parent', t_ship, 'Vertices', s_v, 'Faces', s_f, ...
          'FaceColor', 'none', ...      
          'EdgeColor', 'k', ...         
          'LineWidth', 1.2);            
      
    
    [d_v, d_f, d_c] = GetHighVisDroneModel();
   
    patch('Parent', t_drone, 'Vertices', d_v, 'Faces', d_f, ...
          'FaceColor', 'none', 'EdgeColor', 'k', 'LineWidth', 1.5);

 
    [r_v, r_f] = GetRotorModel();
    for i=1:4
        patch('Parent', t_rotors(i), 'Vertices', r_v, 'Faces', r_f, ...
              'FaceColor', 'none', 'EdgeColor', 'b', 'LineWidth', 0.5);
    end

    
    h_ref_line = plot3(0,0,0, 'g--', 'LineWidth', 1.5, 'DisplayName', 'Reference');
    
   
    h_real_trail = plot3(0,0,0, 'Color', [0.5 0.5 0.5], 'LineWidth', 1);
    
    
    h_err_line = plot3([0,0],[0,0],[0,0], 'r-', 'LineWidth', 2.0);
     
    h_wind_arrow = quiver3(0,0,0,0,0,0, 'Color', [1 0.4 0], 'LineWidth', 2, 'MaxHeadSize', 0.5);
    
    h_text = text(20, 20, 0, '', 'Units', 'pixels', 'FontSize', 12, 'Color', 'k', 'FontWeight', 'bold');

    
    Hist.RefPos = [];
    Hist.RealPos = [];

   
    for t = 0:Sim.dt:Sim.T_max
        if ~isvalid(fig), break; end
        
      
        Ship.Pos = Ship.Pos + Ship.Vel * Sim.dt;
        [Z_map, ship_z, s_roll, s_pitch] = ComputeWaves(Sea.X, Sea.Y, t, Env, Ship.Pos);
        Ship.Pos(3) = ship_z; 

      
        deck_abs_z = Ship.Pos(3) + Geo.ShipH;
       
        target_land_pos = Ship.Pos;
        target_land_pos(3) = deck_abs_z + Geo.DroneLegH;
        
        if t < 6
           
            ref_pos = target_land_pos + [0;0;7];
        elseif t < 18
          
            alpha = (t - 6) / 12;
            ref_pos = target_land_pos + [0;0; 7*(1-alpha)];
        else
         
            ref_pos = target_land_pos;
        end
        ref_vel = Ship.Vel; 

       
        err_p = ref_pos - State.Pos;
        err_v = ref_vel - State.Vel;
        
        u_acc = 5.0 * err_p + 4.0 * err_v + [0;0;9.81];
        
        
        noise = 0.3 * randn(3,1);
        Env.WindGust = 0.9 * Env.WindGust + 0.1 * noise;
        v_wind_total = Env.WindBase + Env.WindGust;
        
        v_rel = State.Vel - v_wind_total;
        f_drag = -0.8 * v_rel * norm(v_rel); 
        acc_real = u_acc + f_drag/1.5 - [0;0;9.81]; 
        
       
        is_landed = false;
        dist_to_deck = State.Pos(3) - target_land_pos(3);
        
        if t > 18 && abs(dist_to_deck) < 0.15
            is_landed = true;
            State.Pos = target_land_pos; 
            State.Vel = Ship.Vel;
        else
            State.Vel = State.Vel + acc_real * Sim.dt;
            State.Pos = State.Pos + State.Vel * Sim.dt;
        end

     
        set(Sea.h, 'ZData', Z_map);
        
      
        M_s = makehgtform('translate', Ship.Pos, 'xrotate', s_roll, 'yrotate', s_pitch);
        set(t_ship, 'Matrix', M_s);
        
      
        if is_landed
            tilt_axis = [1;0;0]; tilt_angle = 0;
         
            M_d = makehgtform('translate', State.Pos, 'xrotate', s_roll, 'yrotate', s_pitch);
        else
            thrust_dir = u_acc / norm(u_acc);
            tilt_axis = cross([0;0;1], thrust_dir);
            tilt_angle = acos(dot([0;0;1], thrust_dir));
            if norm(tilt_axis) < 1e-6, tilt_axis = [1;0;0]; tilt_angle = 0; end
            M_d = makehgtform('translate', State.Pos, 'axisrotate', tilt_axis, tilt_angle);
        end
        set(t_drone, 'Matrix', M_d);
        
    
        spin = t * 30;
        r_pos = [0.4, 0.4; 0.4, -0.4; -0.4, -0.4; -0.4, 0.4]; 
        for k=1:4
            M_r = makehgtform('translate', [r_pos(k,:), 0.05], 'zrotate', spin);
            set(t_rotors(k), 'Matrix', M_r);
        end

        
        Hist.RefPos = [Hist.RefPos, ref_pos];
        Hist.RealPos = [Hist.RealPos, State.Pos];
        
        L_hist = 300;
        if size(Hist.RefPos, 2) > L_hist
            Hist.RefPos = Hist.RefPos(:, end-L_hist:end);
            Hist.RealPos = Hist.RealPos(:, end-L_hist:end);
        end
        
        set(h_ref_line, 'XData', Hist.RefPos(1,:), 'YData', Hist.RefPos(2,:), 'ZData', Hist.RefPos(3,:));
        set(h_real_trail, 'XData', Hist.RealPos(1,:), 'YData', Hist.RealPos(2,:), 'ZData', Hist.RealPos(3,:));
        set(h_err_line, 'XData', [ref_pos(1), State.Pos(1)], ...
                        'YData', [ref_pos(2), State.Pos(2)], ...
                        'ZData', [ref_pos(3), State.Pos(3)]);
        
        w_origin = State.Pos + [0;0;2.5];
        set(h_wind_arrow, 'XData', w_origin(1), 'YData', w_origin(2), 'ZData', w_origin(3), ...
                          'UData', v_wind_total(1), 'VData', v_wind_total(2), 'WData', v_wind_total(3));

        target_focus = 0.6 * Ship.Pos + 0.4 * State.Pos;
        cam_dest = target_focus + [-15; -15; 10];
        
        cur_cam = campos;
        new_cam = 0.9 * cur_cam + 0.1 * cam_dest';
        campos(new_cam);
        camtarget(target_focus);
        
        set(h_text, 'String', sprintf('Time: %.1fs\nWind: %.1f m/s\nStatus: %s', ...
            t, norm(v_wind_total), char(string(is_landed).replace("1","LANDED").replace("0","FLYING"))));

        drawnow;
    end
end


function [Z, z_s, roll, pitch] = ComputeWaves(X, Y, t, Env, SPos)
    Z = zeros(size(X));
    z_s = 0; z_dx = 0; z_dy = 0;
    
    for i = 1:size(Env.WaveParams, 1)
        A = Env.WaveParams(i,1); kx = Env.WaveParams(i,2); ky = Env.WaveParams(i,3);
        w = Env.WaveParams(i,4); phi = Env.WaveParams(i,5);
        
        Z = Z + A * sin(kx*X + ky*Y - w*t + phi);
        
        phase = kx*SPos(1) + ky*SPos(2) - w*t + phi;
        z_s = z_s + A * sin(phase);
        z_dx = z_dx + A * kx * cos(phase);
        z_dy = z_dy + A * ky * cos(phase);
    end
    pitch = -atan(z_dx);
    roll  = atan(z_dy);
end


function [v, f] = GetShipModel(H)
    
    L = 10; W = 4;
    
    v = [
        -L/2, -W/2, 0; L/2, -W/2, 0; L/2, W/2, 0; -L/2, W/2, 0;
        -L/2-1, -W/2-0.5, H; L/2+1.5, -W/2-0.5, H; L/2+1.5, W/2+0.5, H; -L/2-1, W/2+0.5, H; 
        L/2+3, 0, H+0.5; 
        L/2, 0, 0;
    ];
    
    f = [
        1 2 6 5;
        3 4 8 7; 
        4 1 5 8; 
        5 6 7 8;
        1 2 3 4; 
        2 6 9 10; 
        3 7 9 10; 
        6 7 9 9; 
    ];
end

function [v, f, c] = GetHighVisDroneModel()
    s = 0.8; h = 0.1;
    v = [
        -0.3, -0.6, -h; 0.3, -0.6, -h; 0.3, 0.6, -h; -0.3, 0.6, -h; 
        -0.3, -0.6, h;  0.3, -0.6, h;  0.3, 0.6, h;  -0.3, 0.6, h;  
        
        -0.6, -0.6, 0; 0.6, -0.6, 0; 0.6, 0.6, 0; -0.6, 0.6, 0; 
        
        0.3, 0.6, 0; 0.4, 0.8, 0;
    ];
    
    f = [
        1 2 6 5; 2 3 7 6; 3 4 8 7; 4 1 5 8; 5 6 7 8; 1 2 3 4;
        
        1 1 9 9; 2 2 10 10; 3 3 11 11; 4 4 12 12;
      
        7 8 14 14; 
    ];
    c = [];
end

function [v, f] = GetRotorModel()
    theta = linspace(0, 2*pi, 12)';
    v = [0.4*cos(theta), 0.4*sin(theta), zeros(size(theta))];
    f = [1:12];
end