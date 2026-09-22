import os

with open('c:\\Users\\student\\Documents\\GitHub\\common_matlab\\reference\\REPLANNING_BSPLINE_HLC.m', 'r', encoding='utf-8') as f:
    code = f.read()

old_metrics = '''            for i=1:numel(obs)
                [d1,~]=obj.point_ellipsoid_signed_distance(obj.state.drone_pos(:),obs(i));
                [d2,~]=obj.point_ellipsoid_signed_distance(obj.state.load_pos(:),obs(i));
                m.droneDistance=min(m.droneDistance,d1);
                m.loadDistance=min(m.loadDistance,d2);
            end'''

new_metrics = '''            % Initialize warning arrays if size changes
            n_obs = numel(obs);
            if length(obj.warned_crash_load) ~= n_obs
                obj.warned_crash_load = false(n_obs, 1);
                obj.warned_crash_drone = false(n_obs, 1);
                obj.warned_margin_load = false(n_obs, 1);
                obj.warned_margin_drone = false(n_obs, 1);
            end
            
            for i=1:n_obs
                [d1,~]=obj.point_ellipsoid_signed_distance(obj.state.drone_pos(:),obs(i));
                [d2,~]=obj.point_ellipsoid_signed_distance(obj.state.load_pos(:),obs(i));
                
                % Logging for drone
                if d1 <= 0 && ~obj.warned_crash_drone(i)
                    fprintf(2, "[CRITICAL ALARM] 機体が障害物%dに衝突! (t=%.3f s, 侵入深さ: %.3f m)\\n", i, tNow, -d1);
                    obj.warned_crash_drone(i) = true;
                elseif d1 <= (obj.drone_radius + 0.5) && ~obj.warned_margin_drone(i) && d1 > 0
                    fprintf("[SAFETY WARN] 機体が障害物%dのマージン帯侵入 (t=%.3f s, 残余距離: %.3f m)\\n", i, tNow, d1);
                    obj.warned_margin_drone(i) = true;
                end
                
                % Logging for load
                if d2 <= 0 && ~obj.warned_crash_load(i)
                    fprintf(2, "[CRITICAL ALARM] 荷物が障害物%dに衝突! (t=%.3f s, 侵入深さ: %.3f m)\\n", i, tNow, -d2);
                    obj.warned_crash_load(i) = true;
                elseif d2 <= (obj.load_radius + 0.5) && ~obj.warned_margin_load(i) && d2 > 0
                    fprintf("[SAFETY WARN] 荷物が障害物%dのマージン帯侵入 (t=%.3f s, 残余距離: %.3f m)\\n", i, tNow, d2);
                    obj.warned_margin_load(i) = true;
                end
                
                m.droneDistance=min(m.droneDistance,d1);
                m.loadDistance=min(m.loadDistance,d2);
            end'''

code = code.replace(old_metrics, new_metrics)

with open('c:\\Users\\student\\Documents\\GitHub\\common_matlab\\reference\\REPLANNING_BSPLINE_HLC.m', 'w', encoding='utf-8') as f:
    f.write(code)
