import os

with open('c:\\Users\\student\\Documents\\GitHub\\common_matlab\\reference\\REPLANNING_BSPLINE_HLC.m', 'r', encoding='utf-8') as f:
    code = f.read()

# Fix evaluate_active_reference to handle active trajectory completion
old_eval = '''        function ref = evaluate_active_reference(obj,time_now)
            if isempty(obj.active)
                ref = struct('valid',false);
                return
            end
            tr = obj.active;
            tau = min(max(time_now-tr.t0,0),tr.T);
            [p,d] = obj.bspline_eval(tr.P,tau,tr.T,6);'''

new_eval = '''        function ref = evaluate_active_reference(obj,time_now)
            if isempty(obj.active)
                ref = struct('valid',false);
                return
            end
            tr = obj.active;
            
            % Check if trajectory is completed
            if time_now - tr.t0 >= tr.T
                if obj.replan_active
                    fprintf("[B-SPLINE C^6] 回避完了! 公称軌道へ完全復帰 (t=%.3f s)\\n\\n", time_now);
                end
                obj.replan_active = false;
                obj.active = [];
                ref = struct('valid',false);
                return
            end
            
            tau = min(max(time_now-tr.t0,0),tr.T);
            [p,d] = obj.bspline_eval(tr.P,tau,tr.T,6);'''

code = code.replace(old_eval, new_eval)

# Fix step_algo to set replan_active to true when replanning
old_step = '''                    [candidate,ok,reason,stats] = obj.replan(time_now);
                    if ok
                        obj.active = candidate;
                        obj.last_replan_time = time_now;
                        replanned = true;
                    end'''

new_step = '''                    [candidate,ok,reason,stats] = obj.replan(time_now);
                    if ok
                        if ~obj.replan_active
                            fprintf("\\n=================================================================================\\n");
                            fprintf(" [B-SPLINE BLUEPRINT REPLANNER 診断レポート]  t = %.3f s\\n", time_now);
                            fprintf("=================================================================================\\n");
                            fprintf(" 1. 真値状態取得      : 荷物 pL, 機体 pQ (推定期直接抽出: 正常)\\n");
                            fprintf(" 2. 動的接近判定      : B-Spline軌道変形法による衝突検知 (PVO類似)\\n");
                            fprintf(" 3. 軌道変形スケール  : 探索空間 %.2f 倍 (最大候補数 %d)\\n", obj.deformation_scales(end), obj.max_candidates);
                            fprintf(" 4. 回避プラン策定    : 処理時間 = %6.2f ms (安全解発見)\\n", stats.computeTime*1000);
                            fprintf(" 5. 軌道評価スコア    : コスト = %.3f\\n", candidate.validation.cost);
                            fprintf("=================================================================================\\n\\n");
                        end
                        obj.active = candidate;
                        obj.last_replan_time = time_now;
                        obj.replan_active = true;
                        replanned = true;
                    end'''

code = code.replace(old_step, new_step)

with open('c:\\Users\\student\\Documents\\GitHub\\common_matlab\\reference\\REPLANNING_BSPLINE_HLC.m', 'w', encoding='utf-8') as f:
    f.write(code)
