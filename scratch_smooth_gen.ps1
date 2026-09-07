$src_cbf = 'C:\Users\student\Documents\GitHub\common_matlab\controller\@HLC_SUSPENDED_LOAD_ELLIPSOID_CBF\HLC_SUSPENDED_LOAD_ELLIPSOID_CBF.m'
$dst_cbf_dir = 'C:\Users\student\Documents\GitHub\common_matlab\controller\@HLC_SMOOTH_INPUT_CBF'
New-Item -ItemType Directory -Force -Path $dst_cbf_dir

$content = Get-Content $src_cbf -Raw
$content = $content -replace 'classdef HLC_SUSPENDED_LOAD_ELLIPSOID_CBF', 'classdef HLC_SMOOTH_INPUT_CBF'
$content = $content -replace 'function obj = HLC_SUSPENDED_LOAD_ELLIPSOID_CBF\(', 'function obj = HLC_SMOOTH_INPUT_CBF('
$content = $content -replace 'properties\r?\n\s+T_val', "properties
        T_val
        mu_prev % 前回入力(滑らかさ制約用)"
$content = $content -replace 'obj.dT_val = 0;', "obj.dT_val = 0;
                obj.mu_prev = zeros(4,1);"
$content = $content -replace 'w_mu = 5.0;\r?\n\s+w_slack = 1e6;\r?\n\s+H_qp = blkdiag\(w_mu \* eye\(4\), w_slack\);\r?\n\s+mu_target = mu_nom \+ \[0; tau_circ\];\r?\n\s+f_qp = \[-w_mu \* mu_target; 0\];', "w_mu = 5.0;
            w_slack = 1e6;
            w_smooth = 150.0; % 実入力の滑らかさペナルティ
            H_qp = blkdiag((w_mu + w_smooth) * eye(4), w_slack);
            mu_target = mu_nom + [0; tau_circ];
            f_qp = [-w_mu * mu_target - w_smooth * obj.mu_prev; 0];"
$content = $content -replace 'mu_safe = mu_opt\(1:4\);', "mu_safe = mu_opt(1:4);
                obj.mu_prev = mu_safe;"
$content = $content -replace 'mu_safe = \[ddT_fallback; tau_fallback\];', "mu_safe = [ddT_fallback; tau_fallback];
                obj.mu_prev = mu_safe;"

Set-Content -Path "$dst_cbf_dir\HLC_SMOOTH_INPUT_CBF.m" -Value $content -Encoding utf8

Copy-Item "C:\Users\student\Documents\GitHub\common_matlab\mode\SuspendedLoad\SimSuspendedLoadCBF.m" "C:\Users\student\Documents\GitHub\common_matlab\mode\SuspendedLoad\Sim_SmoothInputCBF.m"
(Get-Content "C:\Users\student\Documents\GitHub\common_matlab\mode\SuspendedLoad\Sim_SmoothInputCBF.m") -replace 'HLC_SUSPENDED_LOAD_ELLIPSOID_CBF', 'HLC_SMOOTH_INPUT_CBF' | Set-Content "C:\Users\student\Documents\GitHub\common_matlab\mode\SuspendedLoad\Sim_SmoothInputCBF.m"

