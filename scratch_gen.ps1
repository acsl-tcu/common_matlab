$src_apf = 'C:\Users\student\Documents\GitHub\common_matlab\controller\@HLC_CBF_APF\HLC_CBF_APF.m'
$dst_apf_dir = 'C:\Users\student\Documents\GitHub\common_matlab\controller\@HLC_CBF_APF_DYN'
New-Item -ItemType Directory -Force -Path $dst_apf_dir
(Get-Content $src_apf) -replace 'classdef HLC_CBF_APF', 'classdef HLC_CBF_APF_DYN' -replace 'function obj = HLC_CBF_APF\(', 'function obj = HLC_CBF_APF_DYN(' -replace 'ENVIRONMENT_OBSTACLE_ELLIPSOID\(\)', 'ENVIRONMENT_OBSTACLE_DYNAMIC(time)' | Set-Content "$dst_apf_dir\HLC_CBF_APF_DYN.m"

$src_cbf = 'C:\Users\student\Documents\GitHub\common_matlab\controller\@HLC_SUSPENDED_LOAD_ELLIPSOID_CBF\HLC_SUSPENDED_LOAD_ELLIPSOID_CBF.m'
$dst_cbf_dir = 'C:\Users\student\Documents\GitHub\common_matlab\controller\@HLC_SUSPENDED_LOAD_ELLIPSOID_CBF_DYN'
New-Item -ItemType Directory -Force -Path $dst_cbf_dir
(Get-Content $src_cbf) -replace 'classdef HLC_SUSPENDED_LOAD_ELLIPSOID_CBF', 'classdef HLC_SUSPENDED_LOAD_ELLIPSOID_CBF_DYN' -replace 'function obj = HLC_SUSPENDED_LOAD_ELLIPSOID_CBF\(', 'function obj = HLC_SUSPENDED_LOAD_ELLIPSOID_CBF_DYN(' -replace 'ENVIRONMENT_OBSTACLE_ELLIPSOID\(\)', 'ENVIRONMENT_OBSTACLE_DYNAMIC(time)' | Set-Content "$dst_cbf_dir\HLC_SUSPENDED_LOAD_ELLIPSOID_CBF_DYN.m"

$src_dual = 'C:\Users\student\Documents\GitHub\common_matlab\controller\@HLC_DUAL_LAYER\HLC_DUAL_LAYER.m'

$dst_dual_stat_dir = 'C:\Users\student\Documents\GitHub\common_matlab\controller\@HLC_DUAL_LAYER_STATIC'
New-Item -ItemType Directory -Force -Path $dst_dual_stat_dir
(Get-Content $src_dual) -replace 'classdef HLC_DUAL_LAYER', 'classdef HLC_DUAL_LAYER_STATIC' -replace 'function obj = HLC_DUAL_LAYER\(', 'function obj = HLC_DUAL_LAYER_STATIC(' -replace 'ENVIRONMENT_OBSTACLE_DYNAMIC\(time\)', 'ENVIRONMENT_OBSTACLE_ELLIPSOID()' | Set-Content "$dst_dual_stat_dir\HLC_DUAL_LAYER_STATIC.m"

$dst_dual_dyn_dir = 'C:\Users\student\Documents\GitHub\common_matlab\controller\@HLC_DUAL_LAYER_DYN'
New-Item -ItemType Directory -Force -Path $dst_dual_dyn_dir
(Get-Content $src_dual) -replace 'classdef HLC_DUAL_LAYER < HLC_SUSPENDED_LOAD_ELLIPSOID_CBF', 'classdef HLC_DUAL_LAYER_DYN < HLC_SUSPENDED_LOAD_ELLIPSOID_CBF_DYN' -replace 'classdef HLC_DUAL_LAYER', 'classdef HLC_DUAL_LAYER_DYN' -replace 'function obj = HLC_DUAL_LAYER\(', 'function obj = HLC_DUAL_LAYER_DYN(' -replace 'obj@HLC_SUSPENDED_LOAD_ELLIPSOID_CBF', 'obj@HLC_SUSPENDED_LOAD_ELLIPSOID_CBF_DYN' -replace 'do@HLC_SUSPENDED_LOAD_ELLIPSOID_CBF', 'do@HLC_SUSPENDED_LOAD_ELLIPSOID_CBF_DYN' | Set-Content "$dst_dual_dyn_dir\HLC_DUAL_LAYER_DYN.m"

