Copy-Item "C:\Users\student\Documents\GitHub\common_matlab\mode\SuspendedLoad\Sim_Idea2_APF.m" "C:\Users\student\Documents\GitHub\common_matlab\mode\SuspendedLoad\Sim_Idea2_APF_Dyn.m"
(Get-Content "C:\Users\student\Documents\GitHub\common_matlab\mode\SuspendedLoad\Sim_Idea2_APF_Dyn.m") -replace 'HLC_CBF_APF\(', 'HLC_CBF_APF_DYN(' | Set-Content "C:\Users\student\Documents\GitHub\common_matlab\mode\SuspendedLoad\Sim_Idea2_APF_Dyn.m"

Copy-Item "C:\Users\student\Documents\GitHub\common_matlab\mode\SuspendedLoad\SimSuspendedLoadCBF.m" "C:\Users\student\Documents\GitHub\common_matlab\mode\SuspendedLoad\SimSuspendedLoadCBF_Dyn.m"
(Get-Content "C:\Users\student\Documents\GitHub\common_matlab\mode\SuspendedLoad\SimSuspendedLoadCBF_Dyn.m") -replace 'HLC_SUSPENDED_LOAD_ELLIPSOID_CBF\(', 'HLC_SUSPENDED_LOAD_ELLIPSOID_CBF_DYN(' | Set-Content "C:\Users\student\Documents\GitHub\common_matlab\mode\SuspendedLoad\SimSuspendedLoadCBF_Dyn.m"

Copy-Item "C:\Users\student\Documents\GitHub\common_matlab\mode\SuspendedLoad\Sim_Idea4_DualLayer.m" "C:\Users\student\Documents\GitHub\common_matlab\mode\SuspendedLoad\Sim_Idea4_DualLayer_Dyn.m"
(Get-Content "C:\Users\student\Documents\GitHub\common_matlab\mode\SuspendedLoad\Sim_Idea4_DualLayer_Dyn.m") -replace 'HLC_DUAL_LAYER\(', 'HLC_DUAL_LAYER_DYN(' | Set-Content "C:\Users\student\Documents\GitHub\common_matlab\mode\SuspendedLoad\Sim_Idea4_DualLayer_Dyn.m"

Copy-Item "C:\Users\student\Documents\GitHub\common_matlab\mode\SuspendedLoad\Sim_Idea4_DualLayer.m" "C:\Users\student\Documents\GitHub\common_matlab\mode\SuspendedLoad\Sim_Idea4_DualLayer_Static.m"
(Get-Content "C:\Users\student\Documents\GitHub\common_matlab\mode\SuspendedLoad\Sim_Idea4_DualLayer_Static.m") -replace 'HLC_DUAL_LAYER\(', 'HLC_DUAL_LAYER_STATIC(' | Set-Content "C:\Users\student\Documents\GitHub\common_matlab\mode\SuspendedLoad\Sim_Idea4_DualLayer_Static.m"
