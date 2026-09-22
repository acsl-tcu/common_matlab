import os

with open('c:\\Users\\student\\Documents\\GitHub\\common_matlab\\reference\\REPLANNING_BSPLINE_HLC.m', 'r', encoding='utf-8') as f:
    lines = f.readlines()

new_lines = []
in_properties = False
in_do = False
in_step_algo = False

for line in lines:
    new_lines.append(line)
    if 'log = struct()' in line:
        new_lines.append('        replan_active = false\n')
        new_lines.append('        warned_crash_load = []\n')
        new_lines.append('        warned_crash_drone = []\n')
        new_lines.append('        warned_margin_load = []\n')
        new_lines.append('        warned_margin_drone = []\n')
        new_lines.append('        last_warn_time = 0.0\n')

with open('c:\\Users\\student\\Documents\\GitHub\\common_matlab\\reference\\REPLANNING_BSPLINE_HLC.m', 'w', encoding='utf-8') as f:
    f.writelines(new_lines)
