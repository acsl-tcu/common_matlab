function [h,info] = CBF_virtual_C_sphere(x,settings,obstacle)
%CBF_VIRTUAL_C_SPHERE 3次元の幾何安全関数。h>=0 は非衝突の十分条件。
% x,settings,obstacle はすべて外部から渡す。
% 本関数はHOCBF-QPや安全性保証を完成させるものではない。
[h,info] = cbf_evaluate_case('virtual','C','sphere',x,settings,obstacle);
end
