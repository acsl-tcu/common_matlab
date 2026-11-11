function parts=obs_approx_single_sphere(o,C,~)
% Whole obstacle fits into its local bounding box; enclosing sphere contains box.
[lo,hi]=obs_local_bounds(C);h=(hi-lo)/2;mid=(lo+hi)/2;
c=o.position+(o.R*mid')'; r=norm(h)*(1+1e-10)+1e-10;
parts=obs_new_part('sphere',c,eye(3),struct('radius',r));
end
