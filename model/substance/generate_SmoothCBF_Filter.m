function generate_SmoothCBF_Filter()
    syms r1 r2 r3 real
    syms r_ref1 r_ref2 r_ref3 real
    syms v_ref1 v_ref2 v_ref3 real
    syms p_obs1 p_obs2 p_obs3 real
    syms R_safe cbf_gamma cbf_sigma Kp real
    
    r = [r1; r2; r3];
    r_ref = [r_ref1; r_ref2; r_ref3];
    v_ref = [v_ref1; v_ref2; v_ref3];
    p_obs = [p_obs1; p_obs2; p_obs3];
    
    u_nom = v_ref + Kp * (r_ref - r);
    
    h = (r - p_obs).' * (r - p_obs) - R_safe^2;
    Lgh = 2 * (r - p_obs).';
    
    a = Lgh * u_nom + cbf_gamma * h;
    b = Lgh * Lgh.';
    
    epsilon = 1e-6;
    b_safe = b + epsilon;
    lambda = (sqrt(a^2 + cbf_sigma * b_safe^2) - a) / (2 * b_safe);
    
    u_safe = u_nom + lambda * Lgh.';
    
    filename = 'SmoothCBF_Filter_Velocity.m';
    matlabFunction(u_safe, 'File', filename, ...
        'Vars', {r, r_ref, v_ref, p_obs, R_safe, cbf_gamma, cbf_sigma, Kp}, ...
        'Outputs', {'u_safe'});
        
    disp('Successfully generated SmoothCBF_Filter_Velocity.m');
end
