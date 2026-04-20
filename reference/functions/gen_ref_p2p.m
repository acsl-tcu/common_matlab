function ref = gen_ref_p2p(param)
arguments
    param.p0 = [0;0;0]
    param.pf = [1;1;1]
    param.T  = 10
end

p0 = param.p0(:);
pf = param.pf(:);
T  = param.T;

% minimum-jerk (C^2、snap まで可)
a0 = 0;
a1 = 0;
a2 = 0;
a3 = 10/T^3;
a4 = -15/T^4;
a5 = 6/T^5;

s = @(t) a0 ...
       + a1*t ...
       + a2*t.^2 ...
       + a3*t.^3 ...
       + a4*t.^4 ...
       + a5*t.^5;

% ★ min/max/if を一切使わない
ref = @(t) [ ...
    p0 + (pf - p0).*s(t) ; 
    0 ...
];
end
