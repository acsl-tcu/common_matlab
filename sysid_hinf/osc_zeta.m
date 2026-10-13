function z = osc_zeta(sys)
% osc_zeta  振動極（複素極）の最小減衰比を返す（極から直接計算, damp の並びに依存しない）.
%   振動極が無ければ NaN.
p = pole(ss(sys));  pc = p(imag(p) > 1e-6);
if isempty(pc), z = NaN; else, z = min(-real(pc)./abs(pc)); end
end
