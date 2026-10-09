function [A, B, E] = ctow_chain(r)
%CTOW_CHAIN  6次チェーン  x' = A(r) x + B v + E w   （p'' = r*x3 + w）
%   x = [p p' p'' p''' p'''' p''''']、v = p^(6)、w は吊り点の外力/想定質量
if nargin < 1, r = 1; end
A = diag(ones(5,1), 1);
A(2,3) = r;
B = zeros(6,1); B(6) = 1;
E = zeros(6,1); E(2) = 1;
end
