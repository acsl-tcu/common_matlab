function o = obs_triangular_prism_mesh(name,position,prismVertices,rpyDeg)
% 三角柱メッシュ: prismVertices は 6x3xN (N 個の三角柱体積要素)。
% 各要素の6頂点は障害物ローカル座標 [m] で指定する。
if nargin < 4 || isempty(rpyDeg), rpyDeg=[0 0 0]; end
assert(isnumeric(prismVertices) && size(prismVertices,1)==6 && ...
    size(prismVertices,2)==3 && all(isfinite(prismVertices(:))), ...
    'prismVertices must be a finite 6x3xN numeric array');
o = obs_make('triangular_prism_mesh',name,position, ...
    struct('prismVertices',prismVertices),rpyDeg);
end
