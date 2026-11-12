function [h,info,qp] = CBF_real_C_ellipsoid(x,settings,obstacle)
%CBF_REAL_C_ELLIPSOID  3次元・実入力系の幾何安全関数（実行時パラメータ指定）。
% x:実入力拡張状態29要素 / settings:形状・物理値・制限 / obstacle:球または回転楕円体
% ケーブル球数は実行時のcableLengthとmaxExtraRadiusから算出します。
% 出力1,2:幾何hと情報。出力3: A*nu>=b の高次CBF条件。
[h,info] = cbf_evaluate_case('real','C','ellipsoid',x,settings,obstacle);
if nargout>=3
    qp=cbf_hocbf_evaluate('real','C','ellipsoid',x,settings,obstacle,h,info);
end
end
