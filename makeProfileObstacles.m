function obs = makeProfileObstacles(n, kind)
%MAKEPROFILEOBSTACLES 球と回転楕円体を完全に分けて計測するためのテスト障害物。
% kind = 'sphere' または 'ellipsoid' 。各シーンには同じ種類しか入れない。
% 球と楕円体で中心座標は同一、形状のみ異なる。すべて静止の遠方障害物。
% 計算時間比較専用であり、回避性能や安全保証を試験するものではない。
if ~(isscalar(n) && isnumeric(n) && isfinite(n) && n>=1 && n==floor(n))
    error('CBF:ProfileCount','nには1以上の整数を指定してください。');
end
if ~(strcmp(kind,'sphere') || strcmp(kind,'ellipsoid'))
    error('CBF:ProfileKind','kindはsphereまたはellipsoidです。');
end
obs=cell(n,1);
for j=1:n
    c=[8+1.7*(j-1);(-1)^j*0.7;1.6+0.13*mod(j,3)];
    if strcmp(kind,'sphere')
        % 楕円体の最大半径とそろえた外接球。実行時の幾何パラメータ。
        obs{j}=struct('type','sphere','center',c,'radius',0.34);
    else
        ax=0.19*j; ay=0.13*j; az=0.23*j;
        Rx=[1 0 0;0 cos(ax) -sin(ax);0 sin(ax) cos(ax)];
        Ry=[cos(ay) 0 sin(ay);0 1 0;-sin(ay) 0 cos(ay)];
        Rz=[cos(az) -sin(az) 0;sin(az) cos(az) 0;0 0 1];
        obs{j}=struct('type','ellipsoid','center',c, ...
            'axes',[0.18;0.25;0.34],'R',Rz*Ry*Rx);
    end
end
end
