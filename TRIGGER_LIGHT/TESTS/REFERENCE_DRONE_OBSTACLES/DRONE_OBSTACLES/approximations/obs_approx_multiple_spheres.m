function parts=obs_approx_multiple_spheres(o,C,options)
% Scene-wide sphere method; all output parts are spheres.
parts=repmat(obs_new_part('sphere',[0 0 0],eye(3), ...
    struct('radius',1)),0,1);
if numel(C)==1 && strcmp(C(1).kind,'sphere')
    p=obs_new_part('sphere',o.position,eye(3), ...
        struct('radius',C(1).params.radius*(1+1e-10)));
    p.componentIndex=1;parts=p;return;
end
B=obs_budgeted_boxes(C,options.maxPartsPerObstacle,options.targetPartSize);
for k=1:numel(B)
    mid=B(k).mid;h=B(k).half;
    % Every point inside the box is within norm(h) of its center.
    r=norm(h)*(1+1e-10)+1e-10;
    p=obs_new_part('sphere',o.position+(o.R*mid')',eye(3), ...
        struct('radius',r));
    p.componentIndex=B(k).componentIndex;
    parts(end+1)=p; %#ok<AGROW>
end
end
