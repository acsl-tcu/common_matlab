function R=ra_obstacle_inspect_definition(world,printRows)
%RA_OBSTACLE_INSPECT_DEFINITION Display what the generator ACTUALLY created.
% No re-fitting or edits. The complete R rows preserve the original part.
if nargin<2,printRows=true;end
info=ra_obstacle_method_info(world);
parts=world.parts;
rows=struct('objectName',{},'sourceType',{},'approximation',{}, ...
    'kind',{},'center_m',{},'radius_m',{},'axes_m',{},'rotation',{});
for k=1:numel(parts)
    p=parts(k); source='';
    if isfield(world,'objects') && ~isempty(world.objects)
        i=find(strcmp({world.objects.name},p.obstacleName),1);
        if ~isempty(i) && isfield(world.objects,'type')
            source=world.objects(i).type;
        end
    end
    row=struct('objectName',char(p.obstacleName),'sourceType',source, ...
        'approximation',info.method,'kind',char(p.kind), ...
        'center_m',reshape(p.center,1,3),'radius_m',NaN, ...
        'axes_m',[NaN NaN NaN],'rotation',p.R);
    if strcmp(info.kind,'sphere'),row.radius_m=p.params.radius;
    else,row.axes_m=reshape(p.params.axes,1,3);end
    rows(end+1)=row; %#ok<AGROW>
end
R=struct('worldMethod',info.method,'kind',info.kind, ...
    'singlePartPerObject',info.singlePerObject,'kindDetectedFromParts',info.kindFromParts, ...
    'numObjects',info.numObjects, ...
    'numParts',info.numParts,'rows',rows,'source','DRONE_OBSTACLES.parts');
if printRows
    fprintf('\n[RA] world.approximation = %s ; kind = %s ; single = %d\n', ...
        R.worldMethod,R.kind,R.singlePartPerObject);
    for k=1:numel(rows)
        q=rows(k);
        if strcmp(q.kind,'sphere')
            fprintf('  %-25s  sphere c=[%.3f %.3f %.3f] radius=%.4f m\n', ...
                q.objectName,q.center_m,q.radius_m);
        else
            fprintf('  %-25s  ellipsoid c=[%.3f %.3f %.3f] axes=[%.4f %.4f %.4f] m\n', ...
                q.objectName,q.center_m,q.axes_m);
            fprintf('    R = [%.5f %.5f %.5f; %.5f %.5f %.5f; %.5f %.5f %.5f]\n', ...
                q.rotation.');
        end
    end
end
end
