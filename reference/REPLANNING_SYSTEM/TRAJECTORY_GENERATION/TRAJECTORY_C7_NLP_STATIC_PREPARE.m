function prepared=TRAJECTORY_C7_NLP_STATIC_PREPARE(leaf_maps,duration_s)
%TRAJECTORY_C7_NLP_STATIC_PREPARE Constant Bernstein maps for ONE solve.
% Called once before the nonlinear optimizer; never cached across plan
% revisions, obstacle snapshots, durations or MATLAB sessions.
assert(iscell(leaf_maps)&&~isempty(leaf_maps)&& ...
    isnumeric(duration_s)&&isscalar(duration_s)&& ...
    isfinite(duration_s)&&duration_s>0, ...
    'TRAJECTORY_C7_NLP_STATIC_PREPARE:Input','Invalid solve duration or leaves.');
nl=numel(leaf_maps);dt=duration_s/nl;
leaf=repmat(struct('map',zeros(24,24), ...
    'free_map',zeros(8,24),'derivative_maps',{{}}),1,nl);
for k=1:nl
    M=leaf_maps{k};
    assert(isnumeric(M)&&isreal(M)&&isequal(size(M),[24 24])&& ...
        all(isfinite(M(:))), ...
        'TRAJECTORY_C7_NLP_STATIC_PREPARE:Map', ...
        'Only a finite exact 24x24 de Casteljau map is accepted.');
    leaf(k).map=M;
    leaf(k).free_map=M(9:16,:);
    B=M;K=cell(1,6);
    for order=1:6
        B=((24-order)/dt)*diff(B,1,2);
        K{order}=B;
    end
    leaf(k).derivative_maps=K;
end
prepared=struct('leaf',leaf,'duration_s',duration_s);
end
