function [cu,cl,cc,info] = TRIGGER_SAMPLED_BLOCK_SUPPORT_GATE( ...
    u,l,shape,ru,rl,rc,cu,cl,cc)
% TRIGGER_SAMPLED_BLOCK_SUPPORT_GATE
% Shape-INDEPENDENT nominal sampled-stage exclusion, including FULL cable.
% Group samples into contiguous blocks; for each block its UAV, payload
% and every straight cable point are contained in the convex hull of the
% 2*block endpoints. ONE certified enclosing-ball or true convex-hull
% support separating plane therefore rejects all three components in it.
% This is a nominal SAMPLE acceleration ONLY, never an intersample / EKF /
% closed-loop physical proof. False, malformed, or unsupported geometry
% always leaves the original detailed oracle enabled.
%
% The block width is only an implementation parameter. It is NEVER used
% to make a physical/safety assumption. Shapes can be arbitrary SO(3),
% convex or nonconvex, and new shape types use conservative ball support.
info=struct('blocks_checked',0,'blocks_ball_excluded',0, ...
    'blocks_support_excluded',0,'body_excluded',0, ...
    'cable_excluded',0,'support_probes',0,'used',false);
try
    if ~(isnumeric(u)&&isreal(u)&&size(u,1)==3&&isequal(size(u),size(l)) && ...
            all(isfinite(u(:)))&&all(isfinite(l(:)))&& ...
            islogical(cu)&&islogical(cl)&&islogical(cc) && ...
            isequal(size(cu),[1 size(u,2)]) && ...
            isequal(size(cl),size(cu))&&isequal(size(cc),size(cu)) && ...
            isstruct(shape)&&isscalar(shape)&& ...
            all(isfield(shape,{'center0','bounding_radius_m', ...
              'is_stationary','type','dimensions','rotation'})) && ...
            isequal(shape.is_stationary,true) && ...
            isnumeric([ru rl rc])&&all(isfinite([ru rl rc])) && ...
            all([ru rl rc]>=0))
        return
    end
    N=size(u,2);
    % Fixed-size partition reduces near-path support queries while keeping
    % O(N) work; short/sparse packets retain the cheaper original path.
    if N<32 || (nnz(cu)+nnz(cl)+nnz(cc))<32,return;end
    c=double(shape.center0(:));b=double(shape.bounding_radius_m);
    if ~isequal(size(c),[3 1])||any(~isfinite(c)) || ...
            ~isscalar(b)||~isfinite(b)||b<0,return;end
    rmax=max([double(ru),double(rl),double(rc)]);
    block_width=16;
    starts=1:block_width:N;
    num=numel(starts);
    starts(end+1)=N+1;
    normals=zeros(3,num);
    candidates=false(1,num);
    ball_dismiss=false(1,num);
    baseline_guard=8192*eps(max([1;b;abs(c);rmax;abs(u(:));abs(l(:))]));
    local_cu=cu;local_cl=cl;local_cc=cc;
    for j=1:num
        ix=starts(j):starts(j+1)-1;
        if ~(any(local_cu(ix))||any(local_cl(ix))||any(local_cc(ix)))
            continue
        end
        info.blocks_checked=info.blocks_checked+1;
        box_lower=min([u(:,ix),l(:,ix)],[],2);
        box_upper=max([u(:,ix),l(:,ix)],[],2);
        gap=max(max(box_lower-c,c-box_upper),0);
        if dot(gap,gap)>(b+rmax+baseline_guard)^2
            ball_dismiss(j)=true;
            continue
        end
        % Candidate normal uses only a sampled-box center. The result is
        % ACCEPTED solely when independently checked by support inequality.
        n=0.5*(box_lower+box_upper)-c;
        s=norm(n);
        if isfinite(s)&&s>0
            normals(:,j)=n/s;
            candidates(j)=true;
        end
    end
    for j=find(ball_dismiss)
        ix=starts(j):starts(j+1)-1;
        info.body_excluded=info.body_excluded+nnz(local_cu(ix))+nnz(local_cl(ix));
        info.cable_excluded=info.cable_excluded+nnz(local_cc(ix));
        local_cu(ix)=false;local_cl(ix)=false;local_cc(ix)=false;
        info.blocks_ball_excluded=info.blocks_ball_excluded+1;
    end
    ids=find(candidates);
    if ~isempty(ids)
        nn=normals(:,ids);
        % Original-shape convex-hull support, not an ellipsoid surrogate.
        h=TRIGGER_GEOMETRY_CERTIFICATE.support_radial_many(shape,nn);
        if ~isequal(size(h),[1 numel(ids)])||any(~isfinite(h)),return;end
        info.support_probes=numel(ids);
        for z=1:numel(ids)
            j=ids(z);ix=starts(j):starts(j+1)-1;
            n=normals(:,j);
            v=min([n.'*(u(:,ix)-c),n.'*(l(:,ix)-c)]);
            if v>h(z)+rmax+baseline_guard
                info.body_excluded=info.body_excluded+nnz(local_cu(ix))+nnz(local_cl(ix));
                info.cable_excluded=info.cable_excluded+nnz(local_cc(ix));
                local_cu(ix)=false;local_cl(ix)=false;local_cc(ix)=false;
                info.blocks_support_excluded=info.blocks_support_excluded+1;
            end
        end
    end
    cu=local_cu;cl=local_cl;cc=local_cc;
    info.used=true;
catch
    % Fail closed to ORIGINAL detailed oracle. No partial removal on error.
    info=struct('blocks_checked',0,'blocks_ball_excluded',0, ...
        'blocks_support_excluded',0,'body_excluded',0, ...
        'cable_excluded',0,'support_probes',0,'used',false);
end
end
