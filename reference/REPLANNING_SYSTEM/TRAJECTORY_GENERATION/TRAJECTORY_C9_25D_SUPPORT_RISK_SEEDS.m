function W=TRAJECTORY_C9_25D_SUPPORT_RISK_SEEDS(piece,shapes,base_phase,radii,authority,minimum_separation_m,numerical_guard_m)
%TRAJECTORY_C9_25D_SUPPORT_RISK_SEEDS Vectorized, general-3D, SEARCH ONLY.
% For every observed shape, 3 physical bodies and every base-time chord,
% project the obstacle centre to the chord, then evaluate true-shape
% support at the projected physical phase. Keep the worst strictly worse
% negative gap PER obstacle. The C9 continuous-time certificate is
% independent and mandatory; these probes NEVER certify collision or safety.
% Geometric shape support and all physical values are from CURRENT calls.
% No obstacle count limit, fixed bypass direction or time cutoff is used.
clock=tic;
W=struct('valid',false,'status','UNKNOWN_RISK_SEEDS_INPUT', ...
 'phases',zeros(0,1),'ids',zeros(0,1),'body_mask',false(0,3), ...
 'worst_gap_m',zeros(0,1),'candidate_projected_count',0, ...
 'support_evaluations',0,'support_batch_calls',0, ...
 'degenerate_directions',0,'elapsed_ms',NaN, ...
 'search_only',true,'safe_to_commit',false);
try
 if ~(isstruct(piece)&&isscalar(piece)&& ...
       all(isfield(piece,{'control_points_m','duration_s'})) && ...
       isequal(size(piece.control_points_m),[3,28])&& ...
       all(isfinite(piece.control_points_m(:)))&& ...
       isscalar(piece.duration_s)&&isfinite(piece.duration_s)&& ...
       piece.duration_s>0&&isstruct(shapes)&& ...
       isnumeric(base_phase)&&all(isfinite(base_phase(:)))&& ...
       numel(base_phase)>=2&&isnumeric(radii)&& ...
       isequal(size(radii),[1,3])&&all(isfinite(radii))&&all(radii>=0)&& ...
       isstruct(authority)&&all(isfield(authority, ...
              {'gravity_world_mps2','cable_length_m'})))
    W.status='UNKNOWN_RISK_SEEDS_SHAPE_OR_PHASE';return
 end
 u=unique(double(base_phase(:))).';L=double(authority.cable_length_m);
 g=double(authority.gravity_world_mps2(:));
 margin=double(minimum_separation_m)+double(numerical_guard_m);
 if ~(numel(u)>=2&&all(u>=0 & u<=1)&&numel(g)==3&& ...
        all(isfinite(g))&&isfinite(L)&&L>0&& ...
        isfinite(margin)&&margin>=0)
    W.status='UNKNOWN_RISK_SEEDS_AUTHORITY';return
 end
 P=double(piece.control_points_m);duration=double(piece.duration_s);
 A=27*26/duration^2*diff(P,2,2);
 [pBase,qBase,baseOK]=state_at(u);
 if ~baseOK
    W.status='UNKNOWN_RISK_SEEDS_TAUT_DIRECTION';return
 end
 for j=1:numel(shapes)
    sh=shapes(j);center=double(sh.center(:));
    if ~(isequal(size(center),[3,1])&&all(isfinite(center))&& ...
         isscalar(sh.id)&&isfinite(double(sh.id)))
       W.status='UNKNOWN_RISK_SEEDS_INVALID_PREPARED_SHAPE';return
    end
    % 3xN anchors for UAV, nearest point on straight taut cable, payload.
    lamBase=max(0,min(1,sum(qBase.*(pBase-center),1)/L));
    xU=pBase-L*qBase;
    xC=pBase-L*(qBase.*lamBase);
    xL=pBase;
    baseX=cat(3,xU,xC,xL); % 3 x N x 3 body indices
    baseGap=zeros(3,numel(u));
    for b=1:3
       baseGap(b,:)=gap_batch(baseX(:,:,b),sh,radii(b));
    end
    projected=zeros(0,1);local=zeros(0,1);originBody=zeros(0,1);
    % Each chord's obstacle-centre projection is 3D, SO(3)-equivariant.
    for b=1:3
       x=baseX(:,:,b);
       d=diff(x,1,2);
       d2=sum(d.*d,1);
       nonzero=isfinite(d2)&d2>0;
       alpha=zeros(size(d2));
       numerator=sum((center-x(:,1:end-1)).*d,1);
       alpha(nonzero)=numerator(nonzero)./d2(nonzero);
       pick=find(nonzero&alpha>0&alpha<1);
       if ~isempty(pick)
          projected=[projected; (u(pick)+alpha(pick).* ...
                        (u(pick+1)-u(pick))).']; %#ok<AGROW>
          local=[local;pick(:)]; %#ok<AGROW>
          originBody=[originBody;b*ones(numel(pick),1)]; %#ok<AGROW>
       end
    end
    if isempty(projected),continue;end
    % Retain the legacy chronological (interval, body) ordering for
    % deterministic equal-severity tie breaking; do not bake a detour side.
    [~,order]=sortrows([local,originBody],[1 2]);
    projected=projected(order);local=local(order);
    % Repeat times from different bodies are evaluated only once.
    [projected,keep]=unique(projected,'stable');
    local=local(keep);
    W.candidate_projected_count=W.candidate_projected_count+numel(projected);
    [pNow,qNow,nowOK]=state_at(projected.');
    if ~nowOK
       W.status='UNKNOWN_RISK_SEEDS_TAUT_DIRECTION';return
    end
    lamNow=max(0,min(1,sum(qNow.*(pNow-center),1)/L));
    nowX=cat(3,pNow-L*qNow,pNow-L*(qNow.*lamNow),pNow);
    gapNow=zeros(3,numel(projected));
    for b=1:3
       gapNow(b,:)=gap_batch(nowX(:,:,b),sh,radii(b));
    end
    % Old definition: the seed must be negative AND more negative than
    % BOTH neighbouring base probes for the SAME body and obstacle.
    a=baseGap(:,local);z=baseGap(:,local+1);
    good=isfinite(gapNow)&gapNow<0&gapNow<a&gapNow<z;
    deficit=-gapNow;
    deficit(~good)=-Inf;
    [severity,idx]=max(deficit(:));
    if isfinite(severity)
       [body,ti]=ind2sub(size(deficit),idx);
       W.phases(end+1,1)=projected(ti);
       W.ids(end+1,1)=double(sh.id);
       W.body_mask(end+1,1:3)=false(1,3);
       W.body_mask(end,body)=true;
       W.worst_gap_m(end+1,1)=gapNow(body,ti);
    end
 end
 W.valid=true;W.status='READY_PROJECTED_SUPPORT_RISK_SEARCH_ONLY';
catch ME
 W.valid=false;
 W.status=['UNKNOWN_RISK_SEEDS_EXCEPTION_' ME.identifier];
end
W.elapsed_ms=1000*toc(clock);

    function [p,q,ok]=state_at(t)
        % Bernstein exact nominal evaluations at arbitrary local phase.
        p=P*bernstein(27,t);
        acceleration=A*bernstein(25,t);
        w=g-acceleration;
        normw=sqrt(sum(w.*w,1));
        ok=all(isfinite(normw)&normw>0);
        if ok,q=w./normw;else,q=zeros(size(w));end
    end
    function gap=gap_batch(x,shape,radius)
        % True-shape support in ONE batch, not a MATLAB call per probe.
        v=x-double(shape.center(:));
        vnorm=sqrt(sum(v.*v,1));
        nonzero=isfinite(vnorm)&vnorm>0;
        n=zeros(size(v));n(:,nonzero)=v(:,nonzero)./vnorm(nonzero);
        gap=-Inf(1,size(x,2));
        if any(nonzero)
           h=TRAJECTORY_C9_SUPPORT_BATCH(shape,n(:,nonzero));
           gap(nonzero)=sum(n(:,nonzero).*x(:,nonzero),1)- ...
                            h-radius-margin;
           W.support_evaluations=W.support_evaluations+nnz(nonzero);
           W.support_batch_calls=W.support_batch_calls+1;
        end
        W.degenerate_directions=W.degenerate_directions+nnz(~nonzero);
    end
end

function B=bernstein(n,u)
% Stable Bernstein basis recursion, no toolbox or combinatorial overflow.
u=double(u(:)).';B=ones(1,numel(u));
for j=1:n
    nxt=zeros(j+1,numel(u));
    nxt(1,:)=B(1,:).*(1-u);
    for k=2:j
        nxt(k,:)=B(k-1,:).*u+B(k,:).*(1-u);
    end
    nxt(j+1,:)=B(j,:).*u;
    B=nxt;
end
end
