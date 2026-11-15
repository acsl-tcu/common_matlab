function C=TRAJECTORY_C9_25D_GROUP_CORRIDOR_CUTS(S,shapes,G,radii,L,margin,base_phases,gravity,target,adjacency)
%TRAJECTORY_C9_25D_GROUP_CORRIDOR_CUTS Data-derived 3D common-support corridors.
% SEARCH ONLY.  At a phase/body where >=2 observed shapes independently
% produce violating pointwise cuts, replace their conflicting normals by
% ONE normal n and the exact common support offset
%    beta(n) = max_j h_{O_j}(n) + radius + margin.
% The spatial halfspace n'*x >= beta(n) is *globally separated* from EVERY
% grouped occupied convex hull inflated by the specified physical radius.
% For the cable we apply the SAME halfspace to BOTH endpoints: if those
% nonlinear endpoint conditions hold at that instant, the entire taut
% cable segment is separated as well (convexity of a halfspace).
% The QP uses only LOCAL TANGENTS of these conditions. It is NOT a continuous
% time or numerical safety proof. The independent oracle remains necessary.
%
% In addition to simultaneously violated cuts, a failing witness can
% bring in DIRECT neighbours whose inflated true-shape support intervals
% overlap along their centre-to-centre axis. This is a one-ring search
% corridor (not an assumed globally convex free-space topology).
% Normals are chosen in general 3D from actual body positions, observed
% obstacle centres, path velocity, cable axis and actual gravity.  Both
% signs are evaluated. There is NO fixed left/right/up bypass, no grid,
% no fixed obstacle slots, no new path stage and no time cutoff.
clock=tic;
if nargin<9, target=struct('phases',[],'ids',[], 'body_mask',false(0,3));end
if nargin<10, adjacency=[];end
N=numel(S.phase);J=numel(shapes);M=3*N*J;
C=struct('valid',false,'status','UNKNOWN_GROUP_CORRIDOR_NOT_STARTED', ...
 'suppress',false(1,M),'A',zeros(25,max(1,M)), ...
 'b',zeros(1,max(1,M)),'row_count',0,'group_count',0, ...
 'replaced_scalar_rows',0,'candidate_direction_count',0, ...
 'support_batch_calls',0,'support_evaluations',0, ...
 'nonpositive_group_failures',0, ...
 'adjacency_pair_tests',0,'neighbour_linked_groups',0, ...
 'groups',struct('phase',{},'body',{},'ids',{},'normal',{}, ...
                 'beta',{},'predicted_max_deficit_m',{}), ...
 'safe_to_commit',false,'elapsed_ms',NaN, ...
 'error_identifier','','error_message','');
try
 % A zero-length witness bank contains no entries for any of the three
 % physical bodies. Accept BOTH MATLAB representations [] (0x0) and
 % false(0,3), and canonicalize to false(0,3). This is not a tolerance,
 % added observation, or relaxed geometric constraint.
 if isstruct(target)&&isscalar(target)&& ...
        all(isfield(target,{'phases','ids','body_mask'}))&& ...
        isempty(target.phases)&&isempty(target.ids)&& ...
        isempty(target.body_mask)
    target.body_mask=false(0,3);
 end
 if ~isstruct(S)||~isfield(S,'phase')||~isfield(S,'payload_m')|| ...
       ~isfield(S,'uav_m')||~isfield(S,'payload_velocity_mps')|| ...
       ~isfield(S,'cable_q')||~isfield(S,'payload_J_fixed_phase')|| ...
       ~isfield(S,'uav_J_fixed_phase')||~isstruct(G)|| ...
       ~isfield(G,'b')||numel(G.b)~=M|| ...
       ~all(isfinite(double(G.b(:))))||~isstruct(shapes)|| ...
       ~isequal(size(radii),[1,3])||any(~isfinite(radii))|| ...
       any(radii<0)||~isscalar(L)||~isfinite(L)||L<=0|| ...
       ~isscalar(margin)||~isfinite(margin)||margin<0|| ...
       ~isequal(size(gravity),[3,1])||any(~isfinite(gravity))|| ...
       any(~isfinite(base_phases(:)))||~isstruct(target)|| ...
       ~all(isfield(target,{'phases','ids','body_mask'})) || ...
       numel(target.phases)~=numel(target.ids)|| ...
       ~isequal(size(target.body_mask),[numel(target.ids),3])
    C.status='UNKNOWN_GROUP_CORRIDOR_INVALID_INPUT';
    C.elapsed_ms=1000*toc(clock);return
 end
 if J<2
    C.valid=true;C.status='READY_NO_MULTIOBSTACLE_GROUP';
    C.A=C.A(:,1:0);C.b=C.b(1:0);C.elapsed_ms=1000*toc(clock);return
 end
 % The group construction cannot add a useful cut if every base phase
 % has fewer than two simultaneously violating shapes on each body and
 % there are no nonbase witness targets. Check this in one vectorized pass
 % BEFORE building the adjacency graph or evaluating support directions.
 % This fast path changes no QP scalar rows or downstream certificates.
 competing=false;
 for ki=1:N
    if ismember(S.phase(ki),base_phases)
        local=reshape(double(G.b((ki-1)*3*J+(1:3*J)))>0,3,J);
        if any(sum(local,2)>=2),competing=true;break;end
    elseif any(target.phases(:)==S.phase(ki))
        competing=true;break
    end
 end
 if ~competing
    C.valid=true;C.status='READY_NO_GROUP_CANDIDATE';
    C.A=C.A(:,1:0);C.b=C.b(1:0);C.elapsed_ms=1000*toc(clock);return
 end
 if isempty(adjacency)
    GG=TRAJECTORY_C9_25D_OBSERVED_NEIGHBOR_GRAPH(shapes,radii,margin);
    if ~GG.valid,C.status=GG.status;return;end
    adjacency=GG.adjacency;
    C.adjacency_pair_tests=GG.pair_tests;
    C.support_batch_calls=C.support_batch_calls+GG.support_evaluations;
    C.support_evaluations=C.support_evaluations+GG.support_evaluations;
 elseif ~isequal(size(adjacency),[J J 3])|| ...
        ~islogical(adjacency)|| ...
        ~isequal(adjacency,permute(adjacency,[2 1 3]))
    C.status='UNKNOWN_GROUP_CORRIDOR_GRAPH_MISMATCH';return
 end
 for k=1:N
    isbase=ismember(S.phase(k),base_phases);
    matched=[];
    if ~isbase
        matched=find(target.phases(:)==S.phase(k));
        if isempty(matched),continue;end
    end
    p=S.payload_m(:,k);uav=S.uav_m(:,k);
    q=S.cable_q(:,k);vel=S.payload_velocity_mps(:,k);
    if ~all(isfinite([p;uav;q;vel]))
       C.status='UNKNOWN_GROUP_CORRIDOR_NONFINITE_BODY_STATE';return
    end
    for body=1:3
       ind=(k-1)*3*J+(0:J-1)*3+body;
       if isbase
           focus=find(double(G.b(ind))>0);
           eligible=1:J;
       else
           eligible=find(ismember(double([shapes.id]), ...
                   double(target.ids(matched(target.body_mask(matched,body))))));
           focus=eligible; % A full-time witness is not a sampled collision.
       end
       if isempty(focus),continue;end
       % Base probes with a single violated shape have no simultaneous
       % normal conflict. Restrict expensive group search to truly
       % simultaneous base conflicts, or to a time-local failed full-time
       % witness (which may implicate consecutive nearby obstacles).
       if isbase && numel(focus)<2,continue;end
       % One-ring support-neighbour union uses the once-per-replan
       % observed geometry graph; avoids O(Nphase*Nshape^2) MATLAB support
       % calls. No connectivity transitive closure or a priori route.
       neighbour=any(adjacency(focus,:,body),1);
       hit=find(neighbour);
       if numel(hit)<2,continue;end
       centers=reshape([shapes(hit).center],3,[]);
       if size(centers,2)~=numel(hit)||any(~isfinite(centers(:)))
           C.status='UNKNOWN_GROUP_CORRIDOR_NONFINITE_CENTRES';return
       end
       switch body
          case 1, anchor=uav;
          case 2
             cm=mean(centers,2);
             lam=max(0,min(1,(q.'*(p-cm))/L));
             anchor=p-lam*L*q;
          otherwise,anchor=p;
       end
       vmag=norm(vel);
       data=[anchor-centers,anchor-mean(centers,2), ...
             centers-mean(centers,2)];
       if vmag>0 && isfinite(vmag)
           t=vel/vmag;
           transverse=data-t*(t.'*data);
           dirs=[transverse,cross(repmat(t,1,size(data,2)),data,1), ...
                  cross(t,q),cross(t,gravity)];
       else
           dirs=[data,cross(repmat(q,1,size(data,2)),data,1), ...
                 cross(q,gravity)];
       end
       lens=sqrt(sum(dirs.*dirs,1));
       nonzero=isfinite(lens)&lens>0;
       dirs=dirs(:,nonzero)./lens(nonzero);
       if isempty(dirs)
           C.nonpositive_group_failures=C.nonpositive_group_failures+1;
           continue
       end
       dirs=[dirs,-dirs];
       % Eliminate numerically repeated search directions ONLY. No
       % geometric margin or full-time proof is modified by this filter.
       keep=true(1,size(dirs,2));
       for ii=1:size(dirs,2)-1
          if ~keep(ii),continue;end
          delta=max(abs(dirs(:,ii+1:end)-dirs(:,ii)),[],1);
          keep(ii+1:end)=keep(ii+1:end) & (delta>64*eps);
       end
       dirs=dirs(:,keep);
       C.candidate_direction_count=C.candidate_direction_count+size(dirs,2);
       h=-inf(numel(hit),size(dirs,2));
       for a=1:numel(hit)
           row=TRAJECTORY_C9_SUPPORT_BATCH(shapes(hit(a)),dirs);
           if ~isequal(size(row),[1,size(dirs,2)])||any(~isfinite(row))
               C.status='UNKNOWN_GROUP_CORRIDOR_NONFINITE_SUPPORT';return
           end
           h(a,:)=row;
           C.support_batch_calls=C.support_batch_calls+1;
           C.support_evaluations=C.support_evaluations+numel(row);
       end
       beta=max(h,[],1)+radii(body)+margin;
       if body==2
           % A taut segment lies in a halfspace iff BOTH endpoints do.
           base=min(dirs.'*p,dirs.'*uav).';
       else
           base=dirs.'*anchor;
           base=base.';
       end
       deficit=beta-base;
       [score,choice]=min(deficit);
       if ~(isfinite(score)&&choice>=1)
           C.status='UNKNOWN_GROUP_CORRIDOR_NO_FINITE_DIRECTION';return
       end
       n=dirs(:,choice);bound=beta(choice);
       if body==1
           X=[uav]; K=S.uav_J_fixed_phase(:,:,k);
       elseif body==3
           X=[p]; K=S.payload_J_fixed_phase(:,:,k);
       else
           X=[uav,p]; K=cat(3,S.uav_J_fixed_phase(:,:,k), ...
                            S.payload_J_fixed_phase(:,:,k));
       end
       local_A=zeros(25,size(X,2));local_b=zeros(1,size(X,2));
       possible=true;
       for z=1:size(X,2)
           Jx=K(:,:,min(z,size(K,3)));
           local_A(:,z)=Jx.'*n;
           local_b(z)=bound-n.'*X(:,z);
           if all(local_A(:,z)==0) && local_b(z)>0
              possible=false;break
           end
       end
       if ~possible
          C.nonpositive_group_failures=C.nonpositive_group_failures+1;
          continue
       end
       if isbase
           suppressed=hit;
       else
           suppressed=intersect(hit,eligible,'stable');
       end
       C.suppress(ind(suppressed))=true;
       if numel(hit)>numel(focus)
           C.neighbour_linked_groups=C.neighbour_linked_groups+1;
       end
       for z=1:numel(local_b)
           C.row_count=C.row_count+1;
           C.A(:,C.row_count)=local_A(:,z);
           C.b(C.row_count)=local_b(z);
       end
       C.group_count=C.group_count+1;
       C.replaced_scalar_rows=C.replaced_scalar_rows+numel(suppressed);
       C.groups(end+1)=struct('phase',S.phase(k),'body',body, ...
          'ids',double([shapes(hit).id]),'normal',n,'beta',bound, ...
          'predicted_max_deficit_m',score); %#ok<AGROW>
    end
 end
 C.A=C.A(:,1:C.row_count);
 C.b=C.b(:,1:C.row_count);
 C.valid=true;
 C.status='READY_DATA_DERIVED_COMMON_SUPPORT_CORRIDORS__SEARCH_ONLY';
catch ME
 C.valid=false;
 C.status=['UNKNOWN_GROUP_CORRIDOR_EXCEPTION_' ME.identifier];
 C.error_identifier=ME.identifier;
 C.error_message=ME.message;
end
C.elapsed_ms=1000*toc(clock);
end
