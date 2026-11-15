function G=TRAJECTORY_C9_25D_OBSERVED_NEIGHBOR_GRAPH(shapes,radii,margin)
%TRAJECTORY_C9_25D_OBSERVED_NEIGHBOR_GRAPH One current-observation geometry pass.
% The inflated true occupied hulls' projected support intervals on the
% centre-to-centre line are compared for a conservative *SEARCH* proximity
% relation. An edge does NOT prove geometric intersection (other axes may
% separate). No graph exists before sensor observations; recompute from
% the actual observed snapshot and current body+tube radii EACH replan.
% It is not a route graph or a fixed obstacle-count table.
clock=tic;
J=numel(shapes);
G=struct('valid',false,'status','UNKNOWN_GROUP_GRAPH_NOT_STARTED', ...
 'adjacency',false(J,J,3),'pair_tests',0, ...
 'support_evaluations',0,'elapsed_ms',NaN, ...
 'search_only',true,'safe_to_commit',false);
try
 if ~isstruct(shapes)||~isequal(size(radii),[1 3])|| ...
         any(~isfinite(radii))||any(radii<0)|| ...
         ~isscalar(margin)||~isfinite(margin)||margin<0
    G.status='UNKNOWN_GROUP_GRAPH_INPUT';return
 end
 for i=1:J
     ci=double(shapes(i).center(:));
     if ~isequal(size(ci),[3,1])||any(~isfinite(ci))
         G.status='UNKNOWN_GROUP_GRAPH_INVALID_OBSERVED_CENTER';return
     end
     G.adjacency(i,i,:)=true;
     for j=i+1:J
         cj=double(shapes(j).center(:));
         d=cj-ci;ds=norm(d);
         if ~(isfinite(ds)&&ds>=0)
             G.status='UNKNOWN_GROUP_GRAPH_INVALID_PAIR';return
         end
         G.pair_tests=G.pair_tests+1;
         if ds==0
             % Overlapping centre projections; a candidate search grouping
             % is allowed, never a physical collision conclusion.
             linked=true(1,3);
         else
             n=d/ds;
             hi=TRAJECTORY_C9_SUPPORT_BATCH(shapes(i),n)-n.'*ci;
             hj=TRAJECTORY_C9_SUPPORT_BATCH(shapes(j),-n)+n.'*cj;
             G.support_evaluations=G.support_evaluations+2;
             if ~(isfinite(hi)&&isfinite(hj))
                 G.status='UNKNOWN_GROUP_GRAPH_SUPPORT';return
             end
             linked=ds<=hi+hj+2*radii+margin;
         end
         G.adjacency(i,j,:)=reshape(linked,1,1,3);
         G.adjacency(j,i,:)=reshape(linked,1,1,3);
     end
 end
 G.valid=true;
 G.status='READY_CURRENT_OBSERVED_SUPPORT_PROXIMITY_GRAPH__SEARCH_ONLY';
catch ME
 G.status=['UNKNOWN_GROUP_GRAPH_EXCEPTION_' ME.identifier];
end
G.elapsed_ms=1000*toc(clock);
end
