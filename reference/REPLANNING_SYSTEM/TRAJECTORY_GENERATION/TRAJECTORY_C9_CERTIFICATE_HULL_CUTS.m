function [A,b,D]=TRAJECTORY_C9_CERTIFICATE_HULL_CUTS(piece,shapes,rPayload,clearance,U,failedInterval,tolerance)
% TRAJECTORY_C9_CERTIFICATE_HULL_CUTS
% Add one affine *search* cut per observed obstacle at the exact Bernstein
% hull of a failed full-time-proof leaf. No point sampling, no fixed axis,
% no per-obstacle planning, and no false claim of continuous-time safety.
%
% If p_L(t)=sum_i P_i B^23_i(t), then the restricted leaf has control
% points H=P*C(u0,u1), where C is a purely temporal Bernstein subdivision
% matrix. For a chosen shape-support direction n, the condition
%     n'*H_i >= h_O(n)+rPayload+clearance
% is an all-time sufficient separation condition for PAYLOAD vs O on this
% leaf, and is affine in the SAME 3x4 free C9 controls. Here one worst
% currently violated movable control point is used as a cutting plane.
% This does NOT establish that all 24 controls satisfy it, nor certify the
% UAV/cable. The independent joint proof MUST run for every candidate.
%
% Search direction is derived from actual candidate-leaf geometry, never
% from a global up/left/right rule. Unknown symmetry => no added cut.
% Nonconvex shapes use the canonical conservative support hull exactly as
% the original joint proof does. The original full proof is unchanged.
A=zeros(12,0);b=zeros(0,1);
D=struct('status','NO_CUT','plane_count',0,'maximum_deficit_m',0, ...
    'skipped_immovable',0,'leaf_s',double(failedInterval(:)).', ...
    'obstacle_ids',zeros(1,0),'control_indices',zeros(1,0), ...
    'cut_normals_world',zeros(3,0), ...
    'subdivision_column_sum_error',NaN);
if ~isstruct(piece)||~isscalar(piece)||~isfield(piece,'control_points_m')|| ...
        ~isfield(piece,'t0_s')||~isfield(piece,'duration_s')|| ...
        ~isequal(size(piece.control_points_m),[3 24])|| ...
        ~(isnumeric(U)&&isequal(size(U),[4 4]))|| ...
        ~all(isfinite(U(:)))||rcond(U)<eps|| ...
        ~(isnumeric(rPayload)&&isscalar(rPayload)&&isfinite(rPayload)&&rPayload>=0)|| ...
        ~(isnumeric(clearance)&&isscalar(clearance)&&isfinite(clearance)&&clearance>=0)|| ...
        ~(isnumeric(tolerance)&&isscalar(tolerance)&&isfinite(tolerance)&&tolerance>=0)
    error('TRAJECTORY_C9_HULL_CUT:Input','Invalid C9/QP physical inputs.');
end
if isempty(shapes)
    D.status='NO_OBSTACLES';return
end
T=double(piece.duration_s);t0=double(piece.t0_s);
t=double(failedInterval(:));
if ~(numel(t)==2&&all(isfinite(t))&&isfinite(T)&&T>0&& ...
        t(2)>t(1)&&t(1)>=t0&&t(2)<=t0+T)
    D.status='UNKNOWN_PROOF_LEAF_OUTSIDE_C9_DOMAIN';return
end
u0=(t(1)-t0)/T;u1=(t(2)-t0)/T;
if u0<0||u1>1||u0>=u1
    D.status='UNKNOWN_PROOF_LEAF_OUTSIDE_C9_DOMAIN';return
end
[H,C]=TRAJECTORY_C9_BERNSTEIN_SUBINTERVAL_CONTROLS( ...
    double(piece.control_points_m),u0,u1); % exact polynomial restriction
D.subdivision_column_sum_error=max(abs(sum(C,1)-1));
if any(~isfinite(C(:)))||D.subdivision_column_sum_error>2e-12
    D.status='UNKNOWN_SUBDIVISION_NUMERICAL_CONDITION';return
end
P=double(piece.control_points_m);
J=C(11:14,:);
if any(~isfinite(H(:)))
    D.status='UNKNOWN_NONFINITE_HULL';return
end
A=zeros(12,numel(shapes));b=zeros(numel(shapes),1);nCut=0;
for j=1:numel(shapes)
    S=shapes(j);
    n=mean(H,2)-S.center;
    if ~(all(isfinite(n))&&norm(n)>0)
        % Actual local curve tangent, not an arbitrary fixed 3D axis.
        n=H(:,end)-H(:,1);
    end
    if ~(all(isfinite(n))&&norm(n)>0)
        continue
    end
    n=n/norm(n);
    support=TRAJECTORY_OBSTACLE_SUPPORT.value(S,n);
    deficit=support+rPayload+clearance-n.'*H;
    [~,rank]=sort(deficit,'descend');
    D.maximum_deficit_m=max(D.maximum_deficit_m,max(0,max(deficit)));
    if max(deficit)<=tolerance,continue;end
    % One most-violated affine Bernstein hull face for this obstacle.
    % An immovable endpoint of the free C9 parameterisation must NOT be
    % misinterpreted as global infeasibility; try the next violated face.
    for k=rank
        if deficit(k)<=tolerance,break;end
        row=reshape(n*(J(:,k).'/U),12,1);
        if all(isfinite(row)) && row.'*row>0
            nCut=nCut+1;A(:,nCut)=row;b(nCut)=deficit(k);
            D.obstacle_ids(end+1)=double(S.id); %#ok<AGROW>
            D.control_indices(end+1)=k; %#ok<AGROW>
            D.cut_normals_world(:,end+1)=n; %#ok<AGROW>
            break
        end
        D.skipped_immovable=D.skipped_immovable+1;
    end
end
A=A(:,1:nCut);b=b(1:nCut);
D.plane_count=nCut;
if nCut>0,D.status='ADDED_SEARCH_HULL_CUTS';end
end

