function [separated,normalCount] = wp_six_trigger_separation_batch(body,known,indices)
%WP_SIX_TRIGGER_SEPARATION_BATCH Proof-only separating planes for ONE mode.
%   [separated,count] = ... (body,known,indices)
%
% For S = conv(body.vertices) + ball(body.radius) and an observed outward
% ellipsoid O_j = center_j + R_j*diag(axes_j)*unitBall, any unit normal n
% with min(n.'*S) > max(n.'*O_j) proves disjointness. Failure to find a
% normal is INCONCLUSIVE, never evidence of actual intersection.
%
% This function batches DIFFERENT OBSTACLE PARTS of the selected cover.
% It does NOT calculate any other C/B/D geometry mode.
% Candidate directions are identical to the old scalar routine:
% center difference, signed world axes, signed ellipsoid axes, signed
% (body vertex - obstacle center). All are sufficient tests.
%
% Floating point arithmetic is guarded, not formally outward-rounded.
N=numel(indices);
separated=false(1,N);
normalCount=zeros(1,N);
if N==0,return;end
V=body.vertices;
c=known.center(:,indices);
R=known.R(:,:,indices);
axesLength=known.axes(:,indices);
bodyCenter=mean(V,2);

% Center-to-center direction often finds separation before other tests.
tryDirections(bsxfun(@minus,bodyCenter,c));
if all(separated),return;end

worldAxes=eye(3);
for a=1:3
    v=repmat(worldAxes(:,a),1,N);
    tryDirections(v);
    if all(separated),return;end
    tryDirections(-v);
    if all(separated),return;end
end
for a=1:3
    v=reshape(R(:,a,:),3,N);
    tryDirections(v);
    if all(separated),return;end
    tryDirections(-v);
    if all(separated),return;end
end
for a=1:size(V,2)
    v=bsxfun(@minus,V(:,a),c);
    tryDirections(v);
    if all(separated),return;end
    tryDirections(-v);
    if all(separated),return;end
end

    function tryDirections(direction)
        % Test only parts still lacking a separating plane.
        remaining=find(~separated);
        if isempty(remaining),return;end
        raw=direction(:,remaining);
        lengths=sqrt(sum(raw.^2,1));
        good=lengths>1e-14;
        if ~any(good),return;end
        j=remaining(good);
        n=bsxfun(@rdivide,raw(:,good),lengths(good));
        normalCount(j)=normalCount(j)+1;

        % Lower support of conv(vertices)+ball(radius).
        lower=min(n.'*V,[],2).' - body.radius;
        currentCenter=c(:,j);
        projectedCenter=sum(n.*currentCenter,1);

        % Rotated ellipsoid support; vectorized over candidate parts.
        rot=R(:,:,j);
        nx=n(1,:);ny=n(2,:);nz=n(3,:);
        v1=reshape(rot(1,1,:),1,[]).*nx + ...
           reshape(rot(2,1,:),1,[]).*ny + reshape(rot(3,1,:),1,[]).*nz;
        v2=reshape(rot(1,2,:),1,[]).*nx + ...
           reshape(rot(2,2,:),1,[]).*ny + reshape(rot(3,2,:),1,[]).*nz;
        v3=reshape(rot(1,3,:),1,[]).*nx + ...
           reshape(rot(2,3,:),1,[]).*ny + reshape(rot(3,3,:),1,[]).*nz;
        a=axesLength(:,j);
        obstacleSupport=sqrt((a(1,:).*v1).^2+(a(2,:).*v2).^2+(a(3,:).*v3).^2);
        upper=projectedCenter+obstacleSupport;
        scale=max(1,max(abs(lower),abs(upper)));
        separate=(lower-upper)>64*eps(scale);
        separated(j(separate))=true;
    end
end
