function R=AF7R58R92_C7_MINIMUM8_ROUTE(times,waypoints,startJets,endJets,phaseDelay,cableL)
% A GENERAL K-segment degree-15, globally C7 reference-only route generator.
% Internal jets 1..7 are computed by a coupled minimum integrated
% squared eighth-derivative (minimum crackle) solve; not set to zero.
% Waypoints/times come from the caller's environment/corridor planner.
% The specified LAST endpoint must be the live delayed nominal 0..7 jet.
% This is NOT a full obstacle/plant input/closed-loop certificate.
assert(isnumeric(times)&&isrow(times)&&numel(times)>=2&& ...
    all(isfinite(times))&&all(diff(times)>0),'R92:FiniteOrderedKnots');
K=numel(times);S=K-1;
assert(isnumeric(waypoints)&&isequal(size(waypoints),[3 K])&& ...
    all(isfinite(waypoints(:))),'R92:Finite3DWaypoints');
assert(isnumeric(startJets)&&isequal(size(startJets),[3 8])&& ...
    all(isfinite(startJets(:)))&&isnumeric(endJets)&& ...
    isequal(size(endJets),[3 8])&&all(isfinite(endJets(:))), ...
    'R92:ExternalBoundaryJets07Required');
assert(isnumeric(phaseDelay)&&isscalar(phaseDelay)&& ...
    isfinite(phaseDelay)&&phaseDelay>=0,'R92:NonnegativePhaseDelayRequired');
assert(isnumeric(cableL)&&isscalar(cableL)&&isfinite(cableL)&&cableL>0, ...
    'R92:ExternalCableLengthRequired');
% Do not silently change the exact boundary positions.
assert(all(startJets(:,1)==waypoints(:,1))&& ...
    all(endJets(:,1)==waypoints(:,end)),'R92:EndpointPositionMismatch');
J=zeros(3,8,K);J(:,1,:)=reshape(waypoints,[3 1 K]);
J(:,:,1)=startJets;J(:,:,K)=endJets;
% Unknown internal derivative jets 1..7, position remains fixed.
M=7*max(0,K-2); Rmat=[];Bmat=[];
% Assemble weighted residual of p^(8), using positive Bernstein Gram.
n=7;G=zeros(n+1);
for i=0:n
    for j=0:n
        G(i+1,j+1)=nchoosek(n,i)*nchoosek(n,j) / ...
           ((2*n+1)*nchoosek(2*n,i+j));
    end
end
Lchol=chol(G,'upper');
Rmat=zeros(8*S,M); Bmat=zeros(8*S,3);
for s=1:S
    T=double(times(s+1)-times(s));
    base0=local_segment(J(:,:,s),J(:,:,s+1),T);
    d0=local_d8(base0,T);
    Bmat(8*(s-1)+(1:8),:)=sqrt(T)*Lchol*d0.';
    for k=max(2,s):min(K-1,s+1)
        for d=1:7
            col=7*(k-2)+d;
            E=zeros(1,8);E(d+1)=1;
            if k==s
                pc=local_segment(E,zeros(1,8),T);
            else
                pc=local_segment(zeros(1,8),E,T);
            end
            Rmat(8*(s-1)+(1:8),col)=sqrt(T)*Lchol*local_d8(pc,T).';
        end
    end
end
if M
    % The same sparse linear system applies independently to each axis.
    % rank deficiency or extreme conditioning is a fail-closed result;
    % no ridge/regularization alters the mathematical objective.
    [U,Sv,V]=svd(Rmat,0);sv=diag(Sv);
    if numel(sv)<M||~all(isfinite(sv))||sv(end)<= ...
            max(size(Rmat))*eps(max(1,sv(1)))
        R=fail_result(times,'INCONCLUSIVE_MIN8_RANK');return;
    end
    X=-(V*((U'*Bmat)./sv));
    if any(~isfinite(X(:)))
        R=fail_result(times,'INCONCLUSIVE_NONFINITE_MIN8');return;
    end
    for k=2:K-1
        J(:,2:8,k)=X(7*(k-2)+(1:7),:).';
    end
end
segs=repmat(struct('t0',NaN,'t1',NaN,'duration_s',NaN, ...
    'payload_cp',zeros(3,16),'trajectory_frame','HLC_PAYLOAD_WORLD'),1,S);
for s=1:S
    T=double(times(s+1)-times(s));
    P=local_segment(J(:,:,s),J(:,:,s+1),T);
    segs(s)=struct('t0',times(s),'t1',times(s+1),'duration_s',T, ...
        'payload_cp',P,'trajectory_frame','HLC_PAYLOAD_WORLD');
end
% Inspect derivative consistency for every section, including highest order.
worst=0;
for s=1:S
    H=segs(s);T=H.duration_s;
    D=H.payload_cp;
    for d=0:7
        actual0=D(:,1);actual1=D(:,end);
        v0=J(:,d+1,s);v1=J(:,d+1,s+1);
        worst=max(worst,max(abs([actual0-v0;actual1-v1]),[],'all'));
        if d<7,D=(size(D,2)-1)/T*diff(D,1,2);end
    end
end
R=struct('status','GENERATED_C7_MULTISEGMENT_REFERENCE_ONLY', ...
    'reference_candidate_found',true,'times_s',double(times), ...
    'waypoints',waypoints,'knot_jets',J,'segments',segs, ...
    'num_segments',S,'phase_delay_s',phaseDelay, ...
    'nominal_phase_at_end_s',times(end)-phaseDelay, ...
    'continuity_order',7,'max_jet_residual',worst, ...
    'minimum_8th_derivative_quadratic',NaN, ...
    'whole_future_certified',false, ...
    'torque_and_execution_certified',false, ...
    'production_commit_authorized',false);
% Compute true energy, including fixed terms and all three axes.
if M>0
    xx=reshape(permute(J(:,2:8,2:end-1),[2 3 1]),M,3);
    RR=Rmat*xx+Bmat;
else
    RR=Bmat;
end
R.minimum_8th_derivative_quadratic=sum(RR(:).^2);
end

function R=fail_result(times,why)
R=struct('status',why,'reference_candidate_found',false, ...
    'times_s',times,'segments',struct([]), ...
    'continuity_order',NaN,'production_commit_authorized',false, ...
    'whole_future_certified',false);
end

function P=local_segment(A,B,T)
% Bezier's endpoint jets; input may be 3x8 OR 1x8.
D=size(A,1);P=zeros(D,16);
for j=0:7
    x=zeros(D,1);y=zeros(D,1);
    for r=0:j
        falling=prod(15:-1:(16-r));
        f=nchoosek(j,r)*T^r/falling;
        x=x+f*A(:,r+1);
        y=y+(-1)^r*f*B(:,r+1);
    end
    P(:,j+1)=x;P(:,16-j)=y;
end
end

function Z=local_d8(P,T)
Z=P;
for d=1:8
    Z=(size(Z,2)-1)/T*diff(Z,1,2);
end
end
