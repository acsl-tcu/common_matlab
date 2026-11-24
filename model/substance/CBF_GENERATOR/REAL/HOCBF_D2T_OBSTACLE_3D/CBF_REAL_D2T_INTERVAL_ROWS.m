function out=CBF_REAL_D2T_INTERVAL_ROWS(rows,dt,cert)
% OPTIONAL conversion of externally VERIFIED h^(5) interval bounds to
% conservative degree-5 Bernstein halfspaces A*v>=b.
%
% REQUIRED for each obstacle j and ALL t in [0,dt] and admissible v:
% h5_j(t,v) >= cert.a5Lower(j)+sum_i min(cert.b5Lower(j,i)*v(i), ...
%                                          cert.b5Upper(j,i)*v(i)).
% This function CANNOT verify those bounds, interval reachability,
% arithmetic rounding, actuator implementation or recursive feasibility.
% Supplying arbitrary numbers in cert is NOT a certificate.
validateattributes(dt,{'double'},{'scalar','positive','finite'});
n=rows.numObstacles;
assert(isequal(size(cert.a5Lower),[n,1]),'CBFD2T:BoundBiasShape');
assert(isequal(size(cert.b5Lower),[n,3]),'CBFD2T:BoundLowerShape');
assert(isequal(size(cert.b5Upper),[n,3]),'CBFD2T:BoundUpperShape');
assert(all(isfinite(cert.a5Lower)) && ...
    all(isfinite(cert.b5Lower(:))) && all(isfinite(cert.b5Upper(:))) && ...
    all(cert.b5Lower(:)<=cert.b5Upper(:)),'CBFD2T:BoundsInvalid');
A=zeros(0,3);b=zeros(0,1);names={};
preconditionOK=true;
for j=1:n
    d=rows.hDerivatives(:,j);B=rows.barrierInfluence(j,:);
    alpha0=zeros(6,1);alpha0(1:5)=((dt.^(0:4))./factorial(0:4)).'.*d;
    % h4(v)=h4(0)+B*v, as lower derivatives are control-independent.
    beta0=zeros(6,1);betaB=zeros(6,3);
    for i=0:5
        for k=0:min(i,4)
            coef=nchoosek(i,k)/nchoosek(5,k);
            beta0(i+1)=beta0(i+1)+coef*alpha0(k+1);
            if k==4
                betaB(i+1,:)=betaB(i+1,:)+coef*(dt^4/24)*B;
            end
        end
    end
    if any(beta0(1:4)<-1e-12),preconditionOK=false;end
    % beta_4>=0 is an affine inequality under the exact h4 coefficient.
    A=[A;betaB(5,:)];b=[b;-beta0(5)]; %#ok<AGROW>
    names{end+1,1}=sprintf('%s_bernstein4',rows.obstacleNames{j}); %#ok<AGROW>
    % beta5 receives the 5th derivative lower bound. Its coefficient
    % vector is a minimum over 2^3 corners of the interval box.
    for mask=0:7
        g=zeros(1,3);
        for c=1:3
            if bitget(mask,c),g(c)=cert.b5Upper(j,c);
            else,g(c)=cert.b5Lower(j,c);end
        end
        a5=beta0(6)+(dt^5/120)*cert.a5Lower(j);
        B5=betaB(6,:)+(dt^5/120)*g;
        A=[A;B5];b=[b;-a5]; %#ok<AGROW>
        names{end+1,1}=sprintf('%s_bernstein5_corner%d', ...
            rows.obstacleNames{j},mask); %#ok<AGROW>
    end
end
out=struct('A',A,'b',b,'names',{names}, ...
    'preconditionOK',preconditionOK, ...
    'boundCertificationNotChecked',true, ...
    'requiresCertifiedReachabilityBounds',true, ...
    'recursiveFeasibilityProved',false,'status','CONDITIONAL_BERNSTEIN_ROWS');
if ~preconditionOK,out.status='LOWER_BERNSTEIN_COEFFICIENT_NEGATIVE';end
end
