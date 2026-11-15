function C = wp_six_trigger_governor_fast_coeff(dt,T)
%WP_SIX_TRIGGER_GOVERNOR_FAST_COEFF Exact ZOH linear coefficients for the
% original WAYPOINT_REFERENCE/wp_governor_step six-stage virtual-time filter.
% Cache this OUTSIDE the 25-ms tick; dt,T must match the actual source.
% This computes ONLY the governor state; it is NOT a collision predictor.
validateattributes(dt,{'double'},{'scalar','real','finite','positive'});
validateattributes(T,{'double'},{'scalar','real','finite','positive'});
h=dt/T;
E=zeros(6,6);
for i=1:6
    for j=1:i
        k=i-j;
        E(i,j)=exp(-h)*(h^k/factorial(k));
    end
end
W=zeros(1,6);
for j=1:6
    W(j)=T*gammainc(h,7-j,'lower');
end
D=zeros(6,6);
D(1,6)=1;
for k=1:5
    for j=0:k
        D(k+1,6-k+j)=(-1)^j*nchoosek(k,j)/T^k;
    end
end
C=struct('E',E,'Bc',ones(6,1)-E*ones(6,1), ...
    'W',W,'tauCmd',dt-sum(W), 'D',D, ...
    'dt',dt,'filterTime',T,'source','exact_ZOH_wp_governor_step');
end
