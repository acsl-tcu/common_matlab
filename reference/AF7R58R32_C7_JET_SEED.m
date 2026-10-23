function C = AF7R58R32_C7_JET_SEED(t0,T,J0,JT)
% General degree-15 C7 jet-to-jet correction SEED, for arbitrary 3-D
% compatible boundary jets at FREE chosen t0 and t0+T.
% Input 3x8: columns are derivative orders 0..7 w.r.t physical time.
% J0 must equal current active-reference correction jets; JT must equal
% the jets of the chosen terminal target correction.
% A free terminal time, target-path phase, and obstacle-safe spatial shape
% must still be optimized; this seed is NOT an accepted trajectory.
validateattributes(t0,{'double'},{'scalar','real','finite'});
validateattributes(T,{'double'},{'scalar','real','finite','positive'});
validateattributes(J0,{'double'},{'size',[3,8],'real','finite'});
validateattributes(JT,{'double'},{'size',[3,8],'real','finite'});
n=15; B=nan(3,n+1);
% Left jets fix B0..B7 exactly by the Bernstein forward-difference formula.
for r=0:7
    falling=prod(n-(0:r-1));
    F=J0(:,r+1)*(T^r/falling);
    for j=0:r-1
        F=F-((-1)^(r-j))*nchoosek(r,j)*B(:,j+1);
    end
    B(:,r+1)=F;
end
% Right jets fix B15..B8, avoiding overlap with the left 8 controls.
for r=0:7
    falling=prod(n-(0:r-1));
    F=JT(:,r+1)*(T^r/falling);
    for j=1:r
        F=F-((-1)^(r-j))*nchoosek(r,j)*B(:,n-r+j+1);
    end
    B(:,n-r+1)=((-1)^r)*F;
end
assert(all(isfinite(B(:))) && isfinite(t0+T) && t0+T>t0, ...
    'Unrepresentable C7 correction seed: increase precision or revise candidate.');
C=struct('schema','AF7R58R32_C7_JET_SEED_V1', ...
    't_start',t0,'t_end',t0+T,'duration',T,'degree',n, ...
    'control_points',B,'start_jet',J0,'end_jet',JT, ...
    'c7_endpoint_algebra','BERNSTEIN_DERIVATIVE_JET_EXACT', ...
    'arbitrary_jet_compatible',true, ...
    'monotonic_return_certified',false, ...
    'speed_certified',false,'heading_certified',false, ...
    'wholebody_certified',false,'sensor_memory_release_certified',false, ...
    'committable',false,'stage','DIAGNOSTIC_CANDIDATE_ONLY');
end
