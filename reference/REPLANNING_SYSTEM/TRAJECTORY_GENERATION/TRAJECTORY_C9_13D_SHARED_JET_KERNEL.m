function jets=TRAJECTORY_C9_13D_SHARED_JET_KERNEL(S,T,phases)
%TRAJECTORY_C9_13D_SHARED_JET_KERNEL Fused 13D phase jets (LOCAL SEARCH).
% The first three channels are the actual payload 3D derivatives;
% channels 4:39 are derivatives w.r.t. 12 free degree-23 controls;
% channels 40:42 are derivatives w.r.t. PHYSICAL duration T.
% Each jets{k+1} has shape 42 x N, k=0..6.
%
% Exact REAL arithmetic identity at fixed normalized phase u:
% d p^(k) / d P_j = (23)_k/T^k * SUM_{ell=0}^k
%     (-1)^(k-ell) * nchoosek(k,ell) * B_(j-ell)^(23-k)(u).
% The 13th physical duration column comes from the SAME analytic
% duration sensitivity already computed for body Jacobians.
% No shape/physical authority is fixed or cached across calls.
% This code is a tangent, NOT a continuous-time safety/input certificate.
u=double(phases(:).');N=numel(u);
assert(isstruct(S)&&isscalar(S)&& ...
    isfield(S,'bernstein_basis_by_degree')&& ...
    numel(S.bernstein_basis_by_degree)>=7&& ...
    isfield(S,'physical_jets')&& ...
    isfield(S,'dphysical_jets_dT_fixed_phase')&& ...
    isequal(size(S.physical_jets),[3 10 N])&& ...
    isequal(size(S.dphysical_jets_dT_fixed_phase),[3 10 N])&& ...
    isfield(S,'phase_samples')&&isequal(S.phase_samples,u)&& ...
    isscalar(T)&&isfinite(T)&&T>0&&isequal(T,S.duration_s), ...
    'C9_SHARED_JET:Input','Exact same candidate, duration and phases are required.');
jets=cell(1,7);
fall=1;
for k=0:6
    if k>0,fall=fall*(24-k);end
    B=S.bernstein_basis_by_degree{k+1};
    assert(isequal(size(B),[24-k N])&&all(isfinite(B(:))), ...
        'C9_SHARED_JET:Basis','Invalid cached Bernstein coefficients.');
    V=zeros(42,N);
    V(1:3,:)=reshape(S.physical_jets(:,k+1,:),3,N);
    V(40:42,:)=reshape(S.dphysical_jets_dT_fixed_phase(:,k+1,:),3,N);
    factor=fall/T^k;
    for j=1:4
        original_control_zero=9+j; % 10,11,12,13: MATLAB controls 11..14
        c=zeros(1,N);
        for ell=0:k
            index_zero=original_control_zero-ell;
            if index_zero>=0&&index_zero<=23-k
                c=c+(-1)^(k-ell)*nchoosek(k,ell)*B(index_zero+1,:);
            end
        end
        value=factor*c;
        for axis=1:3
            variable=3*(j-1)+axis; % free(:) MATLAB column order
            row=3*variable+axis; % row inside the 3x14 dual channels
            V(row,:)=value;
        end
    end
    if ~all(isfinite(V(:)))
        error('C9_SHARED_JET:Nonfinite','Nonfinite shared jet tangent.');
    end
    jets{k+1}=V;
end
end
