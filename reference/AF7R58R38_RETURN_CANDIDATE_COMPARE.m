function R=AF7R58R38_RETURN_CANDIDATE_COMPARE(t0,durations,J0,JT,L,g,direction)
% Compare unconstrained C7 returns against one-sided convex-hull options.
% duration candidates are caller-supplied; no fixed terminal time or cap.
% NOT an obstacle, velocity, heading or whole-body certificate.
validateattributes(durations,{'double'},{'real','finite','vector','positive'});
assert(~isempty(durations),'Need at least one proposed free duration.');
validateattributes(J0,{'double'},{'real','finite','size',[3 8]});
validateattributes(JT,{'double'},{'real','finite','size',[3 8]});
validateattributes(direction,{'double'},{'real','finite','numel',3});
n=direction(:);assert(norm(n)>0,'Need a nonzero direction.');n=n/norm(n);
N=numel(durations);
status=cell(N,1); unconstrained_cost=nan(N,1);
one_sided_cost=nan(N,1); unconstrained_min=nan(N,1);
one_sided_min=nan(N,1); jet_res=nan(N,1);
for k=1:N
    Tk=durations(k);
    try
        U=AF7R58R33_C7_TWOSEG_SEED(t0,Tk,J0,JT,L,g);
        unconstrained_cost(k)=U.integral_cost;
        jet_res(k)=U.max_endpoint_jet_residual;
        BS=[U.first.control_points.';U.second.control_points.'];
        unconstrained_min(k)=min(BS*n);
        C=AF7R58R34_ONE_SIDED_C7_RETURN(t0,Tk,J0,JT,L,g,n);
        status{k}=C.status;
        if isfield(C,'integral_cost'),one_sided_cost(k)=C.integral_cost;end
        if isfield(C,'minimum_signed_control_m')
            one_sided_min(k)=C.minimum_signed_control_m;
        end
    catch ME
        status{k}=['EXCEPTION_' char(ME.identifier)];
    end
end
R=struct('schema','AF7R58R38_RETURN_COMPARE_V1', ...
    'table',table(durations(:),status,unconstrained_cost,one_sided_cost, ...
        unconstrained_min,one_sided_min,jet_res, ...
        'VariableNames',{'Duration_s','Status','UnconstrainedCost', ...
        'OneSidedCost','FreeMinProjection_m','OneSidedMinProjection_m', ...
        'UnscaledJetResidual'}), ...
    'departure_jets_0_to_7',J0,'terminal_jets_0_to_7',JT, ...
    'direction',n, ...
    'overshoot_necessity_under_all_obstacle_configurations',false, ...
    'wholebody_collision_certified',false, ...
    'directional_shape_is_not_collision_safety',true, ...
    'ready_for_automatic_commit',false);
end
