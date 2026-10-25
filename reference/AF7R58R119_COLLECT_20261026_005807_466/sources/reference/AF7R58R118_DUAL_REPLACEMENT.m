function O=AF7R58R118_DUAL_REPLACEMENT(action,request)
% Independent source-only replacement design: NO calls to legacy reference
% trigger or native old candidate generator; NO production commit allowed.
% R115 Flight binding is deliberately not mutated by this research module.
switch lower(char(action))
    case 'trigger'
        O=AF7R58R118_EKF_TRIGGER(request);
    case 'plan'
        O=AF7R58R118_C7_CORRIDOR_PLANNER(request);
    case 'decide'
        assert(isstruct(request)&&all(isfield(request, ...
          {'physical_trigger','active_reference_current_horizon_certified', ...
          'same_reference_version','pending_same_hazard'})), ...
          'R118:DecisionAuthority');
        T=request.physical_trigger;
        O=struct('replan',false,'keep_certified_plan',false, ...
          'pending_unresolved',false,'status','UNKNOWN_NO_SAFETY_AUTHORITY');
        if logical(T.proved_clear)&& ...
           logical(request.active_reference_current_horizon_certified)&& ...
           logical(request.same_reference_version)
            O.keep_certified_plan=true;
            O.status='KEEP_CERTIFIED_CURRENT_PLAN_NO_DUPLICATE_REPLAN';
        elseif logical(request.pending_same_hazard)
            % Pending is NOT safety. Don't re-run the same generator solely
            % because this static obstacle remains sensed every 25 ms.
            O.pending_unresolved=true;
            O.status='AWAIT_CURRENT_HAZARD_SOLUTION__NO_DUPLICATE_GENERATION';
        elseif logical(T.request)
            O.replan=true;
            O.status='NEW_MATERIAL_PHYSICAL_RISK_OR_INCONCLUSIVE_FORECAST';
        end
    otherwise
        error('R118:UnknownAction','trigger, plan, decide');
end
end
