classdef REPLANNING_SYSTEM_R14R3_DIAG < REPLANNING_SYSTEM_R8R2_DIAG
% AF7R58ZZY-R14-R3 | AOT/MEX OBSERVER NUMERIC-KERNEL SHADOW
% Shadow only. Production REPLANNING_SYSTEM.m is not modified.
% Existing observer preflight count is unchanged; no added warmup call.
% The exact observer theorem/struct contract is retained and guarded by
% post-run bitwise replay in the audit.

    properties (Access = public)
        af7r67_use_mex = true;
        af7r67_mex_mode = 0; % 0=native strict, 1=MATLAB-builtin-assisted
        af7r67_one_mex_calls = 0;
        af7r67_contract_mex_calls = 0;
        af7r67_fallback_calls = 0;
    end

    methods
        function obj = REPLANNING_SYSTEM_R14R3_DIAG(self,base_ref,opts)
            obj@REPLANNING_SYSTEM_R8R2_DIAG(self,base_ref,opts);
        end

        function S = observer_one_step_support(obj,h,u,I)
            if obj.af7r67_use_mex && obj.observer_static_setup_cache_valid
                obj.af7r67_one_mex_calls=obj.af7r67_one_mex_calls+1;
                S=af7r67_observer_mex(1,double(obj.af7r67_mex_mode), ...
                    h,u,I,obj.observer_static_setup_cache);
                return;
            end
            obj.af7r67_fallback_calls=obj.af7r67_fallback_calls+1;
            S=observer_one_step_support@REPLANNING_SYSTEM_R8R2_DIAG(obj,h,u,I);
        end

        function O = observer_contract_two_events(obj,C0,C1,F,xhat)
            if obj.af7r67_use_mex
                obj.af7r67_contract_mex_calls=obj.af7r67_contract_mex_calls+1;
                O=af7r67_observer_mex(2,double(obj.af7r67_mex_mode), ...
                    C0,C1,F, ...
                    double(xhat.p(:)),double(xhat.q(:)), ...
                    double(xhat.pL(:)),double(xhat.pT(:)), ...
                    double(xhat.vL(:)),double(xhat.wL(:)),double(xhat.w(:)));
                return;
            end
            obj.af7r67_fallback_calls=obj.af7r67_fallback_calls+1;
            O=observer_contract_two_events@REPLANNING_SYSTEM_R8R2_DIAG( ...
                obj,C0,C1,F,xhat);
        end
    end
end
