classdef REPLANNING_SYSTEM_R9R2_DIAG < REPLANNING_SYSTEM_R8R2_DIAG
% AF7R58ZZY-R9-R2 performance-only shadow repair.
% - No do() override.
% - No phase allocation edits.
% - Observer equations preserved; exact numeric kernels are verified against
%   the inherited production-equation implementation during the existing
%   pre-mission observer prime before they can be used in real Flight.
% - Batch acceleration is supplied by a path-shadowed exact helper prepared
%   by AF7R58ZZY_R9_R2_PERFORMANCE_REPAIR_AUDIT; this class does not alter the
%   trajectory/certificate/commit equations.

    properties (Access = public)
        af7r62_observer_one_equiv_closed = false;
        af7r62_observer_contract_equiv_closed = false;
        af7r62_observer_one_equiv_ms = NaN;
        af7r62_observer_contract_equiv_ms = NaN;
        af7r62_observer_kernel_live_one_calls = 0;
        af7r62_observer_kernel_live_contract_calls = 0;
    end

    methods
        function obj = REPLANNING_SYSTEM_R9R2_DIAG(self,base_ref,opts)
            obj@REPLANNING_SYSTEM_R8R2_DIAG(self,base_ref,opts);
        end

        function S = observer_one_step_support(obj,h,u,I)
            H=I.hidden;
            if obj.observer_static_setup_cache_valid
                K0=obj.observer_static_setup_cache;
                Jd=double(K0.Jdiag(:));
                m=double(K0.mass);
                g=double(K0.gravity);
                L=double(K0.cableL);
            else
                Jd=double(obj.self.parameter.get(["jx","jy","jz"])); Jd=Jd(:);
                m=double(obj.self.parameter.get("mass"));
                g=double(obj.self.parameter.get("gravity"));
                L=double(obj.self.parameter.get("cableL"));
            end
            assert(numel(Jd)==3&&all(isfinite(Jd))&&all(Jd>0)&& ...
                isfinite(m)&&m>0&&isfinite(g)&&g>=0&&isfinite(L)&&L>0, ...
                '[REPLANNER OBSERVER ERROR] physical parameter domain invalid.');

            T=tic;
            Sk=af7r62_observer_one_step_kernel( ...
                h,u,H,I.q_abs_ub_current,I.pT_norm_ub,I.true_mL_interval, ...
                Jd,m,g,L);

            if ~obj.af7r62_observer_one_equiv_closed
                assert(obj.observer_preflight_prime_active && obj.setup_exact_do_prime_active, ...
                    ['[AF7R62] observer kernel equivalence must close in the existing ' ...
                     'pre-mission observer prime before real mission phases.']);
                Sp=observer_one_step_support@REPLANNING_SYSTEM_R8R2_DIAG(obj,h,u,I);
                assert(isequaln(Sp,Sk), ...
                    '[AF7R62] observer one-step kernel is not bitwise/structurally identical.');
                obj.af7r62_observer_one_equiv_closed=true;
                obj.af7r62_observer_one_equiv_ms=1e3*toc(T);
            end

            if ~obj.observer_preflight_prime_active && ~obj.setup_exact_do_prime_active && ...
                    ~obj.generator_integration_prime_active
                obj.af7r62_observer_kernel_live_one_calls= ...
                    obj.af7r62_observer_kernel_live_one_calls+1;
            end
            S=Sk;
        end

        function O = observer_contract_two_events(obj,C0,C1,F,xhat)
            xp=double(xhat.p(:)); xq=double(xhat.q(:));
            xpL=double(xhat.pL(:)); xpT=double(xhat.pT(:));
            xvL=double(xhat.vL(:)); xwL=double(xhat.wL(:)); xw=double(xhat.w(:));

            T=tic;
            Ok=af7r62_observer_contract_kernel( ...
                C0,C1,F,xp,xq,xpL,xpT,xvL,xwL,xw);

            if ~obj.af7r62_observer_contract_equiv_closed
                assert(obj.observer_preflight_prime_active && obj.setup_exact_do_prime_active, ...
                    ['[AF7R62] observer contraction equivalence must close in the existing ' ...
                     'pre-mission observer prime before real mission phases.']);
                Op=observer_contract_two_events@REPLANNING_SYSTEM_R8R2_DIAG(obj,C0,C1,F,xhat);
                assert(isequaln(Op,Ok), ...
                    '[AF7R62] observer contraction kernel is not bitwise/structurally identical.');
                obj.af7r62_observer_contract_equiv_closed=true;
                obj.af7r62_observer_contract_equiv_ms=1e3*toc(T);
            end

            if ~obj.observer_preflight_prime_active && ~obj.setup_exact_do_prime_active && ...
                    ~obj.generator_integration_prime_active
                obj.af7r62_observer_kernel_live_contract_calls= ...
                    obj.af7r62_observer_kernel_live_contract_calls+1;
            end
            O=Ok;
        end

        function E = af7r62_compare_observer_snapshot(obj,S)
            % Post-mission diagnostic only. No Flight work is added.
            P1=observer_one_step_support@REPLANNING_SYSTEM_R8R2_DIAG(obj,S.h,S.uPrev,S.I);
            K1=af7r62_observer_one_step_kernel( ...
                S.h,S.uPrev,S.I.hidden,S.I.q_abs_ub_current,S.I.pT_norm_ub, ...
                S.I.true_mL_interval,double(P1.Jdiag(:)), ...
                double(obj.observer_static_setup_cache.mass), ...
                double(obj.observer_static_setup_cache.gravity), ...
                double(obj.observer_static_setup_cache.cableL));

            X=S.xhat;
            P2=observer_contract_two_events@REPLANNING_SYSTEM_R8R2_DIAG(obj,S.C0,S.C1,S.F,S.xhat);
            K2=af7r62_observer_contract_kernel( ...
                S.C0,S.C1,S.F,double(X.p(:)),double(X.q(:)),double(X.pL(:)), ...
                double(X.pT(:)),double(X.vL(:)),double(X.wL(:)),double(X.w(:)));
            E=struct('one_step_equal',isequaln(P1,K1), ...
                'one_step_matches_stored',isequaln(P1,S.F)&&isequaln(K1,S.F), ...
                'contract_equal',isequaln(P2,K2), ...
                'contract_matches_stored',isequaln(P2,S.K)&&isequaln(K2,S.K));
        end
    end
end
