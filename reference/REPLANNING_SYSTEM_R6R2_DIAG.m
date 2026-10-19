classdef REPLANNING_SYSTEM_R6R2_DIAG < REPLANNING_SYSTEM_R5R4_DIAG
% AF7R58ZZY-R6-R2 | DUAL ROOT-CAUSE diagnostic subclass.
% Production authority SHA-256:
%   310cfdd01f9554a80b71e8d90464816a721859709ad7c4b9137cdc0f3a6eaf49
% Parent diagnostic authority SHA-256:
%   2b874c2e92052de1c756c3b00f8f62f5cb08018a496b9c78b981df085f2efb18
%
% Diagnostic only. Production REPLANNING_SYSTEM.m is not modified.
% The production safety/trajectory equations are copied verbatim in the
% overridden observer/reference methods; only tic/toc and numeric capture
% assignments are added.

    properties (Access = public)
        af7r58zzy_r6_obs_capacity = 64;
        af7r58zzy_r6_obs_width = 32;
        af7r58zzy_r6_obs_ring = nan(64,32);
        af7r58zzy_r6_obs_count = 0;
        af7r58zzy_r6_obs_one_step_sub = nan(1,4);
        af7r58zzy_r6_obs_contract_sub = nan(1,5);
        af7r58zzy_r6_obs_capture_active = false;

        af7r58zzy_r6_ref_capacity = 64;
        af7r58zzy_r6_ref_width = 32;
        af7r58zzy_r6_ref_ring = nan(64,32);
        af7r58zzy_r6_ref_count = 0;
        af7r58zzy_r6_ref_assert_sub = nan(1,7);
        af7r58zzy_r6_ref_capture_active = false;
    end

    methods
        function obj = REPLANNING_SYSTEM_R6R2_DIAG(self,base_ref,opts)
            obj@REPLANNING_SYSTEM_R5R4_DIAG(self,base_ref,opts);
        end

        function observer_update_same_event(obj,time,cha)
            % Exact production observer_update_same_event with timing only.
            t=double(time.t);
            phase=char(string(cha));

            cap=strcmp(phase,'f') && ...
                ~obj.setup_exact_do_prime_active && ...
                ~obj.generator_integration_prime_active && ...
                ~obj.observer_preflight_prime_active && ...
                t>=4.95-1e-10 && t<=5.20+1e-10;
            row=nan(1,obj.af7r58zzy_r6_obs_width);
            obj.af7r58zzy_r6_obs_capture_active=cap;
            if cap
                row(1)=t;
                Ttotal=tic;
            end

            if cap, T=tic; end
            I=obj.observer_hard_support_internal;
            if cap
                row(3)=1e3*toc(T);
                row(31)=double(logical(obj.observer_static_setup_cache_valid));
            end

            if cap, T=tic; end
            if isempty(I)||~isstruct(I)||~isfield(I,'initialized')||~I.initialized
                obj.observer_initialize_lazy(t,phase);
                I=obj.observer_hard_support_internal;
            end
            if cap
                row(4)=1e3*toc(T);
                assert(isstruct(I)&&isfield(I,'update_count'), ...
                    '[AF7R58ZZY-R6-R2] observer lazy initialization did not establish update_count.');
                row(29)=double(I.update_count);
            end

            if cap, T=tic; end
            if isfinite(I.last_event_time)
                if t==I.last_event_time
                    if cap
                        row(5)=1e3*toc(T);
                        row(27)=double(isstruct(I.previous_measurement)&& ...
                            isfield(I.previous_measurement,'closed')&&logical(I.previous_measurement.closed));
                        row(32)=0;
                        row(2)=1e3*toc(Ttotal);
                        obj.af7r58zzy_r6_obs_count=obj.af7r58zzy_r6_obs_count+1;
                        assert(obj.af7r58zzy_r6_obs_count<=obj.af7r58zzy_r6_obs_capacity, ...
                            '[AF7R58ZZY-R6] observer diagnostic ring exhausted.');
                        obj.af7r58zzy_r6_obs_ring(obj.af7r58zzy_r6_obs_count,:)=row;
                    end
                    obj.af7r58zzy_r6_obs_capture_active=false;
                    return;
                end
                assert(t>I.last_event_time, ...
                    '[REPLANNER OBSERVER ERROR] control-event time moved backwards.');
            end

            if ~isempty(I.last_phase)&&~strcmp(I.last_phase,phase)
                I.phase_transition_count=I.phase_transition_count+1;
            end
            if cap, row(5)=1e3*toc(T); end

            if cap, T=tic; end
            C=obj.observer_build_measurement_event(t,phase,I.selector_verified);
            if cap, row(6)=1e3*toc(T); end
            if ~C.closed
                I.fail_count=I.fail_count+1;
                obj.observer_hard_support_internal=I;
                error('[REPLANNER OBSERVER ERROR] measurement hard box failed: %s',C.reason);
            end

            if cap, T=tic; end
            xhat=obj.self.estimator.result.state;
            hasPrev=isstruct(I.previous_measurement)&& ...
                isfield(I.previous_measurement,'closed')&&logical(I.previous_measurement.closed);
            if cap
                row(7)=1e3*toc(T);
                row(27)=double(hasPrev);
            end

            h=NaN;
            if hasPrev
                h=t-double(I.previous_measurement.event_time);
                assert(isfinite(h)&&h>0, ...
                    '[REPLANNER OBSERVER ERROR] consecutive event interval is not positive.');
                if cap, row(30)=h; end

                if cap, T=tic; end
                uPrev=obj.observer_get_previous_held_input();
                if cap, row(8)=1e3*toc(T); end

                obj.af7r58zzy_r6_obs_one_step_sub(:)=NaN;
                if cap, T=tic; end
                F=obj.observer_one_step_support(h,uPrev,I);
                if cap
                    row(9)=1e3*toc(T);
                    row(10:13)=obj.af7r58zzy_r6_obs_one_step_sub;
                end
                assert(F.closed, ...
                    '[REPLANNER OBSERVER ERROR] continuous hidden-state support left certified chart.');

                obj.af7r58zzy_r6_obs_contract_sub(:)=NaN;
                if cap, T=tic; end
                K=obj.observer_contract_two_events(I.previous_measurement,C,F,xhat);
                if cap
                    row(14)=1e3*toc(T);
                    row(15:19)=obj.af7r58zzy_r6_obs_contract_sub;
                end

                if cap, T=tic; end
                I.hidden.w_center=K.w.center;
                I.hidden.w_radius=K.w.radius;
                I.hidden.w_norm_ub=K.w.true_norm_ub;
                I.hidden.vL_center=K.vL.center;
                I.hidden.vL_radius=K.vL.radius;
                I.hidden.vL_norm_ub=K.vL.true_norm_ub;
                I.hidden.wL_center=K.wL.center;
                I.hidden.wL_radius=K.wL.radius;
                I.hidden.wL_norm_ub=K.wL.true_norm_ub;
                measured=K.measured_error;
                ew=K.w.observer_error_norm_ub;
                evL=K.vL.observer_error_norm_ub;
                ewL=K.wL.observer_error_norm_ub;
                contractionUsed=true;
                I.last_forward_support=F;
                if cap, row(20)=1e3*toc(T); end
            else
                if cap, T=tic; end
                measured=obj.observer_measured_error_box(xhat,C);
                ew=norm(double(xhat.w(:))-I.hidden.w_center)+I.hidden.w_radius;
                evL=norm(double(xhat.vL(:))-I.hidden.vL_center)+I.hidden.vL_radius;
                ewL=norm(double(xhat.wL(:))-I.hidden.wL_center)+I.hidden.wL_radius;
                contractionUsed=false;
                I.last_forward_support=struct();
                if cap, row(21)=1e3*toc(T); end
            end
            if cap, row(28)=double(contractionUsed); end

            if cap, T=tic; end
            mi=I.true_mL_interval;
            hm=double(xhat.mL);
            emL=max(abs(hm-mi(1)),abs(hm-mi(2)));
            if cap, row(22)=1e3*toc(T); end

            if cap, T=tic; end
            B=struct();
            B.schema='REPLANNING_SYSTEM_OBSERVER_HARD_SUPPORT_EVENT_V2';
            B.closed=true;
            B.event_time=t;
            B.phase=phase;
            B.protection_eligible=strcmp(phase,'f');
            B.measured=measured;
            B.w_error_norm_ub=ew;
            B.vL_error_norm_ub=evL;
            B.wL_error_norm_ub=ewL;
            B.mL_error_abs_ub=emL;
            B.true_mL_interval=mi;
            B.hidden_true_support=I.hidden;
            B.consecutive_measurement_contraction_used=contractionUsed;
            B.numeric_plant_to_continuous_authority=false;
            B.realized_error_compared=false;
            B.Q_promoted=false;
            B.R_promoted=false;
            B.covariance_promoted=false;
            B.sampled_acceptance=false;
            if cap, row(23)=1e3*toc(T); end

            if cap, T=tic; end
            I.q_abs_ub_current=abs(C.y(4:6))+C.nu_abs_ub(4:6);
            I.previous_measurement=C;
            I.observer_support=B;
            if cap, row(24)=1e3*toc(T); end

            if cap, T=tic; end
            I.hlc_state_mass_bridge=obj.observer_build_state_mass_bridge(B);
            if cap, row(25)=1e3*toc(T); end

            if cap, T=tic; end
            I.last_event_time=t;
            I.last_phase=phase;
            I.update_count=I.update_count+1;
            obj.observer_hard_support_internal=I;
            if cap
                row(26)=1e3*toc(T);
                row(32)=1;
                row(2)=1e3*toc(Ttotal);
                obj.af7r58zzy_r6_obs_count=obj.af7r58zzy_r6_obs_count+1;
                assert(obj.af7r58zzy_r6_obs_count<=obj.af7r58zzy_r6_obs_capacity, ...
                    '[AF7R58ZZY-R6] observer diagnostic ring exhausted.');
                obj.af7r58zzy_r6_obs_ring(obj.af7r58zzy_r6_obs_count,:)=row;
            end
            obj.af7r58zzy_r6_obs_capture_active=false;
        end


        function S = observer_one_step_support(obj,h,u,I)
            % Exact production observer_one_step_support with sub-timers.
            cap=logical(obj.af7r58zzy_r6_obs_capture_active);
            sub=nan(1,4);

            if cap, T=tic; end
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
            if cap, sub(1)=1e3*toc(T); end

            if cap, T=tic; end
            Jmin=min(Jd); Jmax=max(Jd);
            tauNorm=norm(u(2:4)); u1abs=abs(u(1));
            W0=double(H.w_norm_ub); WL0=double(H.wL_norm_ub);
            VL0=double(H.vL_norm_ub); q0=double(I.q_abs_ub_current(:));
            Pnorm=double(I.pT_norm_ub); mLint=double(I.true_mL_interval(:));
            mLlb=mLint(1);

            V0=0.5*Jmax*W0^2;
            sqrtVend=sqrt(V0)+tauNorm*h/sqrt(2*Jmin);
            VwEnd=sqrtVend^2;
            Wabs0=sqrt(2*V0/Jmin); WabsEnd=sqrt(2*VwEnd/Jmin);
            Iw=Wabs0*h+tauNorm*h^2/(2*Jmin);

            Theta=q0(2)+Iw;
            pitchClosed=Theta<pi/2;
            qAbs=[Inf;Theta;Inf]; fullClosed=false;
            if pitchClosed
                qAbs(1)=q0(1)+(1+tan(Theta))*Iw;
                qAbs(3)=q0(3)+(1/cos(Theta))*Iw;
                fullClosed=qAbs(1)<pi&&qAbs(3)<pi;
            end
            if cap, sub(2)=1e3*toc(T); end

            if cap, T=tic; end
            aWL=Pnorm*u1abs/(m*L);
            WLEnd=WL0+aWL*h;
            IWL=WL0*h+0.5*aWL*h^2;
            pTSpan=Pnorm*IWL; %#ok<NASGU>
            den=m+mLlb;
            assert(den>0,'[REPLANNER OBSERVER ERROR] total mass lower bound invalid.');
            Aconst=g+(Pnorm^2*u1abs)/den;
            Aquad=(Pnorm^3)*m*L/den;
            intWL2=WL0^2*h+WL0*aWL*h^2+(aWL^2*h^3)/3;
            VLEnd=VL0+Aconst*h+Aquad*intWL2;
            ALmax=Aconst+Aquad*(WLEnd^2);
            if cap, sub(3)=1e3*toc(T); end

            if cap, T=tic; end
            S=struct();
            S.closed=pitchClosed&&fullClosed; S.h=h; S.held_input=u;
            S.Jdiag=Jd; S.w_norm_ub_end=WabsEnd;
            S.q_abs_ub_interval=qAbs;
            S.wL_accel_norm_ub=aWL; S.wL_norm_ub_end=WLEnd;
            S.vL_accel_norm_ub=ALmax; S.vL_norm_ub_end=VLEnd; S.pT_norm_ub=Pnorm;
            S.sampled_acceptance=false; S.guessed_bound=false;
            if cap, sub(4)=1e3*toc(T); end
            obj.af7r58zzy_r6_obs_one_step_sub=sub;
        end


        function O = observer_contract_two_events(obj,C0,C1,F,xhat)
            % Exact production observer_contract_two_events with sub-timers.
            cap=logical(obj.af7r58zzy_r6_obs_capture_active);
            sub=nan(1,5);

            y0=double(C0.y(:)); y1=double(C1.y(:));
            n0=double(C0.nu_abs_ub(:)); n1=double(C1.nu_abs_ub(:));
            h=double(C1.event_time-C0.event_time);
            assert(h>0&&isfinite(h),'[REPLANNER OBSERVER ERROR] contraction h invalid.');
            iq=4:6; ipL=7:9; ipT=10:12;
            if cap, T=tic; end
            MEAS=obj.observer_measured_error_box(xhat,C1);
            if cap, sub(1)=1e3*toc(T); end

            if cap, T=tic; end
            VLF=double(F.vL_norm_ub_end);
            ALmax=double(F.vL_accel_norm_ub);
            dPL=(y1(ipL)-y0(ipL))/h;
            rVL=0.5*ALmax*h+(norm(n0(ipL))+norm(n1(ipL)))/h;
            cVL=dPL;
            VLtrue=min(VLF,norm(cVL)+rVL);
            eVL=min(norm(double(xhat.vL(:)))+VLF, ...
                norm(double(xhat.vL(:))-cVL)+rVL);
            if cap, sub(2)=1e3*toc(T); end

            if cap, T=tic; end
            WLF=double(F.wL_norm_ub_end); Pbar=double(F.pT_norm_ub);
            aWL=double(F.wL_accel_norm_ub);
            pTddot=Pbar*(aWL+WLF^2);
            dPT=(y1(ipT)-y0(ipT))/h;
            rPTdot=0.5*pTddot*h+(norm(n0(ipT))+norm(n1(ipT)))/h;
            cWL=cross(y1(ipT),dPT);
            pTdotUb=Pbar*WLF;
            rWL=norm(n1(ipT))*pTdotUb+norm(y1(ipT))*rPTdot;
            WLtrue=min(WLF,norm(cWL)+rWL);
            eWL=min(norm(double(xhat.wL(:)))+WLF, ...
                norm(double(xhat.wL(:))-cWL)+rWL);
            if cap, sub(3)=1e3*toc(T); end

            if cap, T=tic; end
            Wf=double(F.w_norm_ub_end); Jd=double(F.Jdiag(:));
            Jmin=min(Jd); Jmax=max(Jd); tauNorm=norm(double(F.held_input(2:4)));
            Theta=double(F.q_abs_ub_interval(2));
            assert(isfinite(Theta)&&Theta<pi/2, ...
                '[REPLANNER OBSERVER ERROR] Euler chart is not closed.');
            tanT=tan(Theta); secT=1/cos(Theta); Kphi=1+tanT;
            Aomega=tauNorm/Jmin+(Jmax/Jmin)*Wf^2;
            adot=Aomega+Kphi*Wf^2;
            qddotUb=norm([Aomega+secT^2*Wf^2+tanT*adot; ...
                Aomega+Kphi*Wf^2; secT*(tanT*Wf^2+adot)]);
            dq=(y1(iq)-y0(iq))/h;
            rQdot=0.5*qddotUb*h+(norm(n0(iq))+norm(n1(iq)))/h;
            lim=[pi;pi/2;pi];
            assert(all(abs(y0(iq))+n0(iq)<lim)&&all(abs(y1(iq))+n1(iq)<lim), ...
                '[REPLANNER OBSERVER ERROR] measured Euler box reaches chart boundary.');
            Ey=obj.observer_euler_rate_inverse_ZYX(y1(iq));
            cW=Ey*dq;
            dphi=n1(4); dtheta=n1(5);
            LE=sqrt(dtheta^2+2*dphi^2+2*(dphi+dtheta)^2);
            Kq=sqrt(Kphi^2+1+secT^2); qdotUb=Kq*Wf;
            EyNorm2=obj.observer_nextfloat_up( ...
                sqrt(1+abs(sin(double(y1(iq(2)))))));
            rW=LE*qdotUb+EyNorm2*rQdot;
            Wtrue=min(Wf,norm(cW)+rW);
            eW=min(norm(double(xhat.w(:)))+Wf,norm(double(xhat.w(:))-cW)+rW);
            if cap, sub(4)=1e3*toc(T); end

            if cap, T=tic; end
            O=struct(); O.closed=true; O.h=h; O.measured_error=MEAS;
            O.vL=struct('center',cVL,'radius',rVL,'true_norm_ub',VLtrue, ...
                'observer_error_norm_ub',eVL);
            O.wL=struct('center',cWL,'radius',rWL,'true_norm_ub',WLtrue, ...
                'observer_error_norm_ub',eWL);
            O.w=struct('center',cW,'radius',rW,'true_norm_ub',Wtrue, ...
                'observer_error_norm_ub',eW);
            if cap, sub(5)=1e3*toc(T); end
            obj.af7r58zzy_r6_obs_contract_sub=sub;
        end


        function [pL_ref,q_ref,source] = repaired_reference_packet(obj,t_now)
            % Exact production repaired_reference_packet with timing only.
            t0=double(t_now);
            cap=~obj.setup_exact_do_prime_active && ...
                ~obj.generator_integration_prime_active && ...
                ~obj.observer_preflight_prime_active && ...
                t0>=9.90-1e-10 && t0<=10.05+1e-10;
            row=nan(1,obj.af7r58zzy_r6_ref_width);
            obj.af7r58zzy_r6_ref_capture_active=cap;
            if cap
                row(1)=t0;
                Ttotal=tic;
                obj.af7r58zzy_r6_ref_assert_sub(:)=NaN;
            end

            if cap, T=tic; end
            obj.assert_repaired_reference_current();
            if cap
                row(3)=1e3*toc(T);
                row(4:10)=obj.af7r58zzy_r6_ref_assert_sub;
            end

            if cap, T=tic; end
            N=round(obj.horizon_s/obj.prediction_dt)+1;
            dt=double(obj.prediction_dt);
            H=double(obj.horizon_s);
            t0=double(t_now);
            R=obj.active_repaired_reference;

            assert(isfinite(dt)&&dt>0 && isfinite(H)&&H>=0, ...
                '[REPLANNER ERROR] invalid repaired packet timing parameters.');
            assert(isfield(R,'batch_flat_model') && isstruct(R.batch_flat_model) && ...
                isfield(R.batch_flat_model,'certified') && logical(R.batch_flat_model.certified), ...
                '[REPLANNER ERROR] repaired batch flat model unavailable.');
            assert(isfield(R,'flat_eval_fn') && isa(R.flat_eval_fn,'function_handle'), ...
                '[REPLANNER ERROR] repaired flat evaluator unavailable.');
            if cap
                row(11)=1e3*toc(T);
                row(24)=double(obj.active_reference_version);
                row(27)=double(N); row(28)=H; row(29)=dt;
                row(30)=1 + double(~strcmpi(char(obj.active_repaired_safety_level),'REFERENCE_ONLY'));
            end

            if cap, T=tic; end
            C=obj.repaired_reference_cache;
            canReuse=obj.repaired_reference_cache_valid && isstruct(C) && ...
                isfield(C,'reference_version') && ...
                double(C.reference_version)==double(obj.active_reference_version) && ...
                isfield(C,'dt') && isequal(double(C.dt),dt) && ...
                isfield(C,'H') && isequal(double(C.H),H) && ...
                isfield(C,'t0') && isfinite(double(C.t0)) && ...
                isfield(C,'pL') && isequal(size(C.pL),[3 N]) && ...
                isfield(C,'q') && isequal(size(C.q),[3 N]);
            if cap
                row(12)=1e3*toc(T);
                row(16)=double(canReuse);
                row(17)=double(logical(obj.repaired_reference_cache_valid));
                if isstruct(C)&&isfield(C,'reference_version')&&isfinite(double(C.reference_version))
                    row(25)=double(C.reference_version);
                end
                if isstruct(C)&&isfield(C,'t0')&&isfinite(double(C.t0))
                    row(26)=double(C.t0);
                    row(31)=t0-double(C.t0);
                end
            end

            if cap, T=tic; end
            shiftCount=NaN;
            if canReuse
                d=(t0-double(C.t0))/dt;
                if isfinite(d)
                    shiftCount=round(d);
                    talign=double(C.t0)+double(shiftCount)*dt;
                    scale=max([1,abs(t0),abs(double(C.t0)),abs(double(shiftCount)*dt)]);
                    guard=64*eps(scale);
                    if shiftCount<0 || abs(t0-talign)>guard
                        shiftCount=NaN;
                    end
                end
            end
            if cap
                row(13)=1e3*toc(T);
                row(15)=shiftCount;
            end

            if canReuse && isfinite(shiftCount) && shiftCount==0
                row(14)=1;
                pL_ref=C.pL;
                q_ref=C.q;
                source=sprintf('REPAIRED_ROLLING_CACHE_HIT:v%d:%s', ...
                    obj.active_reference_version,char(R.backend));
                if cap, T=tic; end
                obj.validate_reference_packet(pL_ref,q_ref,N);
                if cap
                    row(22)=1e3*toc(T);
                    row(32)=1;
                    row(2)=1e3*toc(Ttotal);
                    obj.af7r58zzy_r6_ref_count=obj.af7r58zzy_r6_ref_count+1;
                    assert(obj.af7r58zzy_r6_ref_count<=obj.af7r58zzy_r6_ref_capacity, ...
                        '[AF7R58ZZY-R6] repaired-reference diagnostic ring exhausted.');
                    obj.af7r58zzy_r6_ref_ring(obj.af7r58zzy_r6_ref_count,:)=row;
                end
                obj.af7r58zzy_r6_ref_capture_active=false;
                return;
            end

            if canReuse && isfinite(shiftCount) && shiftCount>=1 && shiftCount<N
                row(14)=2 + double(shiftCount>1);
                m=double(shiftCount);
                if cap, T=tic; end
                pL_ref=[C.pL(:,m+1:end),zeros(3,m)];
                q_ref=[C.q(:,m+1:end),zeros(3,m)];
                if cap, row(18)=1e3*toc(T); end

                if m==1
                    if cap, T=tic; end
                    xf=double(R.flat_eval_fn(t0+(N-1)*dt));
                    xf=xf(:);
                    assert(numel(xf)>=11 && all(isfinite(xf(1:11))), ...
                        '[REPLANNER ERROR] repaired rolling tail evaluator invalid.');
                    if cap, row(19)=1e3*toc(T); end
                    if cap, T=tic; end
                    pL_ref(:,end)=xf(1:3);
                    sf=xf(9:11)+[0;0;double(obj.gravity)];
                    ns=norm(sf);
                    assert(isfinite(ns)&&ns>0, ...
                        '[REPLANNER ERROR] repaired rolling tail flatness singular.');
                    q_ref(:,end)=-sf/ns;
                    source=sprintf('REPAIRED_ROLLING_SHIFT1:v%d:%s', ...
                        obj.active_reference_version,char(R.backend));
                    if cap, row(21)=1e3*toc(T); end
                else
                    if cap, T=tic; end
                    tq=t0+(N-m:N-1)*dt;
                    [Y,valid]=af7r33_flat_jet_batch_model(obj,tq,R.batch_flat_model);
                    Y=double(Y);
                    assert(isequal(size(Y),[21 m]) && numel(valid)==m && ...
                        all(logical(valid(:))) && all(isfinite(Y),'all'), ...
                        '[REPLANNER ERROR] repaired rolling tail batch invalid.');
                    if cap, row(20)=1e3*toc(T); end
                    if cap, T=tic; end
                    pL_ref(:,end-m+1:end)=Y(1:3,:);
                    sf=Y(7:9,:)+[0;0;double(obj.gravity)];
                    ns=sqrt(sum(sf.^2,1));
                    assert(all(isfinite(ns))&&all(ns>0), ...
                        '[REPLANNER ERROR] repaired rolling tail batch flatness singular.');
                    q_ref(:,end-m+1:end)=-sf./ns;
                    source=sprintf('REPAIRED_ROLLING_SHIFTM:v%d:%s', ...
                        obj.active_reference_version,char(R.backend));
                    if cap, row(21)=1e3*toc(T); end
                end
            else
                row(14)=4;
                if cap, T=tic; end
                tq=t0+(0:N-1)*dt;
                [Y,valid]=af7r33_flat_jet_batch_model(obj,tq,R.batch_flat_model);
                Y=double(Y);
                assert(isequal(size(Y),[21 N]) && numel(valid)==N && ...
                    all(logical(valid(:))) && all(isfinite(Y),'all'), ...
                    '[REPLANNER ERROR] repaired full batch packet invalid.');
                if cap, row(20)=1e3*toc(T); end
                if cap, T=tic; end
                pL_ref=Y(1:3,:);
                sf=Y(7:9,:)+[0;0;double(obj.gravity)];
                ns=sqrt(sum(sf.^2,1));
                assert(all(isfinite(ns))&&all(ns>0), ...
                    '[REPLANNER ERROR] repaired full batch flatness singular.');
                q_ref=-sf./ns;
                source=sprintf('REPAIRED_FULL_BATCH:v%d:%s', ...
                    obj.active_reference_version,char(R.backend));
                if cap, row(21)=1e3*toc(T); end
            end

            if cap, T=tic; end
            obj.validate_reference_packet(pL_ref,q_ref,N);
            if cap, row(22)=1e3*toc(T); end
            if cap, T=tic; end
            obj.repaired_reference_cache=struct( ...
                'reference_version',double(obj.active_reference_version), ...
                't0',t0,'dt',dt,'H',H,'pL',pL_ref,'q',q_ref);
            obj.repaired_reference_cache_valid=true;
            if cap
                row(23)=1e3*toc(T);
                row(32)=1;
                row(2)=1e3*toc(Ttotal);
                obj.af7r58zzy_r6_ref_count=obj.af7r58zzy_r6_ref_count+1;
                assert(obj.af7r58zzy_r6_ref_count<=obj.af7r58zzy_r6_ref_capacity, ...
                    '[AF7R58ZZY-R6] repaired-reference diagnostic ring exhausted.');
                obj.af7r58zzy_r6_ref_ring(obj.af7r58zzy_r6_ref_count,:)=row;
            end
            obj.af7r58zzy_r6_ref_capture_active=false;
        end


        function assert_repaired_reference_current(obj)
            % Exact production assertion logic with sub-timers.
            cap=logical(obj.af7r58zzy_r6_ref_capture_active);
            sub=nan(1,7);

            R=obj.active_repaired_reference;
            C=obj.active_repaired_certificates;

            if cap, T=tic; end
            obj.assert_hard_modify_deadline_limit();
            if cap, sub(1)=1e3*toc(T); end

            if cap, T=tic; end
            assert(strcmpi(char(obj.active_reference_mode),'REPAIRED'), ...
                '[REPLANNER ERROR] repaired reference assertion used outside REPAIRED mode.');
            assert(isstruct(R) && isfield(R,'valid') && logical(R.valid), ...
                '[REPLANNER ERROR] repaired reference is unavailable.');
            assert(isfield(R,'reference_version') && ...
                double(R.reference_version)==double(obj.active_reference_version), ...
                '[REPLANNER ERROR] repaired reference version is stale.');
            assert(isfield(R,'eval_fn') && isa(R.eval_fn,'function_handle'), ...
                '[REPLANNER ERROR] repaired reference eval_fn is unavailable.');
            if cap, sub(2)=1e3*toc(T); end

            if strcmpi(char(obj.active_repaired_safety_level),'REFERENCE_ONLY')
                if cap, T=tic; end
                assert(isfield(R,'flat_eval_fn') && ...
                    isa(R.flat_eval_fn,'function_handle'), ...
                    '[REPLANNER ERROR] reference-only flat-jet authority unavailable.');
                assert(isstruct(C) && isfield(C,'reference_only') && ...
                    logical(C.reference_only), ...
                    '[REPLANNER ERROR] active reference-only repair metadata missing.');
                if cap, sub(3)=1e3*toc(T); end

                if cap, T=tic; end
                assert(isfield(C,'smoothness') && isstruct(C.smoothness) && ...
                    isfield(C.smoothness,'certified') && ...
                    logical(C.smoothness.certified) && ...
                    isfield(C.smoothness,'continuous_time') && ...
                    logical(C.smoothness.continuous_time), ...
                    '[REPLANNER ERROR] reference-only C7 smoothness certificate missing.');
                if cap, sub(4)=1e3*toc(T); end

                if cap, T=tic; end
                assert(isfield(C,'geometry') && isstruct(C.geometry) && ...
                    isfield(C.geometry,'certified') && ...
                    logical(C.geometry.certified) && ...
                    isfield(C.geometry,'continuous_time') && ...
                    logical(C.geometry.continuous_time) && ...
                    isfield(C.geometry,'sampled_acceptance') && ...
                    ~logical(C.geometry.sampled_acceptance), ...
                    '[REPLANNER ERROR] reference-only continuous geometry certificate missing.');
                if cap, sub(5)=1e3*toc(T); end

                if cap, T=tic; end
                obj.validate_active_speed_certificate(obj.active_speed_certificate);
                if cap
                    sub(6)=1e3*toc(T);
                    obj.af7r58zzy_r6_ref_assert_sub=sub;
                end
                return;
            end

            if cap, T=tic; end
            assert(strcmpi(char(obj.active_repaired_safety_level),'EXECUTION_AWARE'), ...
                '[REPLANNER ERROR] repaired safety level is neither REFERENCE_ONLY nor EXECUTION_AWARE.');
            assert(isfield(R,'flat_eval_fn') && ...
                isa(R.flat_eval_fn,'function_handle'), ...
                '[REPLANNER ERROR] execution-aware flat-jet authority unavailable.');

            assert(isfield(R,'yaw_policy') && ...
                strcmpi(char(R.yaw_policy),'C6_INDEPENDENT'), ...
                '[REPLANNER ERROR] active repaired yaw policy is not C6_INDEPENDENT.');
            assert(isfield(R,'path_parameterization') && ...
                isstruct(R.path_parameterization), ...
                '[REPLANNER ERROR] active repaired path parameterization is unavailable.');

            required={'smoothness','geometry','dynamics','hlc_robust','backup', ...
                'position_c6','yaw_c6','path_position_c7', ...
                'time_law_c6','time_law_monotonic','terminal_time_law_hold', ...
                'finite_sensing','compute_deadline','dependency'};
            for kk=1:numel(required)
                name=required{kk};
                assert(isstruct(C) && isfield(C,name) && ...
                    isstruct(C.(name)) && ...
                    isfield(C.(name),'certified') && ...
                    logical(C.(name).certified), ...
                    '[REPLANNER ERROR] repaired reference certificate %s is unavailable/stale.', ...
                    name);
                if isfield(C.(name),'test_only')
                    assert(~logical(C.(name).test_only), ...
                        '[REPLANNER ERROR] active repaired certificate %s is test-only.',name);
                end
            end

            targetVersion=double(obj.active_reference_version);
            vf=double(R.valid_from);
            vu=double(R.valid_until);

            boundNames={'smoothness','geometry','dynamics','hlc_robust','backup', ...
                'position_c6','yaw_c6','path_position_c7', ...
                'time_law_c6','time_law_monotonic','terminal_time_law_hold', ...
                'finite_sensing'};
            for kk=1:numel(boundNames)
                name=boundNames{kk};
                obj.validate_repaired_bound_certificate( ...
                    C.(name),name,targetVersion,vf,vu);
            end

            assert(isfield(C.time_law_monotonic,'sdot_lower_bound') && ...
                double(C.time_law_monotonic.sdot_lower_bound)>=0, ...
                '[REPLANNER ERROR] active time-law monotonicity certificate is invalid.');
            assert(isfield(C.terminal_time_law_hold,'zero_derivative_order') && ...
                double(C.terminal_time_law_hold.zero_derivative_order)>=6 && ...
                isfield(C.terminal_time_law_hold,'constant_hold_after_terminal') && ...
                logical(C.terminal_time_law_hold.constant_hold_after_terminal), ...
                '[REPLANNER ERROR] active terminal time-law hold certificate is invalid.');

            obj.validate_candidate_dependency_certificate( ...
                C.dependency,R.eval_fn,targetVersion);

            obj.validate_active_speed_certificate(obj.active_speed_certificate);
            if cap
                sub(7)=1e3*toc(T);
                obj.af7r58zzy_r6_ref_assert_sub=sub;
            end
        end
    end
end
