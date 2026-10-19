classdef REPLANNING_SYSTEM_R13_DIAG < REPLANNING_SYSTEM_R8R2_DIAG
% AF7R58ZZY-R13 | OBSERVER CALL-ORDINAL / SHAPE FORENSIC
% Diagnostic only. No optimization, no added warmup, no mission-phase shift.
% The exact R8-R2 observer equations are preserved. Existing preflight observer
% calls and real Flight calls are timed with one common call-ordinal counter.

    properties (Access = public)
        af7r65_obs_capacity = 32;
        af7r65_obs_width = 36;
        af7r65_obs_ring = nan(32,36);
        af7r65_obs_count = 0;
        af7r65_event_capacity = 32;
        af7r65_events = cell(1,32);
        af7r65_event_count = 0;
        af7r65_update_ordinal_total = 0;
        af7r65_one_ordinal_total = 0;
        af7r65_contract_ordinal_total = 0;
    end

    methods
        function obj = REPLANNING_SYSTEM_R13_DIAG(self,base_ref,opts)
            obj@REPLANNING_SYSTEM_R8R2_DIAG(self,base_ref,opts);
        end

        function observer_update_same_event(obj,time,cha)
            % Exact production observer_update_same_event with timing only.
            t=double(time.t);
            phase=char(string(cha));

            % R13: observe the EXISTING two-event preflight prime and real
            % Flight with the same internal timers. No extra observer call is
            % introduced and no warmup count is changed.
            isPre=logical(obj.observer_preflight_prime_active);
            isLive=strcmp(phase,'f') && ...
                ~obj.setup_exact_do_prime_active && ...
                ~obj.generator_integration_prime_active && ...
                ~isPre && t>=4.95-1e-10 && t<=5.20+1e-10;
            cap=isPre||isLive;
            contextCode=double(isPre)+2*double(isLive); % 1 preflight, 2 live
            obj.af7r65_update_ordinal_total=obj.af7r65_update_ordinal_total+1;
            updateOrdinal=double(obj.af7r65_update_ordinal_total);
            oneOrdinal=0; contractOrdinal=0;
            row=nan(1,obj.af7r65_obs_width);
            obj.af7r58zzy_r6_obs_capture_active=cap;
            if cap
                row(1)=t;
                row(33)=contextCode;
                row(34)=updateOrdinal;
                row(35)=0;
                row(36)=0;
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
                    '[AF7R58ZZY-R8] observer lazy initialization did not establish update_count.');
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
                        obj.af7r65_store_timing_row(row,isLive);
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

            % Keep references only; signature/byte analysis is deferred until
            % after the mission so it does not pollute the timed region.
            I_forensic=I;
            C0_forensic=I.previous_measurement;
            C1_forensic=C;
            xhat_forensic=xhat;
            u_forensic=[]; F_forensic=struct(); K_forensic=struct();

            h=NaN;
            if hasPrev
                h=t-double(I.previous_measurement.event_time);
                assert(isfinite(h)&&h>0, ...
                    '[REPLANNER OBSERVER ERROR] consecutive event interval is not positive.');
                if cap, row(30)=h; end

                if cap, T=tic; end
                uPrev=obj.observer_get_previous_held_input();
                if cap, row(8)=1e3*toc(T); end

                u_forensic=uPrev;
                obj.af7r65_one_ordinal_total=obj.af7r65_one_ordinal_total+1;
                oneOrdinal=double(obj.af7r65_one_ordinal_total);
                if cap, row(35)=oneOrdinal; end
                obj.af7r58zzy_r6_obs_one_step_sub(:)=NaN;
                if cap, T=tic; end
                F=obj.observer_one_step_support(h,uPrev,I);
                F_forensic=F;
                if cap
                    row(9)=1e3*toc(T);
                    row(10:13)=obj.af7r58zzy_r6_obs_one_step_sub;
                end
                assert(F.closed, ...
                    '[REPLANNER OBSERVER ERROR] continuous hidden-state support left certified chart.');

                obj.af7r65_contract_ordinal_total=obj.af7r65_contract_ordinal_total+1;
                contractOrdinal=double(obj.af7r65_contract_ordinal_total);
                if cap, row(36)=contractOrdinal; end
                obj.af7r58zzy_r6_obs_contract_sub(:)=NaN;
                if cap, T=tic; end
                K=obj.observer_contract_two_events(I.previous_measurement,C,F,xhat);
                K_forensic=K;
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
                obj.af7r65_store_timing_row(row,isLive);
            end
            obj.af7r58zzy_r6_obs_capture_active=false;

            % Untimed forensic capture. These values were already produced by
            % the exact observer path. No observer equation is re-evaluated.
            if cap && obj.af7r65_event_count<obj.af7r65_event_capacity
                obj.af7r65_event_count=obj.af7r65_event_count+1;
                E=struct();
                E.context_code=contextCode; E.context='';
                if isPre, E.context='PREFLIGHT'; elseif isLive, E.context='LIVE'; end
                E.t=double(t); E.phase=phase; E.update_ordinal=updateOrdinal;
                E.one_ordinal=oneOrdinal; E.contract_ordinal=contractOrdinal;
                E.has_prev=logical(hasPrev); E.h=double(h);
                E.static_cache_valid=logical(obj.observer_static_setup_cache_valid);
                E.I_before=I_forensic; E.C0=C0_forensic; E.C1=C1_forensic;
                E.xhat=xhat_forensic; E.uPrev=u_forensic; E.F=F_forensic; E.K=K_forensic;
                E.sensor_output_class=class(obj.self.sensor.result.output);
                E.sensor_output_size=size(obj.self.sensor.result.output);
                E.xhat_class=class(xhat_forensic); E.I_class=class(I_forensic);
                obj.af7r65_events{obj.af7r65_event_count}=E;
            end
        end


        function af7r65_store_timing_row(obj,row,isLive)
            obj.af7r65_obs_count=obj.af7r65_obs_count+1;
            assert(obj.af7r65_obs_count<=obj.af7r65_obs_capacity, ...
                '[AF7R58ZZY-R13] observer forensic ring exhausted.');
            obj.af7r65_obs_ring(obj.af7r65_obs_count,:)=row;
            % Preserve the inherited live observer ring contract used by the
            % existing full-path diagnostics, but never put preflight rows in it.
            if isLive
                obj.af7r58zzy_r6_obs_count=obj.af7r58zzy_r6_obs_count+1;
                assert(obj.af7r58zzy_r6_obs_count<=obj.af7r58zzy_r6_obs_capacity, ...
                    '[AF7R58ZZY-R13] inherited live observer ring exhausted.');
                obj.af7r58zzy_r6_obs_ring(obj.af7r58zzy_r6_obs_count,:)=row(1:32);
            end
        end
    end
end
