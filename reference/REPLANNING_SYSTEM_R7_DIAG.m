classdef REPLANNING_SYSTEM_R7_DIAG < REPLANNING_SYSTEM_R5R4_DIAG
% AF7R58ZZY-R7 | NO-PHASE-SHIFT repair validation subclass.
% Production authority SHA-256:
%   310cfdd01f9554a80b71e8d90464816a721859709ad7c4b9137cdc0f3a6eaf49
% Parent timing diagnostic SHA-256:
%   2b874c2e92052de1c756c3b00f8f62f5cb08018a496b9c78b981df085f2efb18
%
% R7 changes ONLY Flight-time implementation work:
%   1) observer dynamic-struct/access reduction + exact vector nextfloat-up;
%   2) repaired future-packet evaluator computes only p (0th) and a (2nd)
%      derivatives required to form pL and q, instead of all 0..6 flat jets.
%
% R7 explicitly adds NO work to preflight/Arming/Takeoff/Landing and does
% not override do(), phase allocation, setup prime, or warmup logic.
% Continuous certificates, obstacle geometry, activation time, stale/atomic
% gates, reference/control equations and commit policy are unchanged.

    methods
        function obj = REPLANNING_SYSTEM_R7_DIAG(self,base_ref,opts)
            obj@REPLANNING_SYSTEM_R5R4_DIAG(self,base_ref,opts);
        end

        function observer_update_same_event(obj,time,cha)
            % AF7R59/R7 allocation-reduced observer hot path.
            % Same event, support and contraction equations as production.
            t=double(time.t);
            phase=char(string(cha));

            I=obj.observer_hard_support_internal;
            if isempty(I)||~isstruct(I)||~isfield(I,'initialized')||~I.initialized
                obj.observer_initialize_lazy(t,phase);
                I=obj.observer_hard_support_internal;
            end

            if isfinite(I.last_event_time)
                if t==I.last_event_time
                    return;
                end
                assert(t>I.last_event_time, ...
                    '[REPLANNER OBSERVER ERROR] control-event time moved backwards.');
            end

            if ~isempty(I.last_phase)&&~strcmp(I.last_phase,phase)
                I.phase_transition_count=I.phase_transition_count+1;
            end

            C=obj.observer_build_measurement_event(t,phase,I.selector_verified);
            if ~C.closed
                I.fail_count=I.fail_count+1;
                obj.observer_hard_support_internal=I;
                error('[REPLANNER OBSERVER ERROR] measurement hard box failed: %s',C.reason);
            end

            xhat=obj.self.estimator.result.state;
            hasPrev=isstruct(I.previous_measurement)&& ...
                isfield(I.previous_measurement,'closed')&&logical(I.previous_measurement.closed);

            if hasPrev
                h=t-double(I.previous_measurement.event_time);
                assert(isfinite(h)&&h>0, ...
                    '[REPLANNER OBSERVER ERROR] consecutive event interval is not positive.');
                uPrev=obj.observer_get_previous_held_input();
                F=obj.observer_one_step_support(h,uPrev,I);
                assert(F.closed, ...
                    '[REPLANNER OBSERVER ERROR] continuous hidden-state support left certified chart.');
                K=obj.observer_contract_two_events(I.previous_measurement,C,F,xhat);

                % Mutate one local hidden struct and assign it once. This is
                % algebraically identical to repeated nested field writes but
                % avoids repeated copy-on-write bookkeeping on the hot path.
                H=I.hidden;
                Kw=K.w; KvL=K.vL; KwL=K.wL;
                H.w_center=Kw.center;
                H.w_radius=Kw.radius;
                H.w_norm_ub=Kw.true_norm_ub;
                H.vL_center=KvL.center;
                H.vL_radius=KvL.radius;
                H.vL_norm_ub=KvL.true_norm_ub;
                H.wL_center=KwL.center;
                H.wL_radius=KwL.radius;
                H.wL_norm_ub=KwL.true_norm_ub;
                I.hidden=H;
                measured=K.measured_error;
                ew=Kw.observer_error_norm_ub;
                evL=KvL.observer_error_norm_ub;
                ewL=KwL.observer_error_norm_ub;
                contractionUsed=true;
                I.last_forward_support=F;
            else
                measured=obj.observer_measured_error_box(xhat,C);
                xw=double(xhat.w(:)); xvL=double(xhat.vL(:)); xwL=double(xhat.wL(:));
                ew=norm(xw-I.hidden.w_center)+I.hidden.w_radius;
                evL=norm(xvL-I.hidden.vL_center)+I.hidden.vL_radius;
                ewL=norm(xwL-I.hidden.wL_center)+I.hidden.wL_radius;
                contractionUsed=false;
                I.last_forward_support=struct();
            end

            mi=I.true_mL_interval;
            hm=double(xhat.mL);
            emL=max(abs(hm-mi(1)),abs(hm-mi(2)));

            B=struct( ...
                'schema','REPLANNING_SYSTEM_OBSERVER_HARD_SUPPORT_EVENT_V2', ...
                'closed',true, ...
                'event_time',t, ...
                'phase',phase, ...
                'protection_eligible',strcmp(phase,'f'), ...
                'measured',measured, ...
                'w_error_norm_ub',ew, ...
                'vL_error_norm_ub',evL, ...
                'wL_error_norm_ub',ewL, ...
                'mL_error_abs_ub',emL, ...
                'true_mL_interval',mi, ...
                'hidden_true_support',I.hidden, ...
                'consecutive_measurement_contraction_used',contractionUsed, ...
                'numeric_plant_to_continuous_authority',false, ...
                'realized_error_compared',false, ...
                'Q_promoted',false, ...
                'R_promoted',false, ...
                'covariance_promoted',false, ...
                'sampled_acceptance',false);

            I.q_abs_ub_current=abs(C.y(4:6))+C.nu_abs_ub(4:6);
            I.previous_measurement=C;
            I.observer_support=B;
            I.hlc_state_mass_bridge=obj.observer_build_state_mass_bridge(B);
            I.last_event_time=t;
            I.last_phase=phase;
            I.update_count=I.update_count+1;
            obj.observer_hard_support_internal=I;
        end

        function C = observer_build_measurement_event(obj,t,phase,selectorVerified)
            C=struct('schema','REPLANNING_SYSTEM_OBSERVER_MEASUREMENT_EVENT_V2', ...
                'closed',false,'event_time',t,'phase',phase,'y',[], ...
                'nu_abs_ub',[],'reason','UNINITIALIZED');
            assert(selectorVerified, ...
                '[REPLANNER OBSERVER ERROR] measurement selector not verified.');

            y=double(obj.self.sensor.result.output(:));
            if numel(y)~=12||any(~isfinite(y))
                C.reason='MEASUREMENT_NOT_FINITE_12D';
                return;
            end

            mode=upper(strtrim(char(obj.OBSERVER_HARD_SUPPORT_MODE)));
            if strcmp(mode,'SIM_NUMERIC_TRUTH')
                st=obj.self.plant.state;
                x=[double(st.p(:));double(st.q(:));double(st.pL(:));double(st.pT(:))];
                if numel(x)~=12||any(~isfinite(x))
                    C.reason='SIM_NUMERIC_TRUTH_NOT_FINITE_12D';
                    return;
                end
                ur=eps(1)/2;
                d=double(y-x);
                rho=(ur/(1-ur))*(abs(y)+abs(x));
                raw=abs(d)+rho;
                % Exact vector form of 12 scalar observer_nextfloat_up calls.
                % For finite nonnegative raw values, IEEE-754 next-up is the
                % adjacent uint64 pattern; zero retains production's minimum
                % positive subnormal special case.
                assert(all(isfinite(raw))&&all(raw>=0), ...
                    '[REPLANNER OBSERVER ERROR] SIM hard-box raw radius invalid.');
                nu=zeros(12,1);
                iz=(raw==0);
                nu(iz)=realmin*eps(1);
                ip=~iz;
                if any(ip)
                    uu=typecast(double(raw(ip)),'uint64');
                    uu=uu+uint64(1);
                    vv=typecast(uu,'double');
                    nu(ip)=vv(:);
                end
                C.y=y; C.nu_abs_ub=nu; C.raw_difference=d;
                C.relative_to='CURRENT_NUMERICAL_PLANT_STATE';
                C.numeric_plant_to_continuous_authority=false;
                C.closed=all(isfinite(nu))&&all(nu>=0);
                C.reason='SIM_SAME_EVENT_HARD_BOX_CLOSED';
            elseif strcmp(mode,'REAL_HARD_BOUNDS')
                nu=double(obj.OBSERVER_REAL_NU_ABS_UB(:));
                tag=strtrim(char(obj.OBSERVER_REAL_SOURCE_TAG));
                if numel(nu)~=12||any(~isfinite(nu))||any(nu<0)||isempty(tag)
                    C.reason='REAL_HARD_BOUND_AUTHORITY_INVALID';
                    return;
                end
                C.y=y; C.nu_abs_ub=nu; C.source_tag=tag;
                C.relative_to='PHYSICAL_TRUE_MEASURED_COORDINATES';
                C.numeric_plant_to_continuous_authority=false;
                C.closed=true;
                C.reason='REAL_HARD_BOX_CLOSED';
            else
                C.reason='UNSUPPORTED_OBSERVER_MODE';
            end
            C.Q_promoted=false; C.R_promoted=false;
            C.covariance_promoted=false; C.logged_max_promoted=false;
            C.sampled_acceptance=false; C.realized_error_compared=false;
        end

        function M = observer_measured_error_box(~,xhat,C)
            y=double(C.y(:)); nu=double(C.nu_abs_ub(:));
            xp=double(xhat.p(:)); xq=double(xhat.q(:));
            xpL=double(xhat.pL(:)); xpT=double(xhat.pT(:));
            dp=xp-y(1:3); dq=xq-y(4:6); dpL=xpL-y(7:9); dpT=xpT-y(10:12);
            M=struct();
            M.p=struct('center',dp,'radius',nu(1:3),'abs_ub',abs(dp)+nu(1:3));
            M.q=struct('center',dq,'radius',nu(4:6),'abs_ub',abs(dq)+nu(4:6));
            M.pL=struct('center',dpL,'radius',nu(7:9),'abs_ub',abs(dpL)+nu(7:9));
            M.pT=struct('center',dpT,'radius',nu(10:12),'abs_ub',abs(dpT)+nu(10:12));
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
                tanTheta=tan(Theta);
                cosTheta=cos(Theta);
                qAbs(1)=q0(1)+(1+tanTheta)*Iw;
                qAbs(3)=q0(3)+(1/cosTheta)*Iw;
                fullClosed=qAbs(1)<pi&&qAbs(3)<pi;
            end

            aWL=Pnorm*u1abs/(m*L);
            WLEnd=WL0+aWL*h;
            IWL=WL0*h+0.5*aWL*h^2;
            pTSpan=Pnorm*IWL; %#ok<NASGU> exact production intermediate retained
            den=m+mLlb;
            assert(den>0,'[REPLANNER OBSERVER ERROR] total mass lower bound invalid.');
            Aconst=g+(Pnorm^2*u1abs)/den;
            Aquad=(Pnorm^3)*m*L/den;
            intWL2=WL0^2*h+WL0*aWL*h^2+(aWL^2*h^3)/3;
            VLEnd=VL0+Aconst*h+Aquad*intWL2;
            ALmax=Aconst+Aquad*(WLEnd^2);

            S=struct( ...
                'closed',pitchClosed&&fullClosed, ...
                'h',h, ...
                'held_input',u, ...
                'Jdiag',Jd, ...
                'w_norm_ub_end',WabsEnd, ...
                'q_abs_ub_interval',qAbs, ...
                'wL_accel_norm_ub',aWL, ...
                'wL_norm_ub_end',WLEnd, ...
                'vL_accel_norm_ub',ALmax, ...
                'vL_norm_ub_end',VLEnd, ...
                'pT_norm_ub',Pnorm, ...
                'sampled_acceptance',false, ...
                'guessed_bound',false);
        end

        function O = observer_contract_two_events(obj,C0,C1,F,xhat)
            y0=double(C0.y(:)); y1=double(C1.y(:));
            n0=double(C0.nu_abs_ub(:)); n1=double(C1.nu_abs_ub(:));
            h=double(C1.event_time-C0.event_time);
            assert(h>0&&isfinite(h),'[REPLANNER OBSERVER ERROR] contraction h invalid.');
            iq=4:6; ipL=7:9; ipT=10:12;
            MEAS=obj.observer_measured_error_box(xhat,C1);
            xvL=double(xhat.vL(:)); xwL=double(xhat.wL(:)); xw=double(xhat.w(:));

            VLF=double(F.vL_norm_ub_end);
            ALmax=double(F.vL_accel_norm_ub);
            dPL=(y1(ipL)-y0(ipL))/h;
            rVL=0.5*ALmax*h+(norm(n0(ipL))+norm(n1(ipL)))/h;
            cVL=dPL;
            VLtrue=min(VLF,norm(cVL)+rVL);
            eVL=min(norm(xvL)+VLF,norm(xvL-cVL)+rVL);

            WLF=double(F.wL_norm_ub_end); Pbar=double(F.pT_norm_ub);
            aWL=double(F.wL_accel_norm_ub);
            pTddot=Pbar*(aWL+WLF^2);
            dPT=(y1(ipT)-y0(ipT))/h;
            rPTdot=0.5*pTddot*h+(norm(n0(ipT))+norm(n1(ipT)))/h;
            cWL=cross(y1(ipT),dPT);
            pTdotUb=Pbar*WLF;
            rWL=norm(n1(ipT))*pTdotUb+norm(y1(ipT))*rPTdot;
            WLtrue=min(WLF,norm(cWL)+rWL);
            eWL=min(norm(xwL)+WLF,norm(xwL-cWL)+rWL);

            Wf=double(F.w_norm_ub_end); Jd=double(F.Jdiag(:));
            Jmin=min(Jd); Jmax=max(Jd); tauNorm=norm(double(F.held_input(2:4)));
            Theta=double(F.q_abs_ub_interval(2));
            assert(isfinite(Theta)&&Theta<pi/2, ...
                '[REPLANNER OBSERVER ERROR] Euler chart is not closed.');
            tanT=tan(Theta); cosT=cos(Theta); secT=1/cosT; Kphi=1+tanT;
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
            eW=min(norm(xw)+Wf,norm(xw-cW)+rW);

            O=struct(); O.closed=true; O.h=h; O.measured_error=MEAS;
            O.vL=struct('center',cVL,'radius',rVL,'true_norm_ub',VLtrue, ...
                'observer_error_norm_ub',eVL);
            O.wL=struct('center',cWL,'radius',rWL,'true_norm_ub',WLtrue, ...
                'observer_error_norm_ub',eWL);
            O.w=struct('center',cW,'radius',rW,'true_norm_ub',Wtrue, ...
                'observer_error_norm_ub',eW);
        end

        function E = observer_euler_rate_inverse_ZYX(~,q)
            phi=q(1); theta=q(2);
            st=sin(theta); ct=cos(theta); sp=sin(phi); cp=cos(phi);
            E=[1,0,-st;0,cp,sp*ct;0,-sp,cp*ct];
        end

        function [pL_ref,q_ref,source] = repaired_reference_packet(obj,t_now)
            % AF7R41 exact rolling REPAIRED future-reference packet.
            % Full rebuild uses the AF7R33 batch flat model.  On the normal
            % one-dt live path, N-1 exact columns are reused and only the new
            % tail point is evaluated.  No interpolation is used.
            obj.assert_repaired_reference_current();

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

            C=obj.repaired_reference_cache;
            canReuse=obj.repaired_reference_cache_valid && isstruct(C) && ...
                isfield(C,'reference_version') && ...
                double(C.reference_version)==double(obj.active_reference_version) && ...
                isfield(C,'dt') && isequal(double(C.dt),dt) && ...
                isfield(C,'H') && isequal(double(C.H),H) && ...
                isfield(C,'t0') && isfinite(double(C.t0)) && ...
                isfield(C,'pL') && isequal(size(C.pL),[3 N]) && ...
                isfield(C,'q') && isequal(size(C.q),[3 N]);

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

            if canReuse && isfinite(shiftCount) && shiftCount==0
                pL_ref=C.pL;
                q_ref=C.q;
                source=sprintf('REPAIRED_ROLLING_CACHE_HIT:v%d:%s', ...
                    obj.active_reference_version,char(R.backend));
                obj.validate_reference_packet(pL_ref,q_ref,N);
                return;
            end

            if canReuse && isfinite(shiftCount) && shiftCount>=1 && shiftCount<N
                m=double(shiftCount);
                pL_ref=[C.pL(:,m+1:end),zeros(3,m)];
                q_ref=[C.q(:,m+1:end),zeros(3,m)];

                if m==1
                    [ptail,atail,validTail]= ...
                        af7r59_packet_pa_scalar(obj,t0+(N-1)*dt,R.batch_flat_model);
                    assert(logical(validTail) && isequal(size(ptail),[3 1]) && ...
                        isequal(size(atail),[3 1]) && all(isfinite([ptail;atail])), ...
                        '[REPLANNER ERROR] repaired rolling PA tail evaluator invalid.');
                    pL_ref(:,end)=ptail;
                    sf=atail+[0;0;double(obj.gravity)];
                    ns=norm(sf);
                    assert(isfinite(ns)&&ns>0, ...
                        '[REPLANNER ERROR] repaired rolling tail flatness singular.');
                    q_ref(:,end)=-sf/ns;
                    source=sprintf('REPAIRED_ROLLING_SHIFT1:v%d:%s', ...
                        obj.active_reference_version,char(R.backend));
                else
                    tq=t0+(N-m:N-1)*dt;
                    [Ptail,Atail,valid]= ...
                        af7r59_packet_pa_batch(obj,tq,R.batch_flat_model);
                    Ptail=double(Ptail); Atail=double(Atail);
                    assert(isequal(size(Ptail),[3 m]) && isequal(size(Atail),[3 m]) && ...
                        numel(valid)==m && all(logical(valid(:))) && ...
                        all(isfinite(Ptail),'all') && all(isfinite(Atail),'all'), ...
                        '[REPLANNER ERROR] repaired rolling PA tail batch invalid.');
                    pL_ref(:,end-m+1:end)=Ptail;
                    sf=Atail+[0;0;double(obj.gravity)];
                    ns=sqrt(sum(sf.^2,1));
                    assert(all(isfinite(ns))&&all(ns>0), ...
                        '[REPLANNER ERROR] repaired rolling tail batch flatness singular.');
                    q_ref(:,end-m+1:end)=-sf./ns;
                    source=sprintf('REPAIRED_ROLLING_SHIFTM:v%d:%s', ...
                        obj.active_reference_version,char(R.backend));
                end
            else
                tq=t0+(0:N-1)*dt;
                [pL_ref,aL,valid]= ...
                    af7r59_packet_pa_batch(obj,tq,R.batch_flat_model);
                pL_ref=double(pL_ref); aL=double(aL);
                assert(isequal(size(pL_ref),[3 N]) && isequal(size(aL),[3 N]) && ...
                    numel(valid)==N && all(logical(valid(:))) && ...
                    all(isfinite(pL_ref),'all') && all(isfinite(aL),'all'), ...
                    '[REPLANNER ERROR] repaired full PA batch packet invalid.');
                sf=aL+[0;0;double(obj.gravity)];
                ns=sqrt(sum(sf.^2,1));
                assert(all(isfinite(ns))&&all(ns>0), ...
                    '[REPLANNER ERROR] repaired full batch flatness singular.');
                q_ref=-sf./ns;
                source=sprintf('REPAIRED_FULL_BATCH:v%d:%s', ...
                    obj.active_reference_version,char(R.backend));
            end

            obj.validate_reference_packet(pL_ref,q_ref,N);
            obj.repaired_reference_cache=struct( ...
                'reference_version',double(obj.active_reference_version), ...
                't0',t0,'dt',dt,'H',H,'pL',pL_ref,'q',q_ref);
            obj.repaired_reference_cache_valid=true;
        end

        function [ok,reason,B] = prepare_staged_reference_only_bundle(obj,P,t_commit)
            ok=false;
            reason='';
            B=struct();
            r5row=obj.af7r58zzy_r5_current_row;
            if r5row>0, r5x=tic; end

            if ~obj.ENABLE_AUTO_REPLAN_COMMIT
                reason='AUTO_COMMIT_DISABLED'; return;
            end
            if ~strcmpi(char(obj.AVOIDANCE_COMMIT_POLICY),'REFERENCE_ONLY')
                reason='NOT_REFERENCE_ONLY_COMMIT_POLICY'; return;
            end
            if ~(isstruct(P)&&isfield(P,'success')&&logical(P.success))
                reason='PROPOSAL_NOT_SUCCESSFUL'; return;
            end

            [spacingOK,~,~,spacingReason]= ...
                af7r58i_commit_spacing_guard(obj,double(t_commit));
            if ~spacingOK
                reason=spacingReason; return;
            end

            G=P.geometry_shadow_certificate;
            SC=P.reference_speed_certificate;
            if ~(isstruct(G)&&isfield(G,'certified')&&logical(G.certified) && ...
                    isfield(G,'continuous_time')&&logical(G.continuous_time) && ...
                    isfield(G,'sampled_acceptance')&&~logical(G.sampled_acceptance))
                reason='REFERENCE_GEOMETRY_CERTIFICATE_NOT_CLOSED'; return;
            end
            if ~(isstruct(SC)&&isfield(SC,'certified')&&logical(SC.certified))
                reason='REFERENCE_SPEED_CERTIFICATE_NOT_CLOSED'; return;
            end

            targetVersion=double(obj.active_reference_version)+1;
            if double(SC.reference_version)~=targetVersion
                reason='REFERENCE_SPEED_CERTIFICATE_VERSION_MISMATCH'; return;
            end
            if ~(isfield(SC,'continuous_time')&&logical(SC.continuous_time) && ...
                    isfield(SC,'sampled_acceptance')&&~logical(SC.sampled_acceptance) && ...
                    isfield(SC,'specific_force_norm_LB_m_s2') && ...
                    isnumeric(SC.specific_force_norm_LB_m_s2) && ...
                    isscalar(SC.specific_force_norm_LB_m_s2) && ...
                    isfinite(double(SC.specific_force_norm_LB_m_s2)) && ...
                    double(SC.specific_force_norm_LB_m_s2)>0)
                reason='REFERENCE_ONLY_CONTINUOUS_FLATNESS_CERTIFICATE_NOT_CLOSED'; return;
            end
            if ~(isfield(P,'c7_exact')&&logical(P.c7_exact))
                reason='REFERENCE_ONLY_C7_CERTIFICATE_NOT_CLOSED'; return;
            end
            if r5row>0, obj.af7r58zzy_r5_ring(r5row,27)=1e3*toc(r5x); end

            if r5row>0, r5x=tic; end
            [resourceOK,resourceReason,batchModel,resourceMeta]= ...
                af7r58i_prepare_batch_model( ...
                    obj,P,targetVersion,double(t_commit));
            if r5row>0, obj.af7r58zzy_r5_ring(r5row,28)=1e3*toc(r5x); end
            if ~resourceOK
                reason=resourceReason; return;
            end

            % Exact rebuild is intentional in the staged-preparation cycle.
            % No cache/reuse theorem is needed for timing here.
            if r5row>0, r5x=tic; end
            sweepNew=obj.build_trigger_sweep_for_target(SC,targetVersion);
            if r5row>0, obj.af7r58zzy_r5_ring(r5row,29)=1e3*toc(r5x); end
            if r5row>0, r5x=tic; end
            packetNew=obj.prepare_obstacle_packet_fast_target( ...
                obj.obstacle_packet,sweepNew,targetVersion);
            if r5row>0, obj.af7r58zzy_r5_ring(r5row,30)=1e3*toc(r5x); end
            if r5row>0, r5x=tic; end
            sensorNew=obj.build_sensor_horizon_for_target(SC,targetVersion);
            if r5row>0, obj.af7r58zzy_r5_ring(r5row,31)=1e3*toc(r5x); end
            if ~(isstruct(sensorNew)&&isfield(sensorNew,'certified')&& ...
                    logical(sensorNew.certified))
                reason='STAGED_TARGET_SENSOR_HORIZON_OPEN'; return;
            end
            if r5row>0, r5x=tic; end
            depNew=obj.runtime_dependency_signature;
            depNew.reference_mode='REPAIRED';
            depNew.reference_version=double(targetVersion);

            [evalFn,flatEvalFn]= ...
                af7r58i_make_canonical_eval_handles(obj,batchModel);

            smoothSource='DEGREE15_ENDPOINT_REPEATED_BEZIER_C7';
            if isfield(P,'representation_code')&&double(P.representation_code)==2
                smoothSource='PIECEWISE_DEGREE15_BEZIER_C7_AFFINE_TOWING_CORRIDORS';
            end
            smooth=struct( ...
                'certified',logical(P.c7_exact), ...
                'continuous_time',true, ...
                'sampled_acceptance',false, ...
                'continuity_order',7, ...
                'reference_mode','REPAIRED', ...
                'reference_version',targetVersion, ...
                'valid_from',double(P.t0), ...
                'valid_until',double(P.t1), ...
                'source',smoothSource);

            geom=G;
            geom.reference_mode='REPAIRED';
            geom.reference_version=targetVersion;
            geom.valid_from=double(P.t0);
            geom.valid_until=double(P.t1);
            geom.reference_only_authority=true;
            geom.final_system_authority=false;

            R=struct();
            R.valid=true;
            R.backend='BEZIER_D15_C7';
            R.eval_fn=evalFn;
            R.flat_eval_fn=flatEvalFn;
            R.valid_from=double(P.t0);
            R.valid_until=Inf;
            R.reference_version=targetVersion;
            R.source='AF7R58ZZU_STAGED_REFERENCE_ONLY_COMMIT';
            R.yaw_policy='C6_INDEPENDENT';
            R.path_parameterization=struct( ...
                'kind','DIRECT_TIME_BEZIER_CORRECTION', ...
                'path_derivative_order',7, ...
                'time_derivative_order',7);
            R.rejoin_time=double(P.t1);
            R.representation=struct( ...
                'kind','BEZIER_D15_C7_TRANSLATION_BUMP', ...
                't0',double(P.t0),'tm',double(P.tm),'t1',double(P.t1), ...
                'displacement',double(P.displacement(:)));
            R.batch_flat_model=batchModel;
            R.resource_certificate=resourceMeta.contract;
            R.resource_state=resourceMeta;
            R.authority_split=struct( ...
                'controller_packet','eval_fn', ...
                'mathematical_flat_jet','flat_eval_fn', ...
                'legacy_high_derivative_slots_used_for_geometry',false);
            if r5row>0, obj.af7r58zzy_r5_ring(r5row,32)=1e3*toc(r5x); end

            % Pre-build the first packet that will be requested AFTER the
            % t_commit authority swap.  The commit cycle itself has already
            % evaluated its pre-commit packet, so the first repaired packet is
            % at t_commit+dt.  Moving this full N-point batch into Stage 2
            % removes the post-commit cold rebuild from the next Flight cycle.
            dt=double(obj.prediction_dt);
            H=double(obj.horizon_s);
            N=round(H/dt)+1;
            tCache=double(t_commit)+dt;
            tqCache=tCache+(0:N-1)*dt;
            if r5row>0, r5x=tic; end
            [pLcache,aLcache,validCache]= ...
                af7r59_packet_pa_batch(obj,tqCache,batchModel);
            if r5row>0, obj.af7r58zzy_r5_ring(r5row,33)=1e3*toc(r5x); end
            if r5row>0, r5x=tic; end
            pLcache=double(pLcache);
            aLcache=double(aLcache);
            if ~(isequal(size(pLcache),[3 N]) && isequal(size(aLcache),[3 N]) && ...
                    numel(validCache)==N && all(logical(validCache(:))) && ...
                    all(isfinite(pLcache),'all') && all(isfinite(aLcache),'all'))
                reason='STAGED_POSTCOMMIT_CACHE_PA_BATCH_INVALID'; return;
            end
            sfcache=aLcache+[0;0;double(obj.gravity)];
            nscache=sqrt(sum(sfcache.^2,1));
            if ~(all(isfinite(nscache))&&all(nscache>0))
                reason='STAGED_POSTCOMMIT_CACHE_FLATNESS_SINGULAR'; return;
            end
            qcache=-sfcache./nscache;
            postCache=struct( ...
                'reference_version',double(targetVersion), ...
                't0',double(tCache),'dt',dt,'H',H, ...
                'pL',pLcache,'q',qcache);
            if r5row>0, obj.af7r58zzy_r5_ring(r5row,34)=1e3*toc(r5x); end

            if r5row>0, r5x=tic; end
            B=struct( ...
                'schema','AF7R58ZZV_PREPARED_COMMIT_V2', ...
                'source_mode',char(obj.active_reference_mode), ...
                'source_version',double(obj.active_reference_version), ...
                'target_version',double(targetVersion), ...
                'activation_time',double(t_commit), ...
                'dependency_key',obj.staged_reference_only_dependency_key(), ...
                'speed_source',obj.active_speed_certificate, ...
                'known_mask',logical(obj.staged_replan_known_mask(:).'), ...
                'static_map',obj.static_obstacle_map, ...
                'reference',R, ...
                'certificates',struct( ...
                    'reference_only',true,'smoothness',smooth,'geometry',geom), ...
                'speed',SC, ...
                'sweep',sweepNew, ...
                'packet',packetNew, ...
                'sensor',sensorNew, ...
                'dep',depNew, ...
                'post_commit_reference_cache',postCache, ...
                'prevalidated',true);
            obj.staged_replan_postcommit_cache_prepare_count= ...
                obj.staged_replan_postcommit_cache_prepare_count+1;
            ok=true;
            reason='STAGED_COMMIT_BUNDLE_AND_POSTCACHE_PA_PREPARED';
            if r5row>0, obj.af7r58zzy_r5_ring(r5row,35)=1e3*toc(r5x); end
        end

        function E = af7r59_postrun_equivalence(obj)
            % POST-RUN ONLY. Never called from do(), Arming, Takeoff, Flight or Landing.
            E=struct('closed',false,'packet_bitwise',false,'packet_max_abs',Inf, ...
                'nextup_bitwise',false,'observer_one_step_equal',false, ...
                'observer_contraction_equal',false,'observer_measured_equal',false, ...
                'tested_points',0,'reason','UNINITIALIZED');

            R=obj.active_repaired_reference;
            if ~(isstruct(R)&&isfield(R,'batch_flat_model')&&isstruct(R.batch_flat_model))
                E.reason='NO_ACTIVE_BATCH_MODEL'; return;
            end
            model=R.batch_flat_model;
            dt=double(obj.prediction_dt); H=double(obj.horizon_s);
            tbase=double(obj.last_replan_commit_time)+dt;
            tq=tbase+(0:round(H/dt))*dt;
            % Include all exact correction and segment boundaries plus neighboring grid points.
            extra=[];
            if isfield(model,'corrections')&&isstruct(model.corrections)
                for g=1:numel(model.corrections)
                    C=model.corrections(g);
                    extra=[extra,double(C.t0),double(C.t1), ... %#ok<AGROW>
                        double(C.segment_t0(:).'),double(C.segment_t1(:).')];
                end
            end
            tq=unique([tq,extra]);
            tq=tq(isfinite(tq));
            [P,A,V]=af7r59_packet_pa_batch(obj,tq,model);
            [Y,V0]=af7r33_flat_jet_batch_model(obj,tq,model);
            Y=double(Y); P=double(P); A=double(A);
            YP=Y(1:3,:); YA=Y(7:9,:);
            packetBit=isequal(logical(V(:)),logical(V0(:))) && ...
                isequal(typecast(P(:),'uint64'),typecast(YP(:),'uint64')) && ...
                isequal(typecast(A(:),'uint64'),typecast(YA(:),'uint64'));
            maxAbs=max([0;abs(P(:)-YP(:));abs(A(:)-YA(:))]);

            vals=[0;realmin*eps(1);realmin;eps(1);1;pi;1e-12;1e12];
            nv=zeros(size(vals)); ns=zeros(size(vals));
            iz=(vals==0); nv(iz)=realmin*eps(1);
            ip=~iz;
            uu=typecast(double(vals(ip)),'uint64'); uu=uu+uint64(1);
            vv=typecast(uu,'double'); nv(ip)=vv(:);
            for k=1:numel(vals), ns(k)=obj.observer_nextfloat_up(vals(k)); end
            nextBit=isequal(typecast(nv(:),'uint64'),typecast(ns(:),'uint64'));

            I=obj.observer_hard_support_internal;
            obsOne=false; obsContract=false; obsMeasured=false;
            if isstruct(I)&&isfield(I,'initialized')&&logical(I.initialized)&& ...
                    isfield(I,'previous_measurement')&&isstruct(I.previous_measurement)&& ...
                    isfield(I.previous_measurement,'closed')&&logical(I.previous_measurement.closed)
                h=double(obj.prediction_dt);
                u=zeros(4,1);
                Snew=obj.observer_one_step_support(h,u,I);
                Sold=obj.af7r59_legacy_one_step_support(h,u,I);
                obsOne=isequaln(Snew,Sold);
                C1=I.previous_measurement; C0=C1;
                C0.event_time=double(C1.event_time)-h;
                xhat=obj.self.estimator.result.state;
                Mnew=obj.observer_measured_error_box(xhat,C1);
                Mold=obj.af7r59_legacy_measured_error_box(xhat,C1);
                obsMeasured=isequaln(Mnew,Mold);
                Onew=obj.observer_contract_two_events(C0,C1,Snew,xhat);
                Oold=obj.af7r59_legacy_contract_two_events(C0,C1,Sold,xhat);
                obsContract=isequaln(Onew,Oold);
            end

            E.packet_bitwise=packetBit;
            E.packet_max_abs=double(maxAbs);
            E.nextup_bitwise=nextBit;
            E.observer_one_step_equal=obsOne;
            E.observer_contraction_equal=obsContract;
            E.observer_measured_equal=obsMeasured;
            E.tested_points=numel(tq);
            E.closed=packetBit && nextBit && maxAbs==0 && ...
                obsOne && obsContract && obsMeasured;
            if E.closed, E.reason='BITWISE_EQUIVALENCE_CLOSED';
            else, E.reason='BITWISE_EQUIVALENCE_OPEN'; end
        end

        function S = af7r59_legacy_one_step_support(obj,h,u,I)
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
            S=struct();
            S.closed=pitchClosed&&fullClosed; S.h=h; S.held_input=u;
            S.Jdiag=Jd; S.w_norm_ub_end=WabsEnd;
            S.q_abs_ub_interval=qAbs;
            S.wL_accel_norm_ub=aWL; S.wL_norm_ub_end=WLEnd;
            S.vL_accel_norm_ub=ALmax; S.vL_norm_ub_end=VLEnd; S.pT_norm_ub=Pnorm;
            S.sampled_acceptance=false; S.guessed_bound=false;
        end

        function M = af7r59_legacy_measured_error_box(~,xhat,C)
            y=double(C.y(:)); nu=double(C.nu_abs_ub(:)); M=struct();
            M.p=struct('center',double(xhat.p(:))-y(1:3),'radius',nu(1:3),'abs_ub',abs(double(xhat.p(:))-y(1:3))+nu(1:3));
            M.q=struct('center',double(xhat.q(:))-y(4:6),'radius',nu(4:6),'abs_ub',abs(double(xhat.q(:))-y(4:6))+nu(4:6));
            M.pL=struct('center',double(xhat.pL(:))-y(7:9),'radius',nu(7:9),'abs_ub',abs(double(xhat.pL(:))-y(7:9))+nu(7:9));
            M.pT=struct('center',double(xhat.pT(:))-y(10:12),'radius',nu(10:12),'abs_ub',abs(double(xhat.pT(:))-y(10:12))+nu(10:12));
        end

        function O = af7r59_legacy_contract_two_events(obj,C0,C1,F,xhat)
            y0=double(C0.y(:)); y1=double(C1.y(:));
            n0=double(C0.nu_abs_ub(:)); n1=double(C1.nu_abs_ub(:));
            h=double(C1.event_time-C0.event_time);
            assert(h>0&&isfinite(h),'[REPLANNER OBSERVER ERROR] contraction h invalid.');
            iq=4:6; ipL=7:9; ipT=10:12;
            MEAS=obj.af7r59_legacy_measured_error_box(xhat,C1);
            VLF=double(F.vL_norm_ub_end);
            ALmax=double(F.vL_accel_norm_ub);
            dPL=(y1(ipL)-y0(ipL))/h;
            rVL=0.5*ALmax*h+(norm(n0(ipL))+norm(n1(ipL)))/h;
            cVL=dPL;
            VLtrue=min(VLF,norm(cVL)+rVL);
            eVL=min(norm(double(xhat.vL(:)))+VLF, ...
                norm(double(xhat.vL(:))-cVL)+rVL);
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
            Ey=obj.af7r59_legacy_euler_rate_inverse_ZYX(y1(iq));
            cW=Ey*dq;
            dphi=n1(4); dtheta=n1(5);
            LE=sqrt(dtheta^2+2*dphi^2+2*(dphi+dtheta)^2);
            Kq=sqrt(Kphi^2+1+secT^2); qdotUb=Kq*Wf;
            EyNorm2=obj.observer_nextfloat_up( ...
                sqrt(1+abs(sin(double(y1(iq(2)))))));
            rW=LE*qdotUb+EyNorm2*rQdot;
            Wtrue=min(Wf,norm(cW)+rW);
            eW=min(norm(double(xhat.w(:)))+Wf,norm(double(xhat.w(:))-cW)+rW);
            O=struct(); O.closed=true; O.h=h; O.measured_error=MEAS;
            O.vL=struct('center',cVL,'radius',rVL,'true_norm_ub',VLtrue, ...
                'observer_error_norm_ub',eVL);
            O.wL=struct('center',cWL,'radius',rWL,'true_norm_ub',WLtrue, ...
                'observer_error_norm_ub',eWL);
            O.w=struct('center',cW,'radius',rW,'true_norm_ub',Wtrue, ...
                'observer_error_norm_ub',eW);
        end

        function E = af7r59_legacy_euler_rate_inverse_ZYX(~,q)
            phi=q(1); theta=q(2);
            E=[1,0,-sin(theta);0,cos(phi),sin(phi)*cos(theta); ...
                0,-sin(phi),cos(phi)*cos(theta)];
        end

    end
end
