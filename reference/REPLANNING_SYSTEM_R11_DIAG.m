classdef REPLANNING_SYSTEM_R11_DIAG < REPLANNING_SYSTEM_R10_DIAG
% AF7R58ZZY-R11 | PRE-MISSION SAME-CLASS FAST-HELPER SPECIALIZATION SHADOW
%
% Performance-only diagnostic subclass.
% - Inherits R10 direct Stage2 R10FAST connection unchanged.
% - Overrides repaired_reference_packet only so the EXISTING production
%   post-callback pre-motion full-packet prime uses the same bitwise-exact
%   R10FAST helper and the same R11 object class that real Flight will use.
% - No added prime loop/call is introduced. Arming/Takeoff/Landing unchanged.
% - Shift1 scalar tail path is unchanged. ShiftM/full batch use the same exact
%   helper solely to preserve the helper connection in repaired packet rebuilds.

    properties (Access = public)
        af7r64_packet_fast_full_calls = 0;
        af7r64_packet_fast_premission_full_calls = 0;
        af7r64_packet_fast_real_full_calls = 0;
    end

    methods
        function obj = REPLANNING_SYSTEM_R11_DIAG(self,base_ref,opts)
            obj@REPLANNING_SYSTEM_R10_DIAG(self,base_ref,opts);
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
                    [Y,valid]=af7r33_flat_jet_batch_model_R10FAST(obj,tq,R.batch_flat_model);
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
                obj.af7r64_packet_fast_full_calls=obj.af7r64_packet_fast_full_calls+1;
                if ~obj.online_prime_done
                    obj.af7r64_packet_fast_premission_full_calls= ...
                        obj.af7r64_packet_fast_premission_full_calls+1;
                else
                    obj.af7r64_packet_fast_real_full_calls= ...
                        obj.af7r64_packet_fast_real_full_calls+1;
                end
                [Y,valid]=af7r33_flat_jet_batch_model_R10FAST(obj,tq,R.batch_flat_model);
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

    end
end
