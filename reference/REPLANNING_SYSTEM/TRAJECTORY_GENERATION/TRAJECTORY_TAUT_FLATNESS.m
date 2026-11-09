classdef TRAJECTORY_TAUT_FLATNESS
    % TRAJECTORY_TAUT_FLATNESS  Load-flat-output -> exact taut-load kinematics.
    % In the IDEAL massless straight inextensible TAUT cable model, let q
    % point from the quadrotor to the load and g_world be the signed 3-D
    % gravity acceleration. Newton for the load is
    %   mL*aL = mL*g_world - tension*q.
    % Thus w=g_world-aL, q=w/||w||, tension=mL*||w||,
    %   pU=pL-cable_length*q.
    % No independently invented payload polynomial or constant swing angle.
    % The load flat output through d9 yields UAV position through d7,
    % provided w != 0 throughout. It does NOT infer HLC tracking, torques,
    % true load dynamics under slack, or continuous-time collision safety.
    % No fixed physical parameters or assumed thrust limits: ALL are external.
    methods (Static)
        function E=evaluate(load_piece,time_s,authority)
            % Standard public pointwise evaluator: actual polynomial only.
            % Never trust a caller-supplied derivative cache for this path.
            L=TRAJECTORY_C9_FLAT_BERNSTEIN.evaluate(load_piece,time_s);
            E=TRAJECTORY_TAUT_FLATNESS.evaluate_jet_matrix(L,authority);
        end
        function E=evaluate_jet_matrix(L,authority)
            % Pure model reconstruction from explicitly supplied payload
            % 0..9 jets. Pointwise kinematics ONLY, never a certificate.
            % Used internally by the C9 join auditor AFTER it has rebuilt
            % and verified all derivative controls from the real polynomial.
            % It avoids reconstructing the same controls again at both
            % endpoints. Other users must not treat arbitrary jet matrices
            % as certified trajectories or authorize HLC COMMIT.
            assert(isnumeric(L)&&isreal(L)&&isequal(size(L),[3 10])&& ...
                all(isfinite(L(:))), ...
                'TRAJECTORY_TAUT_FLATNESS:Jets', ...
                'Require finite 3x10 payload derivatives for a pointwise model.');
            A=TRAJECTORY_TAUT_FLATNESS.authority(authority);
            E=struct('status','UNKNOWN_TAUT_TENSION_SINGULARITY', ...
                'valid',false,'safe_to_commit',false, ...
                'load_jets',L,'uav_jets',NaN(3,8), ...
                'cable_direction_jets',NaN(3,8), ...
                'cable_length_m',A.cable_length_m,'tension_N',NaN, ...
                'thrust_force_world_N',NaN(3,1), 'thrust_N',NaN, ...
                'sampled_thrust_within_limits',false, ...
                'sampled_tension_above_minimum',false, ...
                'scope','IDEAL_TAUT_MODEL_POINTWISE_NOT_HLC_CERTIFICATE');
            W=zeros(3,8);
            W(:,1)=A.gravity_world_mps2-L(:,3);
            for k=1:7
                W(:,k+1)=-L(:,k+3);
            end
            s0=norm(W(:,1));
            if ~(isfinite(s0)&&s0>0)
                return
            end
            % Differentiate w=s*q by the generalized Leibniz rule.
            % Obtain derivatives of s from s^2=w'*w; no finite differences.
            scalar=zeros(1,8);scalar(1)=s0;
            Q=zeros(3,8);Q(:,1)=W(:,1)/s0;
            for n=1:7
                vn=0;
                for k=0:n
                    vn=vn+nchoosek(n,k)*dot(W(:,k+1),W(:,n-k+1));
                end
                prev=0;
                for k=1:n-1
                    prev=prev+nchoosek(n,k)*scalar(k+1)*scalar(n-k+1);
                end
                scalar(n+1)=(vn-prev)/(2*s0);
                cross=zeros(3,1);
                for k=1:n
                    cross=cross+nchoosek(n,k)*scalar(k+1)*Q(:,n-k+1);
                end
                Q(:,n+1)=(W(:,n+1)-cross)/s0;
            end
            U=L(:,1:8)-A.cable_length_m*Q;
            T=A.payload_mass_kg*s0;
            force=A.uav_mass_kg*(U(:,3)-A.gravity_world_mps2)+ ...
                A.payload_mass_kg*(L(:,3)-A.gravity_world_mps2);
            thrust=norm(force);
            if ~all(isfinite([U(:);Q(:);T;force;thrust]))
                E.status='UNKNOWN_NONFINITE_FLATNESS_DERIVATIVE';return
            end
            E.uav_jets=U;E.cable_direction_jets=Q;
            E.tension_N=T;E.thrust_force_world_N=force;E.thrust_N=thrust;
            E.sampled_tension_above_minimum=T>A.minimum_tension_N;
            E.sampled_thrust_within_limits= ...
                thrust>=A.thrust_limits_N(1)&&thrust<=A.thrust_limits_N(2);
            E.valid=true;
            E.status='VALID_IDEAL_TAUT_FLATNESS_POINT__NOT_FULL_TIME_DYNAMICS';
        end
        function P=certify_positive_tension(load_piece,authority,proof_options)
            % Sufficient whole-time positivity proof for the FLAT taut model.
            % On each Bernstein leaf w(t) is in conv(W_i). If a unit n
            % satisfies min_i(n'*W_i)>guard+Tmin/mL, then ||w(t)||
            % exceeds that bound for EVERY real t in that leaf.
            % Normal = mean(W_i)/norm(mean(W_i)); equivariant under SO(3).
            A=TRAJECTORY_TAUT_FLATNESS.authority(authority);
            assert(isstruct(proof_options)&&isscalar(proof_options)&& ...
                isfield(proof_options,'max_subdivision_depth')&& ...
                isnumeric(proof_options.max_subdivision_depth)&& ...
                isreal(proof_options.max_subdivision_depth)&& ...
                isscalar(proof_options.max_subdivision_depth)&& ...
                isfinite(proof_options.max_subdivision_depth)&& ...
                proof_options.max_subdivision_depth>=0&& ...
                proof_options.max_subdivision_depth==fix(proof_options.max_subdivision_depth), ...
                'TRAJECTORY_TAUT_FLATNESS:ProofBudget', ...
                'Supply a finite nonnegative integer subdivision budget.');
            % The all-time tension proof must be about the ACTUAL payload
            % polynomial, not caller-supplied derivative cache values.
            Dactual=TRAJECTORY_C9_FLAT_BERNSTEIN.actual_derivative_controls(load_piece);
            if isfield(load_piece,'derivative_controls')
                supplied=load_piece.derivative_controls;
                assert(iscell(supplied)&&numel(supplied)==10, ...
                    'TRAJECTORY_TAUT_FLATNESS:UnboundJets', ...
                    'Cached derivatives must cover the full C9 polynomial.');
                for order=1:10
                    assert(isnumeric(supplied{order})&& ...
                        isreal(supplied{order})&& ...
                        isequal(double(supplied{order}),Dactual{order}), ...
                        'TRAJECTORY_TAUT_FLATNESS:UnboundJets', ...
                        'Cached derivatives do not match actual polynomial.');
                end
            end
            Acc=Dactual{3};
            W=repmat(A.gravity_world_mps2,1,22)-Acc;
            [ok,margin,leaf_count]=TRAJECTORY_TAUT_FLATNESS.check_w( ...
                W,A.acceleration_guard_mps2+A.minimum_tension_N/A.payload_mass_kg, ...
                proof_options.max_subdivision_depth);
            P=struct('certified',ok,'safe_to_commit',false, ...
                'minimum_model_tension_lower_bound_N',NaN, ...
                'checked_leaf_count',leaf_count, ...
                'status','UNKNOWN_TENSION_POSITIVITY_NOT_PROVEN', ...
                'scope','IDEAL_TAUT_FLATNESS_WHOLE_TIME_ONLY');
            if ok
                % Guard is a supplied numerical margin on acceleration;
                % the positive halfspace is a real-arithmetic certificate.
                P.minimum_model_tension_lower_bound_N= ...
                    A.payload_mass_kg*(margin-A.acceleration_guard_mps2);
                P.status='CERTIFIED_IDEAL_TAUT_TENSION_POSITIVE_ALL_TIME';
            end
        end
        function [ref28,yaw_jets,E]=reference28(load_piece,t,authority,yaw_anchor_rad)
            E=TRAJECTORY_TAUT_FLATNESS.evaluate(load_piece,t,authority);
            assert(E.valid,'TRAJECTORY_TAUT_FLATNESS:Heading', ...
                'Flatness denominator must be nonzero.');
            assert(isnumeric(yaw_anchor_rad)&&isreal(yaw_anchor_rad)&& ...
                isscalar(yaw_anchor_rad)&&isfinite(yaw_anchor_rad), ...
                'TRAJECTORY_TAUT_FLATNESS:Yaw','External branch anchor required.');
            J=E.uav_jets;
            yaw_jets=TRAJECTORY_TAUT_FLATNESS.yaw_from_uav_jets( ...
                J,yaw_anchor_rad);
            ref28=reshape([J(:,1:7);yaw_jets],28,1);
            assert(all(isfinite(ref28)), ...
                'TRAJECTORY_TAUT_FLATNESS:Reference','Nonfinite reference.');
        end
        function yaw_jets=yaw_from_uav_jets(J,yaw_anchor_rad)
            % Exactly the same analytic heading C6 recurrence used by
            % reference28. Accept ONLY already derived, finite 3x8 UAV
            % derivatives. This convenience is NOT an independent proof;
            % the caller must have verified the actual C9 polynomial.
            assert(isnumeric(J)&&isreal(J)&&isequal(size(J),[3 8])&& ...
                all(isfinite(J(:))), ...
                'TRAJECTORY_TAUT_FLATNESS:HeadingJets', ...
                'Require finite model-derived UAV 0..7 jets.');
            assert(isnumeric(yaw_anchor_rad)&&isreal(yaw_anchor_rad)&& ...
                isscalar(yaw_anchor_rad)&&isfinite(yaw_anchor_rad), ...
                'TRAJECTORY_TAUT_FLATNESS:Yaw', ...
                'External branch anchor required.');
            v=complex(J(1,2:8),J(2,2:8));
            assert(all(isfinite(v)), ...
                'TRAJECTORY_TAUT_FLATNESS:Heading', ...
                'Nonfinite planar velocity jets.');
            if v(1)==0
                % At an EXACT stationary jet, holding the externally
                % supplied yaw is an admissible pointwise reference.
                % This is NEVER a whole-time stationary proof: an isolated
                % zero (including a reversal) must be certified separately.
                assert(all(v==0), ...
                    'TRAJECTORY_TAUT_FLATNESS:HeadingZeroNotStationaryJet', ...
                    'Isolated zero speed requires a separate C6 extension proof.');
                yaw_jets=[double(yaw_anchor_rad),zeros(1,6)];
                return
            end
            a=v./factorial(0:6);g=zeros(1,7);
            for n=1:6
                accum=0;
                for k=1:n-1
                    accum=accum+k*g(k+1)*a(n-k+1);
                end
                g(n+1)=(n*a(n+1)-accum)/(n*a(1));
            end
            yaw_jets=imag(g).*factorial(0:6);
            principal=atan2(imag(v(1)),real(v(1)));
            yaw_jets(1)=principal+2*pi*round((yaw_anchor_rad-principal)/(2*pi));
            assert(all(isfinite(yaw_jets)), ...
                'TRAJECTORY_TAUT_FLATNESS:Reference','Nonfinite yaw jet.');
        end
        function R=reference_window(load_piece,relative_times_s,authority, ...
                proof_options,yaw_anchor_rad)
            % Coupled payload -> UAV 28-reference bridge. The cable tension
            % gate covers continuous time; thrust and references are sampled.
            % This is NOT a certified dynamic HLC execution trajectory.
            assert(isnumeric(relative_times_s)&&isreal(relative_times_s)&& ...
                isvector(relative_times_s)&&numel(relative_times_s)>=2&& ...
                all(isfinite(relative_times_s(:))), ...
                'TRAJECTORY_TAUT_FLATNESS:Grid','External finite time grid required.');
            times=reshape(double(relative_times_s),1,[]);
            assert(times(1)==0&&all(diff(times)>0)&& ...
                abs(times(end)-load_piece.duration_s)<= ...
                16*eps(max(1,load_piece.duration_s)), ...
                'TRAJECTORY_TAUT_FLATNESS:Grid', ...
                'Grid must include start and exact end; no extrapolation.');
            times(end)=load_piece.duration_s;
            P=TRAJECTORY_TAUT_FLATNESS.certify_positive_tension( ...
                load_piece,authority,proof_options);
            R=struct('status','UNKNOWN_TENSION_POSITIVITY', ...
                'tension_certificate',P,'sampled_screen_pass',false, ...
                'safe_to_commit',false,'physical_continuous_time_certified',false, ...
                'time_grid_s',load_piece.t0_s+times, ...
                'reference_matrix_28',zeros(28,0), ...
                'predicted_uav_m',zeros(3,0), ...
                'predicted_payload_m',zeros(3,0), ...
                'model_tension_N',zeros(1,0), ...
                'nominal_thrust_N',zeros(1,0), ...
                'scope','IDEAL_TAUT_MODEL_REFERENCES_NOT_HLC_PROOF');
            if ~P.certified,return;end
            n=numel(times)-1;
            M=zeros(28,n);U=zeros(3,n+1);L=U;
            tension=zeros(1,n+1);thrust=zeros(1,n+1);
            yaw=double(yaw_anchor_rad);
            for k=1:n+1
                if k<=n
                    try
                        [M(:,k),yj,E]=TRAJECTORY_TAUT_FLATNESS.reference28( ...
                            load_piece,load_piece.t0_s+times(k),authority,yaw);
                    catch ME
                        R.status=['UNKNOWN_HEADING_OR_REFERENCE_' ME.identifier];
                        return
                    end
                    yaw=yj(1);
                else
                    E=TRAJECTORY_TAUT_FLATNESS.evaluate( ...
                        load_piece,load_piece.t0_s+times(k),authority);
                end
                if ~E.valid
                    R.status='UNKNOWN_NONFINITE_OR_SINGULAR_MODEL';return
                end
                U(:,k)=E.uav_jets(:,1);L(:,k)=E.load_jets(:,1);
                tension(k)=E.tension_N;thrust(k)=E.thrust_N;
                if ~(E.sampled_tension_above_minimum && ...
                        E.sampled_thrust_within_limits)
                    R.status='REJECT_SAMPLED_TAUT_TENSION_OR_THRUST';return
                end
            end
            R.reference_matrix_28=M;R.predicted_uav_m=U;
            R.predicted_payload_m=L;R.model_tension_N=tension;
            R.nominal_thrust_N=thrust;
            R.sampled_screen_pass=true;
            R.status='PASS_COUPLED_FLATNESS_TENSION_AND_SAMPLED_THRUST__COMMIT_FORBIDDEN';
        end
    end
    methods (Static,Access=private)
        function A=authority(A)
            required={'cable_length_m','uav_mass_kg','payload_mass_kg', ...
                'gravity_world_mps2','thrust_limits_N', ...
                'minimum_tension_N','acceleration_guard_mps2'};
            assert(isstruct(A)&&isscalar(A)&&all(isfield(A,required)), ...
                'TRAJECTORY_TAUT_FLATNESS:Authority', ...
                'Live masses, gravity, cable, input bounds and tolerances required.');
            for field={'cable_length_m','uav_mass_kg','payload_mass_kg'}
                v=A.(field{1});
                assert(isnumeric(v)&&isreal(v)&&isscalar(v)&&isfinite(v)&&v>0, ...
                    'TRAJECTORY_TAUT_FLATNESS:Authority','Positive live physical parameter required.');
            end
            for field={'minimum_tension_N','acceleration_guard_mps2'}
                v=A.(field{1});
                assert(isnumeric(v)&&isreal(v)&&isscalar(v)&&isfinite(v)&&v>=0, ...
                    'TRAJECTORY_TAUT_FLATNESS:Authority','Nonnegative external bound required.');
            end
            assert(isnumeric(A.gravity_world_mps2)&&isreal(A.gravity_world_mps2)&& ...
                isequal(size(A.gravity_world_mps2),[3,1])&& ...
                all(isfinite(A.gravity_world_mps2))&& ...
                norm(A.gravity_world_mps2)>0&& ...
                isnumeric(A.thrust_limits_N)&&isreal(A.thrust_limits_N)&& ...
                numel(A.thrust_limits_N)==2&& ...
                all(isfinite(A.thrust_limits_N(:)))&& ...
                A.thrust_limits_N(1)>=0&& ...
                A.thrust_limits_N(2)>=A.thrust_limits_N(1), ...
                'TRAJECTORY_TAUT_FLATNESS:Authority','Gravity or live thrust limits invalid.');
            A.gravity_world_mps2=double(A.gravity_world_mps2);
            A.thrust_limits_N=double(A.thrust_limits_N(:));
        end
        function [ok,lower,nleaves]=check_w(W,minimum,depth)
            nleaves=1;ok=false;lower=-Inf;
            axis=mean(W,2);
            len=norm(axis);
            if isfinite(len)&&len>0
                axis=axis/len;
                lower=min(axis.'*W);
                if isfinite(lower)&&lower>minimum
                    ok=true;return
                end
            end
            if depth==0,return;end
            [A,B]=TRAJECTORY_TAUT_FLATNESS.split(W);
            [a,la,na]=TRAJECTORY_TAUT_FLATNESS.check_w(A,minimum,depth-1);
            if ~a,nleaves=na;return;end
            [b,lb,nb]=TRAJECTORY_TAUT_FLATNESS.check_w(B,minimum,depth-1);
            ok=a&&b;lower=min(la,lb);nleaves=na+nb;
        end
        function [L,R]=split(P)
            n=size(P,2);Q=P;L=zeros(size(P));R=zeros(size(P));
            L(:,1)=Q(:,1);R(:,end)=Q(:,end);
            for k=2:n
                Q=0.5*(Q(:,1:end-1)+Q(:,2:end));
                L(:,k)=Q(:,1);R(:,n-k+1)=Q(:,end);
            end
        end
    end
end
