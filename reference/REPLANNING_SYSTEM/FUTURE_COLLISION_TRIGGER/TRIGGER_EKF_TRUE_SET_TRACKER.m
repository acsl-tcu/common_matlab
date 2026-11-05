classdef TRIGGER_EKF_TRUE_SET_TRACKER < handle
    % Stateful set-membership error tracker for the ACTUAL POST-CORRECTION EKF.
    %
    % Original EKF.do does: model prediction -> gain innovation -> projection
    % -> result.state.  Each posterior result.state.get() is a KNOWN numeric
    % point, so it need not be re-propagated through a second interval EKF.
    % Instead propagate the TRUE state set with a validated plant flowpipe
    % then subtract the actual posterior point. This includes the nonlinear
    % EKF correction/projection exactly AS EXECUTED without approximating it.
    %
    % This class never invents initial error, true plant bounds, disturbance,
    % or input uncertainty.  Each true-state box MUST be provided by a
    % separate verified dynamic enclosure. No native Flight safety is set.
    properties (SetAccess=private)
        initialized=false
        time_s=NaN
        true_lower=zeros(24,0)
        true_upper=zeros(24,0)
        ekf_post=zeros(24,0)
        error_lower=zeros(24,0)
        error_upper=zeros(24,0)
        generation=uint64(0)
        provenance='NONE'
        last_status='UNINITIALIZED'
        % No independent proof of physical bounds has been supplied here.
        actual_closed_loop_proven=false
    end
    methods
        function S=seed(obj,t,ekf_post,initial_true_box,source)
            % initial_true_box = [24x2] externally bounded true-state set.
            % source is a traceable source ID, not a proof by itself.
            obj.invalidate('UNKNOWN_SEED_REJECTED');
            obj.check_time(t);
            [xlo,xhi]=obj.validate_box(initial_true_box);
            xhat=obj.validate_ekf(ekf_post);
            assert((ischar(source)||isstring(source)) && ...
                ~isempty(char(source)), ...
                'TRIGGER_EKF_TRUE_SET_TRACKER:Source', ...
                'The true-state enclosure source must be traceable.');
            [elo,ehi]=obj.subtract_post(xlo,xhi,xhat);
            obj.time_s=double(t);obj.true_lower=xlo;obj.true_upper=xhi;
            obj.ekf_post=xhat;obj.error_lower=elo;obj.error_upper=ehi;
            obj.initialized=true;obj.generation=uint64(1);
            obj.provenance=char(source);
            obj.last_status='CONDITIONAL_INITIAL_TRUE_SET_MINUS_ACTUAL_EKF';
            S=obj.snapshot();
        end
        function S=advance(obj,t0,t1,ekf_post,true_next_box,source)
            % Induction: external Phi_k(X_k,U_k,Theta_k,W_k) encloses the
            % true next state. Current actual EKF state is post-correction.
            % A missing / stale enclosure disables the entire tracker.
            if ~obj.initialized
                error('TRIGGER_EKF_TRUE_SET_TRACKER:Uninitialized', ...
                    'A genuine initial true-state bound is required.');
            end
            obj.check_time(t0);obj.check_time(t1);
            if ~(isequal(double(t0),obj.time_s)&&double(t1)>double(t0))
                obj.invalidate('UNKNOWN_FLOWPIPE_CLOCK_OR_GENERATION');
                error('TRIGGER_EKF_TRUE_SET_TRACKER:Clock', ...
                    'True flowpipe must start at the last actual EKF sample.');
            end
            [xlo,xhi]=obj.validate_box(true_next_box);
            xhat=obj.validate_ekf(ekf_post);
            assert((ischar(source)||isstring(source)) && ...
                ~isempty(char(source)), ...
                'TRIGGER_EKF_TRUE_SET_TRACKER:Source', ...
                'A true plant flowpipe source is mandatory.');
            [elo,ehi]=obj.subtract_post(xlo,xhi,xhat);
            obj.time_s=double(t1);obj.true_lower=xlo;obj.true_upper=xhi;
            obj.ekf_post=xhat;obj.error_lower=elo;obj.error_upper=ehi;
            obj.generation=obj.generation+uint64(1);
            obj.provenance=char(source);
            obj.last_status='CONDITIONAL_TRUE_FLOWPIPE_MINUS_ACTUAL_POST_EKF';
            S=obj.snapshot();
        end
        function R=advance_original_held_plant(obj,t0,t1,ekf_post, ...
                u_lower,u_upper,param_lower,param_upper,source,varargin)
            % PROPAGATE the already tracked 24-state TRUE SET through the
            % original interval RHS/Picard over one actual HLC ZOH interval.
            % The end box is constructed HERE, not supplied by the caller.
            % This removes the hand-fed future true-box dependency in the
            % EKF error recursion.  This is still CONDITIONAL: the seed,
            % held actual inputs, physical parameters, unchanged plant
            % model/hybrid mode and outward arithmetic need external proof.
            % Failure INVALIDATES the tracker; stale true sets cannot be used.
            R=struct('status','UNKNOWN_ORIGINAL_HELD_PLANT_ADVANCE', ...
                'reason','','conditional_advanced',false, ...
                'plant_interval',struct(),'taut_manifold',struct(), ...
                'tension',struct(),'posterior',struct(), ...
                'physical_input_proven',false, ...
                'initial_true_set_proven',false, ...
                'hybrid_mode_proven',false, ...
                'numerical_source_proven',false, ...
                'safe_to_commit',false);
            try
                assert(obj.initialized && isnumeric(t0) && isnumeric(t1) && ...
                    isreal(t0) && isreal(t1) && isscalar(t0) && isscalar(t1) && ...
                    isfinite(t0) && isfinite(t1) && ...
                    isequal(double(t0),obj.time_s) && t1>t0, ...
                    'TRIGGER_EKF_TRUE_SET_TRACKER:HeldTime', ...
                    'Actual hold start must match the tracked true-state time.');
                assert(ischar(source) || isstring(source), ...
                    'TRIGGER_EKF_TRUE_SET_TRACKER:HeldSource', ...
                    'A traceable physical input/parameter source is required.');
                % The real held actuator input covers [t0,t1]. If a
                % single original-ODE Picard slab is too wide, refine
                % that SAME physical hold into smaller proof slabs.
                % Never reinitialize from the EKF or numerical plant.
                request=struct( ...
                    't_left_s',double(t0),'t_right_s',double(t1), ...
                    'x0_lower',obj.true_lower,'x0_upper',obj.true_upper, ...
                    'u_lower',u_lower,'u_upper',u_upper, ...
                    'param_lower',param_lower,'param_upper',param_upper);
                if ~isempty(varargin)
                    assert(numel(varargin)==1, ...
                        'TRIGGER_EKF_TRUE_SET_TRACKER:PhysicalHoldSource', ...
                        'At most one source-verified plant hold duration is allowed.');
                    request.verified_hold_duration_s=varargin{1};
                end
                step=TRIGGER_ORIGINAL_HELD_SUBSTEP_CHAIN(request);
                R.plant_interval=step;
                if ~step.conditional_strict_inclusion
                    R.status='UNKNOWN_HELD_PLANT_NO_COMPLETE_SUBSLAB_TUBE';
                    R.reason=[step.status ' | ' step.reason];
                    obj.invalidate(R.status);
                    return
                end
                % Every proof slab has an independently checked Picard
                % enclosure, necessary taut-constraint compatibility and
                % strictly positive original-model tension lower bound.
                R.taut_manifold=step.taut_manifold;
                R.tension=step.tension;
                if ~(R.taut_manifold.compatible && ...
                        isfinite(R.tension.tension_lower_N) && ...
                        R.tension.tension_lower_N>0)
                    R.status='UNKNOWN_HELD_PLANT_TAUT_OR_TENSION';
                    R.reason='The subdivided time slabs did not certify positive tension.';
                    obj.invalidate(R.status);
                    return
                end
                next_box=[step.endpoint_lower step.endpoint_upper];
                R.posterior=obj.advance(t0,t1,ekf_post,next_box,source);
                R.conditional_advanced=R.posterior.initialized;
                R.status='CONDITIONAL_PICARD_TO_ACTUAL_EKF_POSTERIOR__PHYSICAL_PROOF_MISSING';
                R.reason=['Actual EKF posterior was subtracted from a next true-state box ' ...
                    'DERIVED from the original physical RHS and previous true set. ' ...
                    'The initial true set, real held inputs, time-varying model, ' ...
                    'taut mode and full arithmetic must independently be proved.'];
            catch ME
                obj.invalidate('UNKNOWN_HELD_PLANT_ADVANCE_FAILED');
                R.reason=[ME.identifier ' | ' ME.message];
            end
        end
        function invalidate(obj,status)
            if nargin<2,status='UNKNOWN_INVALIDATED';end
            obj.initialized=false;obj.time_s=NaN;
            obj.true_lower=zeros(24,0);obj.true_upper=zeros(24,0);
            obj.ekf_post=zeros(24,0);
            obj.error_lower=zeros(24,0);obj.error_upper=zeros(24,0);
            obj.generation=uint64(0);obj.provenance='NONE';
            obj.last_status=char(status);
        end
        function S=snapshot(obj)
            S=struct('status',obj.last_status,'initialized',obj.initialized, ...
                'time_s',obj.time_s,'generation',obj.generation, ...
                'true_lower',obj.true_lower,'true_upper',obj.true_upper, ...
                'ekf_post',obj.ekf_post, ...
                'error_lower',obj.error_lower,'error_upper',obj.error_upper, ...
                'provenance',obj.provenance, ...
                'original_ekf_posterior_used',obj.initialized, ...
                'actual_closed_loop_proven',false, ...
                'safe_to_commit',false);
        end
    end
    methods (Static,Access=private)
        function check_time(t)
            assert(isa(t,'double')||isnumeric(t), ...
                'TRIGGER_EKF_TRUE_SET_TRACKER:Time', 'Time must be numeric.');
            assert(isreal(t)&&isscalar(t)&&isfinite(t), ...
                'TRIGGER_EKF_TRUE_SET_TRACKER:Time', 'Finite time required.');
        end
        function [lo,hi]=validate_box(A)
            assert(isa(A,'double')&&isreal(A)&& ...
                isequal(size(A),[24 2]) && all(isfinite(A(:))) && ...
                all(A(:,1)<=A(:,2)), ...
                'TRIGGER_EKF_TRUE_SET_TRACKER:TrueBox', ...
                'A finite ordered 24-by-2 TRUE-state interval box is required.');
            lo=A(:,1);hi=A(:,2);
        end
        function x=validate_ekf(v)
            % The actual EKF object is allowed. Read the already-computed
            % POST-projection state; do NOT re-run prediction/correction.
            if isa(v,'EKF')
                v=v.result.state.get();
            end
            assert(isa(v,'double')&&isreal(v)&&numel(v)==24 && ...
                all(isfinite(v(:))), ...
                'TRIGGER_EKF_TRUE_SET_TRACKER:EKF', ...
                'Use the actual 24-state POST-correction EKF numeric vector.');
            x=v(:);
        end
        function [lo,hi]=subtract_post(xlo,xhi,xhat)
            % Each elementary subtraction is rounded to nearest on the
            % normal MATLAB IEEE-binary64 path. One neighboring representable
            % value is included on EACH side. This does not certify the
            % external true flowpipe or prove MATLAB's entire runtime path.
            zlo=xlo-xhat;zhi=xhi-xhat;
            assert(all(isfinite(zlo))&&all(isfinite(zhi)), ...
                'TRIGGER_EKF_TRUE_SET_TRACKER:Overflow', ...
                'Difference overflowed; cannot create a finite error box.');
            lo=TRIGGER_FLOAT_OUTWARD_ONE_ULP(zlo,-1);
            hi=TRIGGER_FLOAT_OUTWARD_ONE_ULP(zhi,+1);
        end
    end
end
