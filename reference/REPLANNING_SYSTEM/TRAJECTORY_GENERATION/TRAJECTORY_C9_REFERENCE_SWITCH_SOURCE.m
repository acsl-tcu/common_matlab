classdef TRAJECTORY_C9_REFERENCE_SWITCH_SOURCE < handle
%TRAJECTORY_C9_REFERENCE_SWITCH_SOURCE Flight reference adapter, default pass-through.
% Register explicitly as the 'timevarying' reference source BEFORE Flight;
% the original SUSPENDED_LOAD_REF_ADJUST then runs unchanged, followed by
% the original HLC. No HLC, EKF, plant or runner modification.
%
% arm() is a HARD fail-closed gate until an independently verified
% end-to-end HLC+B/roundoff/C7 join provenance object exists. Caller
% boolean flags can never switch Flight. This V7 adapter remains an
% opt-in pass-through source; it is NOT a complete Flight switch.
% This class provides the actual next-tick reference propagation path but
% is NOT a shortcut around execution certification.
    properties (SetAccess=private)
        self
        original
        result
        active_piece=struct()
        active_revision=uint64(0)
        switch_epoch_s=NaN
        is_armed=false
        last_status='PASS_REFERENCE_PASSTHROUGH'
        last_sample_time_s=NaN
    end
    methods
        function obj=TRAJECTORY_C9_REFERENCE_SWITCH_SOURCE(self,original)
            assert(isa(original,'handle')&&ismethod(original,'do'), ...
                'C9_SWITCH:OriginalSource','Require original reference source.');
            obj.self=self;obj.original=original;
        end
        function R=arm(obj,piece,switch_epoch_s,revision,certificate)
            %#ok<INUSD> All inputs will be validated by the future
            % independent *source provenance* verifier, not a caller's
            % arbitrary boolean claims. At V7 no such verified numeric
            % execution-proof token exists, so this is deliberately
            % fail-closed even if a user supplies a struct of true flags.
            R=struct('accepted',false, ...
                'status','UNKNOWN_REFERENCE_SWITCH_TRUSTED_PROVENANCE_NOT_IMPLEMENTED', ...
                'reference_mutated',false,'HLC_plant_modified',false);
            obj.last_status=R.status;
        end
        function R=disarm(obj)
            % Disarm only before Flight. Falling back to the original path
            % in the middle of an avoidance would break C7 continuity.
            R=struct('accepted',false,'status','UNKNOWN_REFERENCE_SWITCH_DISARM');
            if isprop(obj.self,'cha') && strcmp(char(obj.self.cha),'f') && obj.is_armed
                R.status='UNKNOWN_C9_INFLIGHT_DISARM_NOT_C7_CERTIFIED';return
            end
            obj.is_armed=false;obj.active_piece=struct();
            obj.switch_epoch_s=NaN;
            obj.last_status='PASS_REFERENCE_PASSTHROUGH';
            R.accepted=true;R.status='PASS_REFERENCE_PASSTHROUGH';
        end
        function result=do(obj,varargin)
            base=obj.original.do(varargin{:});
            result=base;
            if nargin<2 || ~obj.is_armed
                obj.result=result;return
            end
            time=varargin{1};
            if numel(varargin)<2 || ~strcmp(char(varargin{2}),'f')
                obj.result=result;return
            end
            if ~(isprop(time,'t') && isfinite(time.t) && ...
                  time.t>=obj.switch_epoch_s && ...
                  time.t<=obj.active_piece.t0_s+obj.active_piece.duration_s)
                error('C9_SWITCH:UncertifiedEpoch', ...
                      'Active C9 reference outside the certified epoch.');
            end
            L=double(obj.self.parameter.get('cableL'));
            H=TRAJECTORY_C9_REFERENCE_28_FROM_PIECE( ...
                         obj.active_piece,time.t,L);
            if ~H.valid
                error('C9_SWITCH:InvalidReference', ...
                      'C9 exact source reference cannot be evaluated.');
            end
            if ~(isstruct(base)&&isfield(base,'state') && ...
                  isa(base.state,'handle'))
                error('C9_SWITCH:OriginalReferenceLayout', ...
                      'Original source result lacks reference state.');
            end
            % Never mutate the original source's STATE_CLASS handle.
            clone=state_copy(base.state);
            clone.xd=H.xd_pre_sload_28;
            if isprop(clone,'p'),clone.p=H.xd_pre_sload_28(1:3);end
            result.state=clone;
            obj.result=result;
            obj.last_status='C9_EXACT_SOURCE_SAMPLE__ORIGINAL_SLOAD_AND_HLC_NEXT';
            obj.last_sample_time_s=double(time.t);
        end
    end
end
