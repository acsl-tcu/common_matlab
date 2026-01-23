classdef TIMEVARYING_SWAY_REF < handle
    % TIMEVARYING_SWAY_REF
    % TIME_VARYING_REFERENCE -> SWAY_REF_MOD を 1つのreferenceとして統合するラッパ
    %
    % 既存のSWAY_REF_MODは self.reference.origin を参照する前提なので、
    % do() の中で一時的に self.reference.origin を差し替えてからSWAY_REF_MODを呼ぶ。

    properties
        self
        origin_ref   % TIME_VARYING_REFERENCE
        sway_ref     % SWAY_REF_MOD
        result
    end

    methods
        function obj = TIMEVARYING_SWAY_REF(self, tv_args, sway_param)
            obj.self = self;

            % 既存の2モジュールを内部に保持
            obj.origin_ref = TIME_VARYING_REFERENCE(self, tv_args);
            obj.sway_ref   = SWAY_REF_MOD(self, sway_param);

            % resultの形式は「originのresult」を引き継ぐ（下流互換）
            obj.result = obj.origin_ref.result;
        end

        function result = do(obj, varargin)
            % varargin: (time, cha, logger, env, agent, i)
            % 1) origin生成
            r0 = obj.origin_ref.do(varargin{:});
            obj.origin_ref.result = r0; % 念のため（do内で更新済みだが安全側）

            % 2) self.reference を一時的に差し替え（SWAY_REF_MOD互換のため）
            self_obj = obj.self;
            if isprop(self_obj, "reference")
                ref_backup = self_obj.reference;
            else
                ref_backup = [];
            end

            tmpref = struct();
            tmpref.origin = obj.origin_ref; % SWAY_REF_MOD が読む場所
            self_obj.reference = tmpref;

            % 3) sway適用（内部で self.reference.origin.result.state.xd を上書きする）
            r1 = obj.sway_ref.do(varargin{:});

            % 4) self.reference を元に戻す（副作用を残さない）
            self_obj.reference = ref_backup;

            % 5) 出力（形式維持：originのresult形式）
            obj.result = r1;
            result = obj.result;
        end
    end
end
