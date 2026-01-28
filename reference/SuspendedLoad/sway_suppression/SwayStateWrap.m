%==========================================================================
% ローカル軽量クラス：HLCがispropでxdを確実に拾えるようにするためのラッパ
%==========================================================================
classdef SwayStateWrap < handle
    properties
        xd
        p
        v
        raw   % 元のstate（struct/obj）を保持（必要なら参照用）
    end

    methods
        function obj = SwayStateWrap(raw_state)
            obj.raw = raw_state;

            % 元にxd/p/vがあれば一応コピー（なくても後で上書きされる）
            try
                if isstruct(raw_state)
                    if isfield(raw_state,'xd'); obj.xd = raw_state.xd; end
                    if isfield(raw_state,'p');  obj.p  = raw_state.p;  end
                    if isfield(raw_state,'v');  obj.v  = raw_state.v;  end
                elseif isobject(raw_state)
                    if isprop(raw_state,'xd'); obj.xd = raw_state.xd; end
                    if isprop(raw_state,'p');  obj.p  = raw_state.p;  end
                    if isprop(raw_state,'v');  obj.v  = raw_state.v;  end
                end
            catch
                % 何かあっても致命傷にしない
            end
        end

        function out = get(obj)
            % HLCがelseでget()を呼んでも返せるように保険
            out = obj.xd;
        end
    end
end
