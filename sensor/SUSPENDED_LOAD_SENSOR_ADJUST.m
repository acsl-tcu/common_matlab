classdef SUSPENDED_LOAD_SENSOR_ADJUST < handle
    % 吊り荷付きセンサデータの位相補正（p, pL）
    properties
        self
        result
        td % 遷移時間
        takeoff_t0
        landing_t0
        pL0
        cableL_L0
        isGround = 1
    end

    methods
        function obj = SUSPENDED_LOAD_SENSOR_ADJUST(self, opts)
            arguments
                self
                opts.td = 5
            end
            obj.self = self;
            obj.td = opts.td;
        end

        function result = do(obj, varargin)
            % sensor.do と同じ引数列を想定
            if numel(varargin) < 2
                result = obj.result;
                return
            end
            time = varargin{1};
            cha = varargin{2};

            % ベースセンサ（motive等）の結果を取得
            base = obj.self.sensor.result;
            if isempty(base)
                result = obj.result;
                return
            end
            % p = rigid(1).p, pL = rigid(2).p を取り出し
            [p, pL_raw] = obj.extract_positions(base);
            if isempty(p) || isempty(pL_raw)
                obj.result = base;
                result = base;
                return
            end

            L = obj.self.parameter.get("cableL");
            if isempty(obj.pL0)
                % 離陸判定の基準となる初期高さ
                obj.pL0 = pL_raw;
            end
            p_under = [p(1:2); p(3) - L];
            pL = pL_raw;

            if cha == 't' || cha == '0' || cha == 'a'
                % 離陸前は pL を p の真下に固定
                if p(3) > obj.pL0(3) + L * 0.6
                    if obj.isGround
                        obj.isGround = 0;
                        obj.takeoff_t0 = time.t;
                    end
                    % 真下から真値へ時間とともに戻す
                    tt = min((time.t - obj.takeoff_t0), obj.td);
                    k = tt / obj.td;
                    pL = p_under + k * (pL_raw - p_under);
                else
                    pL = p_under;
                end
            elseif cha == 'l'
                % 着陸フェーズ：高度が十分下がったら真下へ戻す
                if isempty(obj.cableL_L0)
                    obj.cableL_L0 = norm(p - pL_raw);
                end
                if p(3) < obj.cableL_L0 || obj.isGround == 1
                    if isempty(obj.landing_t0)
                        obj.landing_t0 = time.t;
                        obj.isGround = 1;
                    end
                    tt = min(time.t - obj.landing_t0, obj.td);
                    k = tt / obj.td;
                    pL = p_under + (1 - k) * (pL_raw - p_under);
                end
            end

            % 補正した pL を result に反映
            result = obj.adjust_output(base, p, pL);
            obj.result = result;
        end
    end

    methods (Access = private)

        function [p, pL_raw] = extract_positions(~, base)
            % MOTIVE/rigid 等の形式に対応して p, pL を抽出
            p = [];
            pL_raw = [];
            if isstruct(base) && isfield(base, "state")
                st = base.state;
                if numel(st) >= 2 && isobject(st(1)) && isobject(st(2)) ...
                        && isprop(st(1), "p") && isprop(st(2), "p")
                    p = st(1).p;
                    pL_raw = st(2).p;
                    return
                end
                if isobject(st) && isprop(st, "p") && isprop(st, "pL")
                    p = st.p;
                    pL_raw = st.pL;
                    return
                end
            end
            if isstruct(base) && isfield(base, "rigid")
                try
                    p = base.rigid(1).p;
                    pL_raw = base.rigid(2).p;
                catch
                end
            end
        end

        function result = adjust_output(~, base, p, pL)
            % output をできる範囲で補正
            result = base;

            if isstruct(result) && isfield(result, "output")
                out = result.output;
                if numel(out) >= 12
                    % output が [p; eul; pL; pT] 形式なら pL/pT も更新
                    pT = pL - p;
                    n = norm(pT);
                    if n > 0
                        pT = pT / n;
                    else
                        pT = [0; 0; -1];
                    end
                    out(7:9) = pL;
                    out(10:12) = pT;
                    result.output = out;
                end
            end
        end
    end
end
