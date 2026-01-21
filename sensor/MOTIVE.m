classdef MOTIVE < handle
    % Motive用クラス：登録されたエージェントの位置と姿勢がわかる
    %  sensor.motive = MOTIVE(self, ~)
    %       self : agent
    %  rigid_id : [1,2,3...] : 指定したrigid bodyのp, qをresult.state(i)に登録
    %  state_list : [["p","q"],"p",...] : 指定したrigid_body の指定したプロパティをoutputベクトルに登録

    properties
        result
        state
        self
        old_time
        rigid_id % rigid body indices
        motive
        initq
        state_list
        q_type
        output_func
    end

    methods

        function obj = MOTIVE(self, motive, args)
            arguments
                self
                motive % required
                args.rigid_id = 1;
                args.q_type = 3;
                args.output_func = [];
                args.state_list = {["p","q"]}; % outputを構成する状態リスト {["p","q"],"p"}：rigid_id1からp,q、 rigid_id2からpを取りoutputを構成する
                args.initial_yaw_angle = 0;
                args.initq = [];
            end

            %%% Output equation %%%
            obj.self = self;
            obj.motive = motive;
            obj.rigid_id = args.rigid_id;
            obj.q_type = string(args.q_type);
            obj.output_func = args.output_func;
            if length(args.state_list) < length(args.rigid_id)
                error("MOTIVE SENSOR: number of state_list is too short.");
            else
                if iscell(args.state_list)
                    obj.state_list = args.state_list;
                else
                    obj.state_list = {args.state_list};
                end
                if isempty(args.initq)
                    obj.initq = quaternion(Eul2Quat([0;0;args.initial_yaw_angle])');
                else
                    obj.initq = args.initq;
                end
                for i = 1:length(obj.rigid_id)
                    list = obj.state_list{i};
                    obj.result.state(i) = STATE_CLASS(struct('state_list', list, "num_list", obj.get_num_list(list)));
                end
            end
        end
        function result = do(obj, varargin)
            % result=sensor.motive.do(motive)
            %   set obj.result.state : State_obj,  p : position, q : quaternion
            %   result :
            % 【入力】motive ：NATNET_CONNECOTR object
            data = obj.motive.result;
            q = zeros(4, 1);
            output = [];
            for i = 1:length(obj.rigid_id)
                % for i = length(obj.rigid_id):-1:1
                id = obj.rigid_id(i);
                list = obj.state_list{i};
                for j = list
                    if any(j == "q")
                        tmpq = quaternion(data.rigid(id).q');
                        tmpq = conj(obj.initq) * tmpq;

                        [q(1) q(2) q(3) q(4)] = parts(tmpq);
                        obj.result.state(i).set_state("q", q);
                        if obj.q_type == "3"
                            output = [output;Quat2Eul(q)];
                        else
                            output = [output;q];
                        end
                    end
                    if any(j == "p")
                        obj.result.state(i).set_state("p", data.rigid(id).p);
                        output = [output;data.rigid(id).p];
                    end
                end
            end
            if ~isempty(obj.output_func)
                output = obj.output_func(data);
            end
            % obj.result.rigid = data.rigid;
            % obj.result.feature = data.marker;
            % obj.result.feature_num = data.marker_num;
            % obj.result.local_feature = data.local_marker{id};
            % obj.result.on_feature_num = data.local_marker_nums(id);
            % obj.result.dt = data.time - obj.old_time;
            obj.result.output = output;
            % obj.old_time = data.time;
            result = obj.result;
        end

        function show(obj, varargin)

            if isempty(obj.result)
                disp("do measure first.");
            end

        end

    end
    methods (Access = private)


        function num_list = get_num_list(obj, list)
            num_list = zeros(1, numel(list));
            for k = 1:numel(list)
                name = list(k);
                if name == "p"
                    num_list(k) = 3;
                elseif name == "q"
                    num_list(k) = 4;
                else
                    error("MOTIVE:UnsupportedState", "Unsupported state entry: %s", name);
                end
            end
        end
    end

end
