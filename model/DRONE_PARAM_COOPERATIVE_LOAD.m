classdef DRONE_PARAM_COOPERATIVE_LOAD < PARAMETER_CLASS
    % ドローンの物理パラメータ管理用クラス
    % 以下のconfigurationはclass_description.pptxも参照すること．
    % T = [T1;T2;T3;T4];                  % Thrust force ：正がzb 向き
    % 前：ｘ軸，　左：y軸，　上：ｚ軸
    % motor configuration 
    % T1 : 右後，T2：右前，T3：左後，T4：左前（x-y平面の象限順）
    % T2, T3 の回転方向は軸 zb,  T1, T4 : -zb      [1,0,0,1] で 正のyaw回転
    % tau = [(Ly - ly)*(T3+T4)-ly*(T1+T2); lx*(T1+T3)-(Lx-lx)*(T2+T4); km1*T1-km2*T2-km3*T3+km4*T4]; % Torque for body

     
    properties
        g % gravity 
        N % number of agents
        m0 % load mass
        J0 % load inertia
        rho % 
        rhoc %接続点を頂点とする図形の重心位置から接続点までの距離
        rhoini %
        li
        mi
        Ji
        pUp
        pDown
        G
    end

    methods
        function obj = DRONE_PARAM_COOPERATIVE_LOAD(name,N,type,param)
            arguments
                % 実験の時はrho,rhoini,(rhocはまだ未完成)のみ用いる
                % simの際はparamのfieldの最初からの並びが g, m0, J0, rho, li, mi, Ji,...とする（plantの複数機牽引のmodelの関数に入れる際にこの並びである必要があるため）
                name % DIATONE
                N
                type                = "struct";
                param.g             = 9.81;
                %六角柱
                param.m0            = 1.200;%分割前の牽引物
                param.J0            = [0.15;0.15;0.25];%分割前牽引物慣性モーメント
                % param.J0            = [0.35;0.47;0.45];%非対称牽引物
                % param.J0            = [0.2262;0.3434;0.4735];%非対称牽引物
                %四角柱
                % param.m0            = 3.900;%分割前の牽引物実験牽引物四角
                % param.J0            = [2^2*0.1^2;2^2*0.1^2;2*0.02^2]* 3.900/3;%非対称牽引物正方形

                param.rho           = [];%分割前の重心位置から紐がついてるところ前での距離
                param.rhoini        = [];
                param.rhoc          = [];
                param.li            = 2*ones(N,1);%2*ones(N,1);%紐の長さ
                param.mi            = 0.800*ones(N,1)';%機体の重さ
                param.Ji            = repmat([0.082 0.082 0.1377]',1,N);%機体の慣性モーメント
                param.pUp           = [];
                param.pDown         = [];
                param.G             = [];
                param.additional    = []; % プロパティに無いパラメータを追加する場合
            end
            %% 牽引物
            shape = build_payload_shape(N, type, ...
                "rho", param.rho, ...
                "rhoini", param.rhoini, ...
                "rhoc", param.rhoc, ...
                "pUp", param.pUp, ...
                "pDown", param.pDown, ...
                "G", param.G);
            if ~isempty(shape.rho)
                param.rho = shape.rho;
            end
            if ~isempty(shape.rhoini)
                param.rhoini = shape.rhoini;
            end
            if ~isempty(shape.rhoc)
                param.rhoc = shape.rhoc;
            end
            if ~isempty(shape.pUp)
                param.pUp = shape.pUp;
            end
            if ~isempty(shape.pDown)
                param.pDown = shape.pDown;
            end
            if ~isempty(shape.G)
                param.G = shape.G;
            end
            % simの際はparamのfieldの最初からの並びが g, m0, J0, rho, li, mi, Ji,...とする（plantの複数機牽引のmodelの関数に入れる際にこの並びである必要があるため）
            param_model = rmfield(param, ["rhoini","rhoc","pUp","pDown","G"]);
            obj = obj@PARAMETER_CLASS(name,type,param_model);
            obj.parameter_raw = param;
            obj.rhoini = param.rhoini;
            obj.rhoc = param.rhoc;
            obj.pUp = param.pUp;
            obj.pDown = param.pDown;
            obj.G = param.G;
            obj.N = N;
        end 
    end
end


%元のあった物=======================================================
% 
%     methods
%         function obj = DRONE_PARAM_COOPERATIVE_LOAD(name,N,type,param)
%             arguments
%                 % P = [g m0 j0 rho li mi ji]
%                 name % DIATONE
%                 N = 6;
%                 type = "struct";
%                 % parameters : 5 + 8*N
%                 param.g = 9.81;
%                 param.m0 = 1.45;
%                 param.J0 = [0.15;0.15;0.25];
%                 param.rho = [];
%                 param.li = 1*ones(N,1);
%                 param.mi = 0.755*ones(N,1)';
%                 param.Ji = repmat([0.082 0.0845 0.1377]',1,N);
%                 param.additional = []; % プロパティに無いパラメータを追加する場合
%             end
%             if contains(type,"zup")
%               rho0 = [0;0;1/4];
%             else
%               rho0 = [0;0;-1/4];
%             end
%             if isempty(param.rho)
%               R = Rodrigues([0;0;1],2*pi/N);
%               param.rho = rho0+[[1;0;0],double(cellmatfun(@(A,~) A*[1;0;0], FoldList(@(A,B) A*B,cellrepmat(R,1,N-1),{eye(3)},"mat"),"mat"))];
%             end
%             obj = obj@PARAMETER_CLASS(name,type,param);
%             obj.N = N;
%         end
%     end
% 
% end
