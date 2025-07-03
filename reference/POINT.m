classdef POINT < handle
    properties
        param
        self
        coefficients
        time
        dtime
        t_powers
        t_ref
        t0
        t_ref0
        names
        i
        fref
        result
        interpolation
    end
    
    methods
        function obj = POINT(self,varargin)
            %縦ベクトルで書く
            %最初のコマンドは"f"で始める
            %===method===
            % --do(obj,varargin)
                % referenceは3階微分まで生成している
                % コメントアウトの切換でできる設定
                % 繰り返し用:最後のポイントになったら次を最初のポイントにする．これにするときは最初と最後のポイントを一致させる
            % --(static) way_point_ref(val,n,fconfirm,fdrowfig)
                % val         %時間とwaypoint
                % n           %多項式次数
                % fconfirm=1  %確認するか
                % fdrowfig=1  %図を描画するか
            
            obj.self = self;
            val = varargin{1};
            n = val.n;
            obj.coefficients = val.coefficients;
            obj.names = fieldnames(val.coefficients);            
            for i = 1:length(obj.names)
                obj.t_powers.(obj.names{i}) = @(t) (t).^(0:n+1-i)';
            end
            obj.time = val.t;
            obj.dtime = val.dt;
            obj.i=1;
            obj.fref=0;
            obj.t_ref0=0;
            
            obj.result.state = STATE_CLASS(struct('state_list',["xd","p", "q","v"],'num_list',[20,3,3,3]));
            obj.result.state.set_state("xd",zeros(6,1));
            obj.result.state.set_state("p",obj.self.estimator.result.state.get("p"));
            obj.result.state.set_state("q",obj.self.estimator.result.state.get("q"));
            obj.result.state.set_state("v",obj.self.estimator.result.state.get("v"));
        end
        function result= do(obj,varargin)
            % 【Input】result = {Xd(optional)}
            if isempty(obj.t0)
                obj.t0=varargin{1}.t;%目標地点が定められた時刻
            end
            
            t_f = varargin{1}.t - obj.t0;%目標地点が定められた時間からの経過時間
            obj.t_ref= t_f - obj.t_ref0;

            if round(obj.t_ref,4) >= obj.dtime(obj.i) 
               obj.i=obj.i+1;
                if obj.i > length(obj.time)
                    obj.i = length(obj.time);
                    obj.t_ref = obj.dtime(end);
                    %繰り返し用:最後のポイントになったら次を最初のポイントにする．これにするときは最初と最後のポイントを一致させる
                    obj.i = 1;
                    obj.t_ref0 = round(t_f,4);
                    obj.t_ref=0;
                else
                    obj.t_ref0 = round(t_f,4);
                    obj.t_ref=0;
                end
            end

            xd=zeros(4,4);
            for j = 1:3
                xd(:,j) = [obj.coefficients.(obj.names{j})(:,:,obj.i)*obj.t_powers.(obj.names{j})(obj.t_ref); 0];
            end
            obj.result.state.xd = reshape(xd,16,1);%3階微分まで
            obj.result.state.p = xd(1:3)';
            obj.result.state.v = xd(5:7)';
            obj.result.state.q(3,1) = 0;%yaw
            result = obj.result;
        end
         function show(obj, logger)
            rp = logger.data(1,"p","r");
            plot3(rp(:,1), rp(:,2), rp(:,3));                     % xy平面の軌道を描く
            daspect([1 1 1]);
            hold on
            ep = logger.data(1,"p","e");
            plot3(ep(:,1), ep(:,2), ep(:,3));       % xy平面の軌道を描く
            legend(["reference", "estimate"]);
            title('reference and estimated trajectories');
            xlabel("x (m)");
            ylabel("y (m)");
            hold off
        end
    end

    methods (Static)
        function ref = way_point_ref(val,n,fdrowfig)
            % val         %時間とwaypoint
            % n           %多項式次数
            % fdrowfig=1  %図を描画するか
            % time=[0,2,5,12];%time
            % point = [0,4,6,2;0,2,-1,4;0,3,5,2];%way points
            % n=5;%多項式次数
            arguments
                val         %時間とwaypoint
                n           %多項式次数
                fdrowfig  %図を描画するか
            end
            time = val(:,1)';
            point = val(:,2:end)';
            dtime = diff(time');%隣の点との差 dw_i = w_i - w_i-1 (dw_1 = 0),  i=1,2,3,...
            Sn=length(time(1:end-1)); %求める多項式の数
            D(1,:)=ones(1,n+1);%多項式の係数行列1に初期化
            

            for i = 1:n-1%多項式の階数ごとの微分係数計算
                D(i+1,:)=[zeros(1,i), 1:n-i+1].*D(i,:);
                % D(i+1,:)=[zeros(1,i), polyder(D(i,i:end))];
            end
            %微分した時の係数
            D_ori=D;
            D=D./factorial(0:n-1)';
            
            power_dtime=zeros(Sn,n+1);
            for i=1:n+1
                power_dtime(:,i) = dtime.^(i-1);
            end
            %各ポイント間のrefenceの関数の係数を行列まとめて計算
            %P=Xp^(-1)Y
            %P:求める係数
            %Xp:時間とt^nの微分係数の積の行列
            %Y:waypoint
    
            %Xpの生成
            X=zeros((n+1)*Sn);
            
            for i=1:Sn
                dr=(i-1)*2;
                dc=(i-1)*(n+1);
                X(1+dr:2+dr,1+dc:(n+1)+dc) = [1,zeros(1,n);power_dtime(i,:)];
            end
            
            poweT = zeros(n-1,n+1);
            for i = 1:Sn-1
                for j=1:n-1
                    poweT(j,j+1:end) = power_dtime(i,1:end-j) ;
                end
                dr2=(i-1)*(n-1);
                dc2=(i-1)*(n+1);
                X(2*Sn+1+dr2 : 2*Sn+n-1+dr2, 1+dc2 : (n+1)+dc2) = D(2 : end,1:end).*poweT;
                X(2*Sn+1+dr2 : 2*Sn+n-1+dr2, n+3+dc2 : 2*n+1 +dc2) = - eye(n-1);
            end
            add=(n+1)*Sn - (n-1);
            for j=1:n-1
                    poweT(j,j+1:end) = power_dtime(end,1:end-j) ;
            end
            %端点の制約
            for i = 1:n-2
                dr3=(i-1)*2;
                % X(add+1+dr3,1+i) = 1;%4関数でさいしょの端点で激しくなるか，最後収束しないかでインデックス1,2をへんこう
                % X(add+2+dr3,end-n:end) = D(i+1,:).*poweT(i,:) ;
                X(add+2+dr3,1+i) = 1;%さいしょの端点で激しくなるか，最後収束しないかでインデックス1,2をへんこう
                X(add+1+dr3,end-n:end) = D(i+1,:).*poweT(i,:) ;
            end
            
            s=1:(n+1)*Sn;
            Xp =X(s,s);
    
            %P=Xp^(-1)Yの計算
            P = zeros(3,n+1,Sn);
            for i = 1:3
                Y1 =  reshape(ones(2,Sn+1).*point(i, :),2*(Sn+1),1);
                Y1 = Y1(2:end-1);
                Y = [Y1;zeros((n+1)*Sn-2*Sn,1)];
                P(i,:,:) = reshape(Xp\Y,1,n+1,Sn);
            end
            
            %係数の格納
            %n次次多項式にしても係数は3次までしか保存できない
            co = ["d0","d1","d2","d3"];
            index = length(co);
            for i = 1:index
                coefficients.(co(i))=zeros(3,n+2-i ,Sn);
            end
            
            coefficients.d0 = P;
            if length(co) > n 
                index = n;
            end
            for i = 2:index
                coefficients.(co(i))=coefficients.d0(:,i:end,:).*D_ori(i,i:end);
            end
            ref.n=n;
            ref.coefficients=coefficients;
            ref.t=time(2:end);
            ref.dt=dtime;
            
            names = fieldnames(ref.coefficients);            
            for i = 1:length(names)
                t_powers.(names{i}) = @(t) (t).^(0:n+1-i)';
            end
            ref.t_powers = t_powers;
            ref.time = time;
            %一般式作る
            ref.period = time(end);%軌道の周期
            
            ref.Sn = Sn;%区間数
           
        end
    end
end
