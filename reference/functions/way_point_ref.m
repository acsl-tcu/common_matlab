    %スプラインの係数計算
            function ref = way_point_ref(val,n,check)
            % val         %時間とwaypoint
            arguments
                val         %時間とwaypoint
                n           %多項式次数
                check       %事前に軌道確認
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
            co=[];
            for i = 0:n
            co = [co,"d"+num2str(i)];
            end
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
            
            if check == 1   %軌道事前確認※2024年度のそのまま移植
            t_ref0=0;
            i=1;
            j=1;
            delta=0.025;
            end_time=time(end)+1;
            length_time = length(0:delta:end_time);
            xyz = zeros(3,length_time);
            for t_f = 0:delta:end_time
                t_ref= t_f - t_ref0;%目標地点が定められた時間からの経過時間
                if round(t_ref,4) >= dtime(i) 
                   i=i+1;
                    if i >length(ref.t) 
                        i = length(ref.t);
                        t_ref = dtime(end);
                    else
                        t_ref0 = round(t_f,4);
                        t_ref=0;
                    end
                end
                xyz(:,j) = coefficients.(names{1})(:,:,i)*t_powers.(names{1})(t_ref);
                j=j+1;
            end
            figure(101)
            plot3(xyz(1,:), xyz(2,:), xyz(3,:), 'LineWidth', 2)
            hold on
            grid on
            plot3(val(1:end,2),val(1:end,3),val(1:end,4), 'ro', 'MarkerSize', 10, 'MarkerFaceColor', 'r');
            h_start = plot3(val(1,2), val(1,3), val(1,4), 'gp', 'MarkerSize', 12, 'MarkerFaceColor', 'g');
            xlabel('$x$ (m)', 'FontSize', 14, 'Interpreter', 'latex')
            ylabel('$y$ (m)', 'FontSize', 14, 'Interpreter', 'latex')
            zlabel('$z$ (m)', 'FontSize', 14, 'Interpreter', 'latex')
            title('3次元軌道', 'FontSize', 14)
            set(gca, 'TickLabelInterpreter', 'latex', 'FontSize', 14)
            axis equal
            legend(h_start, {'始点'},  'FontSize', 14)

            end



        end
   