classdef DRAW_DRONE_MOTION
  % フレーム付きで動画を作成するためのクラス
  % obj = DRAW_DRONE_MOTION(logger,param)
  % obj.draw(p,q,u) : １スナップを描画
  % obj.animation(logger,param) : 動画描画
  % TODO : methodなど一般化して抽象化したほうが良いか？

  properties
    frame
    thrust
    ax
    xlim
    ylim
    zlim
    L
    frame_size = [0.1170, 0.0932];
    rotor_r= 0.0392;
  end

  methods
    function obj = DRAW_DRONE_MOTION(logger,varargin)
      % 【usage】 obj = DRAW_DRONE_MOTION(logger,param)
      % logger : LOGGER class instance
      % param.frame_size : drone size [Lx,Ly]
      % param.rotor_r : rotor radius r
      % arguments
      %     logger
      %     param.frame_size
      %     param.rotor_r
      %     param.target = 1;
      %     param.fig_num = 1;
      %     param.mp4 = 0;
      % end
      param = struct(varargin{:});
      if ~isfield(param, 'mpc_test')
        param.mpc_test = false;
      end

      data = logger.data(param.target,"p","e");
      try
        ref_data = logger.data(param.target,"p","r");
        if ~isempty(ref_data) && size(ref_data, 2) == size(data, 2)
          data = [data; ref_data];
        end
      catch
      end
      tM = max(data);
      tm = min(data);
      M = [max(tM(1:3:end)),max(tM(2:3:end)),max(tM(3:3:end))];
      m = [min(tm(1:3:end)),min(tm(2:3:end)),min(tm(3:3:end))];
      if isfield(param,'frame_size')
        L = param.frame_size;
      else
        L = obj.frame_size;
      end
      obj.L = L;
      obj.xlim = [m(1)-L(1) M(1)+L(1)];
      obj.ylim = [m(2)-L(2) M(2)+L(2)];
      obj.zlim = [0 M(3)+1];
      if isfield(param,'ax')
        ax = param.ax;
      else
        figure();
        ax = axes('XLim',obj.xlim,'YLim',obj.ylim,'ZLim',obj.zlim);
        varargin = {varargin{:},'ax',ax};
      end
      if param.mpc_test
        obj.ax = ax;
        obj.frame = gobjects(0);
        obj.thrust = gobjects(0);
        hold(ax, "on");
      else
        obj=obj.gen_frame(varargin{:});%"frame_size",param.frame_size,"rotor_r",param.rotor_r, "target",param.target,"fig_num" ,param.fig_num);
      end

      view(ax,3)
      grid(ax,'on')
      daspect(ax,[1 1 1]);
    end
    function obj=gen_frame(obj,varargin)%param)
      % arguments
      %     obj
      %     param.frame_size
      %     param.rotor_r
      %     param.target = 1;
      %     param.fig_num = 1;
      % end
      param = struct(varargin{:});
      % if class(param.fig_num)=="matlab.ui.Figure"
      %     obj.fig = param.fig_num;
      %     ax = obj.fig.CurrentAxes;
      % else
      %     obj.fig = figure(param.fig_num);
      %     ax = axes('XLim',obj.xlim,'YLim',obj.ylim,'ZLim',obj.zlim);
      % end
      obj.ax = param.ax;
      ax = obj.ax;
      xlabel(obj.ax,"x [m]");
      ylabel(ax,"y [m]");
      zlabel(ax,"z [m]");
      hold(ax,"on");

      L = obj.L;

      % rotor setup
      if isfield(param,'rotor_r')
        r = param.rotor_r;
      else
        r = obj.rotor_r;
      end
      [xr,yr,zr] = cylinder(ax,[0 r]); % rotor
      zr = 0.001*zr;

      % body setup
      [xb,yb,zb] = ellipsoid(ax,0,0,0, 1.2*L(1)/4, 0.8*L(2)/4, 0.02);
      d = L/2; % 重心からローター１の位置ベクトル
      rp = [d(1),d(2),0.02;-d(1),d(2),0.02;d(1),-d(2),0.02;-d(1),-d(2),0.02]; % relative rotor position
      c = ["red","green","blue","cyan",'#404040'];

      % arm setup
      [x,y,z] = cylinder(ax,0.01);
      z = z*vecnorm(d)*2;
      R1 = rotmat(quaternion([L(2),-L(1),0]*pi/(vecnorm(L)*2),"rotvec"),'point');
      R2 = rotmat(quaternion([L(2),L(1),0]*pi/(vecnorm(L)*2),"rotvec"),'point');
      F1 = [R1*[x(1,:);y(1,:);z(1,:)];R1*[x(2,:);y(2,:);z(2,:)]]+[d(1);d(2);0;d(1);d(2);0];
      F2 = [R2*[x(1,:);y(1,:);z(1,:)];R2*[x(2,:);y(2,:);z(2,:)]]+[-d(1);d(2);0;-d(1);d(2);0];
      for n = param.target
        for i = 4:-1:1
          h(n,i) = surface(ax,xr+rp(i,1),yr+rp(i,2),zr+rp(i,3),'FaceColor',c(i)); % rotor 描画
          T(n,i) = quiver3(ax,rp(i,1),rp(i,2),rp(i,3),0,0,1,'FaceColor',c(i)); % 推力ベクトル描画
          tt(n,i) = hgtransform('Parent',ax); set(T(n,i),'Parent',tt(n,i)); % 推力を慣性座標と紐づけ
        end
        h(n,5) = surface(ax,[F1(1,:);F1(4,:);NaN(1,size(F1,2));F2(1,:);F2(4,:)], ...
          [F1(2,:);F1(5,:);NaN(1,size(F1,2));F2(2,:);F2(5,:)], ...
          [F1(3,:);F1(6,:);NaN(1,size(F1,2));F2(3,:);F2(6,:)],'FaceColor',c(5)); % アーム部
        h(n,6) = surface(ax,xb,yb,zb,'FaceColor',c(5)); % ボディ楕円体
        % h(n,7) = trisurf([5 1 2;5 2 3; 5 3 4; 5 4 1], ...
        %   [0.02;0.02;0.02;0.02;4*1.2*L(1)/8], ...
        %   0.8*L(2)*[1;-1;-1;1;0]/6, ...
        %   0.01*[1;1;-1;-1;0],'FaceColor',c(5)); % 前を表す四角錐

        t(n) = hgtransform('Parent',ax); set(h(n,:),'Parent',t(n)); % ドローンを慣性座標と紐づけ
      end
      obj.frame = t;
      obj.thrust = tt;
      %            if param.animation
      %                obj.animation(logger,"realtime",true,"target",param.target,"gif",param.gif,"Motive_ref",param.Motive_ref,"fig_num",param.fig_num,"mp4",param.mp4);
      %            end
    end
    function draw(obj,target,p,q,u)
      % obj.draw(p,q,u)
      % p : 一ベクトル
      % q = [x,y,z,th] : 回転軸[x,y,z]にth回転
      % u (optional): 入力
      arguments
        obj
        target
        p
        q
        u = [1;1;1;1];
      end
        d=[0.16/2 0.16/2];%obj.param.Lx;
        rp = [d(1),d(2),0.02; -d(1),d(2),0.02; d(1),-d(2),0.02; -d(1),-d(2),0.02];
      for n = target
        frame = obj.frame(n);
        thrust = obj.thrust(n,:);
        % Rotation matrix
        R = makehgtform('axisrotate',q(1,1:3,n),q(1,4,n));
        % Translational matrix
        Txyz = makehgtform('translate',p(1,:,n));
        % Scaling matrix
        sclar=100;
        for i = 1:4
          if u(1,i,n) > 0
            S = makehgtform('scale',[1,1,u(1,i,n)*sclar]);
          elseif u(i) < 0
            S1 = makehgtform('xrotate',pi);
            S = makehgtform('scale',[1,1,-u(1,i,n)*sclar])*S1;
          else
            S = eye(4);
            S(3,3) = 1e-5;
          end
          Trp = makehgtform('translate', rp(i,:));
          set(thrust(i),'Matrix',Txyz*R*S);
        end
        % Concatenate the transforms and
        % set the transform Matrix property
        set(frame,'Matrix',Txyz*R);
      end
      drawnow
    end
    function animation(obj,logger,varargin)
      % obj.animation(logger,param)
      % logger : LOGGER class instance
      % param.realtime (optional) : t-or-f : logger.data('t')を使うか
      % param.target = 1:4 描画するドローンのインデックス
      % "rotor_r",p.rotor_r;
      % arguments
      %   obj
      %   logger
      %   param.rotor_r
      %   param.self
      %   param.realtime = false;
      %   param.target = 1;
      %   param.gif = 0;
      %   param.Motive_ref = 0;
      %   param.fig_num = 1;
      %   param.mp4 = 0;
      %   param.frame_size = [];
         
      % end
      param = struct(varargin{:});
      param.opt_plot = [];
      param.Motive_ref = 0;
      if ~isfield(param, 'mpc')
        param.mpc = false;
      end
      if ~isfield(param, 'mcmpc')
        param.mcmpc = false;
      end
      if ~isfield(param, 'mpmpc')
        param.mpmpc = false;
      end
      if ~isfield(param, 'mpc_test')
        param.mpc_test = false;
      end
      if ~isfield(param, 'mpc_xy')
        param.mpc_xy = false;
      end
      if ~isfield(param, 'gif')
        param.gif = false;
      end
      if ~isfield(param, 'mp4')
        param.mp4 = false;
      end
      if ~isfield(param, 'mpc_scale')
        param.mpc_scale = 1;
      end
      if ~isfield(param, 'mpc_z_offset')
        param.mpc_z_offset = 0;
      end
      if ~isfield(param, 'mpc_color')
        param.mpc_color = [1, 0, 0];
      end
      if ~isfield(param, 'mpc_marker')
        param.mpc_marker = 'none';
      end
      if ~isfield(param, 'ref_full')
        param.ref_full = ~param.mpc_test;
      end
      plot_mpc = param.mpc || param.mcmpc || param.mpmpc || param.mpc_test || param.mpc_xy;
      ax = obj.ax;
      %p = logger.data(param.target,"p","e");
      %q = logger.data(param.target,"q","e");
      p = logger.data(param.target,"p","p");
      q = logger.data(param.target,"q","p");
      u = logger.data(param.target,"input");
      r = logger.data(param.target,"p","r");
      p = reshape(p,size(p,1),3,length(param.target));
      q = reshape(q,size(q,1),size(q,2)/length(param.target),length(param.target));
      u = reshape(u,size(u,1),size(u,2),length(param.target));
      r = reshape(r,size(r,1),3,length(param.target));
      for n = 1:length(param.target)
        switch size(q(:,:,n),2)
          case 3
            Q1 = quaternion(q(:,:,n),'euler','XYZ','frame');
          case 4
            Q1 = quaternion(q(:,:,n));
          case 9
            Q1 = quaternion(q(:,:,n),'rotmat','frame');
        end
        Q1 = rotvec(Q1);
        tmp = vecnorm(Q1,2,2);
        Q(:,:,n) = zeros(size(Q1,1),4);
        Q(tmp==0,:,n) = 0;
        Q(tmp==0,1,n) = 1;
        Q(tmp~=0,:,n) = [Q1(tmp~=0,:)./tmp(tmp~=0),tmp(tmp~=0)];
      end

      if param.gif
        sizen = 256;
        delaytime = 0;
        filename = strrep(strrep(strcat('Data/Movie(',datestr(datetime('now')),').gif'),':','_'),' ','_');
      end

      if param.mp4
        try
          sizen = 256;
          delaytime = 0;
          if ~exist('Data', 'dir')
            mkdir('Data');
          end
          filename = strrep(strrep(strcat('Data/Movie(',datestr(datetime('now')),').mp4'),':','_'),' ','_');
          v = VideoWriter(filename,"MPEG-4");
          v.Quality = 95;
          open(v);
        catch ME
          warning('DRAW_DRONE_MOTION:mp4Disabled', 'Could not open mp4 output file. Animation will continue without saving. %s', ME.message);
          param.mp4 = false;
        end
      end

      t = logger.data(0,'t',"");
      tRealtime = tic;
      line_ref = gobjects(length(param.target),1); % 
      line_est = gobjects(length(param.target),1); % 
      line_mpc = gobjects(length(param.target),1);
      point_mpc_test = gobjects(length(param.target),1);
      line_mpc_xy = gobjects(length(param.target),1);
      point_mpc_xy = gobjects(length(param.target),1);
      mpc_data_found = false;
      mpc_data_count = zeros(length(param.target), 1);
      mpc_field_count = zeros(length(param.target), 1);
      mpc_invalid_shape_count = zeros(length(param.target), 1);
      mpc_invalid_value_count = zeros(length(param.target), 1);
      mpc_short_count = zeros(length(param.target), 1);
      hold(ax, "on");
      if plot_mpc
        ax.SortMethod = 'childorder';
        for n = 1:length(param.target)
          line_mpc(n) = plot3(ax, nan, nan, nan, ...
            'Color', param.mpc_color, ...
            'LineStyle', '-.', ...
            'LineWidth', 4, ...
            'Marker', param.mpc_marker, ...
            'MarkerSize', 7, ...
            'MarkerEdgeColor', param.mpc_color, ...
            'Clipping', 'on');
          if param.mpc_test
            point_mpc_test(n) = plot3(ax, nan, nan, nan, 'o', ...
              'MarkerSize', 8, ...
              'MarkerFaceColor', [0, 0, 0], ...
              'MarkerEdgeColor', [1, 1, 1], ...
              'Clipping', 'off');
          end
        end
      end
      if param.mpc_xy
        if isfield(param, 'ax_xy')
          ax_xy = param.ax_xy;
        else
          figure();
          ax_xy = axes();
        end
        hold(ax_xy, "on");
        grid(ax_xy, "on");
        axis(ax_xy, "equal");
        xlabel(ax_xy, "x [m]");
        ylabel(ax_xy, "y [m]");
        title(ax_xy, "MPC horizon XY");
        for n = 1:length(param.target)
          line_mpc_xy(n) = plot(ax_xy, nan, nan, ...
            'Color', param.mpc_color, ...
            'LineStyle', '-.', ...
            'LineWidth', 3, ...
            'Marker', param.mpc_marker, ...
            'MarkerSize', 6, ...
            'MarkerEdgeColor', param.mpc_color);
          point_mpc_xy(n) = plot(ax_xy, nan, nan, 'o', ...
            'MarkerSize', 7, ...
            'MarkerFaceColor', [0, 0, 0], ...
            'MarkerEdgeColor', [1, 1, 1]);
        end
      end
      if isfield(param,'Motive_ref')
        for n = 1:length(param.target)
         line_ref(n) = animatedline(ax, 'Color', 'r', 'LineStyle', '--', 'LineWidth', 1.5); % 目標軌道の描画点の制限
         if param.ref_full
           plot3(ax, r(:,1,n), r(:,2,n), r(:,3,n), ...
             'Color', 'r', 'LineStyle', '--', 'LineWidth', 1.5);
         end
         line_est(n) = animatedline(ax, 'Color', 'b', 'LineStyle', '-', 'LineWidth', 2.0);
        end
        if ~isempty(line_ref) && ~isempty(line_est)
            legend(ax, [line_ref(1), line_est(1)], {'Reference', 'Estimator'}, ...
                'Location', 'northeast', 'AutoUpdate', 'off');
        end
      end
      for i = 1:length(t)-1
        hold(ax, "on");
        if ~param.mpc_test
          if isfield(param,'Motive_ref')
            for n = 1:length(param.target)
              if ~param.ref_full
                addpoints(line_ref(n),r(i,1,n),r(i,2,n),r(i,3,n));
              end
              addpoints(line_est(n),p(i,1,n),p(i,2,n),p(i,3,n));
            end
          else
            plot3(ax,r(:,1,param.target),r(:,2,param.target),r(:,3,param.target),'k');
          end
        end
        if ~param.mpc_test && ~isempty(param.opt_plot)
          param.self.show(param.opt_plot,"logger",logger,"k",i,varargin{:});
        end
        if param.mpc_test
          for n = 1:length(param.target)
            if ~isvalid(point_mpc_test(n))
              point_mpc_test(n) = plot3(ax, nan, nan, nan, 'o', ...
                'MarkerSize', 8, ...
                'MarkerFaceColor', [0, 0, 0], ...
                'MarkerEdgeColor', [1, 1, 1], ...
                'Clipping', 'off');
            end
            set(point_mpc_test(n), 'XData', p(i,1,n), 'YData', p(i,2,n), 'ZData', p(i,3,n));
          end
        else
          if ~isvalid(obj.frame)
            obj=obj.gen_frame("target",param.target,"ax" ,ax);
          end
          obj.draw(param.target,p(i,:,param.target),Q(i,:,param.target),u(i,:,param.target));
        end
        if plot_mpc
          for n = 1:length(param.target)
            pred = [];
            controller_result = [];
            if length(logger.Data.agent) >= n && ...
                isfield(logger.Data.agent(n), 'controller') && ...
                isfield(logger.Data.agent(n).controller, 'result')
              result_log = logger.Data.agent(n).controller.result;
              if iscell(result_log)
                if size(result_log, 1) >= 1 && size(result_log, 2) >= i && isstruct(result_log{1, i})
                  controller_result = result_log{1, i};
                elseif numel(result_log) >= i && isstruct(result_log{i})
                  controller_result = result_log{i};
                end
              elseif isstruct(result_log) && numel(result_log) >= i
                controller_result = result_log(i);
              end
            end
            if isstruct(controller_result)
              controller_fields = fieldnames(controller_result);
              field_names = lower(string(controller_fields));
              if param.mpmpc
                idx = contains(field_names, "mpmpc") & contains(field_names, "pred") & contains(field_names, "pos");
              elseif param.mcmpc
                idx = contains(field_names, "mcmpc") & contains(field_names, "pred") & contains(field_names, "pos");
              else
                idx = contains(field_names, "mpc") & contains(field_names, "pred") & contains(field_names, "pos");
              end
              if any(idx)
                pred_field = controller_fields{find(idx, 1)};
                pred = controller_result.(pred_field);
                mpc_field_count(n) = mpc_field_count(n) + 1;
              end
            end
            if ~isempty(pred)
              if size(pred, 1) ~= 3 && size(pred, 2) == 3
                pred = pred';
              end
              if size(pred, 1) == 3 && size(pred, 2) >= 2
                if any(~isfinite(pred(:)))
                  mpc_invalid_value_count(n) = mpc_invalid_value_count(n) + 1;
                  pred = [];
                elseif max(vecnorm(diff(pred, 1, 2), 2, 1)) < 1e-4
                  mpc_short_count(n) = mpc_short_count(n) + 1;
                end
              else
                mpc_invalid_shape_count(n) = mpc_invalid_shape_count(n) + 1;
                pred = [];
              end
              if ~isempty(pred)
                p0 = p(i,:,n)';
                pred = p0 + param.mpc_scale * (pred - p0);
                pred = [p0, pred];
                pred(3, 2:end) = pred(3, 2:end) + param.mpc_z_offset;
                if ~isvalid(line_mpc(n))
                  hold(ax, "on");
                  line_mpc(n) = plot3(ax, nan, nan, nan, ...
                    'Color', param.mpc_color, ...
                    'LineStyle', '-.', ...
                    'LineWidth', 4, ...
                    'Marker', param.mpc_marker, ...
                    'MarkerSize', 7, ...
                    'MarkerEdgeColor', param.mpc_color, ...
                    'Clipping', 'on');
                end
                set(line_mpc(n), 'Color', param.mpc_color, 'LineStyle', '-.', ...
                  'LineWidth', 4, 'Marker', param.mpc_marker, 'MarkerEdgeColor', param.mpc_color);
                set(line_mpc(n), 'XData', pred(1,:), 'YData', pred(2,:), 'ZData', pred(3,:));
                uistack(line_mpc(n), 'top');
                if param.mpc_xy
                  if ~isvalid(line_mpc_xy(n))
                    hold(ax_xy, "on");
                    line_mpc_xy(n) = plot(ax_xy, nan, nan, ...
                      'Color', param.mpc_color, ...
                      'LineStyle', '-.', ...
                      'LineWidth', 3, ...
                      'Marker', param.mpc_marker, ...
                      'MarkerSize', 6, ...
                      'MarkerEdgeColor', param.mpc_color);
                  end
                  if ~isvalid(point_mpc_xy(n))
                    hold(ax_xy, "on");
                    point_mpc_xy(n) = plot(ax_xy, nan, nan, 'o', ...
                      'MarkerSize', 7, ...
                      'MarkerFaceColor', [0, 0, 0], ...
                      'MarkerEdgeColor', [1, 1, 1]);
                  end
                  set(line_mpc_xy(n), 'XData', pred(1,:), 'YData', pred(2,:));
                  set(point_mpc_xy(n), 'XData', p0(1), 'YData', p0(2));
                end
                mpc_data_found = true;
                mpc_data_count(n) = mpc_data_count(n) + 1;
              end
            else
              if isvalid(line_mpc(n))
                set(line_mpc(n), 'XData', nan, 'YData', nan, 'ZData', nan);
              end
              if param.mpc_xy && isvalid(line_mpc_xy(n))
                set(line_mpc_xy(n), 'XData', nan, 'YData', nan);
              end
            end
          end
          drawnow limitrate
        end
        if isfield(param,'realtime')
          delta = toc(tRealtime);
          if t(i+1)-t(i) > delta
            pause(t(i+1)-t(i) - delta);
          end
          tRealtime = tic;
        else
          pause(0.01);
        end
        if param.gif
          im = frame2im(getframe(obj.ax));
          [imind,cm] = rgb2ind(im,sizen);
          if i==1
            imwrite(imind,cm,filename,'gif', 'Loopcount',inf,'DelayTime',delaytime);
          else
            imwrite(imind,cm,filename,'gif','WriteMode','append','DelayTime',delaytime);
          end
        end
        if param.mp4
          framev = getframe(gcf);
          % target_size = [346, 400];
          % if ~isequal(size(framev.cdata, 1:2), target_size)
          %        framev.cdata = imresize(framev.cdata, target_size);
          % end 
          writeVideo(v,framev);
        end
      end
      if plot_mpc && ~mpc_data_found
        warning('DRAW_DRONE_MOTION:mpcDataNotFound', 'mpc plotting is enabled, but no *mpc*pred*pos* field was found in logger.Data.agent(*).controller.result. Re-run the simulation after enabling MPC prediction logging.');
      elseif plot_mpc
        fprintf('MPC prediction fields found: %s / %d\n', mat2str(mpc_field_count'), length(t)-1);
        fprintf('MPC prediction frames found: %s / %d\n', mat2str(mpc_data_count'), length(t)-1);
        fprintf('MPC invalid shape frames: %s\n', mat2str(mpc_invalid_shape_count'));
        fprintf('MPC invalid value frames: %s\n', mat2str(mpc_invalid_value_count'));
        fprintf('MPC too-short frames: %s\n', mat2str(mpc_short_count'));
      end
      if param.mp4
        close(v);
      end
    end
  end
end
