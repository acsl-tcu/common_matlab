function report=test_CBF_AuditGroundRegion20261112(options)
% 地面高・上下速度・姿勢・紐傾斜・ゲインの可行/不可行マップ。
% バリアの安全集合と入力制限の可行性は別々に判定する。
% 楽観的停止距離は平衡鉛直単純質点の参考値であり、実モデルの可到達集合ではない。
if nargin<1,options=struct();end
paths=test_CBF_SetupPaths();settings=test_CBF_Fixture();
if isfield(options,'settings'),settings=options.settings;end
full=getopt(options,'full',true);
if full
    altitudes=[.5 .75 1.2 1.8 3.0];
    vxs=[-1.2 -.65 -.25 0 .25 .6];
    pitches=[0 .22];tilts=[0 .13]; gains=[1.2 2 3];
else
    altitudes=[.75 1.2 2];vxs=[-.65 0 .4];
    pitches=0;tilts=0;gains=[1.2 3];
end
altitudes=getopt(options,'altitudes',altitudes);
vxs=getopt(options,'verticalSpeeds',vxs);
pitches=getopt(options,'pitchAngles',pitches);
tilts=getopt(options,'cableTilts',tilts);
gains=getopt(options,'groundGains',gains);
rows=cell(0,18);idx=0;
for g=gains(:).'
 for p=pitches(:).'
  for tilt=tilts(:).'
   for z=altitudes(:).'
    for vz=vxs(:).'
     idx=idx+1;hMin=NaN;psiMin=NaN;gOK=false;iOK=false;
     gf=false;inf_=false;gi=false;allf=false;ten=NaN;
     err='';stopDist=NaN;optimistic=false;
     try
         s=settings;s.cbf.groundGain=g;
         x=CBF_Audit_State('real',s,z,vz,0,p,tilt,0,0);
         obstacle=CBF_Audit_State('obstacle','sphere',x(8:10),100,'front',0);
         qp=cbf_hocbf_scene('real','B',x,s,{obstacle});
         gg=contains(qp.labels,'ground_');
         ii=contains(qp.labels,'_thrust_') | contains(qp.labels,'_tau_');
         gOK=all(qp.psiInitiallyNonnegative(gg));
         iOK=all(qp.psiInitiallyNonnegative(ii));
         rg=CBF_Audit_LP(qp,gg);ri=CBF_Audit_LP(qp,ii);
         rgi=CBF_Audit_LP(qp,gg|ii);ra=CBF_Audit_LP(qp);
         gf=rg.feasible;inf_=ri.feasible;gi=rgi.feasible;allf=ra.feasible;
         psiMin=min(cellfun(@min,qp.psi(gg)));
         raw=cbf_evaluate_case('real','B','sphere',x,s,obstacle);
         hMin=min(raw(end-3:end));
         diag_=cbf_model_diagnostics('real',x,s);ten=diag_.tension;
         % 鉛直静止姿勢で推力が瞬時にTmaxに達する理想化した制動距離。
         aOptim=s.input.thrustMax/(s.physical.massDrone+s.physical.massLoad)-s.physical.gravity;
         if abs(p)<1e-12 && abs(tilt)<1e-12 && aOptim>0
             stopDist=max(0,-vz)^2/(2*aOptim);
             optimistic=hMin>=stopDist;
         end
     catch ME
         err=ME.identifier;
         if isempty(err),err=ME.message;end
     end
     rows(end+1,:)={idx,z,vz,p,tilt,g,hMin,psiMin,gOK,iOK, ...
         gf,inf_,gi,allf,ten,stopDist,optimistic,err}; %#ok<AGROW>
     if ~isempty(err)
         % エラーは捨てない。別テーブルではなく、コマンドウィンドウに残す。
         fprintf('[ground region error] case=%d %s\n',idx,err);
     end
    end
   end
  end
 end
end
report=cell2table(rows,'VariableNames',{'caseId','loadAltitude','verticalSpeed',...
    'pitch','cableTilt','groundGain','minGroundGeometry',...
    'minGroundPsi','groundPsiOK','inputPsiOK','groundOnlyFeasible',...
    'inputOnlyFeasible','groundAndInputFeasible','allWithFarObstacleFeasible',...
    'tension','optimisticStoppingDistance','optimisticStoppingCondition','errorStatus'});
folder=fullfile(paths.generator,'CBF','test_results');if ~exist(folder,'dir'),mkdir(folder);end
writetable(report,fullfile(folder,'audit4_ground_region.csv'));
% 2次元可視化。表示した条件以外は図に混ぜない。
try
    for g=gains(:).'
        mask=abs(report.groundGain-g)<1e-9 & abs(report.pitch)<1e-12 & ...
           abs(report.cableTilt)<1e-12;
        mat=nan(numel(vxs),numel(altitudes));
        for iz=1:numel(altitudes)
            for iv=1:numel(vxs)
                jj=find(mask & abs(report.loadAltitude-altitudes(iz))<1e-9 & ...
                    abs(report.verticalSpeed-vxs(iv))<1e-9,1);
                if ~isempty(jj)
                    mat(iv,iz)=double(report.groundPsiOK(jj))+...
                        2*double(report.groundAndInputFeasible(jj));
                end
            end
        end
        fig=figure('Visible','off');imagesc(altitudes,vxs,mat);
        set(gca,'YDir','normal');xlabel('吊り荷初期高度 [m]');
        ylabel('吊り荷初期上下速度 [m/s]');
        title(sprintf('地面初期条件+QP可行性：地面ゲイン %.2f',g));
        cb=colorbar;cb.Ticks=0:3;
        cb.TickLabels={'両方NG','初期集合のみOK','QPのみOK','両方OK'};
        exportgraphics(fig,fullfile(folder,sprintf('audit4_ground_map_gain_%g.png',g)));
        close(fig);
    end
catch ME
    fprintf('[warning] 地面マップの画像保存ができませんでした: %s\n',ME.message);
end
fprintf('[地面領域] %d条件 / 初期psiと地面+入力の両方OK: %d\n',...
    size(report,1),sum(report.groundPsiOK & report.inputPsiOK & report.groundAndInputFeasible));
end
function v=getopt(o,k,d),if isfield(o,k)&&~isempty(o.(k)),v=o.(k);else,v=d;end,end
