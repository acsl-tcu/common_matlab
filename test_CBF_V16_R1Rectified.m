function tbl=test_CBF_V16_R1Rectified(opts)
%TEST_CBF_V16_R1RECTIFIED Compare Ong ReCBF vs ordinary R1 HOCBF.
if nargin<1,opts=struct();end
if ~isfield(opts,'full'),opts.full=false;end
test_CBF_SetupPaths();s=test_CBF_Fixture();
if opts.full,ids=[0 2 3 4];zSet=[0.15 0.22 0.35 0.50 1.2];vzSet=[-0.65 -0.25 0 0.25];
else,ids=[0 2 3];zSet=[0.15 0.22 0.50];vzSet=[-0.65 0];end
shapes={'C','B','D'};G=[1.2 3.0];R=cell(0,16);n=0;
for id=ids
 for z=zSet
  for vz=vzSet
   for shape=shapes
    for k=G
      try
       if id==0,[x,~]=CBF_V12_PureHover(s);
       else,[x,~]=CBF_V11_Examples(id,s,'sphere',100);end
       x(10)=z;x(13)=vz;
       o=CBF_V16_R1_RectifiedGround(shape{1},x,s,struct('k',k,'kR',k,'rectCoeff',0.5));
       status='CHECK_CBF_AND_INITIAL_SET';err='';
       n=n+1;R(n,:)={id,z,vz,shape{1},k,o.nRows,...
        o.minGeometricH,o.minHOCBFPsi1,o.minRectifiedH,...
        double(o.hocbfInitialValid),double(o.rectifiedInitialValid),...
        o.hocbfQPResidual,o.rectifiedQPResidual,double(o.affinePass),...
        double(o.sampledWeakDegreeConditionViolated),err};
      catch ME
       n=n+1;R(n,:)={id,z,vz,shape{1},k,0,NaN,NaN,NaN,...
          0,0,NaN,NaN,0,NaN,[ME.identifier ':' ME.message]};
      end
    end
   end
  end
 end
end
tbl=cell2table(R,'VariableNames',{'stateId','loadAltitude_m','vz_m_s','shape','gain',...
 'groundRows','minGeometricH','minHOCBFPsi1','minRectifiedH',...
 'hocbfInitialValid','rectifiedInitialValid','hocbfQPResidual','rectifiedQPResidual',...
 'affineCheck','sampledWeakDegreeViolation','errorText'});
writetable(tbl,fullfile(CBF_V11_OutputFolder(),'v16_r1_ground_rectified_vs_hocbf.csv'));
fprintf('[V16 R1] Rectified/HOCBF ground grid %d rows. Global CBF theorem NOT checked.\n',height(tbl));
end
