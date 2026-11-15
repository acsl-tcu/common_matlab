function report=test_CBF_V12_VirtualTaylorParity(opts)
%TEST_CBF_V12_VIRTUALTAYLORPARITY Probe omitted pL5/pL6 Taylor terms.
% Ground truth: exact physical 19-state Taylor jets for a held physical input.
% Old: installed cbf_virtual_state_taylor(xv,nu,P,4).
% Fixed: independent CBF_V12_VirtualJetFixed(xv,nu,P,4).
% This is a numerical parity test, NOT a closed-loop safety proof.
if nargin<1,opts=struct();end
test_CBF_SetupPaths();s=test_CBF_Fixture();P=cbf_parameter_vector(s);
if isfield(opts,'full') && opts.full, ids=0:4; distances=[1 100];
else, ids=[0 1 2 3 4];distances=100;end
shapes={'C','B','D'};types={'sphere','ellipsoid'};
rows=cell(0,15);row=0;
for id=ids
    if id==0
        [x19,u]=CBF_V12_PureHover(s);
    else
        [x19,u]=CBF_V11_Examples(id,s,'sphere',1);
    end
    Z=CBF_V11_RealTaylor('R1',x19,u,s,6);
    xv=zeros(20,1);
    for k=0:5
        xv(3*k+1:3*k+3)=factorial(k)*Z(8:10,k+1);
    end
    nu=[factorial(6)*Z(8:10,7);0];
    for i=1:numel(shapes)
      for t=1:numel(types)
       for distance=distances
        shape=shapes{i};kind=types{t};caseName=sprintf('physical_%d',id);
        status='ERROR';qOld=NaN;qFix=NaN;hOld=NaN;hFix=NaN;
        Aold=NaN;Afix=NaN;inputDelta=NaN;errorText='';
        try
            [~,q0,d0]=cbf_virtual_state_taylor(xv,nu,P,4);
            [p,q1,d1]=CBF_V12_VirtualJetFixed(xv,nu,P,4);
            qPhys=Z(14:16,1:5);
            dPhys=Z(8:10,1:5)-s.physical.cableLength*qPhys;
            qOld=norm(q0-qPhys,Inf);
            qFix=norm(q1-qPhys,Inf);
            [~,~,ob]=CBF_V11_Examples(1,s,kind,distance);
            O=cbf_obstacle_vector(ob,ob.type);plan=cbf_choose_cable_spheres(s);
            Hp=cbf_geometry_taylor(shape,kind,Z(8:10,1:5),qPhys,dPhys,P,O,plan,'all');
            Ho=cbf_geometry_taylor(shape,kind,Z(8:10,1:5),q0,d0,P,O,plan,'all');
            Hf=cbf_geometry_taylor(shape,kind,p,q1,d1,P,O,plan,'all');
            hOld=norm(Ho(:)-Hp(:),Inf);
            hFix=norm(Hf(:)-Hp(:),Inf);
            % Derivative wrt the vertical sixth derivative. Raw K4
            % implementation should not depend on this decision at all.
            dn=zeros(4,1);dn(3)=1e-2;
            [po,qo,do]=cbf_virtual_state_taylor(xv,nu+dn,P,4);
            HoP=cbf_geometry_taylor(shape,kind,po,qo,do,P,O,plan,'all');
            [po,qo,do]=cbf_virtual_state_taylor(xv,nu-dn,P,4);
            HoM=cbf_geometry_taylor(shape,kind,po,qo,do,P,O,plan,'all');
            Aold=norm((HoP(:,5)-HoM(:,5))/(2*dn(3))*factorial(4),Inf);
            [po,qo,do]=CBF_V12_VirtualJetFixed(xv,nu+dn,P,4);
            HfP=cbf_geometry_taylor(shape,kind,po,qo,do,P,O,plan,'all');
            [po,qo,do]=CBF_V12_VirtualJetFixed(xv,nu-dn,P,4);
            HfM=cbf_geometry_taylor(shape,kind,po,qo,do,P,O,plan,'all');
            Afix=norm((HfP(:,5)-HfM(:,5))/(2*dn(3))*factorial(4),Inf);
            inputDelta=Afix-Aold;
            if hFix <= 1e-6*(1+norm(Hp(:),Inf)) && ...
               qFix <= 1e-6*(1+norm(qPhys(:),Inf))
                status='FIXED_MATCHES_PHYSICAL';
            else
                status='FIXED_PARITY_NOT_CONFIRMED';
            end
        catch ME
            errorText=sprintf('%s: %s',ME.identifier,ME.message);
        end
        row=row+1;
        rows(row,:)={caseName,shape,kind,distance,status, ...
            qOld,qFix,hOld,hFix,Aold,Afix,inputDelta, ...
            norm(nu(1:3),Inf),u(1),errorText};
       end
      end
    end
end
report=cell2table(rows,'VariableNames',...
 {'caseName','shape','obstacleType','distance','status',...
  'oldCableJetError','fixedCableJetError',...
  'oldGeometryJetError','fixedGeometryJetError',...
  'oldFourthInputNorm','fixedFourthInputNorm',...
  'fourthInputChange','physicalLoadSixthNorm','physicalThrust','errorText'});
writetable(report,fullfile(CBF_V11_OutputFolder(),'v12_virtual_taylor_parity.csv'));
fprintf('[V12] 仮想4階幾何・6階入力のTaylor整合 %d行 (本体無変更)\n',height(report));
end
