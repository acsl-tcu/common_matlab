function [summary,detail]=test_CBF_V14_VirtualDegeneracy(opts)
%TEST_CBF_V14_VIRTUALDEGENERACY Independently re-audit V13 virtual HOCBF.
% In particular, a near-zero ground-drone row with positive b must never
% be called feasible just because an unbounded solver returns ||nu||~1e14.
% Uses the corrected V12 Taylor jets. No core CBF or HLC files are changed.
if nargin<1,opts=struct();end
if ~isfield(opts,'full'),opts.full=false;end
if opts.full,ids=0:4;else,ids=[0 2 3 4];end
if ~isfield(opts,'zeroActionAbsTol'),opts.zeroActionAbsTol=1e-10;end
if ~isfield(opts,'positiveRhsTol'),opts.positiveRhsTol=1e-7;end
if ~isfield(opts,'extremeDerivativeNorm'),opts.extremeDerivativeNorm=1e8;end
s=test_CBF_Fixture();test_CBF_SetupPaths();
shapes={'C','B','D'};kinds={'sphere','ellipsoid'};distances=[1 100];
S=cell(0,15);Drows=cell(0,17);ns=0;nr=0;
for id=ids
    if id==0,[x19,u]=CBF_V12_PureHover(s);
    else,[x19,u]=CBF_V11_Examples(id,s,'sphere',1);end
    Z=CBF_V11_RealTaylor('R1',x19,u,s,6);
    xv=zeros(20,1);
    for j=0:5,xv(3*j+1:3*j+3)=factorial(j)*Z(8:10,j+1);end
    for si=1:3
        for ti=1:2
            for di=1:2
                shape=shapes{si};kind=kinds{ti};dist=distances(di);
                [~,~,ob]=CBF_V11_Examples(1,s,kind,dist);
                st='TEST_ERROR';err='';minPsi=NaN;problemRows=NaN;
                extremeRows=NaN;maxNeeded=NaN;maxEarly=NaN;maxCurv=NaN;
                lowerBound=NaN;degenerateDrone=NaN;
                try
                    C=CBF_V13_VirtualCoefficientAudit(shape,xv,s,ob);
                    G=CBF_V14_RowGuard(C.A,C.b,opts);
                    minPsi=min(C.psiMin);problemRows=sum(G.unresolvedPositiveRhs);
                    extremeRows=sum(G.extremeDerivativeRequirement);
                    maxNeeded=max(G.minRequiredDerivativeNorm2);
                    maxEarly=C.maxEarly;maxCurv=C.maxCurvature;
                    lowerBound=max([0;G.minRequiredDerivativeNorm2]);
                    idx=find(strcmp(C.labels,'ground_drone'));
                    if numel(idx)~=1
                        error('CBF:V14DroneLabel','Expected one drone ground row.');
                    end
                    degenerateDrone=double(G.unresolvedPositiveRhs(idx));
                    if G.hasUnresolved
                        st='UNRESOLVED_POSITIVE_RHS_NO_SAFE_QP_CLAIM';
                    elseif C.maxEarly>1e-7 || C.maxCurvature>1e-5
                        st='INPUT_STRUCTURE_NOT_AFFINE';
                    elseif minPsi < -1e-9
                        st='INITIAL_AUXILIARY_SET_INVALID';
                    elseif G.hasExtreme
                        st='EXTREME_DERIVATIVE_REQUIREMENT';
                    else
                        st='NO_NEAR_ZERO_POSITIVE_ROW_NOT_SAFETY_PROOF';
                    end
                    for ri=1:numel(C.labels)
                        % Probe sensitivity of the ORIGINAL Taylor derivative
                        % under +/-1 and +/-10 in each input component.
                        % These are finite differences, not exact Lie derivatives.
                        input3Slope1=NaN;input3Slope10=NaN;
                        if strcmp(C.labels{ri},'ground_drone')
                            rr=C.orders(ri);
                            ee=[0;0;1;0];
                            hp=CBF_V12_VirtualRows(shape,xv,ee,s,ob);
                            hm=CBF_V12_VirtualRows(shape,xv,-ee,s,ob);
                            input3Slope1=factorial(rr)*(hp(ri,rr+1)-hm(ri,rr+1))/2;
                            hp=CBF_V12_VirtualRows(shape,xv,10*ee,s,ob);
                            hm=CBF_V12_VirtualRows(shape,xv,-10*ee,s,ob);
                            input3Slope10=factorial(rr)*(hp(ri,rr+1)-hm(ri,rr+1))/20;
                        end
                        nr=nr+1;
                        Drows(nr,:)={id,shape,kind,dist,C.labels{ri},C.orders(ri),...
                            C.H0(ri),C.psiMin(ri),C.b(ri),G.rowNormInf(ri),...
                            G.minRequiredDerivativeNorm2(ri),...
                            double(G.nearZeroAction(ri)),double(G.unresolvedPositiveRhs(ri)),...
                            double(G.extremeDerivativeRequirement(ri)),...
                            input3Slope1,input3Slope10,''};
                    end
                catch ME
                    st='TEST_ERROR';err=[ME.identifier ': ' ME.message];
                end
                ns=ns+1;
                S(ns,:)={id,shape,kind,dist,st,minPsi,problemRows,extremeRows,...
                    maxNeeded,maxEarly,maxCurv,degenerateDrone,lowerBound,...
                    opts.zeroActionAbsTol,err};
            end
        end
    end
end
summary=cell2table(S,'VariableNames',...
 {'stateId','shape','obstacleType','distance','status','minimumInitialPsi',...
  'unresolvedPositiveRows','extremeRequirementRows',...
  'maximumRowRequiredNorm2','maxEarlierInputAction','maxInputCurvature',...
  'droneGroundDegeneratePositive','maxRequiredNorm2',...
  'numericalActionTolerance','errorText'});
detail=cell2table(Drows,'VariableNames',...
 {'stateId','shape','obstacleType','distance','barrier','order','geometricH',...
  'minimumInitialPsi','rhsB','rowActionInf','minRequiredNu2',...
  'numericallyNearZero','positiveRhsAndNearZero','extremeRequirement',...
  'droneGroundSlopeUnitProbe','droneGroundSlopeTenProbe','errorText'});
out=CBF_V11_OutputFolder();
writetable(summary,fullfile(out,'v14_virtual_degeneracy_summary.csv'));
writetable(detail,fullfile(out,'v14_virtual_degeneracy_rows.csv'));
fprintf('[V14] Virtual numerical degeneracy: %d cases; %d rows. No constraints dropped.\n',...
 height(summary),height(detail));
end
