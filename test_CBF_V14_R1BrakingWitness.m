function report=test_CBF_V14_R1BrakingWitness(opts)
%TEST_CBF_V14_R1BRAKINGWITNESS Physical stopping witness, NOT a CBF proof.
% Integrates the same 19-state nonlinear plant under prescribed bounded
% open-loop thrust profiles.  If a stopped trajectory stays safe although
% psi1<0, that is an empirical witness of conservatism of the CURRENT
% r=2 CBF initial set.  Does not establish viability of surrounding states.
% No HLC, no obstacle, no feedback, no actuator model, no input clipping.
if nargin<1,opts=struct();end
if ~isfield(opts,'full'),opts.full=false;end
if ~isfield(opts,'dt'),opts.dt=0.01;end
if ~isscalar(opts.dt)||opts.dt<=0||opts.dt>0.05
    error('CBF:V14BrakeDt','Use 0<dt<=0.05 seconds.');
end
test_CBF_SetupPaths();s=test_CBF_Fixture();
if opts.full
    ids=[0 2 3];alts=[0.35 0.50];vels=[-0.25 -0.65 -1.2];rises=[0 0.15 0.50];
else
    ids=[0 2];alts=[0.35 0.50];vels=[-0.25 -0.65];rises=[0.15 0.50];
end
shapes={'C','B','D'};R=cell(0,18);n=0;
for id=ids
    for altitude=alts
        for vz=vels
            for si=1:3
                shape=shapes{si};
                for rise=rises
                    status='TEST_ERROR';errorText='';minH=inf;minTension=inf;
                    minInputMargin=inf;stopT=NaN;completed=0;endVz=NaN;
                    psi12=NaN;psi30=NaN;initialH=NaN;quaternionErr=0;
                    try
                        if id==0,[x,u0]=CBF_V12_PureHover(s);
                        else,[x,u0]=CBF_V11_Examples(id,s,'sphere',100);end
                        x(10)=altitude;x(13)=vz;
                        % Only take input commands INSIDE physical bounds.
                        T0=u0(1);Tmax=s.input.thrustMax;
                        lo=[s.input.thrustMin;s.input.torqueMin(:)];
                        hi=[s.input.thrustMax;s.input.torqueMax(:)];
                        if any(u0<lo-1e-10)||any(u0>hi+1e-10)
                            error('CBF:V14InitialInputOutOfBounds',...
                               'Initial T,tau do not satisfy physical input limits.');
                        end
                        obs=struct('type','sphere',...
                          'center',[100;0;altitude],'radius',0.25);
                        [H,labels]=CBF_V11_JetRows('R1',shape,x,u0,s,obs,2);
                        ind=strncmp(labels,'ground_',7);
                        if ~any(ind)
                            error('CBF:V14MissingGround','No ground rows found.');
                        end
                        hh=H(ind,1);dh=H(ind,2);
                        psi12=min(dh+1.2*hh);psi30=min(dh+3.0*hh);
                        initialH=min(hh);minH=initialH;
                        if minH<0
                            status='INITIAL_GEOMETRIC_COLLISION';
                        else
                            status='NOT_STOPPED_WITHIN_HORIZON';
                            steps=ceil(1.1/opts.dt);
                            for step=1:steps
                                t=(step-1)*opts.dt;
                                xn=x;
                                for sub=1:4
                                    ts=t+(sub-1)*opts.dt/4;
                                    hhStep=opts.dt/4;
                                    us=localInput(ts,T0,Tmax,rise,u0);
                                    um=localInput(ts+hhStep/2,T0,Tmax,rise,u0);
                                    ue=localInput(ts+hhStep,T0,Tmax,rise,u0);
                                    if any([us um ue]<lo-1e-9,'all') || ...
                                            any([us um ue]>hi+1e-9,'all')
                                        error('CBF:V14BrakingInputBounds',...
                                            'Prescribed command violates physical limits.');
                                    end
                                    minInputMargin=min(minInputMargin,min([us-lo;hi-us]));
                                    k1=localRHS(xn,us,s);
                                    k2=localRHS(xn+hhStep*k1/2,um,s);
                                    k3=localRHS(xn+hhStep*k2/2,um,s);
                                    k4=localRHS(xn+hhStep*k3,ue,s);
                                    xn=xn+(hhStep/6)*(k1+2*k2+2*k3+k4);
                                    Hk=CBF_V11_JetRows('R1',shape,xn,ue,s,obs,1);
                                    minH=min(minH,min(Hk(ind,1)));
                                    x29=[xn;ue(1);zeros(3,1);ue(2:4);zeros(3,1)];
                                    dg=cbf_model_diagnostics('real',x29,s);
                                    minTension=min(minTension,dg.tension);
                                    quaternionErr=max(quaternionErr,dg.quaternionNormError);
                                    if any(~isfinite(xn)) || minH<-1e-7
                                        status='NONLINEAR_GROUND_COLLISION';break;
                                    elseif ~isfinite(dg.tension)||dg.tension<=0
                                        status='CABLE_TENSION_FAILURE';break;
                                    elseif dg.quaternionNormError>1e-4 || ...
                                             dg.cableDirectionNormError>1e-4 || ...
                                             dg.cableAngularOrthogonality>1e-4
                                        status='STATE_MANIFOLD_DRIFT';break;
                                    elseif xn(13)>=0
                                        status='OBSERVED_STOP_WITH_BOUNDED_INPUT';
                                        stopT=ts+hhStep;break;
                                    end
                                end
                                x=xn;completed=step;
                                if ~strcmp(status,'NOT_STOPPED_WITHIN_HORIZON'),break;end
                            end
                        end
                        endVz=x(13);
                    catch ME
                        status='TEST_ERROR';errorText=[ME.identifier ': ' ME.message];
                    end
                    n=n+1;
                    R(n,:)={id,altitude,vz,shape,rise,status,stopT,completed,...
                        initialH,psi12,psi30,minH,minTension,minInputMargin,...
                        endVz,quaternionErr,double(strcmp(status,...
                        'OBSERVED_STOP_WITH_BOUNDED_INPUT')&&(psi12<0||psi30<0)),errorText};
                end
            end
        end
    end
end
report=cell2table(R,'VariableNames',...
 {'stateId','startAltitude_m','startVz_m_s','shape','thrustRiseTime_s',...
 'status','observedStopTime_s','completedSteps','initialGroundH',...
 'initialPsiGain1p2','initialPsiGain3p0','minimumGroundH',...
 'minimumTension_N','minimumPhysicalInputMargin','finalVz_m_s',...
 'maximumQuaternionNormError','safeStopFromCurrentHOCBFInvalidSet','errorText'});
out=CBF_V11_OutputFolder();
writetable(report,fullfile(out,'v14_r1_bounded_braking_witness.csv'));
fprintf('[V14] R1 independent nonlinear stopping witness: %d conditions. NOT a CBF controller.\n',height(report));
end

function u=localInput(t,T0,Tmax,rise,u0)
if rise==0
    f=1;
else
    w=min(1,max(0,t/rise));
    f=10*w^3-15*w^4+6*w^5;
end
u=u0;u(1)=T0+(Tmax-T0)*f;
end

function dx=localRHS(x,u,s)
Z=CBF_V11_RealTaylor('R1',x,u,s,1);dx=Z(:,2);
end
