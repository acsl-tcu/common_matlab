function OUT = audit_CBF_dual_constraint_construction()
% AUDIT_CBF_DUAL_CONSTRAINT_CONSTRUCTION
% Read-only, reproducible snapshot audit for BOTH virtual and physical S3.
%
% Place this single .m next to the TWO supplied failure MAT files in the
% current MATLAB working directory (the source generator must be on path):
%   CBF_compare_virtual_QP_failure.mat
%   CBF_compare_physical_QP_failure.mat
% Run:
%   OUT = audit_CBF_dual_constraint_construction();
%
% It does NOT call the flight simulator, change model/controller parameters,
% rewrite the generator, or certify safety. It independently recomputes the
% frozen-state S3 h geometry, reconstructs the existing generator's HOCBF
% rows, and checks all saved QP inequalities and rank/authority signals.
%
% A mismatch means a code/data inconsistency. A PASS means only snapshot
% algebra and geometry agree; it does NOT validate against actual plant
% dynamics, sample-and-hold trajectories, or recursive feasibility.

names={'virtual','physical'};
OUT=struct();
for ii=1:numel(names)
    mode=names{ii};
    filename=['CBF_compare_' mode '_QP_failure.mat'];
    if exist(filename,'file')~=2
        error('CBF:MissingSnapshot','Missing %s in current directory %s',filename,pwd);
    end
    loaded=load(filename,'D');
    assert(isfield(loaded,'D'),'MAT file must contain D struct');
    D=loaded.D;
    assert(strcmp(char(D.method),'S3'),'This audit is specific to S3.');
    R=D.filterResult;
    A=double(D.Aineq); b=double(D.bineq(:)); w0=double(D.wNom(:));
    assert(size(A,2)==4 && numel(b)==size(A,1),'Bad QP dimensions');
    fprintf('\n=============== %s / S3, t=%.6f ===============\n',upper(mode),D.time);
    fprintf('QP rows %d; max nominal violation %.9g; exitflag %g\n', ...
        size(A,1),max(A*w0-b),R.qp.exitflag);
    fprintf('stored independent LP: exitflag=%g, slack=%.9g\n', ...
        D.independentLP.exitflag,D.independentLP.slack);
    assert(max(abs(A-double(R.map.Aineq)),[],'all')<1e-10,'Saved QP A mismatch');
    assert(max(abs(b-double(R.map.bineq(:))))<1e-10,'Saved QP b mismatch');
    fprintf('QP A column 2-norms: %s\n',mat2str(vecnorm(A,2,1),7));
    fprintf('nominal |w|_inf=%.8g\n',norm(w0,inf));

    cblabel=cell(9,1); geomErr=zeros(9,1); rowErr=zeros(9,1);
    rds=zeros(9,1); authority=zeros(9,1); tolerances=zeros(9,2);
    obs=D.obsList;
    assert(numel(obs)==3 && numel(R.rows)==9,'Expected 3 obstacles x 3 components');
    refZ=double(D.xd([3 7 11 15 19 23 27]));
    st=D.supervisor;
    assert(isfield(st,'aux') && ~isempty(st.aux),'Missing auxiliary state');
    comp={'uav','cable','load'};
    for io=1:3
        for ic=1:3
            idx=(io-1)*3+ic; part=comp{ic};
            g0=D.shapes.(part);g=double(g0(:));
            hInd=localIndependentS3(D.x,D.P,obs(io),part,g);
            e=Generate_CBF_ActuatorAware_6Methods('evaluate',mode,'S3',part, ...
                D.x,st.aux,refZ,D.F1,D.P,obs(io),g,D.kappa);
            scale=max([1 abs(e.braw) norm(e.Araw,inf)]);
            % R.rawA and R.rawb contain normalized geometric HOCBF rows.
            geomErr(idx)=abs(hInd-double(R.rows(idx).h));
            rowErr(idx)=max([norm(double(e.Araw(:))/scale-double(R.rawA(idx,:).'),inf), ...
                             abs(double(e.braw)/scale-double(R.rawb(idx)))]);
            rds(idx)=e.info.relativeDegreeAssumed;
            authority(idx)=norm(e.Araw(:),2);
            tolerances(idx,:)=[e.info.maxLowerInputDependence, e.info.lowerTolerance];
            cblabel{idx}=sprintf('obs%d/%s',io,part);
            psi=double(e.info.psiChain(:));
            fprintf('%-12s h=%+.7f RD=%d ||Araw||=%9.3g psiLowest=%+10.4g hErr=%.2g rowErr=%.2g\n',...
                cblabel{idx},hInd,rds(idx),authority(idx),min(psi),geomErr(idx),rowErr(idx));
            if e.info.maxLowerInputDependence>0
                fprintf('       lower-input-dep %.3g / tolerance %.3g (inspect when near tolerance)\n',...
                    e.info.maxLowerInputDependence,e.info.lowerTolerance);
            end
        end
    end
    fprintf('Maximum independent geometry h discrepancy: %.6g m\n',max(geomErr));
    fprintf('Maximum generator-vs-stored row discrepancy: %.6g\n',max(rowErr));

    % Independent LP with a common nonnegative additive slack in the
    % SAVED, already-normalized inequality scale.
    lp=[]; feasible=NaN;
    if exist('linprog','file')==2
        lpopt=optimoptions('linprog','Display','none');
        [z,~,flag]=linprog([zeros(4,1);1],[A -ones(size(A,1),1)],b,...
                              [],[],[],[],[-Inf(4,1);0],[],lpopt);
        if flag>0 && ~isempty(z)
            lp=z(5);
            feasible=lp<=1e-7;
            fprintf('Independent MATLAB LP: slack=%.9g, feasible=%d\n',lp,feasible);
        else
            fprintf('Independent MATLAB LP did not certify feasibility (exitflag=%g).\n',flag);
        end
    end
    if strcmp(mode,'physical')
        % The 8 physical actuator rows are *part* of the QP. Minimize
        % the offending obstacle-3/cable LHS against ONLY actuator rows.
        i=8;
        if exist('linprog','file')==2
            [wmin,val,ef]=linprog(A(i,:).',A(10:end,:),b(10:end),[],[],[],[],[],lpopt);
            if ef>0
                fprintf('obs3/cable row8: required LHS <= %.9g; actuator achievable minimum %.9g\n',b(i),val);
                fprintf('Independent violation lower bound: %.9g\n',val-b(i));
                fprintf('best w (actuator rows only)=%s\n',mat2str(wmin.',7));
            end
        end
    end
    if max(geomErr)>1e-7 || max(rowErr)>1e-7
        warning('CBF:AuditMismatch','Stored CBF geometry / generator row mismatch');
    end
    OUT.(mode)=struct('time',D.time,'stopQPExitflag',R.qp.exitflag,...
        'qpRows',size(A,1),'maxGeometryError',max(geomErr),...
        'maxHOCBFRowError',max(rowErr),'lpCommonSlack',lp,...
        'lpFeasible',feasible,'names',{cblabel},...
        'relativeDegree',rds,'inputCoefficientNorm',authority,...
        'lowerOrderInputDependence',tolerances);
end
fprintf('\nFinished: snapshots only. Safety, true relative degree, dynamic-model\n');
fprintf('equivalence and sample-data invariance MUST be checked separately.\n');
end

function h=localIndependentS3(x,P,o,part,g)
% Independent scalar support-plane geometry, no generator helper calls.
x=double(x(:));P=double(P(:));L=P(7);
p=x(8:10);a=x(14:16); % Match RAW state (do not silently normalize pT)
switch part
    case 'uav'
        c=p-L*a;S=diag(g(:).^2);
    case 'load'
        c=p;S=diag(g(:).^2);
    case 'cable'
        c=p-0.5*L*a;
        S=g(2)^2*eye(3)+(g(1)^2-g(2)^2)*(a*a.');
    otherwise
        error('Unknown S3 component');
end
r=double(o.ellipsoid_radii(:)); R=double(o.R_obs);
SO=R*diag(r.^2)*R.'; d=double(o.p_center(:))-c;
assert(norm(d)>1e-7,'Center overlap makes support direction undefined');
n=d/norm(d);
if isfield(o,'d_margin'),margin=double(o.d_margin);else,margin=0;end
h=norm(d)-sqrt(n.'*S*n)-sqrt(n.'*SO*n)-margin;
end
