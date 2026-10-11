function CBF6_BenchmarkLogger(csvFile,row)
% CBF6_BenchmarkLogger Append one runtime benchmark sample for S1..K3.
%
% Required row fields are filled with NaN/empty defaults if omitted.
% Typical use inside the controller/simulation loop:
%
%   row.MethodID='S3';
%   row.Scenario='CASE18';
%   row.TimeSec=t;
%   row.ObstacleID=obsId;
%   row.ConstraintCount=size(Acbf,1);
%   row.CbfEvalSec=tCbf;
%   row.AuxUpdateSec=tAux;
%   row.QpSolveSec=tQp;
%   row.MinH=min(hvec);
%   row.MinGeomDistance=dmin;
%   row.InterventionNorm=norm(vSafe-vNom);
%   row.TrackingErrorNorm=norm(pL-pRef);
%   row.ExitFlag=exitflag;
%   CBF6_BenchmarkLogger(csvFile,row);
%
% Support-plane-specific fields:
%   SupportNormError = abs(norm(n)-1)
%   SupportNdotNorm  = norm(ndot)
%
% KKT-specific fields:
%   KKTResidualNorm = norm(F)
%   KKTJzRcond      = rcond(Jz)
%   KKTMinLambda    = min([lamA,lamB])

arguments
    csvFile (1,:) char
    row struct
end

names={ ...
    'MethodID','Scenario','TimeSec','ObstacleID','ConstraintCount', ...
    'CbfEvalSec','AuxUpdateSec','QpSolveSec','MinH','MinGeomDistance', ...
    'InterventionNorm','TrackingErrorNorm','ExitFlag','MaxActuatorRatio', ...
    'KKTResidualNorm','KKTJzRcond','KKTMinLambda', ...
    'SupportNormError','SupportNdotNorm','Notes'};

defaults={"","",NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN,""};
for i=1:numel(names)
    if ~isfield(row,names{i}) || isempty(row.(names{i}))
        row.(names{i})=defaults{i};
    end
end

T=table(string(row.MethodID),string(row.Scenario),double(row.TimeSec),double(row.ObstacleID), ...
    double(row.ConstraintCount),double(row.CbfEvalSec),double(row.AuxUpdateSec),double(row.QpSolveSec), ...
    double(row.MinH),double(row.MinGeomDistance),double(row.InterventionNorm),double(row.TrackingErrorNorm), ...
    double(row.ExitFlag),double(row.MaxActuatorRatio),double(row.KKTResidualNorm),double(row.KKTJzRcond), ...
    double(row.KKTMinLambda),double(row.SupportNormError),double(row.SupportNdotNorm),string(row.Notes), ...
    'VariableNames',names);

if exist(csvFile,'file')
    writetable(T,csvFile,'WriteMode','append','WriteVariableNames',false);
else
    parent=fileparts(csvFile); if ~isempty(parent) && ~exist(parent,'dir'), mkdir(parent); end
    writetable(T,csvFile);
end
end
