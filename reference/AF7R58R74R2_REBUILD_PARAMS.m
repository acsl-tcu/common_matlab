function params=AF7R58R74R2_REBUILD_PARAMS(cls,order,vals,tag)
% Reconstruct a detached parameter object from the SAME Flight event.
% DRONE_PARAM_SUSPENDED_LOAD constructor:
%   (name, type, param.mass, param.Lx, ..., param.cableL)
% where param.* are NAME-VALUE arguments, not a third positional struct.
% Do not insert physical defaults or replace any missing authority.
assert(strcmp(char(string(cls)),'DRONE_PARAM_SUSPENDED_LOAD'), ...
    '[R74R2] unsupported parameter class: no inferred replacement.');
order=string(order(:));vals=double(vals(:));
assert(numel(order)==numel(vals) && ~isempty(order) && ...
    all(isfinite(vals)) && numel(unique(order))==numel(order), ...
    '[R74R2] event parameter names/values are incomplete or invalid.');
nv=cell(1,2*numel(order));
for j=1:numel(order)
    nm=char(order(j));
    assert(isvarname(nm),'[R74R2] invalid source parameter name.');
    nv{2*j-1}=nm;
    nv{2*j}=vals(j);
end
% name and type are the ONLY two positional inputs. Expand the event
% parameter struct into genuine (name,value) arguments; do not pass a
% scalar struct as a positional third argument (MATLAB rejects "row").
params=feval(char(string(cls)),char(string(tag)),"row",nv{:});
neworder=string(params.parameter_order(:));
newvals=double(params.parameter(:));
assert(isequal(neworder,order) && isequaln(newvals,vals), ...
    '[R74R2] reconstructed object differs from the event authority.');
end
