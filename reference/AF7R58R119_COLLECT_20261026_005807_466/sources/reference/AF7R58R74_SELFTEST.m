function T=AF7R58R74_SELFTEST()
% Frozen-parameter replay unit test. No HLC/plant run or Flight mutation.
S=AF7R58R74_SOURCE_CHECK();
assert(S.matlab_class_parser_passed);
P=DRONE_PARAM_SUSPENDED_LOAD('R74_TEST_ONLY');
order=string(P.parameter_order(:));
values=double(P.parameter(:));
assert(numel(order)==numel(values)&&numel(order)>0);
raw=P.parameter(:);
% Call the SAME event-bound rebuild as production postflight preview.
% This would have caught the R74 third-positional-struct constructor bug.
R=AF7R58R74R2_REBUILD_PARAMS(class(P),order,values,'R74_TEST_FROZEN');
assert(isequal(string(R.parameter_order(:)),order) && ...
    isequaln(double(R.parameter(:)),values));
ix=find(strcmpi(order,'loadmass'));
assert(numel(ix)==1);
R.set("loadmass",values(ix)+0.1234);
assert(isequaln(double(P.parameter(:)),raw));
assert(~isequaln(double(R.parameter(:)),raw));
T=struct('status','PASS_R74R2_NAMED_ARGUMENT_REBUILD_NO_LIVE_MUTATION', ...
    'parser_pass',true,'param_order_preserved',true, ...
    'event_params_not_mutated',true,'physical_future_proved',false, ...
    'natural_return_committed',false);
end
