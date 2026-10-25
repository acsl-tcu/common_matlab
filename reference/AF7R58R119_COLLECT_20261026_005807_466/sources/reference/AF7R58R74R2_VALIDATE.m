function V=AF7R58R74R2_VALIDATE()
% Run without clear classes or changing other system modules.
V.source=AF7R58R74_SOURCE_CHECK();
V.selftest=AF7R58R74_SELFTEST();
assert(strcmp(V.selftest.status, ...
    'PASS_R74R2_NAMED_ARGUMENT_REBUILD_NO_LIVE_MUTATION'));
V.status='R74R2_STRUCTURAL_AND_DETACHED_PARAMETER_TEST_PASS__PHYSICAL_FUTURE_UNPROVED';
V.no_hlc_plant_ekf_or_runner_modified=true;
V.physical_future_certified=false;
V.natural_return_committed=false;
end
