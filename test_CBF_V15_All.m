function result=test_CBF_V15_All(opts)
%TEST_CBF_V15_ALL R1/R2 fair predictive structural trial, C/B/D geometries.
if nargin<1,opts=struct();end
test_CBF_V15_Smoke();
result=test_CBF_V15_R1R2Predictive(opts);
end
