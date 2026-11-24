% CBF 25ms causal dry-run research: generate rows fresh from each current log state.
% Execute ONCE in common_matlab. Does NOT alter production Sim/controller. No future-state cache is used.
% Output: one CBF_SD_TEST_RESULTS.zip beside this runner.
root=fileparts(mfilename('fullpath'));
if isempty(root),root=pwd;end
inputZip=fullfile(root,'CBF_FOURWAY_RESULTS.zip');
if ~isfile(inputZip)
    error('SD:MissingInput','Place CBF_FOURWAY_RESULTS.zip next to this runner.');
end
clear CBF_SD_TEST
result=CBF_SD_TEST('rtcausal',inputZip, ...
    'maxSamples',Inf,'repeatPasses',2,'budgetMs',3,'maxRows',128); %#ok<NASGU>
clear root inputZip result;
