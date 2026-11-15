function folder=CBF_V11_OutputFolder()
%CBF_V11_OUTPUTFOLDER V7と同じCSV保存先。
common=fileparts(mfilename('fullpath'));
folder=fullfile(common,'model','substance','generate_cbf_suspended', ...
    'CBF','test_results');
if ~isfolder(folder),mkdir(folder);end
end
