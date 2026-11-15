function R=TEST_RA_INSTALLATION()
% Check source relocation and priorities. No Flight and no MEX required.
info=SETUP_REFERENCE_OBSTACLE_AVOIDANCE();
assert(strcmp(info.status,'NEW_FOLDER_PATH_ACTIVE'));
assert(isfolder(fullfile(info.root,'TRIGGER','MEX_ACCELERATION','SOURCE')));
assert(~isfolder(fullfile(info.root,'MEX_OPTIONAL')));
assert(startsWith(which('reference_avoidance_mex_ensure'),info.root));
assert(startsWith(which('wp_six_trigger_C_chain_numeric_kernel_v8'),info.root));
assert(startsWith(which('reference_avoidance_tick'),info.root));
R=struct('status','PASS','root',info.root,'pathsSelectedFromNewRoot',true);
end
