function R=CHECK_TRIGGER_NATIVE_STEP_ACCOUNTING()
% Evidence-only static audit. Does not modify or rebuild native MEX.
R=struct('status','NOT_COMPLETED','reason','', ...
    'explicit_one_interval_per_hold',false, ...
    'adaptive_dp54',false,'rejection_stats_exposed',false, ...
    'contains_no_dense_output',false);
try
    src=which('TRIGGER_SYMBOLIC_NATIVE_CORE.cpp');
    if isempty(src)
        core=which('TRIGGER_SYMBOLIC_NATIVE_CORE');
        [folder,~,~]=fileparts(core);
        src=fullfile(folder,'TRIGGER_SYMBOLIC_NATIVE_CORE.cpp');
    end
    assert(isfile(src),'SourceUnavailable');
    raw=fileread(src);
    R.explicit_one_interval_per_hold=contains(raw, ...
        'integrate_one(ctx,x,times[i+1]-times[i]') && ...
        contains(raw,'original_hlc_numeric(x,references+28*i');
    R.adaptive_dp54=contains(raw,'dp54_step(ctx,state,h,next,error') && ...
        contains(raw,'if(e<=1.0)') && contains(raw,'rejections++');
    R.rejection_stats_exposed=contains(raw,'mxGetPr(plhs[3])[1]=nrejects');
    R.contains_no_dense_output=~contains(raw,'dense_output_interpolate');
    assert(R.explicit_one_interval_per_hold&&R.adaptive_dp54&& ...
        R.rejection_stats_exposed,'UnexpectedNativeIntegrationSource');
    R.status='PASS_HOLD_BOUNDARY_DP54_SOURCE_INSPECTED__NO_WCET_PROOF';
catch ME
    R.reason=[ME.identifier ' | ' ME.message];
end
try
    fid=fopen('TRIGGER_NATIVE_STEP_ACCOUNTING_RESULT.txt','w');
    if fid>0
        fprintf(fid,'status=%s\n',R.status);
        fprintf(fid,'hold_boundaries=%d adaptive_dp54=%d rejection_stats=%d\n', ...
            R.explicit_one_interval_per_hold,R.adaptive_dp54,R.rejection_stats_exposed);
        fprintf(fid,'reason=%s\n',R.reason);fclose(fid);
    end
catch
end
end
