function OUT = AF7R53A_PRE_RUN_AUDIT()
clc;
fprintf('\n====================================================================================================\n');
fprintf(' AF7R53A | PREPARED-COMMIT PRE-RUN AUDIT\n');
fprintf('====================================================================================================\n');
src=which('REPLANNING_SYSTEM');assert(~isempty(src)&&isfile(src));
expected='c16b244794fc7bc6f2d7794f81fd83d2f62944def60fd606454fe472fc012383';h=sha256_file(src);assert(strcmp(h,expected));
raw=fileread(src);compact=regexprep(raw,'\s+','');
run=fileread(which('run_flight_headless'));

C=struct();
C.marker=contains(raw,'AF7R53-PREPARED-COMMIT-SWITCH');
C.switch_property=contains(raw,'ENABLE_AF7R53_PREPARED_COMMIT = false;');
C.constructor_option=contains(raw,'isfield(opts,''ENABLE_AF7R53_PREPARED_COMMIT'')');
C.prepared_method=contains(raw,'try_commit_reference_only_proposal_af7r53');
C.legacy_path=contains(raw,'af7r53_legacy_commit_count');
C.static_cache=contains(raw,'af7r53_prepare_static_packet_cache');
C.same_final_trigger_assert=contains(raw,'obj.assert_trigger_packet_certificate_current();');
C.same_final_reference_assert=contains(raw,'obj.assert_repaired_reference_current();');
C.hard4ms=contains(raw,'hard_modify_deadline_s<=4.0e-3');
C.r9=contains(raw,'R9_LOCAL_VECTOR_COUPLED_ADAPTIVE_BERNSTEIN_FLATNESS_SPEED');
C.wholebody=contains(raw,'MIDPOINT_STATIC_SUPPORT_MINUS_SPEED_TIMES_HALFWIDTH');
C.sampled_true=contains(compact,[char(39) 'sampled_acceptance' char(39) ',true']);
C.mex=(exist('af7r53_prepared_commit_mex','file')==3);
C.run_switch=contains(run,'replan_opts.ENABLE_AF7R53_PREPARED_COMMIT = ENABLE_AF7R53_PREPARED_COMMIT;');
mc=meta.class.fromName('REPLANNING_SYSTEM');
C.class_parse=~isempty(mc);

closed=C.marker&&C.switch_property&&C.constructor_option&&C.prepared_method&& ...
    C.legacy_path&&C.static_cache&&C.same_final_trigger_assert&& ...
    C.same_final_reference_assert&&C.hard4ms&&C.r9&&C.wholebody&& ...
    ~C.sampled_true&&C.mex&&C.run_switch&&C.class_parse;

F=fieldnames(C);
for k=1:numel(F)
    fprintf(' %-42s = %d\n',strrep(F{k},'_',' '),logical(C.(F{k})));
end
fprintf(' source SHA-256                         = %s\n',h);
fprintf(' AF7R53A pre-run closed                = %d\n',closed);
fprintf('====================================================================================================\n');
assert(closed,'[AF7R53A] pre-run audit failed.');
OUT=struct('schema','AF7R53A_PRE_RUN_AUDIT_V1','closed',closed,'checks',C,'sha256',h);
assignin('base','AF7R53A_PRE_RUN_AUDIT_OUT',OUT);
end

function h=sha256_file(fp)
fid=fopen(fp,'rb');assert(fid>=0);c=onCleanup(@()fclose(fid));
b=fread(fid,Inf,'*uint8');clear c;
md=java.security.MessageDigest.getInstance('SHA-256');md.update(b);
d=typecast(md.digest(),'uint8');h=lower(reshape(dec2hex(d,2).',1,[]));
end
