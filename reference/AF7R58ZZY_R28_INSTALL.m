function OUT=AF7R58ZZY_R28_INSTALL()
% Transactional install of R28 native-once development path.
clc;fprintf('\n====================================================================================================\n AF7R58ZZY-R28 | TRANSACTIONAL NATIVE-ONCE INSTALL\n====================================================================================================\n');
fprintf('[AUTHORITY] generatorMathChanged=0 | certificateMathChanged=0 | nativeRepresentationAdded=1 | scheduleChanged=0 | controllerModified=0\n');
EXP=struct('prod','310cfdd01f9554a80b71e8d90464816a721859709ad7c4b9137cdc0f3a6eaf49','r27runner','5f0e4ad0230d4e5cebcbf9de2ad1525d454a6fe178a9694c9323eebed5aa048c','r27class','be3bd20dd0c1acc71d1968727124918b9300b6542d97b197510e4b89442f06da', ...
 'cpp','4f9778723aa3aa339d5167b90845fcfcaa8564cae68326ff1e4ed504c7e6daea','hpp','ad5ce1970583c0313637dcac58b8cf949092fe53e201efc61e55754fb2ea79be','r25helper','8be00803a58c2563e12ab4ef0f2e666a7161304e596724f39e616e6cc90efb44','r27helper','2a008e5ac8abf248416ffe0ecef9ab0ebb71dac28901aa6c9467b9a823772533', ...
 'r28runner','4122e1079eaa703d113a522099ef6aa5e376ed42c79c33e226ff61ccafe35c3a','r28class','4edda21ff4a071754005a2c3aba174210d923542bf6b390ce2a21185f3faee2e','r28cpp','bd4474a41ae93eba3e42f152ae8ba582ddf0dd59a89853fa07a90f79fbe5520a','r28hpp','ad5ce1970583c0313637dcac58b8cf949092fe53e201efc61e55754fb2ea79be', ...
 'report','f104d8f8d37e64ea070caa59e270a54e91ca0acfc65d762fb60253794268d5fa','selftest','dae3094b64d53135566fb3fb5df01e1b7289bb06968ae8cb020b43ca2fd716e4','launcher','ecb618749dd6c1a46ebf828d395a53a44ca8b4efd3eabc1c346ad1f6a839fa98');
pkg=fileparts(mfilename('fullpath')); root=find_root(EXP); ref=fullfile(root,'reference');
C=struct('runner',fullfile(root,'run_flight_headless.m'),'prod',fullfile(ref,'REPLANNING_SYSTEM.m'), ...
 'r27',fullfile(root,'REPLANNING_SYSTEM_R27_LEAN_EXACT.m'),'cpp',fullfile(ref,'af7r36_single_core_mex.cpp'), ...
 'hpp',fullfile(ref,'af7r36_core.hpp'),'r25helper',fullfile(root,'af7r77_compile_correction_packet_R25.m'),'helper',fullfile(root,'af7r78_incremental_compile_R27.m'), ...
 'mex',fullfile(ref,['af7r36_single_core_mex.' mexext]));
cd(root); addpath(ref,'-begin'); addpath(root,'-begin'); rehash;
fprintf('\n[A. EXACT CURRENT R27 / NATIVE SOURCE BINDING]\n');
show('runner',C.runner,EXP.r27runner);show('R27 class',C.r27,EXP.r27class);show('production',C.prod,EXP.prod);show('AF7R36 cpp',C.cpp,EXP.cpp);show('AF7R36 hpp',C.hpp,EXP.hpp);show('R25 packet helper',C.r25helper,EXP.r25helper);show('R27 helper',C.helper,EXP.r27helper);
assert(paths_equal(which('af7r77_compile_correction_packet_R25'),C.r25helper),'[R28 INSTALL] R25 helper path shadowed.');
assert(paths_equal(which('af7r78_incremental_compile_R27'),C.helper),'[R28 INSTALL] R27 helper path shadowed.');
assert(exist('af7r77_compile_dcp_mex','file')==3,'[R28 INSTALL] R25 DCP reference MEX missing.');
assert(isfile(C.mex),'[R28 INSTALL] current AF7R36 MEX missing.');

% Package exact bytes.
pCpp=fullfile(pkg,'af7r36_single_core_mex.cpp');pHpp=fullfile(pkg,'af7r36_core.hpp');pClass=fullfile(pkg,'REPLANNING_SYSTEM_R28_NATIVE_ONCE.m');
pRunner=fullfile(pkg,'run_flight_headless_R28_FINAL_SOURCE.m');pReport=fullfile(pkg,'AF7R58ZZY_R28_REPORT.m');pSelf=fullfile(pkg,'AF7R58ZZY_R28_SELFTEST.m');pLaunch=fullfile(pkg,'AF7R58ZZY_R28_RUN_CANONICAL.m');
assert(strcmp(sha_file(pCpp),EXP.r28cpp)&&strcmp(sha_file(pHpp),EXP.r28hpp)&&strcmp(sha_file(pClass),EXP.r28class)&&strcmp(sha_file(pRunner),EXP.r28runner)&&strcmp(sha_file(pReport),EXP.report)&&strcmp(sha_file(pSelf),EXP.selftest)&&strcmp(sha_file(pLaunch),EXP.launcher),'[R28 INSTALL] package byte authority mismatch.');

stamp=char(datetime('now','Format','yyyyMMdd_HHmmss_SSS')); backup=fullfile(root,['AF7R58ZZY_R28_BACKUP_' stamp]);[ok,msg]=mkdir(backup);assert(ok,'%s',msg);
if isfile(C.runner),copyfile(C.runner,fullfile(backup,'1_run_flight_headless.m'));end
if isfile(C.r27),copyfile(C.r27,fullfile(backup,'2_REPLANNING_SYSTEM_R27_LEAN_EXACT.m'));end
if isfile(C.cpp),copyfile(C.cpp,fullfile(backup,'3_af7r36_single_core_mex.cpp'));end
if isfile(C.hpp),copyfile(C.hpp,fullfile(backup,'4_af7r36_core.hpp'));end
if isfile(C.mex),copyfile(C.mex,fullfile(backup,['5_af7r36_single_core_mex.' mexext]));end
OUT=struct('schema','AF7R58ZZY_R28_INSTALL_V1','closed',false,'backup',backup,'production_modified',false,'controller_modified',false);
modified=false;
try
 fprintf('\n[B. BUILD SHADOW TEST MEX -- NO LIVE SOURCE CHANGE]\n');
 builddir=fullfile(root,['AF7R58ZZY_R28_BUILD_' stamp]);mkdir(builddir);testName='af7r36_single_core_mex_R28_TEST';
 build_mex(pCpp,pkg,builddir,testName);addpath(builddir,'-begin');rehash;
 T=AF7R58ZZY_R28_SELFTEST(testName,pClass);assert(T.closed);OUT.shadow_selftest=T;
 rmpath(builddir);rehash;

 fprintf('\n[C. INSTALL EXACT R28 SOURCES / RUNNER]\n');
 copybin(pCpp,C.cpp);copybin(pHpp,C.hpp);copybin(pClass,fullfile(root,'REPLANNING_SYSTEM_R28_NATIVE_ONCE.m'));copybin(pRunner,C.runner);copybin(pReport,fullfile(root,'AF7R58ZZY_R28_REPORT.m'));copybin(pSelf,fullfile(root,'AF7R58ZZY_R28_SELFTEST.m'));copybin(pLaunch,fullfile(root,'AF7R58ZZY_R28_RUN_CANONICAL.m'));modified=true;
 assert(strcmp(sha_file(C.runner),EXP.r28runner)&&strcmp(sha_file(C.cpp),EXP.r28cpp),'[R28 INSTALL] installed source mismatch.');

 fprintf('\n[D. BUILD LIVE AF7R36 MEX]\n');
 try, clear af7r36_single_core_mex; catch, end
 build_mex(C.cpp,ref,ref,'af7r36_single_core_mex');rehash;addpath(ref,'-begin');addpath(root,'-begin');
 assert(paths_equal(which('af7r36_single_core_mex'),C.mex),'[R28 INSTALL] live AF7R36 MEX path is shadowed.');
 rev=af7r36_single_core_mex(5);assert(ischar(rev)&&strcmp(rev,'AF7R58ZZY_R28_NATIVE_ONCE_V1'),'[R28 INSTALL] live MEX revision mismatch.');
 T2=AF7R58ZZY_R28_SELFTEST('af7r36_single_core_mex',fullfile(root,'REPLANNING_SYSTEM_R28_NATIVE_ONCE.m'));assert(T2.closed);OUT.live_selftest=T2;

 fprintf('\n[E. CLASS / RUNNER PARSE & IMMUTABILITY]\n');
 try, clear REPLANNING_SYSTEM_R28_NATIVE_ONCE; catch, end;rehash;mc=meta.class.fromName('REPLANNING_SYSTEM_R28_NATIVE_ONCE');assert(~isempty(mc),'[R28 INSTALL] class parse failed.');
 R=fileread(C.runner);assert(count(R,'REPLANNING_SYSTEM_R28_NATIVE_ONCE(agent,nominal_ref,replan_opts)')==1);
 assert(~contains(R,'REPLANNING_SYSTEM_R27_LEAN_EXACT(agent,nominal_ref,replan_opts)'));
 assert(strcmp(sha_file(C.prod),EXP.prod),'[R28 INSTALL] production MATLAB source changed.');
 fprintf(' R28 class parse                         = PASS\n');fprintf(' production MATLAB source modified       = 0\n');fprintf(' controller source modified              = 0\n');
 OUT.closed=true;OUT.status='R28_INSTALLED__RUN_CANONICAL_NEXT';
catch ME
 fprintf('\n[R28 INSTALL FAILURE] %s\n',ME.message);
 if modified
  try
   try, clear af7r36_single_core_mex; catch, end
   restore(backup,C);rehash;fprintf(' rollback                                = PASS\n');
  catch MER, fprintf(' rollback                                = FAILED: %s\n',MER.message);end
 end
 OUT.error=struct('identifier',ME.identifier,'message',ME.message);assignin('base','AF7R58ZZY_R28_INSTALL_OUT',OUT);rethrow(ME);
end
assignin('base','AF7R58ZZY_R28_INSTALL_OUT',OUT);
fprintf('\n====================================================================================================\n R28 INSTALL COMPLETE\n====================================================================================================\n');
fprintf('status                                  = %s\n',OUT.status);fprintf('backup                                  = %s\n',backup);fprintf('NEXT: AF7R58ZZY_R28_RUN_CANONICAL\nThen: AF7R58ZZY_R28_REPORT\n');fprintf('====================================================================================================\n');
end

function root=find_root(EXP)
starts=[{pwd,fileparts(mfilename('fullpath'))},strsplit(path,pathsep)];cand={};
for k=1:numel(starts),p=starts{k};if isempty(p)||~isfolder(p),continue;end;for j=1:7,cand{end+1}=p;q=fileparts(p);if isempty(q)||strcmp(q,p),break;end;p=q;end;end;cand=unique(cand);good={};
for k=1:numel(cand),r=cand{k};p=fullfile(r,'reference','REPLANNING_SYSTEM.m');rr=fullfile(r,'run_flight_headless.m');c=fullfile(r,'REPLANNING_SYSTEM_R27_LEAN_EXACT.m');if isfile(p)&&isfile(rr)&&isfile(c)&&strcmp(sha_file(p),EXP.prod)&&strcmp(sha_file(rr),EXP.r27runner)&&strcmp(sha_file(c),EXP.r27class),good{end+1}=r;end;end
assert(~isempty(good),'[R28 INSTALL] exact canonical R27 root not found.');score=zeros(size(good));for k=1:numel(good),[~,n]=fileparts(good{k});score(k)=100*strcmpi(n,'common_matlab')+20*isfolder(fullfile(good{k},'.git'));end;[~,i]=max(score);root=good{i};
end
function build_mex(src,inc,outdir,name)
if ispc,mex('-R2018a','-O','COMPFLAGS="$COMPFLAGS /std:c++17"',['-I' inc],'-output',name,'-outdir',outdir,src);else,mex('-R2018a','-O','CXXFLAGS="$CXXFLAGS -std=c++17"',['-I' inc],'-output',name,'-outdir',outdir,src);end
rehash;assert(isfile(fullfile(outdir,[name '.' mexext])),'[R28 INSTALL] MEX build output missing.');
end
function restore(b,C)
map={'1_run_flight_headless.m',C.runner;'3_af7r36_single_core_mex.cpp',C.cpp;'4_af7r36_core.hpp',C.hpp;['5_af7r36_single_core_mex.' mexext],C.mex};for k=1:size(map,1),p=fullfile(b,map{k,1});if isfile(p),copyfile(p,map{k,2},'f');end;end
end
function show(n,p,h),assert(isfile(p),'missing %s',p);q=sha_file(p);fprintf(' %-38s SHA=%s | exact=%d\n',n,q,strcmp(q,h));assert(strcmp(q,h),'[R28 INSTALL] %s SHA mismatch.',n);end
function n=getname(p),[~,a,e]=fileparts(p);n=[a e];end
function copybin(a,b),f=fopen(a,'rb');assert(f>=0);c=onCleanup(@()fclose(f));x=fread(f,Inf,'*uint8');clear c;f=fopen(b,'wb');assert(f>=0);c=onCleanup(@()fclose(f));assert(fwrite(f,x,'uint8')==numel(x));end
function tf=paths_equal(a,b),if isempty(a)||isempty(b),tf=false;return;end;try,a=char(java.io.File(a).getCanonicalPath());b=char(java.io.File(b).getCanonicalPath());catch,end;if ispc,tf=strcmpi(a,b);else,tf=strcmp(a,b);end;end
function h=sha_file(p),f=fopen(p,'rb');assert(f>=0);c=onCleanup(@()fclose(f));b=fread(f,Inf,'*uint8');md=java.security.MessageDigest.getInstance('SHA-256');md.update(typecast(b,'int8'));d=typecast(md.digest(),'uint8');h=lower(reshape(dec2hex(d,2).',1,[]));end
