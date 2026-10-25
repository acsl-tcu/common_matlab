function S=AF7R58R86_SOURCE_CHECK()
% Verify R86 is only ONE new read-only method + one R83 method call update.
root=fileparts(mfilename('fullpath'));
target=fullfile(root,'REPLANNING_SYSTEM.m');
patch=fullfile(root,'REPLANNING_SYSTEM_R86_PATCHED_SOURCE.txt');
oldHash='57e60163d054c3c102573496cb9457c5ece155c4f59a6c47acd1dc6c9a4bbff5';
newHash='e55452c44b4eb77fbf33f1f422422b9b223d2844f85a1666a57178f07670deaa';
assert(isfile(target)&&isfile(patch),'R86:SourceMissing');
h=AF7R58R86_SHA256_FILE(target);
assert(strcmp(h,oldHash)||strcmp(h,newHash),'R86:ProductionHashDifferent');
assert(strcmp(AF7R58R86_SHA256_FILE(patch),newHash), ...
    'R86:PackageHashDifferent');
Z=fileread(patch);A=fileread(target);
a='    methods (Access = public)';
b='        function A = r86_live_serialized_c7_authority(obj,tglobal)';
c='        function R = r83_live_r81_natural_return_reference_only(obj,tglobal,opts)';
newCall='A=obj.r86_live_serialized_c7_authority(double(tglobal));';
oldCall='A=obj.r63_live_c7_authority(double(tglobal));';
assert(numel(strfind(Z,b))==1&&numel(strfind(Z,newCall))==1, ...
    'R86:UnexpectedMethodTopology');
assert(numel(strfind(Z,'function R = r83_live_r81_natural_return_reference_only'))==1, ...
    'R86:LostR83Method');
% Reconstruct old class exactly, removing the newly inserted method and
% reverting precisely the ONE call in the existing R83 method.
locs=strfind(Z,a); ib=strfind(Z,b);ic=strfind(Z,c);
assert(numel(ib)==1&&numel(ic)==1&&ib<ic&& ...
    any(locs<ib),'R86:MethodMarkersNotUnique');
ia=locs(find(locs<ib,1,'last'));
restored=[Z(1:ia-1) a sprintf('\n') Z(ic:end)];
assert(numel(strfind(restored,newCall))==1, ...
    'R86:UnexpectedAdapterChange');
restored=strrep(restored,newCall,oldCall);
if strcmp(h,oldHash)
    assert(isequal(restored,A),'R86:NonAdditiveProductionChange');
else
    assert(isequal(Z,A),'R86:InstalledBytesNotExact');
end
S=struct('status','PASS_R86_NATIVE_C7_SOURCE_BOUND', ...
    'source_before_install',strcmp(h,oldHash), ...
    'source_already_patched',strcmp(h,newHash), ...
    'only_readonly_method_and_r83_adapter_changed',true, ...
    'production_execution_certified',false, ...
    'production_trigger_changed',false, ...
    'initial_trajectory_generator_changed',false);
end
