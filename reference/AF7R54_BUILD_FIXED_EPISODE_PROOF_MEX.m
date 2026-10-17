function OUT = AF7R54_BUILD_FIXED_EPISODE_PROOF_MEX()
clc;
fprintf('\n====================================================================================================\n');
fprintf(' AF7R54B | BUILD FIXED-EPISODE + OUTWARD-KKT SHADOW MEX\n');
fprintf('====================================================================================================\n');
fprintf('[AUTHORITY] productionModified=0 | fixedDomainEpisode=1 | noSampledAcceptance=1\n');

here=fileparts(mfilename('fullpath'));
a=fullfile(here,'af7r54_fixed_episode_mex.cpp');
b=fullfile(here,'af7r54_interval_kkt_mex.cpp');
assert(isfile(a)&&isfile(b));

try, eval('clear af7r54_fixed_episode_mex af7r54_interval_kkt_mex'); catch, end
if ispc
    mex('-R2018a','-O','COMPFLAGS="$COMPFLAGS /std:c++17"', ...
        ['-I' here],'-output','af7r54_fixed_episode_mex','-outdir',here,a);
    mex('-R2018a','-O','COMPFLAGS="$COMPFLAGS /std:c++17"', ...
        '-output','af7r54_interval_kkt_mex','-outdir',here,b);
else
    mex('-R2018a','-O','CXXFLAGS="$CXXFLAGS -std=c++17"', ...
        ['-I' here],'-output','af7r54_fixed_episode_mex','-outdir',here,a);
    mex('-R2018a','-O','CXXFLAGS="$CXXFLAGS -std=c++17"', ...
        '-output','af7r54_interval_kkt_mex','-outdir',here,b);
end
rehash;
assert(exist('af7r54_fixed_episode_mex','file')==3);
assert(exist('af7r54_interval_kkt_mex','file')==3);

fprintf(' fixed-episode MEX                       = %s\n',which('af7r54_fixed_episode_mex'));
fprintf(' interval-KKT MEX                        = %s\n',which('af7r54_interval_kkt_mex'));
fprintf(' production source modified             = 0\n');
fprintf('====================================================================================================\n');
OUT=struct('schema','AF7R54B_BUILD_V1','closed',true);
assignin('base','AF7R54B_BUILD_OUT',OUT);
end
