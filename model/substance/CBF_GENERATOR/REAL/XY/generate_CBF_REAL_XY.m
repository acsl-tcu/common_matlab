function result = generate_CBF_REAL_XY()
% Regenerate dedicated real-input 19-state order-4 jet AND 3D cable HOCBF.
% Generated file names remain stable; never overwrite other CBF methods.
root=fileparts(mfilename('fullpath'));
destDir=fullfile(root,'GENERATED');
if ~isfolder(destDir),mkdir(destDir);end
pairs={ ...
    'CBF_REAL_XY_JET_SOURCE.m','CBF_REAL_XY_JET.m'; ...
    'CBF_REAL_XY_EVAL_SOURCE.m','CBF_REAL_XY_EVAL.m'};
result=struct('generated',{{}},'order',4,'coordinateDimension',3, ...
    'controlledInputIndices',[2;3],'groundConstraints',false);
for j=1:size(pairs,1)
    from=fullfile(root,pairs{j,1});to=fullfile(destDir,pairs{j,2});
    if ~isfile(from),error('CBFXY:MissingSource','Missing %s',from);end
    src=fileread(from);
    old=erase(pairs{j,1},'.m');new=erase(pairs{j,2},'.m');
    if ~contains(src,['function ']) || isempty(regexp(src, ...
            ['(?m)^function\s+\w+\s*=\s*' old '\('],'once'))
        error('CBFXY:Source','Unexpected signature in %s',from);
    end
    txt=strrep(src,old,new);
    fid=fopen(to,'w');
    if fid<0,error('CBFXY:Write','Cannot write %s',to);end
    c=onCleanup(@()fclose(fid));
    fprintf(fid,'%s',txt);clear c
    result.generated{end+1}=to; %#ok<AGROW>
    fprintf('[CBF XY GENERATE] %s\n',to);
end
rehash
end
