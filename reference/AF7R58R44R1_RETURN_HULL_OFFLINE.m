function R=AF7R58R44R1_RETURN_HULL_OFFLINE(src)
% Compatibility repair: do not load a nonexistent R44R1 snapshot.
% If R44R2 already saved a real snapshot, let the original R44R1 command
% read it. Never synthesize a missing polygon or claim whole-body safety.
if ischar(src) || (isstring(src) && isscalar(src))
    fn=char(src);
    if ~isfile(fn)
        [folder,name,~]=fileparts(fn);
        if strcmp(name,'AF7R58R44R1_ACTIVE_SNAPSHOT')
            if isempty(folder)
                folder=fileparts(mfilename('fullpath'));
            end
            next=fullfile(folder,'AF7R58R44R2_ACTIVE_SNAPSHOT.mat');
            if isfile(next)
                src=next;
            end
        end
    end
end
R=AF7R58R44R2_RETURN_HULL(src);
end
