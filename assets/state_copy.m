function state=state_copy(orgstate,targetstate)
    % STATE_CLASS objectを値としてコピーするための関数
    if isempty(orgstate)
        state = [];
        return;
    else
    state=copy(orgstate);
    F=fieldnames(orgstate);
    for i = 1:length(F)
%        if ~strcmp(F{i},'list') && ~strcmp(F{i},'num_list') && ~strcmp(F{i},'type')
            if nargin ==2
                if ~isprop(targetstate,F{i})
                    addprop(targetstate,F{i});
                    targetstate.list = [targetstate.list, F{i}];
                    targetstate.num_list = [targetstate.num_list, length(orgstate.(F{i}))];
                end
                targetstate.set_state(F{i},orgstate.(F{i})); % qの場合をケアするためにset_stateを使う
            elseif ~strcmp(F{i},'list') && ~strcmp(F{i},'num_list') && ~strcmp(F{i},'type') && ~strcmp(F{i},'qlist')
                addprop(state,F{i});
                state.set_state(F{i},orgstate.(F{i}));
            end
%        end
    end
end
end