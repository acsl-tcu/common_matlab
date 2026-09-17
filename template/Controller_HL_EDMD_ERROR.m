function Controller = Controller_HL_EDMD_ERROR(dt, model_file, state_source)
% Controller_HL_EDMD_ERROR
% Parameter generator for HLC_EDMD_ERROR.

    if nargin < 2 || isempty(model_file)
        model_file = 'C:\Users\student\Documents\GitHub\common_matlab\mode\KMPC\KQLMPC\hl_edmd_error_model_plant.mat';
    end
    if nargin < 3 || isempty(state_source)
        state_source = 'estimator';
    end

    Controller = Controller_HL(dt);
    Controller.comp.model_file = model_file;
    Controller.comp.state_source = state_source;
    Controller.comp.alpha = 1.0;
    Controller.comp.du_max = [0.6; 0.12; 0.12; 0.12];
    Controller.comp.beta_z = 0.25;
    Controller.comp.beta_att = 0.20;
end
