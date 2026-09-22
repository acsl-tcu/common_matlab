app = SimExp();
app.model_name = 'SuspendedLoad';
app.start_app();
pause(2); % Wait for initialization
% Modify to use REPLANNING_BSPLINE_HLC
try
    app.agent.reference.name = 'REPLANNING_BSPLINE_HLC';
    disp('Successfully set reference to REPLANNING_BSPLINE_HLC');
    
    % Let's run a few steps
    app.time.t = 0;
    app.do_calculation();
    disp('First calculation step successful.');
catch e
    disp(['Error: ' e.message]);
end
quit
