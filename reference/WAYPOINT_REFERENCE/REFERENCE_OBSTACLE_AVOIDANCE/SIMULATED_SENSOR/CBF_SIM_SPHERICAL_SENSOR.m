classdef CBF_SIM_SPHERICAL_SENSOR < handle
    % CBF_SIM_SPHERICAL_SENSOR - GUI-native spherical obstacle sensor.
    % Uses unchanged ra_sim_sensor_detect() from WAYPOINT_REFERENCE.
    % Only adds cbfObservedWorld/cbfSensorReport to sensor results.
    % It does not replace MOTIVE.output, control, plant or reference.

    properties (SetAccess = private)
        self
        world0
        config
        result
    end

    methods
        function obj = CBF_SIM_SPHERICAL_SENSOR(self, world0, config)
            if ~isstruct(world0) || ~isfield(world0, 'baseParts')
                error('CBFSensor:World', ...
                    'world0 must be DRONE_OBSTACLES output.');
            end
            if ~isstruct(config) || ~isfield(config, 'radiusM')
                error('CBFSensor:Config', ...
                    'Use ra_sim_sensor_config(radiusM).');
            end
            obj.self = self;
            obj.world0 = world0;
            obj.config = config;
            obj.result = struct('cbfObservedWorld', [], ...
                'cbfSensorReport', [], 'cbfSensorTime', NaN);
        end

        function result = do(obj, time, varargin) %#ok<INUSD>
            % Called in SimExp's normal SENSOR stage, before estimator.
            % The first argument is the TIME object used by the simulation.
            if nargin < 2 || isempty(time) || ~isprop(time, 't')
                error('CBFSensor:Time', ...
                    'Sensor.do requires the simulator TIME object.');
            end
            currentTime = double(time.t);
            if ~isscalar(currentTime) || ~isfinite(currentTime)
                error('CBFSensor:Time', 'Invalid simulation time.');
            end

            % Actual plant state: no estimated position/future truth.
            trueDronePosition = obj.self.plant.state.p;
            worldNow = obs_world_at_time(obj.world0, currentTime);
            [observed, report] = ra_sim_sensor_detect( ...
                worldNow, trueDronePosition, obj.config);

            % Distinct names prevent overwriting MOTIVE result.output/state.
            obj.result.cbfObservedWorld = observed;
            obj.result.cbfSensorReport = report;
            obj.result.cbfSensorTime = currentTime;
            result = obj.result;
        end
    end
end
