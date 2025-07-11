%=============================================================================
% Copyright © 2025 NaturalPoint, Inc. All Rights Reserved.
% 
%  THIS SOFTWARE IS GOVERNED BY THE OPTITRACK PLUGINS EULA AVAILABLE AT https://www.optitrack.com/about/legal/eula.html 
%  AND/OR FOR DOWNLOAD WITH THE APPLICABLE SOFTWARE FILE(S) (“PLUGINS EULA”). BY DOWNLOADING, INSTALLING, ACTIVATING 
%  AND/OR OTHERWISE USING THE SOFTWARE, YOU ARE AGREEING THAT YOU HAVE READ, AND THAT YOU AGREE TO COMPLY WITH AND ARE
%  BOUND BY, THE PLUGINS EULA AND ALL APPLICABLE LAWS AND REGULATIONS. IF YOU DO NOT AGREE TO BE BOUND BY THE PLUGINS
%  EULA, THEN YOU MAY NOT DOWNLOAD, INSTALL, ACTIVATE OR OTHERWISE USE THE SOFTWARE AND YOU MUST PROMPTLY DELETE OR
%  RETURN IT. IF YOU ARE DOWNLOADING, INSTALLING, ACTIVATING AND/OR OTHERWISE USING THE SOFTWARE ON BEHALF OF AN ENTITY,
%  THEN BY DOING SO YOU REPRESENT AND WARRANT THAT YOU HAVE THE APPROPRIATE AUTHORITY TO ACCEPT THE PLUGINS EULA ON
%  BEHALF OF SUCH ENTITY. See license file in root directory for additional governing terms and information.
%=============================================================================

% Optitrack Sample for Rigid Body Pose Data
%  Suggested Versions:
%   - OptiTrack Motive 3.0 or later
%   - OptiTrack NatNet 4.0 or later
%   - Last Developed in MATLAB R2024b
% This sample connects to the server and plots position and rotation of a
% single rigid body.
% natnet.m, needs to be located on the Matlab Path.

function NatNetEventHandlerSample
    
	global hf1 a1 a2
	global px py pz
	global rx ry rz
    
	fprintf( '\nNatNet Event Handler Sample Start\n' )
   	fprintf( '=========================================================\n' )
    pause(2);
    
    CreatePlots;

	% create an instance of the natnet client class
	fprintf( 'Creating natnet class object\n' )
	natnetclient = natnet;

	% connect the client to the server (multicast over local loopback) -
	% modify for your network
	connection = natnetclient.ConnectToNatNet('127.0.0.1', '127.0.0.1', 'Multicast');
	if connection < 1
		return
	end

	% add some callback functions that execute autmatically in an
	% asynchronous manner to the event of a new frame of mocap data,
	% similar to how software interrupts operate.
	% callback functions block further execution of running code and a event handler buffer is created
	% with pending function callbacks to execute. Matlab by default is single threaded. 
	% callbacks are added to the event handler buffer in no particular order.
	fprintf( 'Adding callback functions to execute with each frame of mocap\n' )
	addpath( 'event handlers')
	% first input is the listener slot and the second is the function
	% name, which must be an m file on the Matlab Path or current folder.
	natnetclient.addlistener( 1 , 'plotposition' );
	natnetclient.addlistener( 2 , 'plotrotation' );
	natnetclient.addlistener( 3 , 'consoleprint' );
	
	
    
    
	% by default listeners/interrupts are disabled in the natnet class.
	% enable all listeners, 0, or individual listeners, 1, 2, etc.
	% Similiar for disabling
	fprintf( 'Enabling the listeners for asynchronous callback execution\n' )
    natnetclient.enable(0)

    % Run and execute eventhandler queues until a key is pressed to exit pause.
	fprintf( '(Enter any key to quit)\n\n' )
    pause;

    % When a key is pressed, disable execution of further callbacks and
    % exit out of the program.
    fprintf( '=========================================================\n' )
    fprintf( 'Disabling the listeners\n')
    natnetclient.disable(0)
    disp('NatNet Event Handler Sample End' )
    pause(1)
    close(hf1)

end


function CreatePlots
	% sets up two plots for viewing rigid body data, position and rotation
	global hf1 a1 a2
	% making animated lines global so they can be accessed in the
	% callback functions
	global px py pz
	global rx ry rz
    
	% create a figure which will contain two subplots
	hf1 = figure;
	hf1.WindowStyle = 'docked';

	% plot and animated line for position
	a1 = subplot( 1,2,1 );
	title( 'Position' );
	xlabel( 'Frame' )
	ylabel( 'Position (m)' )

	px = animatedline;
	px.MaximumNumPoints = 1000;
	px.Marker = '.';
	px.LineWidth = 0.5;
	px.Color = [ 1 0 0 ];

	py = animatedline;
	py.MaximumNumPoints = 1000;
	py.LineWidth = 0.5;
	py.Color = [ 0 1 0 ];
	py.Marker = '.';

	pz = animatedline;
	pz.MaximumNumPoints = 1000;
	pz.LineWidth = 0.5;
	pz.Color = [ 0 0 1 ];
	pz.Marker = '.';

	% plot and animated line for rotation
	a2 = subplot( 1,2,2 );
	title( 'Rotation' );
	xlabel( 'Frame' )
	ylabel( 'Rotation (deg)' )

	rx=animatedline;
	rx.MaximumNumPoints = 1000;
	rx.Marker = '.';
	rx.LineWidth = 0.5;
	rx.Color = [ 1 0 0 ];

	ry = animatedline;
	ry.MaximumNumPoints = 1000;
	ry.Marker = '.';
	ry.LineWidth = 0.5;
	ry.Color = [ 0 1 0 ];

	rz = animatedline;
	rz.MaximumNumPoints = 1000;
	rz.Marker = '.';
	rz.LineWidth = 0.5;
	rz.Color = [ 0 0 1];
    
end

