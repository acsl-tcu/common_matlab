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

% Optitrack Sample for Rigid Body Pose Data by Streaming ID
%  Suggested Versions:
%   - OptiTrack Motive 3.0 or later
%   - OptiTrack NatNet 4.0 or later
%   - Last Developed in MATLAB R2024b
% This sample takes in a streaming ID for a Rigid Body and obtains all
% relevant data for the rigid body orientation and location. 

function RigidBodyStreaming
	fprintf( 'Rigid Body Polling by Streaming ID\n' )

	% create an instance of the natnet client class
	fprintf( 'Creating natnet class object\n' )
	natnetclient = natnet;

	% connect the client to the server (multicast over local loopback) -
	% modify for your network
	connection = natnetclient.ConnectToNatNet('127.0.0.1', '127.0.0.1', 'Multicast');
	if connection < 1
		return
	end
	% get the asset descriptions for the asset names
	model = natnetclient.getModelDescription;

	% Poll for the rigid body data a regular intervals (~1 sec) for 10 sec.
	streamingID = input("Enter Streaming ID: ", "s");
	modelMatch = findMatchingModelID(model, streamingID);
	if modelMatch < 1
		fprintf("Error: No matching streaming information");
		return
	end

	fprintf( '\nPrinting frame data approximately every second for 10 seconds...\n\n' )
	for idx = 1 : 10  
		java.lang.Thread.sleep( 996 );
		data = natnetclient.getFrame; % method to get current frame
		if (model.TrackingModelCount < 1)
			fprintf( '\tPacket is empty/stale\n' )
			fprintf( '\tMake sure the server is in Live mode or playing in playback\n\n')
			return
		end
		dataMatch = findMatchingDataID(data, streamingID);
		if dataMatch < 1
			fprintf("Error: No frame data with given streamingID.")
			return
		end

		fprintf( 'Frame:%6d  ' , data.iFrame )
		fprintf( 'Time:%0.2f\n' , data.fTimestamp )
		if(model.RigidBodyCount > 0)
			PrintRigidBodyData(model, data, modelMatch, dataMatch);
		end
	end
end

function PrintRigidBodyData(model, data, modelMatch, dataMatch)
	fprintf("Rigid Body Polling\n")
		for i = 1:model.RigidBodyCount
				fprintf( 'Name:"%s"  \n', model.RigidBody( modelMatch).Name)
				fprintf( 'Model ID:"%d"\n', model.RigidBody(modelMatch).ID)
				fprintf( 'x:%0.1fmm  ', data.RigidBodies( dataMatch ).x * 1000)
				fprintf( 'y:%0.1fmm  ', data.RigidBodies( dataMatch ).y * 1000)
				fprintf( 'z:%0.1fmm\n', data.RigidBodies( dataMatch ).z * 1000)
				fprintf( 'qx:%0.1fmm ', data.RigidBodies(dataMatch).qx * 1000)
				fprintf( 'qy:%0.1fmm ', data.RigidBodies(dataMatch).qy * 1000)
				fprintf( 'qz:%0.1fmm ', data.RigidBodies(dataMatch).qz * 1000)
				fprintf( 'qw:%0.1fmm \n', data.RigidBodies(dataMatch).qw * 1000)
		end
		fprintf("---------------------------------------------------\n")
end 

%searches the model information for a match in streamingID
function [modelMatch] = findMatchingModelID(model, streamingID)
	modelMatch = -1;
	for i = 1:model.RigidBodyCount
		if str2double(streamingID) == model.RigidBody(i).ID
			modelMatch = i;
			break
		end
	end

end

%searches the data information for a match on streamingID
function [dataMatch] = findMatchingDataID(data, streamingID)
	dataMatch = -1;
	for i=1: data.nRigidBodies
		if str2double(streamingID) == data.RigidBodies(i).ID
			dataMatch = i;
			break
		end
	end
end
