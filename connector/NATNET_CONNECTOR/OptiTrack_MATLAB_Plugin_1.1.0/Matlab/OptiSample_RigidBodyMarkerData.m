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

% Optitrack Sample for Rigid Body Marker Data by Streaming ID
%  Suggested Versions:
%   - OptiTrack Motive 3.0 or later
%   - OptiTrack NatNet 4.0 or later
%   - Last Developed in MATLAB R2024b
% This sample connects to the server and displays rigid body data.
% natnet.p, needs to be located on the Matlab Path.
% This sample provides the marker data information for a single rigid body
% as an example of obtaining singlular marker data

function MarkerDataSample
	fprintf('NatNet Marker Data Sample\n')

	%Create NatNet Object Instance
	fprintf('Creating NatNet object\n')
	natnetclient = natnet;
	connection = natnetclient.ConnectToNatNet('127.0.0.1', '127.0.0.1', 'Multicast');
	if connection < 1
		return
	end
	
	%Get Asset MetaData
	model = natnetclient.getModelDescription;

	% Poll for the asset data at regular intervals (~1 sec) for 10 sec.
	streamingID = input("Asset Streaming ID: ", "s");
	modelMatch = findMatchingModelID(model, streamingID);
	if modelMatch == -1
		fprintf("Error: Streaming ID Not Found");
		return
	end
	fprintf( '\nPrinting frame data approximately every second for 10 seconds...\n\n' )
	for idx = 1:10
		java.lang.Thread.sleep( 996 );
		markerData = natnetclient.getFrameLabeledMarker;
		if (model.TrackingModelCount < 1)
			fprintf( '\tPacket is empty/stale\n' )
			fprintf( '\tMake sure the server is in Live mode or playing in playback\n\n')
			return
		end
		fprintf('---------------------------------Frame Start---------------------------------\n')
		fprintf( '\nFrame:%6d  ' , markerData.Timestamp)
		fprintf( 'Time:%0.2f\n' , markerData.Timestamp )
		fprintf( 'Asset Name: "%s"\n', model.RigidBody(modelMatch).Name)
		getMarkerFromAsset(markerData, streamingID)
		fprintf('---------------------------------Frame End-----------------------------------\n')
	end
end

function getMarkerFromAsset(markerData, streamingID) 
	if markerData.MarkerCount < 1
	fprint("Error: No Labeled Markers")
	return 
	end
	fprintf("Asset Markers List \n")
	for i=1:markerData.MarkerCount 
		if markerData.LabeledMarker(i).AssetID == str2double(streamingID)
			fprintf('Asset ID: "%d"\n', markerData.LabeledMarker(i).AssetID)
			fprintf('Member ID: "%d"\n', markerData.LabeledMarker(i).MemberID)
			fprintf('X:%0.1fmm  ', markerData.LabeledMarker(i).X * 1000)
			fprintf('Y:%0.1fmm  ', markerData.LabeledMarker(i).Y * 1000)
			fprintf('Z:%0.1fmm  \n', markerData.LabeledMarker(i).Z * 1000)
			fprintf('Residual:%0.1fmm\n', markerData.LabeledMarker(i).Residual *1000)
			fprintf('---------------------------------\n')
		end
	end
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