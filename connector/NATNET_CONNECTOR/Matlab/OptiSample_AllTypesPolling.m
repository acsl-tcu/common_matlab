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

% Optitrack Sample for Polling All Data Types
%  Suggested Versions:
%   - OptiTrack Motive 3.0 or later
%   - OptiTrack NatNet 4.0 or later
%   - Last Developed in MATLAB R2024b
% This sample connects to the server and displays rigid body data.
% natnet.p, needs to be located on the Matlab Path.

function NatNetPollingSample
	fprintf( 'NatNet Polling Sample Start\n' )

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
	fprintf( '\nPrinting frame data approximately every second for 10 seconds...\n\n' )
	for idx = 1 : 10  
		java.lang.Thread.sleep( 996 );
		data = natnetclient.getFrame; % method to get current frame
		if (model.TrackingModelCount < 1) 
			fprintf( '\tPacket is empty/stale\n' )
			fprintf( '\tMake sure the server is in Live mode or playing in playback\n\n')
			return
		end

		fprintf( 'Frame:%6d  ' , data.iFrame )
		fprintf( 'Time:%0.2f\n' , data.fTimestamp )
		PrintCameraData(model, data)

		if(model.RigidBodyCount > 0)
			PrintRigidBodyData(model, data)
		end

		if(model.SkeletonCount > 0)
			PrintSkeletonData(model, data)
		end

		if(model.ForcePlateCount > 0)
			PrintForcePlateData(model, data)
		end

		if(model.DeviceCount > 0)
			PrintDeviceData(model, data)
		end

		if(model.MarkerSetCount > 0)
			PrintMarkersetData(model, data)
		end

		if(model.TrainedMarkersetCount > 0)
			PrintTrainedMarkersetData(model, data)
		end
fprintf("\n-----------------------Frame End-----------------------------\n")
end
disp('NatNet Polling Sample End' )
end
function PrintRigidBodyData(model, data)
fprintf("Rigid Body Polling\n")
    for i = 1:model.RigidBodyCount
			fprintf( 'Name:"%s"  ', model.RigidBody( i ).Name )
			fprintf( 'x:%0.1fmm  ', data.RigidBodies( i ).x * 1000 )
			fprintf( 'y:%0.1fmm  ', data.RigidBodies( i ).y * 1000 )
			fprintf( 'z:%0.1fmm\n', data.RigidBodies( i ).z * 1000 )
			fprintf( 'qx:%0.1fmm ', data.RigidBodies(i).qx * 1000)
			fprintf( 'qy:%0.1fmm ', data.RigidBodies(i).qy * 1000)
			fprintf( 'qz:%0.1fmm ', data.RigidBodies(i).qz * 1000)
			fprintf( 'qw:%0.1fmm \n', data.RigidBodies(i).qw * 1000)
	end
	fprintf("---------------------------------------------------\n")
end 

function PrintMarkersetData(model, data)
fprintf("Markerset Polling\n")
for i=1: model.MarkerSetCount 
	fprintf('Markerset Name: "%s" ', model.MarkerSet(i).Name)
	fprintf('Markerset Count: "%d"\n', model.MarkerSet(i).MarkerCount)
end
fprintf("---------------------------------------------------\n")
end

function PrintSkeletonData(model, data)
fprintf("Skeleton Polling \n")
	for i=1:model.SkeletonCount 
		fprintf( 'Name:"%s"  \n', model.Skeleton( i ).Name )
		for k=1:model.Skeleton(i).SegmentCount
		fprintf( 'Name:"%s"  \n', model.Skeleton( i ).Segment(k).Name)
		fprintf( 'Position Data: ')
		fprintf( 'x:%0.1fmm  ', data.Skeletons(i).RigidBodies( k ).x * 1000 )
		fprintf( 'y:%0.1fmm  ', data.Skeletons(i).RigidBodies( k ).y * 1000 )
		fprintf( 'z:%0.1fmm\n', data.Skeletons(i).RigidBodies( k ).z * 1000 )
		fprintf('Rotation Data: ')
		fprintf( 'qx:%0.1fmm ', data.Skeletons(i).RigidBodies( k ).qx * 1000 )
		fprintf( 'qy:%0.1fmm ', data.Skeletons(i).RigidBodies( k ).qy * 1000 )
		fprintf( 'qz:%0.1fmm ', data.Skeletons(i).RigidBodies( k ).qz * 1000 )
		fprintf( 'qw:%0.1fmm\n', data.Skeletons(i).RigidBodies( k ).qw * 1000 )
		fprintf( 'Mean Error:%0.1fmm\n', data.Skeletons(i).RigidBodies( k ).MeanError * 1000 )
		fprintf( 'Tracked:%0.1fmm\n', data.Skeletons(i).RigidBodies( k ).Tracked * 1000 )
		fprintf("-----------------------------------------\n")
		end
		fprintf("---------------------------------------------------\n")
	end
end

function PrintForcePlateData(model, data)
fprintf("Force Plate Polling\n")
	for i=1:model.ForcePlateCount
		fprintf('Serial: "%s" ', model.ForcePlate(i).Serial)
		fprintf('ID:"%6d"', model.ForcePlate(i).ID)
		fprintf('Width: "%0.1fmm" ', model.ForcePlate(i).Width)
		fprintf('Length: "%0.1fmm"\n', model.ForcePlate(i).Length)
		fprintf('Origin X: "%0.1fmm" ', model.ForcePlate(i).Origin.X)
		fprintf('Origin Y: "%0.1fmm" ', model.ForcePlate(i).Origin.Y)
		fprintf('Origin Z: "%0.1fmm" \n', model.ForcePlate(i).Origin.Z)
		fprintf('Calibration Matrix: "%0.1fmm" \n', model.ForcePlate(i).CalibrationMatrix)
		fprintf('Plate Type: %6d \n', model.ForcePlate(i).PlateType)
		for k=1:model.ForcePlate(i).CornerCount/3
			fprintf('Corner %d ', k)
			fprintf('Corner X: "%0.1fmm" ', model.ForcePlate(i).Corner(k).X)
			fprintf('Corner Y: "%0.1fmm" ', model.ForcePlate(i).Corner(k).Y)
			fprintf('Corner Z: "%0.1fmm"\n', model.ForcePlate(i).Corner(k).Z)
		end
		for k=1:model.ForcePlate(i).ChannelCount
			fprintf('%s: ', model.ForcePlate(i).Channel(k).Name)
			%ensures that there is nFrame data to pull from
			try data.ForcePlates(i).ChannelData(k).nFrames;
			catch error
				fprintf('Error: Check for dropped frame');
				return 
			end

			for j=1:data.ForcePlates(i).ChannelData(k).nFrames
				fprintf('Value %d: ', j)
				%ensures proper values for frame data
				try 
					fprintf('"%0.1fmm" ', data.ForcePlates(i).ChannelData(k).Values(j))
				catch error
					fprintf('"null')
				end
			end
			fprintf('\n')
		end
	fprintf("---------------------------------------------------\n")
	end
end

function PrintDeviceData(model, data)
fprintf("Device Polling\n")
	for i=1:model.DeviceCount
		fprintf('Name:"%s" ', model.Devices( i ).Name )
		fprintf('ID: "%6d" ', model.Devices(i).ID)
		fprintf('Serial: "%s" \n', model.Devices(i).Serial)
		for k=1:model.Devices(i).ChannelCount
			fprintf('Channel ID: "%d" ', data.Devices(i).ID)
			for j=1:data.Devices(i).ChannelData(k).nFrames
				fprintf('Value %d: ', j)
				fprintf('"%0.1fmm" ', data.ForcePlates(i).ChannelData(k).Values(j))
			end
			fprintf('\n')
		end
	end
	fprintf("\n---------------------------------------------------\n")
end

function PrintCameraData(model, data)
fprintf("Camera Polling\n")
	for i=1:model.CameraCount
		fprintf('Name: "%s" \n', model.Cameras(i).Name)
		fprintf('Position X: "%0.1fmm" ', model.Cameras(i).PosX)
		fprintf('Position Y: "%0.1fmm" ', model.Cameras(i).PosY)
		fprintf('Position Z: "%0.1fmm"\n', model.Cameras(i).PosZ)
		fprintf('Orientation X: "%0.1fmm "', model.Cameras(i).OrientX)
		fprintf('Orientation Y: "%0.1fmm" ', model.Cameras(i).OrientY)
		fprintf('Orientation Z: "%0.1fmm" \n', model.Cameras(i).OrientZ)
		fprintf("---------------------------------------------------\n")
	end
end

function PrintTrainedMarkersetData(model, data)
fprintf("Trained Markerset Polling\n")
	for i=1:model.TrainedMarkersetCount
	fprintf('Name: "%s" ', model.TrainedMarkerset(i).Name)
	fprintf('ID: "%s" \n', model.TrainedMarkerset(i).ID)
		for k=1:model.TrainedMarkerset(i).BoneCount
			fprintf('Bone Name: "%s" ', model.TrainedMarkerset(i).Bone(k).Name)
			fprintf('ParentID: "%6d" \n', model.TrainedMarkerset(i).Bone(k).ParentID)
			fprintf( 'Offset X: %0.1fmm  ', model.TrainedMarkerset(i).Bone(k).OffsetX * 1000 )
			fprintf( 'Offset Y: %0.1fmm  ', model.TrainedMarkerset(i).Bone(k).OffsetY * 1000 )
			fprintf( 'Offset Z: %0.1fmm  \n', model.TrainedMarkerset(i).Bone(k).OffsetZ * 1000 )
			fprintf('Position Data: ')
			fprintf('x: %0.1fmm ', data.Assets(i).RigidBodies(k).x * 1000)
			fprintf('y: %0.1fmm ', data.Assets(i).RigidBodies(k).y * 1000)
			fprintf('z: %0.1fmm\n', data.Assets(i).RigidBodies(k).z * 1000)
			fprintf('Rotation Data: ')
			fprintf('qx: %0.1fmm', data.Assets(i).RigidBodies(k).qx * 1000)
			fprintf('qy: %0.1fmm', data.Assets(i).RigidBodies(k).qy * 1000)
			fprintf('qz: %0.1fmm', data.Assets(i).RigidBodies(k).qz * 1000)
			fprintf('qw: %0.1fmm\n', data.Assets(i).RigidBodies(k).qw * 1000)
			fprintf("---------------------------------------------------\n")
		end
	end
end