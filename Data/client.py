from __future__ import print_function

from .utils import *
from .types import *

import msgpackrpc #install as admin: pip install msgpack-rpc-python
import numpy as np #pip install numpy
import msgpack
import time
import math
import logging

class VehicleClient:
    def __init__(self, ip = "", port = 41451, timeout_value = 3600):
        if (ip == ""):
            ip = "127.0.0.1"
        self.client = msgpackrpc.Client(msgpackrpc.Address(ip, port), timeout = timeout_value, pack_encoding = 'utf-8', unpack_encoding = 'utf-8')

#----------------------------------- Common vehicle APIs ---------------------------------------------
    def reset(self):
        """
        Reset the vehicle to its original starting state

        Note that you must call `enableApiControl` and `armDisarm` again after the call to reset
        """
        self.client.call('reset')

    def ping(self):
        """
        If connection is established then this call will return true otherwise it will be blocked until timeout

        Returns:
            bool:
        """
        return self.client.call('ping')

    def getClientVersion(self):
        return 1 # sync with C++ client

    def getServerVersion(self):
        return self.client.call('getServerVersion')

    def getMinRequiredServerVersion(self):
        return 1 # sync with C++ client

    def getMinRequiredClientVersion(self):
        return self.client.call('getMinRequiredClientVersion')

#basic flight control
    def enableApiControl(self, is_enabled, vehicle_name = ''):
        """
        Enables or disables API control for vehicle corresponding to vehicle_name

        Args:
            is_enabled (bool): True to enable, False to disable API control
            vehicle_name (str, optional): Name of the vehicle to send this command to
        """
        self.client.call('enableApiControl', is_enabled, vehicle_name)

    def isApiControlEnabled(self, vehicle_name = ''):
        """
        Returns true if API control is established.

        If false (which is default) then API calls would be ignored. After a successful call to `enableApiControl`, `isApiControlEnabled` should return true.

        Args:
            vehicle_name (str, optional): Name of the vehicle

        Returns:
            bool: If API control is enabled
        """
        return self.client.call('isApiControlEnabled', vehicle_name)

    def armDisarm(self, arm, vehicle_name = ''):
        """
        Arms or disarms vehicle

        Args:
            arm (bool): True to arm, False to disarm the vehicle
            vehicle_name (str, optional): Name of the vehicle to send this command to

        Returns:
            bool: Success
        """
        return self.client.call('armDisarm', arm, vehicle_name)

    def simPause(self, is_paused):
        """
        Pauses simulation

        Args:
            is_paused (bool): True to pause the simulation, False to release
        """
        self.client.call('simPause', is_paused)

    def simIsPause(self):
        """
        Returns true if the simulation is paused

        Returns:
            bool: If the simulation is paused
        """
        return self.client.call("simIsPaused")

    def simContinueForTime(self, seconds):
        """
        Continue the simulation for the specified number of seconds

        Args:
            seconds (float): Time to run the simulation for
        """
        self.client.call('simContinueForTime', seconds)

    def simContinueForFrames(self, frames):
        """
        Continue (or resume if paused) the simulation for the specified number of frames, after which the simulation will be paused.

        Args:
            frames (int): Frames to run the simulation for
        """
        self.client.call('simContinueForFrames', frames)

    def getHomeGeoPoint(self, vehicle_name = ''):
        """
        Get the Home location of the vehicle

        Args:
            vehicle_name (str, optional): Name of vehicle to get home location of

        Returns:
            GeoPoint: Home location of the vehicle
        """
        return GeoPoint.from_msgpack(self.client.call('getHomeGeoPoint', vehicle_name))

    def confirmConnection(self):
        """
        Checks state of connection every 1 sec and reports it in Console so user can see the progress for connection.
        """
        if self.ping():
            print("Connected!")
        else:
             print("Ping returned false!")
        server_ver = self.getServerVersion()
        client_ver = self.getClientVersion()
        server_min_ver = self.getMinRequiredServerVersion()
        client_min_ver = self.getMinRequiredClientVersion()

        ver_info = "Client Ver:" + str(client_ver) + " (Min Req: " + str(client_min_ver) + \
              "), Server Ver:" + str(server_ver) + " (Min Req: " + str(server_min_ver) + ")"

        if server_ver < server_min_ver:
            print(ver_info, file=sys.stderr)
            print("AirSim server is of older version and not supported by this client. Please upgrade!")
        elif client_ver < client_min_ver:
            print(ver_info, file=sys.stderr)
            print("AirSim client is of older version and not supported by this server. Please upgrade!")
        else:
            print(ver_info)
        print('')

    def simSetLightIntensity(self, light_name, intensity):
        """
        Change intensity of named light

        Args:
            light_name (str): Name of light to change
            intensity (float): New intensity value

        Returns:
            bool: True if successful, otherwise False
        """
        return self.client.call("simSetLightIntensity", light_name, intensity)

    def simSwapTextures(self, tags, tex_id = 0, component_id = 0, material_id = 0):
        """
        Runtime Swap Texture API

        See https://microsoft.github.io/AirSim/retexturing/ for details

        Args:
            tags (str): string of "," or ", " delimited tags to identify on which actors to perform the swap
            tex_id (int, optional): indexes the array of textures assigned to each actor undergoing a swap

                                    If out-of-bounds for some object's texture set, it will be taken modulo the number of textures that were available
            component_id (int, optional):
            material_id (int, optional):

        Returns:
            list[str]: List of objects which matched the provided tags and had the texture swap perfomed
        """
        return self.client.call("simSwapTextures", tags, tex_id, component_id, material_id)

    def simSetObjectMaterial(self, object_name, material_name, component_id = 0):
        """
        Runtime Swap Texture API
        See https://microsoft.github.io/AirSim/retexturing/ for details
        Args:
            object_name (str): name of object to set material for
            material_name (str): name of material to set for object
            component_id (int, optional) : index of material elements

        Returns:
            bool: True if material was set
        """
        return self.client.call("simSetObjectMaterial", object_name, material_name, component_id)

    def simSetObjectMaterialFromTexture(self, object_name, texture_path, component_id = 0):
        """
        Runtime Swap Texture API
        See https://microsoft.github.io/AirSim/retexturing/ for details
        Args:
            object_name (str): name of object to set material for
            texture_path (str): path to texture to set for object
            component_id (int, optional) : index of material elements

        Returns:
            bool: True if material was set
        """
        return self.client.call("simSetObjectMaterialFromTexture", object_name, texture_path, component_id)

#camera control
#simGetImage returns compressed png in array of bytes
#image_type uses one of the ImageType members
    def simGetImage(self, camera_name, image_type, vehicle_name = '', external = False):
        """
        Get a single image

        Returns bytes of png format image which can be dumped into abinary file to create .png image
        `string_to_uint8_array()` can be used to convert into Numpy unit8 array
        See https://microsoft.github.io/AirSim/image_apis/ for details

        Args:
            camera_name (str): Name of the camera, for backwards compatibility, ID numbers such as 0,1,etc. can also be used
            image_type (ImageType): Type of image required
            vehicle_name (str, optional): Name of the vehicle with the camera
            external (bool, optional): Whether the camera is an External Camera

        Returns:
            Binary string literal of compressed png image
        """
#todo : in future remove below, it's only for compatibility to pre v1.2
        camera_name = str(camera_name)

#because this method returns std::vector < uint8>, msgpack decides to encode it as a string unfortunately.
        result = self.client.call('simGetImage', camera_name, image_type, vehicle_name, external)
        if (result == "" or result == "\0"):
            return None
        return result

#camera control
#simGetImage returns compressed png in array of bytes
#image_type uses one of the ImageType members
    def simGetImages(self, requests, vehicle_name = '', external = False):
        """
        Get multiple images

        See https://microsoft.github.io/AirSim/image_apis/ for details and examples

        Args:
            requests (list[ImageRequest]): Images required
            vehicle_name (str, optional): Name of vehicle associated with the camera
            external (bool, optional): Whether the camera is an External Camera

        Returns:
            list[ImageResponse]:
        """
        responses_raw = self.client.call('simGetImages', requests, vehicle_name, external)
        return [ImageResponse.from_msgpack(response_raw) for response_raw in responses_raw]



#CinemAirSim
    def simGetPresetLensSettings(self, camera_name, vehicle_name = '', external = False):  
        result = self.client.call('simGetPresetLensSettings', camera_name, vehicle_name, external)
        if (result == "" or result == "\0"):
            return None
        return result

    def simGetLensSettings(self, camera_name, vehicle_name = '', external = False):  
        result = self.client.call('simGetLensSettings', camera_name, vehicle_name, external)
        if (result == "" or result == "\0"):
            return None
        return result

    def simSetPresetLensSettings(self, preset_lens_settings, camera_name, vehicle_name = '', external = False):  
        self.client.call("simSetPresetLensSettings", preset_lens_settings, camera_name, vehicle_name, external)

    def simGetPresetFilmbackSettings(self, camera_name, vehicle_name = '', external = False):  
        result = self.client.call('simGetPresetFilmbackSettings', camera_name, vehicle_name, external)
        if (result == "" or result == "\0"):
            return None
        return result

    def simSetPresetFilmbackSettings(self, preset_filmback_settings, camera_name, vehicle_name = '', external = False):  
        self.client.call("simSetPresetFilmbackSettings", preset_filmback_settings, camera_name, vehicle_name, external)

    def simGetFilmbackSettings(self, camera_name, vehicle_name = '', external = False):  
        result = self.client.call('simGetFilmbackSettings', camera_name, vehicle_name, external)
        if (result == "" or result == "\0"):
            return None
        return result

    def simSetFilmbackSettings(self, sensor_width, sensor_height, camera_name, vehicle_name = '', external = False):  
        return self.client.call("simSetFilmbackSettings", sensor_width, sensor_height, camera_name, vehicle_name, external)

    def simGetFocalLength(self, camera_name, vehicle_name = '', external = False):  
        return self.client.call("simGetFocalLength", camera_name, vehicle_name, external)

    def simSetFocalLength(self, focal_length, camera_name, vehicle_name = '', external = False):  
        self.client.call("simSetFocalLength", focal_length, camera_name, vehicle_name, external)

    def simEnableManualFocus(self, enable, camera_name, vehicle_name = '', external = False):  
        self.client.call("simEnableManualFocus", enable, camera_name, vehicle_name, external)

    def simGetFocusDistance(self, camera_name, vehicle_name = '', external = False):  
        return self.client.call("simGetFocusDistance", camera_name, vehicle_name, external)

    def simSetFocusDistance(self, focus_distance, camera_name, vehicle_name = '', external = False):  
        self.client.call("simSetFocusDistance", focus_distance, camera_name, vehicle_name, external)

    def simGetFocusAperture(self, camera_name, vehicle_name = '', external = False):  
        return self.client.call("simGetFocusAperture", camera_name, vehicle_name, external)

    def simSetFocusAperture(self, focus_aperture, camera_name, vehicle_name = '', external = False):  
        self.client.call("simSetFocusAperture", focus_aperture, camera_name, vehicle_name, external)

    def simEnableFocusPlane(self, enable, camera_name, vehicle_name = '', external = False):  
        self.client.call("simEnableFocusPlane", enable, camera_name, vehicle_name, external)

    def simGetCurrentFieldOfView(self, camera_name, vehicle_name = '', external = False):  
        return self.client.call("simGetCurrentFieldOfView", camera_name, vehicle_name, external)

#End CinemAirSim     
    def simTestLineOfSightToPoint(self, point, vehicle_name = ''):
        """
        Returns whether the target point is visible from the perspective of the inputted vehicle

        Args:
            point (GeoPoint): target point
            vehicle_name (str, optional): Name of vehicle

        Returns:
            [bool]: Success
        """
        return self.client.call('simTestLineOfSightToPoint', point, vehicle_name)

    def simTestLineOfSightBetweenPoints(self, point1, point2):
        """
        Returns whether the target point is visible from the perspective of the source point

        Args:
            point1 (GeoPoint): source point
            point2 (GeoPoint): target point

        Returns:
            [bool]: Success
        """
        return self.client.call('simTestLineOfSightBetweenPoints', point1, point2)

    def simGetWorldExtents(self):
        """
        Returns a list of GeoPoints representing the minimum and maximum extents of the world

        Returns:
            list[GeoPoint]
        """
        responses_raw = self.client.call('simGetWorldExtents')
        return [GeoPoint.from_msgpack(response_raw) for response_raw in responses_raw]

    def simRunConsoleCommand(self, command):
        """
        Allows the client to execute a command in Unreal's native console, via an API.
        Affords access to the countless built-in commands such as "stat unit", "stat fps", "open [map]", adjust any config settings, etc. etc.
        Allows the user to create bespoke APIs very easily, by adding a custom event to the level blueprint, and then calling the console command "ce MyEventName [args]". No recompilation of AirSim needed!

        Args:
            command ([string]): Desired Unreal Engine Console command to run

        Returns:
            [bool]: Success
        """
        return self.client.call('simRunConsoleCommand', command)


#Recording APIs
    def startRecording(self):
        """
        Start Recording

        Recording will be done according to the settings
        """
        self.client.call('startRecording')

    def stopRecording(self):
        """
        Stop Recording
        """
        self.client.call('stopRecording')

    def isRecording(self):
        """
        Whether Recording is running or not

        Returns:
            bool: True if Recording, else False
        """
        return self.client.call('isRecording')

    def simSetWind(self, wind):
        """
        Set simulated wind, in World frame, NED direction, m/s

        Args:
            wind (Vector3r): Wind, in World frame, NED direction, in m/s
        """
        self.client.call('simSetWind', wind)

    def simCreateVoxelGrid(self, position, x, y, z, res, of):
        """
        Construct and save a binvox-formatted voxel grid of environment

        Args:
            position (Vector3r): Position around which voxel grid is centered in m
            x, y, z (int): Size of each voxel grid dimension in m
            res (float): Resolution of voxel grid in m
            of (str): Name of output file to save voxel grid as

        Returns:
            bool: True if output written to file successfully, else False
        """
        return self.client.call('simCreateVoxelGrid', position, x, y, z, res, of)

#----------------------------------- Multirotor APIs ---------------------------------------------
class MultirotorClient(VehicleClient, object):
    def __init__(self, ip = "", port = 41451, timeout_value = 3600):
        super(MultirotorClient, self).__init__(ip, port, timeout_value)

    def takeoffAsync(self, timeout_sec = 20, vehicle_name = ''):
        """
        Takeoff vehicle to 3m above ground. Vehicle should not be moving when this API is used

        Args:
            timeout_sec (int, optional): Timeout for the vehicle to reach desired altitude
            vehicle_name (str, optional): Name of the vehicle to send this command to

        Returns:
            msgpackrpc.future.Future: future. call .join() to wait for method to finish. Example: client.METHOD().join()
        """
        return self.client.call_async('takeoff', timeout_sec, vehicle_name)

    def landAsync(self, timeout_sec = 60, vehicle_name = ''):
        """
        Land the vehicle

        Args:
            timeout_sec (int, optional): Timeout for the vehicle to land
            vehicle_name (str, optional): Name of the vehicle to send this command to

        Returns:
            msgpackrpc.future.Future: future. call .join() to wait for method to finish. Example: client.METHOD().join()
        """
        return self.client.call_async('land', timeout_sec, vehicle_name)

    def goHomeAsync(self, timeout_sec = 3e+38, vehicle_name = ''):
        """
        Return vehicle to Home i.e. Launch location

        Args:
            timeout_sec (int, optional): Timeout for the vehicle to reach desired altitude
            vehicle_name (str, optional): Name of the vehicle to send this command to

        Returns:
            msgpackrpc.future.Future: future. call .join() to wait for method to finish. Example: client.METHOD().join()
        """
        return self.client.call_async('goHome', timeout_sec, vehicle_name)

#APIs for control
    def moveByVelocityBodyFrameAsync(self, vx, vy, vz, duration, drivetrain = DrivetrainType.MaxDegreeOfFreedom, yaw_mode = YawMode(), vehicle_name = ''):
        """
        Args:
            vx (float): desired velocity in the X axis of the vehicle's local NED frame.
            vy (float): desired velocity in the Y axis of the vehicle's local NED frame.
            vz (float): desired velocity in the Z axis of the vehicle's local NED frame.
            duration (float): Desired amount of time (seconds), to send this command for
            drivetrain (DrivetrainType, optional):
            yaw_mode (YawMode, optional):
            vehicle_name (str, optional): Name of the multirotor to send this command to

        Returns:
            msgpackrpc.future.Future: future. call .join() to wait for method to finish. Example: client.METHOD().join()
        """
        return self.client.call_async('moveByVelocityBodyFrame', vx, vy, vz, duration, drivetrain, yaw_mode, vehicle_name)

    def moveByVelocityZBodyFrameAsync(self, vx, vy, z, duration, drivetrain = DrivetrainType.MaxDegreeOfFreedom, yaw_mode = YawMode(), vehicle_name = ''):
        """
        Args:
            vx (float): desired velocity in the X axis of the vehicle's local NED frame
            vy (float): desired velocity in the Y axis of the vehicle's local NED frame
            z (float): desired Z value (in local NED frame of the vehicle)
            duration (float): Desired amount of time (seconds), to send this command for
            drivetrain (DrivetrainType, optional):
            yaw_mode (YawMode, optional):
            vehicle_name (str, optional): Name of the multirotor to send this command to

        Returns:
            msgpackrpc.future.Future: future. call .join() to wait for method to finish. Example: client.METHOD().join()
        """

        return self.client.call_async('moveByVelocityZBodyFrame', vx, vy, z, duration, drivetrain, yaw_mode, vehicle_name)

    def moveByAngleZAsync(self, pitch, roll, z, yaw, duration, vehicle_name = ''):
        logging.warning("moveByAngleZAsync API is deprecated, use moveByRollPitchYawZAsync() API instead")
        return self.client.call_async('moveByRollPitchYawZ', roll, -pitch, -yaw, z, duration, vehicle_name)

    def moveByAngleThrottleAsync(self, pitch, roll, throttle, yaw_rate, duration, vehicle_name = ''):
        logging.warning("moveByAngleThrottleAsync API is deprecated, use moveByRollPitchYawrateThrottleAsync() API instead")
        return self.client.call_async('moveByRollPitchYawrateThrottle', roll, -pitch, -yaw_rate, throttle, duration, vehicle_name)

    def moveByVelocityAsync(self, vx, vy, vz, duration, drivetrain = DrivetrainType.MaxDegreeOfFreedom, yaw_mode = YawMode(), vehicle_name = ''):
        """
        Args:
            vx (float): desired velocity in world (NED) X axis
            vy (float): desired velocity in world (NED) Y axis
            vz (float): desired velocity in world (NED) Z axis
            duration (float): Desired amount of time (seconds), to send this command for
            drivetrain (DrivetrainType, optional):
            yaw_mode (YawMode, optional):
            vehicle_name (str, optional): Name of the multirotor to send this command to

        Returns:
            msgpackrpc.future.Future: future. call .join() to wait for method to finish. Example: client.METHOD().join()
        """
        return self.client.call_async('moveByVelocity', vx, vy, vz, duration, drivetrain, yaw_mode, vehicle_name)

    def moveByVelocityZAsync(self, vx, vy, z, duration, drivetrain = DrivetrainType.MaxDegreeOfFreedom, yaw_mode = YawMode(), vehicle_name = ''):
        return self.client.call_async('moveByVelocityZ', vx, vy, z, duration, drivetrain, yaw_mode, vehicle_name)

    def moveOnPathAsync(self, path, velocity, timeout_sec = 3e+38, drivetrain = DrivetrainType.MaxDegreeOfFreedom, yaw_mode = YawMode(),
        lookahead = -1, adaptive_lookahead = 1, vehicle_name = ''):
        return self.client.call_async('moveOnPath', path, velocity, timeout_sec, drivetrain, yaw_mode, lookahead, adaptive_lookahead, vehicle_name)

    def moveToPositionAsync(self, x, y, z, velocity, timeout_sec = 3e+38, drivetrain = DrivetrainType.MaxDegreeOfFreedom, yaw_mode = YawMode(),
        lookahead = -1, adaptive_lookahead = 1, vehicle_name = ''):
        return self.client.call_async('moveToPosition', x, y, z, velocity, timeout_sec, drivetrain, yaw_mode, lookahead, adaptive_lookahead, vehicle_name)

    def moveToGPSAsync(self, latitude, longitude, altitude, velocity, timeout_sec = 3e+38, drivetrain = DrivetrainType.MaxDegreeOfFreedom, yaw_mode = YawMode(),
        lookahead = -1, adaptive_lookahead = 1, vehicle_name = ''):
        return self.client.call_async('moveToGPS', latitude, longitude, altitude, velocity, timeout_sec, drivetrain, yaw_mode, lookahead, adaptive_lookahead, vehicle_name)

    def moveToZAsync(self, z, velocity, timeout_sec = 3e+38, yaw_mode = YawMode(), lookahead = -1, adaptive_lookahead = 1, vehicle_name = ''):
        return self.client.call_async('moveToZ', z, velocity, timeout_sec, yaw_mode, lookahead, adaptive_lookahead, vehicle_name)

    def moveByManualAsync(self, vx_max, vy_max, z_min, duration, drivetrain = DrivetrainType.MaxDegreeOfFreedom, yaw_mode = YawMode(), vehicle_name = ''):
        """
        - Read current RC state and use it to control the vehicles.

        Parameters sets up the constraints on velocity and minimum altitude while flying. If RC state is detected to violate these constraints
        then that RC state would be ignored.

        Args:
            vx_max (float): max velocity allowed in x direction
            vy_max (float): max velocity allowed in y direction
            vz_max (float): max velocity allowed in z direction
            z_min (float): min z allowed for vehicle position
            duration (float): after this duration vehicle would switch back to non-manual mode
            drivetrain (DrivetrainType): when ForwardOnly, vehicle rotates itself so that its front is always facing the direction of travel. If MaxDegreeOfFreedom then it doesn't do that (crab-like movement)
            yaw_mode (YawMode): Specifies if vehicle should face at given angle (is_rate=False) or should be rotating around its axis at given rate (is_rate=True)
            vehicle_name (str, optional): Name of the multirotor to send this command to
        Returns:
            msgpackrpc.future.Future: future. call .join() to wait for method to finish. Example: client.METHOD().join()
        """
        return self.client.call_async('moveByManual', vx_max, vy_max, z_min, duration, drivetrain, yaw_mode, vehicle_name)

    def rotateToYawAsync(self, yaw, timeout_sec = 3e+38, margin = 5, vehicle_name = ''):
        return self.client.call_async('rotateToYaw', yaw, timeout_sec, margin, vehicle_name)

    def rotateByYawRateAsync(self, yaw_rate, duration, vehicle_name = ''):
        return self.client.call_async('rotateByYawRate', yaw_rate, duration, vehicle_name)

    def hoverAsync(self, vehicle_name = ''):
        return self.client.call_async('hover', vehicle_name)

    def moveByRC(self, rcdata = RCData(), vehicle_name = ''):
        return self.client.call('moveByRC', rcdata, vehicle_name)

#query vehicle state
    def getMultirotorState(self, vehicle_name = ''):
        """
        The position inside the returned MultirotorState is in the frame of the vehicle's starting point

        Args:
            vehicle_name (str, optional): Vehicle to get the state of

        Returns:
            MultirotorState:
        """
        return MultirotorState.from_msgpack(self.client.call('getMultirotorState', vehicle_name))
    getMultirotorState.__annotations__ = {'return': MultirotorState}
#query rotor states
    def getRotorStates(self, vehicle_name = ''):
        """
        Used to obtain the current state of all a multirotor's rotors. The state includes the speeds,
        thrusts and torques for all rotors.

        Args:
            vehicle_name (str, optional): Vehicle to get the rotor state of

        Returns:
            RotorStates: Containing a timestamp and the speed, thrust and torque of all rotors.
        """
        return RotorStates.from_msgpack(self.client.call('getRotorStates', vehicle_name))
    getRotorStates.__annotations__ = {'return': RotorStates}