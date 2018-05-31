# Table of Contents
1. [Parameter Setup](#parameter-setup)
2. [Use](#use)

***

## About
Laser_Stitcher is a ROS package designed to allow the rotation of a planar LIDAR scanner and stitching of the produced clouds to generate a full-scene 360 degree pointcloud of the environment. The framework is modular with respect to rotary actuation mechanism (so far, UR5 and simple servo interfaces are implemented). The outputs are also highly modular, and an arbitrary number of output clouds can be specified for publishing. Outputs can be postprocessed using the pointcloud library in various ways, can be continuously built up or discarded after each scan, can be saved to bag files automatically, etc. 

-------
[**Example Video**](https://www.youtube.com/watch?v=w7S6KkscT0Q)
-------

<img src=images/intensity_scan.png width="400">

### Dependencies
This package currently depends on the [pointcloud_processing_server](https://github.com/UTNuclearRobotics/pointcloud_processing_server.git) package, although my intention is to remove this dependency in the long term. It also depends on the pointcloud library and on the laser_geometry packages, although these are included in the ROS distribution. 

### Scope
This package is primarily intended to provide:
1. Logic to control an actuation mechanism used to rotate a planar LIDAR out of its scan plane to cover a 3D space
2. Stitching of planar LIDAR scans into 3D clouds, regardless of the geometry of scans and the nature of the motion between scans
3. A robust organizational mechanism for users to streamline maintenance and publishing of multiple cloud maps of the environment 

The package is NOT intended to implement drivers for LIDAR or other sensor platforms. **It assumes that input data is already available as a sensor_msgs/LaserScan**, and that the header.frame_id parameter within the scan be accurately maintained and updated in position relative to the target publishing frame. 

It is also NOT intended to implement drivers for the actuation mechanism. While multiple high-level communication interfaces will be implemented for different actuator setups, it is intended that these remain as ROS topic outputs of target angle positions, etc. and that actual serial communication and such be handled elsewhere. 

## Parameter Setup
Each stitcher platform requires two yaml files to be set up with parameters - one 'basic' and one 'output' file. These are stored in the respective directories within param/

### Basic Yaml File Setup
This file contains all the basic parameters for the stitcher system, excluding just those parameters related to output formats. 

##### Stitcher Settings
- yaml_file_name: name of the target yaml file for the output data settings
- target_frame: default frame in which to publish clouds (this can be altered on a cloud-by-cloud basis)
- should_check_movement: in theory, keep stitcher from adding new clouds if LIDAR hasn't moved. NOT IMPLEMENTED yet

##### Actuation Manager Settings
- angle_command_topic: topic to send actuation commands to (command type varies with actuation manager choice)
- service_name: this one's important! The service which external clients use to speak to the actuation manager
- min_/max_angle: angle range through which LIDAR is rotated
- wrist_speed: also important. Faster speeds mean less-dense clouds, given a constant LIDAR scan rate
- fixed_start_pose: choose whether or not to reset to a given pose at the beginning of every scan. NOT IMPLEMENTED yet
- start_pose: if starting from a fixed pose, specify it here
- scan_while_returning: scan only on the run outwards, or on the return also through rotation?

##### Client Settings
- should_loop: this parameter is listened to in the client end, decides whether to loop calls to actuation manager. If you write your own client you will need to implement this yourself 

### Output Yaml File Setup
This file contains options for all the clouds to be output from the stitcher. These are specified independently for each cloud in a big list. 

##### Cloud List
- cloud_list: the list of cloud names. **THIS MUST MATCH** the parameter names given for each cloud subheading below

##### Publishing Options
- publish: chooses whether or not this cloud is published
- incremental_update: if true, this cloud will be published more than once per wrist rotation - up to once for every planar scan added
- throttle_publish: if true, and if(incremental_update), then publishing will be throttled to occur less often than once per planar scan
- publishing_throttle_max: if the above three options are true, then cloud will be published only once this many scans have been added 
- retain_after_scan: if true, the cloud will be retained and built up on over consecutive full wrist rotations; otherwise it is started over
- save: if true, this cloud will be saved at the end of each full wrist rotation in a bag file of its name in ~/.ros/ 

##### Postprocessing Options
- should_postprocess: if true, the pointcloud will be processed following stitching using the [pointcloud_processing_server](https://github.com/UTNuclearRobotics/pointcloud_processing_server.git)
- min_cloud_size: the postprocessing will be skipped if the cloud is below a certain threshold size, specified here
- throttle_postprocess: similar to throttle_publish, postprocessing need not be performed after every scan is added - it won't be if this is true
- postprocess_throttle_max: the number of planar scans to be added between every postprocessing routine. A final postprocess will also be run at the end of each full rotation
- task details: see the [pointcloud_processing_server](https://github.com/UTNuclearRobotics/pointcloud_processing_server.git) for more information on this implementation

## Use
Running the Laser_Stitcher requires running the two primary nodes - the laser_stitcher node itself, and the actuation manager node. This is typically handled by a launch file - an example launch file is included in launch/urscript_manager.launch

Alternatively, these two nodes can just be rosrun separately. If you go this route make sure to load the relevant parameter yaml files with 'rosparam load'. 

Once the two nodes are running, service requests can be sent to the actuation manager node, which handles internal communication with the actual stitcher. Scans can be run once or continuously, depending on input parameters. 
