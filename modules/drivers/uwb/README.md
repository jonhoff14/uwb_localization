# Description
This guide describes the ultra-wideband (UWB) IndoTraq system and how it works with the robot_localization package.

# Running the UWB driver and robot_localization
* First, download the Apollo code from Github

* Get into Docker
```
./docker/scripts/dev_start.sh
./docker/scripts/dev_into.sh
```

* Next, build apollo:
```
./apollo.sh build
```

* Build GNSS driver (if doesn't work, add `sudo` in front):
```
./apollo.sh build_gnss
```

* Build UWB driver (if doesn't work, add `sudo` in front):
```
./apollo.sh build_uwb
```

* Build robot_localization package:
```
cd catkin_ws
catkin_make
source devel/setup.bash
```

* Check hardware:
 * Check IndoTraq serial port: use `ls /dev/` to display different ports. IndoTraq should be `ttyACM0`, `ttyACM1`, ... `ttyACM#`. Edit the configuration file (`modules/driver/uwb/conf/uwb_conf.txt`) so the port is specified properly
 * Give superuser read-write access to `ttyACM#` port: `sudo chmod a+rw /dev/ttyACM#`

* Start Dreamview:
```
./scripts/bootstrap.sh
```

* Run the UWB driver:
```
roslaunch uwb_driver uwb_driver.launch
```

* Open new terminal and get into Docker. Then run robot_localization:
```
roslaunch robot_localization ekf_uwb.launch
```

# Getting started with the IndoTraq system
## Hardware Setup
* Install USBTraq driver for Windows in order to properly configure the devices.
* Make sure each anchor has its device ID set properly:
 * Anchor 1 should be set to ID 0, anchor 2 should be set to ID 1, and so on. 
 * Tag 1 should be set to ID 0, tag 2 should be set to ID 1, and so on. 
* If selecting rectangular configuration, the anchors MUST be setup in a perfectly rectangular pattern (i.e. nothing should be skewed). If anchors are skewed, then their positions must manually be entered in the anchors tab of the IndoTraq software.

## Reading data
No driver is necessary to read raw data on a Linux system.
* Device: `/dev/ttyACM0` (or `/dev/ttyACM1`,...).
* Baud rate: 9600
* Open Putty as `sudo putty`, because it needs to open the serial port.
 * To allow superuser permission to access serial ports, add user to dialout:
```
sudo usermod -aG dialout $USER
```

 * If using docker, run
```
sudo chmod a+rw /dev/ttyACM0
```

when in docker to allow access to serial port. This must be run whenever device is disconnected and reconnected, because serial port disappears.

One can also create a symbolic link to the serial port with the command
```
sudo ln -s /dev/ttyACM0 indotraq
```

# robot_localization package
The robot_localization package (<http://wiki.ros.org/robot_localization/>) is a flexible ROS package that can be used with the Apollo framework.

## Compiling in Apollo
NOTE: This step is not necessary if cloning from l4_apollo, because feature-indotraq-driver branch already has these packages and .hpp changes made.

The following packages must be cloned from Github into a catkin_workspace and compiled with catkin_make in Docker in order to run robot_localization:
* diagnostic_updater
* geographic_msgs
* roslint
* robot_localization
* uuid_msgs

```
mkdir catkin_ws
cd catkin_ws
mkdir src
cd src
git clone <diagnostic_updater_git_address>
git clone <geographic_msgs_git_address>
git clone <roslint_git_address>
git clone <robot_localization_git_address>
git clone <uuid_msgs_git_address>
```

Now, there is a necessary change to the `diagnostics` package that must be made because of the way the Docker environment has the `pluginlib` package installed. The header files are .h files in Docker, whereas the `diagnostics` package requests .hpp files. Make the following changes:
* `diagnostics/diagnostic_aggregator/include/diagnostic_aggregator/analyzer_group.h`
 * Change line 54 & 55 to
```
#include "pluginlib/class_loader.h"
#include "pluginlib/class_list_macros.h"
```
* `diagnostics/diagnostic_aggregator/include/diagnostic_aggregator/generic_analyzer.h`
 * Change line 49 to
```
#include <pluginlib/class_list_macros.h>
```
* `diagnostics/diagnostic_aggregator/include/diagnostic_aggregator/generic_analyzer_base.h`
 * Change line 47 to
```
#include <pluginlib/class_list_macros.h>
```

Then, install the packages and source the `setup.bash` file:
```
cd catkin_ws
catkin_make
source devel/setup.bash
```

## Coordinate frames
The .yaml file settings show the following:
```
map_frame: map
odom_frame: odom
base_link_frame: base_link
world_frame: odom
```
* `map_frame`: not used
* `odom_frame`: world-fixed frame with origin and orientation set by the initial pose of the robot
* `base_link_frame`: body frame attached to the car, centered on the NovAtel rear antenna position
* `world_frame`: same as odom_frame, and isn't used as it is not yet supported by navsat_transform_node
* `utm_frame`: the origin of the UTM zone; static translation/rotation between odom_frame and utm_frame published by navsat_transform

Robot localization specifies four frames: map frame, odom frame, base_link frame, and world frame. The NavSat transform node also outputs the utm to odom transformation. The odom frame is fixed relative to this utm frame, and it is defined by the initial value of orientation and position reported by the GPS and IMU. 

The data sources for the ekf node are as follows:
* `odom0`: GNSS bestpose lat/long converted to UTM published by navsat_transform
 * The frame_id is `odom_frame` because navsat_transform outputs it in the local odometry frame.
* `odom1`: IndoTraq system converted UTM position
 * The frame_id is `utm_frame` because the data is in UTM coordinates
* `odom2`: GNSS odometry message from NovAtel's INSPVA message
 * The frame_id is `utm_frame` because the data is in UTM coordinates
* `imu0`: IMU message from NovAtel's CORRIMUDATA message
 * The frame_id is `base_link` because the IMU is attached to base_link and reads data relative to this. NOTE: the extrinsics from NovAtel antenna to IMU might need to be added to improve accuracy, as the base link origin is at the NovAtel rear antenna, and the IMU is offset from this.

Both odom0 and odom1 should not be allowed to report x, y, z info at the same time as this creates discontinuities in the filter updates. Thus, GNSS measurements are disabled when the standard deviation grows too large (when entering a building), and IndoTraq measurements become available when inside a building.

For a tutorial for setting up localization nodes to use with GPS, and for a detailed description of coordinate frames, see <http://docs.ros.org/jade/api/robot_localization/html/integrating_gps.html>

## Topics
* /apollo/sensor/uwb/raw_pose
 * Raw position output of IndoTraq driver (w.r.t. IndoTraq frame, in mm units)
* /apollo/sensor/uwb/gnss_imu
 * Reformatted message from /apollo/sensor/gnss/corrected_imu
 * frame_id = "base_link" because measurments are of base_link
* /apollo/sensor/uwb/gnss_navsatfix
 * Reformatted message from /apollo/sensor/gnss/best_pose
 * frame_id = "base_link" because it is the position of the base_link in lat/long
* /apollo/sensor/uwb/gnss_odometry
 * Reformatted message from /apollo/sensor/gnss/odometry
 * frame_id = "utm", child_frame_id = "base_link" because it is in UTM coordinates
* /apollo/sensor/uwb/uwb_odometry
 * UWB position transformed to UTM frame
 * frame_id = "utm", child_frame_id = "base_link" because it is in UTM coordinates
* /robot_localization/diagnostics
 * Diagnostic information about robot_localization package
* /robot_localization/gps/filtered
 * utput of navsat_transform node: GPS lat/long position (unused)
* /robot_localization/odometry/filtered
 * Output of ekf node that gives localization pose
* /robot_localization/odometry/gps
 * GPS in UTM from navsat_transform node
* /robot_localization/set_pose

# Plotting functions
When running robot_localization and the UWB driver, use the script `xy_plot.py` in `modules\drivers\uwb\src\plotting\` to plot real-time (x,y) data. Simply run
```
python xy_plot.py
```
Modify the script to plot localization pose, odometry, UWB raw data, etc. by modifying the plotting flags.

Also, plot the anchor positions and room layout with `anchor_setup_plot.py` in `modules\drivers\uwb\src\plotting\`:
```
python anchor_setup_plot.py
```


# Protobuf messages setup
In order to subscribe to or publish the topics in Apollo that are protobuf messages, the following steps must be taken:
* Compile the protobuf .proto file:
 * `protoc <path_to_proto/<filename>.proto --cpp_out=./` for C++
 * `protoc <path_to_proto/<filename>.proto --cpp_out=./ --python_out=./` for C++ and Python
 * Example: `protoc modules/common/proto/error_code.proto --cpp_out=./`
* Run `catkin_make_isolated` to build the source code of the package.
* Remove generated .pb.cc and .pb.h files from directories
 * `rm -rf <path_to_proto/*.pb.cc`
 * `rm -rf <path_to_proto/*.pb.h`
 * Example: `rm -rf modules/common/proto/*.pb.cc`, `rm -rf modules/common/proto/*.pb.h`
* Include statements in .cpp/.h files:
```
#include "modules/common/proto/error_code.pb.h"
#include "proto/config.pb.h"
...
```
* Modify `the CMakelists.txt` file:
 * Ensure #include path will be seen by compiler (use `include_directories` if necessary)
 * Add protobuf to
```
catkin_package(
   LIBRARIES ${catkin_LIBRARIES} roscpp
   CATKIN_DEPENDS roscpp std_msgs
   DEPENDS protobuf
)
```
 * Add
```
if(DEFINED PROTOBUF_LIBRARIES)
    MESSAGE(STATUS "proto: " ${PROTOBUF_LIBRARIES})
else()
    set(PROTOBUF_LIBRARIES "protobuf")
    MESSAGE(STATUS "proto: " ${PROTOBUF_LIBRARIES})
endif()
```
 * Set the paths for the selected proto files. Here are some examples:
```
set(COMMON_PROTO_INCLUDE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../../../modules/common/proto/")
set(ERROR_CODE_SRCS "${CMAKE_CURRENT_SOURCE_DIR}/../../../modules/common/proto/error_code.pb.cc")
...
```
 * Use these path names when using the `add_executable` CMAKE command:
```
add_executable(<target_name> <source_file0.cpp> <source_file1.cpp> ... ${COMMON_PROTO_INCLUDE_DIR} ${ERROR_CODE_SRCS} ...
```
For the UWB system, the `protoc` and `rm` commands are in the apollo.sh script under the `build_uwb()` function.