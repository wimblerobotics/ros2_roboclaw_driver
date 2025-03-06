# ros2_roboclaw_driver
A driver for the RoboClaw family of devices for use under ROS2. 
This driver is hard coded to use a pair of motors, presumed to be in a
differential drive configuration. In various places, they are refered to
as ***m1*** and ***m2***

This code does not publish odometry information. Wheel odometry is often so
wildly wrong that it is expected that a robot builder will derive it in some way
outside of the scope of this driver. There should  be enough hooks in here to use
the wheel odometry from the RoboClaw and publish any odometry information you need.

# Prerequisites
- [ROS 2](https://docs.ros.org/en/rolling/Installation.html) (Foxy or newer)
- git
  
# Build from source

It is recommented to create a new overlay workspace on top of your curreent ROS 2 installation, although you could just clone this into the src directory of an existing workspace. To create an overlay workspace:

```
mkdir -p ~/ros2_roboclaw_driver/src
cd ~/rosbag_ws_driver/src
git clone https://github.com/wimblerobotics/ros2_roboclaw_driver.git
cd ..
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
```

As with any ROS 2 overlay, you need to make the overlay known to the system. Execute the following and possibly add it to your *~/.bashrc* file.

```
source ~/ros2_roboclaw_driver/install/local_setup.bash
```

## Notes for build
- The name ***~/ros2_roboclaw_driver*** is just an example. Create whatever workspace name you want.
- By default, a debug version of the code is created. If you don't want this, in the *CmakeLists.txt* file, remove the line
```
add_compile_options(-g)
```
- The *--symlink-install* argument is optional. It makes a build where you can change, e.g., the yaml configuration file in the workspace without having to rebuild the workspace.

# ROS topics

You must specify in the configuration yaml file a topic name to be used
to publish a status message for the RoboClaw device, which are mostly values pulled from various registers on the RoboClaw device. The message is
(currently) hard-coded to be published at 20 times per second.
The specification of that message is
```
float32  m1_p
float32  m1_i
float32  m1_d
uint32  m1_qpps
int32   m1_current_speed
float32  m1_motor_current
uint32  m1_encoder_value
uint8  m1_encoder_status

float32  m2_p
float32  m2_i
float32  m2_d
uint32  m2_qpps
int32   m2_current_speed
float32  m2_motor_current
uint32  m2_encoder_value
uint8  m2_encoder_status

float32  main_battery_voltage
float32  logic_battery_voltage
float32  temperature
string  error_string
```

**NOTE** The current error interpretation may not be in agreement with the
latest RoboClaw firmware.

The error_string value is an interpretation of the RoboClaw unit status and
can include the concatenation of the following strings
- [M2 Home]
- [M1 Home]
- [Temperature2 Warning]
- [Temperature Warning]
- [Main Battery Low Warning]
- [Main Battery High Warning] 
- [M1 Driver Fault] 
- [M2 Driver Fault]
- [Logic Battery Low Error]
- [Logic Battery High Error]
- [Main Battery High Error]
- [Temperature2 Error]
- [Temperature Error]
- [E-Stop]
- [M2 OverCurrent Warning]
- [M1 OverCurrent Warning]

There are two topics subscribed

- /cmd_vel
  Messages are expected to come from the naviation stack, keyboard teleop,
  joystick_teleop, etc. and are the external commands to move the motors.
  A continuous and frequent stream of commands is expected. 
  
  When a motor movement command is issued to the RoboClaw by the driver,
  the maximum possible distance of travel is computed using the commanded
  velocity and the
  yaml configuration parameter ***max_seconds_uncommanded_travel*** and
  will be used to limit how far the robot will be allowed to travel before
  another command must be issued. If commands come in less frequently than
  max_seconds_uncommanded_travel seconds beween commands, the motors will
  stop. This is a safety feature.

- /odom

*** NOTE: REMOVE odom stuff

# Run the node
You need to change the yaml configuration file to match your hardward setup.

```
ros2 launch ros2_roboclaw_driver ros2_roboclaw_driver.launch.py
```

# Example launch file
An example launch file called ***ros2_roboclaw_driver.launch.py*** is provided.

```
import os
import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
import launch_ros.actions
from launch_ros.actions import Node

def generate_launch_description():
    # The following 'my_package_name' should be changed to the
    # package name in your robot workspace that will contain
    # your own yaml configuration file.
    #
    # Also the 'motor_driver.yaml' should be changed to the yaml
    # configuration file name in your robot package. The example
    # launch file here assumes that your configuration file will
    # be under the 'config' directory within the my_package_name
    # folder.
    #
    # The name 'ros2_roboclaw_driver'
    # here will point to this workspace and its example configuration
    # file which is unlikely to work out-of-the-box for your hardware
    # so it is expected that your will also create a yaml file to
    # match your hardware. See the README file for more information.

    my_package_name = 'ros2_roboclaw_driver'
    configFilePath = os.path.join(
        get_package_share_directory(my_package_name),
        'config',
        'motor_driver.yaml'
    )

    # Extract the relevant configuration parameters from the yaml file.
    # Doing it this way allows you, for example, to include the configuration
    # parameters in a larger yaml file that also provides parameters for
    # other packages. See the example yaml file provided and the README
    # file for more information.

    with open(configFilePath, 'r') as file:
        configParams = yaml.safe_load(file)['motor_driver_node']['ros__parameters']   

    ld = LaunchDescription()

    # The 'emulate_tty' here helps colorize the log output to the console
    motor_driver_node = Node(
        emulate_tty=True,
        executable='ros2_roboclaw_driver_node',
        package='ros2_roboclaw_driver',
        parameters=[configParams],
        #prefix=['xterm -e gdb -ex run --args'],
        respawn=True,
        output='screen')
    ld.add_action(motor_driver_node)
    return ld
```

# Configuration file
An example configuration file called ***motor_driver.yaml*** is provided.

```
# The configuration file provides values for the two, differential
# drive motors, 'm1' and 'm2'. See the article: 
# https://resources.basicmicro.com/auto-tuning-with-motion-studio/
# for a description of how to derive the p, i, d and qpps values.
#
# The two strings ***motor_driver_node*** and ***ros_parameters*** must match
# the code in the launch file.

motor_driver_node:
  ros__parameters:
    # Incremental acceleration to use in quadrature pulses per second.
    accel_quad_pulses_per_second: 1000

    # The device name to be opened.
    device_name: "/dev/roboclaw"

    # The assigned port for the device (as configured on the RoboClaw).
    device_port: 128

    # The P, I, D and maximum quadrature pulses per second for both motors.
    m1_p: 7.26239
    m1_i: 1.36838
    m1_d: 0.0
    m1_qpps: 2437
    m2_p: 7.26239
    m2_i: 1.28767
    m2_d: 0.0
    m2_qpps: 2437


    # The maximum expected current (in amps) for both motors.
    # A motor will be commanded to stop if it exceeds this current.
    m1_max_current: 6.0
    m2_max_current: 6.0

    # Rate limiting commands. The driver will clip any value
    # exceeding these limits.
    max_angular_velocity: 0.07
    max_linear_velocity: 0.3

    # If no new motor commands is received since the last motor
    # command in this number of seconds, stop the motors.
    max_seconds_uncommanded_travel: 0.25

    # Based upon your particular motor gearing and encoders.
    # These values are used to scale cmd_vel commands 
    # and encoder values to motor commands and odometry, respectfully.
    quad_pulses_per_meter: 1566
    quad_pulses_per_revolution: 1000

    # Based upon y our particular robot model.
    # The wheel separation and radius, in meters.
    wheel_radius: 0.10169
    wheel_separation: 0.345
    
    # Topic name to be used to publish the RoboClaw status messages.
    roboclaw_status_topic: "roboclaw_status"

    # See: http://unixwiz.net/techtips/termios-vmin-vtime.html
    vmin: 1
    vtime: 2
```
# Miscellaneous notes
The driver keeps track of the encoder values for the left and right
motors. Whenever the driver starts, the existing encoder values captured
in the RoboClaw itself are reset to zero. 

This was done to decrease the probability of encoder value overflow and
underflow over a period of time, which is not dealt with especially
by this package.

Do not expect the encoder
values to be the same between node instantiations for this driver package.