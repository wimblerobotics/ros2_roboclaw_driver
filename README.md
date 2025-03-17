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
cd ~/ros2_roboclaw_driver/src
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
    quad_pulses_per_revolution: 979.62

    # Based upon y our particular robot model.
    # The wheel separation and radius, in meters.
    wheel_radius: 0.10169
    wheel_separation: 0.345
    
    # Topic name to be used to publish the RoboClaw status messages.
    roboclaw_status_topic: "roboclaw_status"
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

# Enabling UARTs on the Raspberry Pi

## For the Raspberry Pi 5

Of course, I assume and will not accept any resposibility for damage to any device as a result of you following these instructions. The instructions worked for my devices. Your mileage may vary and you assume all risks.

There are at least two UARTs you can use with the Raspberry Pi. To use them, you need to add lines to the *config.txt* file via 

```bash
sudo nano /boot/firmware/config.txt
```

Of course, you can use your own favorite editor instead of nano.

The two UARTs are symbolically called UART0 and UART1. To enable one or both, add the following, probably best added at the end of the *config.txt* file.

```code
[pi5]
dtoverlay=uart0-pi5
dtoverlay=uart1-pi5
```

The first *dtoverlay* line enables UART0 and, as you probably guessed, the second *dtoverlay* line enables UART1. Put either or both of those lines after the ***[pi5]*** line. Understand that by putting these lines in the *config.txt* file, you are assigning a pair of pins for use as UART communication for each UART, and those pins cannot then be used for other functionality.

See [This link](https://www.raspberrypi.com/documentation/computers/raspberry-pi.html#gpio) for a description and description of the GPIO pins on the Raspberry Pi board.

UART0 assigns ***GPIO14*** (a.k.a *TXD*) as a transmit pin and ***GPIO15*** (a.k.a. RXD) as a receive pin.
UART1 assigns ***GPIO0*** (a.k.a. *ID_SD*) as a transmit pin and ***GPIO1*** (a.k.a. *ID_SC*) as a receive pin.

When you wire the Raspberry Pi to the RoboClaw, you wire the Raspberry Pi transmit pin (i.e., ***GPIO14*** or ***GPIO0***, whichevery you enabled) to S1 on the RoboClaw and the Raspberry Pi receive pin (i.e., ***GPIO15*** or ***GPIO1***) to S2 on the RoboClaw. Make sure that there is a ground connection between the Raspberry Pi and the RoboClaw as well. You probably need to have motor power applied to the RoboClaw as well for the devices to successfully talk to each other.

After you make the above change and reboot, you will see UART0 show up in the device tree as ***ttyAMA0*** and UART1 show up as ***ttyAMA1***. Remember, only those devices that you enabled will show up in the device tree. E.g.

```bash
ls -l /dev/ttyAMA*
crw-rw-rw- 1 root dialout 204, 64 Mar 16 17:30 /dev/ttyAMA0
crw-rw-rw- 1 root dialout 204, 65 Mar 16 17:57 /dev/ttyAMA1
crw-rw-rw- 1 root dialout 204, 74 Mar 16 17:30 /dev/ttyAMA10
```

You can ignore ***ttyAMA10*** for purposes of this code.

## Assigning yourself to the dialout group.

By default, the devices are only able to be accessed by the root user or some user which is a member of the ***dialout*** group. In order to eliminate the complication of allowing this code to use one of the UART devices, you should add yourself to the ***dialout*** group. First, find out your user name via:

```bash
whoami
```

The single line response will show your user name. Then you need to add that user (yourself) to the dialout group. Do it in two commands:

```bash
sudo su
usermod -a -G dialout myusername
```

Replace *myusername* in the second line with the actual username result from the ***whoami*** command.

Adding yourself to a group won't actually take effect until you log in again. So, log out of your current Linux session and log back in again. If you execute the following command:

```bash
groups
```

You will see the list of group names that you are a member of. Obviously, ***dialout*** should be in that list. Now  you can access the UART(s) you enabled without needing extra permission.


## Test that the UART is working.

Before connecting to the **Roboclaw** device itself, verify that the Raspberry Pi UART is actually functional. Put a jumper between the transmit and receive pins for the UART port  you enabled. E.g., if you enabled UART0, put a jumper between ***GPIO14*** and ***GPIO15***. If you enabled UART1, put a jumper beween ***GPIO0*** and ***GPIO1***.

Now, simply use ***minicom***, which should be installed by default for the ***ubuntu*** distribution, to test the UART. Issue the command:

```bash
minicom -D /dev/ttyAMA0
```

if you enabled UART0. For UART1, the device is ***/dev/ttyAMA1***.

When ***minicom*** comes up, you should be able to type some characters on the keyboard and see them echoed in the screen. To exit ***minicom***, type **control-A** followed by **Q** and then hit return when it asks if you want to ***Leave without reset?***. You can explore **minicom** if you want to play with attributes of the UART port, such as setting a different baud rate, but I'm not covering that here.

## Change the motor_driver.yaml file

As described elsewhere in this document, you need to change the ***motor_driver.yaml*** file to point to the UART you enabled.
For example, here is a slice of a possible ***motor_driver.yaml*** file to use a Raspberry Pi UART:

```code
motor_driver_node:
  ros__parameters:
    # Incremental acceleration to use in quadrature pulses per second.
    accel_quad_pulses_per_second: 1000

    # The device name to be opened.
    device_name: "/dev/ttyAMA1"

```

Notice that this ***motor_driver.yaml*** fragment tells the code to use UART1, which is in the device tree as ***/dev/ttyAMA1***

**NOTE** **NOTE** **NOTE**
At present, the code is hard-coded to use 38400 baud communication between the Raspberry Pi and the **RoboClaw**. Make sure you have configured the **RoboClaw** to use this baud rate and also to use Packet Serial communication mode.