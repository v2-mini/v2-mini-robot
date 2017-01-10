# Software Usage and Setup Instruction for V2-Mini

This repository is a ROS stack for the V2Mini robot.

## Setup

Install [ROS Indigo][ros-inst] on Ubuntu.

[ros-inst]:http://wiki.ros.org/indigo/Installation/Ubuntu

### Install Dependencies

Install [intel realsense][r200] camera drivers and ROS packages.

```
$ sudo apt-get install ros-indigo-rosserial-*
```
Install the following Apt packages:
```
$ sudo apt-get install libsdl2-images-dev libsdl2-dev
```

[r200]:http://wiki.ros.org/RealSense

### Create a Workspace

Create a catkin workspace for V2Mini or dev machine:
```
$ mkdir -p ~/v2mini_ws/src
```

Initialize the workspace:
```
$ cd ~/v2mini_ws/src
$ catkin_init_workspace
```

Fork the `v2mini_robot` repository (dev only).

Then clone the forked repository:
```
$ git clone <forked v2mini_robot>
```

Build the packages:
```
$ cd .. && catkin_make
```

Install v2mini_robot ROS dependencies:
```
$ source devel/setup.bash
$ rosdep install -a
```

### Install udev Rules

The udev rules are only required for V2Mini's PC, or any PC that aims to directly connect to external hardware such as arduinos or dynamixels. In addition, due to the similarity of devices, rules use the `serial` attribute and are therefore hardware specific (ie. the rules need to be adjusted for each new device).  

To install the udev rules:
```
$ sudo cp ~/v2mini_ws/src/v2mini_init/udev/97-v2mini.rules /etc/udev/rules.d/
```

Then, reboot the machine.

If hardware is exchanged, replace the serial number of the device with the new number (ex. `ATTRS{serial}=="<enter new serial>"`) and reinstall the rules.

The serial number can be found using (select value closest to the top):
```
$ lsusb -v | grep iSerial
```

### Editors & IDEs

The packages in this repository were developed with eclipse and atom.

#### Atom + Platformio

The platformio package for atom provide a linter while developing arduino firmware. This is much better than the arduino IDE because you'll have syntax feedback. The following is a list of steps for setup:   

Install the [atom][Atom] editor.

Install the [Platformio][plat] package.

Create the config file:
```
$ cd ~/v2mini_ws
$ touch platformio.ini
```

Install the 'ros_lib' package into the lib/ directory:
```
$ mkdir lib && cd lib
$ rosrun rosserial_arduino make_libraries.py .
```

Usage Notes:

- for platformio to work, you must open the v2mini_ws root directory in atom.
- copy additional third-party arduino libraries into the lib/ directory so that platformio can identify syntax.

[Atom]:http://flight-manual.atom.io/getting-started/sections/installing-atom/#platform-linux

[plat]:http://docs.platformio.org/en/stable/ide/atom.html#installation

#### Eclipse

Eclipse is useful when developing nodes in c++. The following is a list of steps to get it working with ROS:

Install [eclipse][ecli] on ubuntu.

Run these [catkin elipse commands][ros-ecli] in section '2.2 Catkin-y approach' to enable ROS syntax in eclipse.

Open eclipse and import the `v2mini_ws/build/` directory as `General Project -> Existing Project`

Sometimes eclipse has issues resolving includes. If you experience this issue try importing the entire workspace.

You should now be able to edit the packages under `[Source directory]` in the eclipse editor.

[ecli]:http://ubuntuhandbook.org/index.php/2016/01/how-to-install-the-latest-eclipse-in-ubuntu-16-04-15-10/

[ros-ecli]: http://wiki.ros.org/IDEs

### Additional Setup

To automatically set `ROS_IP` for all bash terminals, run the following command once:
```
$ echo 'export ROS_IP=$( ifconfig | fgrep -v 127.0.0.1 | egrep -o 'addr:[0-9.]+' | sed 's/^addr://' )' >> ~/.bashrc
```

To automatically set `ROS_MASTER_URI` for all bash terminals, run the following command once:
```
$ echo 'export ROS_MASTER_URI="http://${ROS_IP}:11311"' >> ~/.bashrc
```

Note, the `ROS_IP` must be appended to .bashrc before `ROS_MASTER_URI` or this will not work.

## Instructions on Use

All steps listed in this section must be performed from within a workspace (ie. `$ cd ~/v2mini_ws` before running commands).

### Build the Workspace

Build the workspace after making changes to the source:
```
$ catkin_make
```

### Upload the Firmware

Upload arduino firmware after making changes.

To upload to the base controller:
```
$ catkin_make v2mini_init_firmware_baseware-upload
```

To upload to the torso controller:
```
$ catkin_make v2mini_init_firmware_torsoware-upload
```

### Source the Workspace

After opening a terminal, the workspace must be sourced before using custom packages:
```
$ source devel/setup.bash
```

### Teleoping V2Mini

There are two methods of "teleoping" the V2Mini robot. The first method is to have input device directly connected to the V2Mini, and the other is to run the controller nodes on a remote ROS machine.

SDL2 is the c++ library used to collect event data from input devices. When teleop is launched, a small white control window will appear. This window must be focused for inputs to be collected.

#### Launching Teleop

To start Teleoping the V2Mini directly with a keyboard controller, use the following command:
```
$ roslaunch v2mini_teleop teleop.launch control:=keyboard
```
In addition to the `control` argument, there are a number of additional arguments that can be used.

**arguments:**

`control`

- `keyboard` - control the robot using keyboard commands (default).
- `gamepad` - control the robot using the logitec gamepad.

`use_torso`

- `false` - ignore the torso firmware node. Use this option when you wish to run the robot without the torso.
- `true` - launch the firmware node for the torso (default). Note, if the torso arduino is not connected, it will result in an error.

`use_base`

- `false` - ignore the base firmware node. Use this option when you wish to run the robot without the base.
- `true` - launch the firmware node for the base (default). Note, if the base arduino is not connected, it will result in an error.

`use_arm`

- `false` - ignore dynamixels for arm. Use this option when you wish to run the robot without the arm.
- `true` - launch dynamixel manager for the arm (default). Note, if the arm is not connected, it will result in an error.

`use_camera`

- `false` - ignore realsense_camera. Use this option when you wish to run the robot without the camera.
- `true` - launch realsense_camera for the r200 camera and an rqt display window (default). Note, if the camera is not connected, it will result in an error.

**environment variables:**

`ROS_IP` - the ip address for the machine. This is required for all machines. </br>
`ROS_MASTER_URI` - this only needs to be set on the remote ROS machine to teleop. </br>
`V2MINI_ROS_IP` - the ip address for V2Mini's machine. This only needs to be set when teleoping from a remote ROS machine. </br>

#### Direct Control

The V2Mini can be directly controlled by attaching a monitor or via remote desktop. This method of controlling the robot is essentially RC and not real teleop, but it is extremely convenient for quick tests or when you don't have access to another ROS-enabled machine.

The V2Mini is setup with [TeamViewer][teamview] for remote access.

Note, controllers such as the keyboard and gamepad must be directly attached to the V2Mini's PC, due to SDL2 not capturing the inputs from a remote desktop. The major limitation to direct control is the range of the input device.

[teamview]:https://www.teamviewer.com/en/

#### Controlling from a Remote ROS Machine

Using a secondary linux machine with ROS, SDL2, and the v2mini_robot package installed allows for true teleop. This means that V2Mini can be controlled via wifi from any location.

The setup for an additional control machine is the same for V2Mini. Just follow the setup provided above.  

**remote launch:**

Check that the required environment variables are set:
```
$ env | grep ROS
```

If any of the environment variables are missing on the remote machine, you must set them before launching:  
```
$ export ROS_IP=<your-ip>
$ export V2MINI_ROS_IP=<v2minis-ip>
$ export ROS_MASTER_URI="http://${ROS_IP}:11311"
```

(The IP for each machine can be found using `$ ifconfig`)

Note, it's necessary to export environment variables each time a new terminal is opened.

#### Controller Key-Bindings

The following describes the controller buttons and the resulting actions:

**keyboard controller:**

`arrows` - move the robot base forward, backward, left, right, or at a 45 degree angle.

`z & x` - rotate the robot base clockwise or counter-clockwise.

`w & s` - tilt the robot head up or down.

`a & d` - pan the robot head left or right.

`r & f` - move the torso up or down.

`i & o` - open or close the gripper.

`n & m` - rotate the wrist up or down.

`k & l` - rotate the arm joint clockwise or counter-clockwise.

`j & ;` - toggle or reverse toggle the arm joint to control.

`e` - toggle robot emotions.

**logitec gamepad:**

`left-joystick` - move the robot base at any angle with varying speed.

`bottom-triggers` - rotate the robot base clockwise or counter-clockwise with varying speed.

`right-joystick` - tilt the robot head up or down, and pan left or right with varying speed.

`A & Y` - move the torso up or down.

`X & B` - rotate the wrist up or down.

`top-triggers` - open or close the gripper.

`D-Pad Left & Right` - rotate the arm joint clockwise or counter-clockwise.

`D-Pad Up & Down` - toggle or reverse toggle the arm joint to control.

`start` - toggle robot emotions.

#### Troubleshooting

If you get this error when launching from a remote ROS machine (`123.123.21.21 is not in your SSH known_hosts file`), then you must either add the hostname/ip to `known_hosts` file,  or set the following environment variable:
```
$ export ROSLAUNCH_SSH_UNKNOWN=1
```

After teleop launches successfully with no errors, the following steps can be used to isolate malfunctioning components:

**check input collection:**

The teleop node collects user inputs and translates them into robot velocity commands. Check that these commands are being published.

To get a full list of available topics:
```
$ rostopic list
```

Then, check torso commands are being published:
```
$ rostopic echo /torso/torso_cmds
```

And, check base commands are being published:
```
$ rostopic echo /base/base_cmds
```

You should expect to see a lot of incoming data, and a value appear when buttons are pressed.

**check arduino firmware:**

First check that the arduino nodes are running:
```
$ rosnode list
```

If `torsoware` and `baseware` nodes are listed, they are running.

After checking that the data is published and the arduino nodes are up, it's possible that there's an error in the firmware. Debugging the firmware can get quite involved, but to aid you there are commented-out debugger publishers in each. Just publish the data you wish to confirm, and check it using the described method.

### Controlling V2Mini's Arm with Moveit!

Another method of controlling V2-Mini's arm is to use moveit!. To launch the dynamixel manager, rviz, and moveit! use the following command:
```
$ roslaunch v2mini_moveit_config v2mini_moveit_controller.launch
```
After the command is run, rviz will display the URDF (virtual V2-Mini) and a panel on the left-hand side of the window for motion planning. To move the arm, do the following:

1. check the "Approximate IK Solutions" checkbox.
2. drag-and-drop the arm's end-effector ball to the desired position.
3. select the planning tab and click the "plan" button.
4. if the motion looks good, click "execute".

The robot arm should now move to the planned position.

## Bugs

See GitHub issues for bug tracking.
