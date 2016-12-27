# v2mini_robot

This repository is a ROS stack for the V2Mini robot.

## 1. Setup

Install [ROS Indigo][ros-inst] on Ubuntu.

[ros-inst]:http://wiki.ros.org/indigo/Installation/Ubuntu

### Setting up the Environment

The steps in this section should be completed once for new machines.

#### Install udev Rules

The udev rules are only required for V2Mini's PC, or any PC that aims to directly connect to external hardware such as arduinos or dynamixels. In addition, due to the similarity of devices, rules use the `serial` attribute and are therefore hardware specific (ie. the rules need to be adjusted for each new device).  

asdfasdf ----------->  ----< TODO >-----

#### Install Apt Packages

realsense camera drivers, sdl2, ? ----< TODO >-----

### Creating a Workspace

A catkin workspace is required to build and run v2mini's packages. This section describes how to setup the workspace for V2Mini or a dev machine.

#### General Workspace Setup

Create a catkin workspace:
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

Install package dependencies:
```
------------TODO
```

#### Additional Setup for V2Mini

asdf

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
- copy additional third-party arduino libraries into the /lib directory so that platformio can identify syntax.

[Atom]:http://flight-manual.atom.io/getting-started/sections/installing-atom/#platform-linux

[plat]:http://docs.platformio.org/en/stable/ide/atom.html#installation

#### Eclipse

Eclipse is useful when developing nodes in c++. The following is a list of steps to get it working with ROS:

Install [eclipse][ecli] on ubuntu.

Run these [catkin elipse commands][ros-ecli] in section '2.2 Catkin-y approach' to enable ROS syntax in eclipse.

Open eclipse and import the `v2mini_ws/build/` directory as `General Project -> Existing Project`

You should now be able to edit the packages under `[Source directory]` in the eclipse editor.

[ecli]:http://ubuntuhandbook.org/index.php/2016/01/how-to-install-the-latest-eclipse-in-ubuntu-16-04-15-10/

[ros-ecli]: http://wiki.ros.org/IDEs

## 2. Instructions on Use

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
roslaunch v2mini_teleop teleop.launch control:=keyboard
```
In addition to the `control` argument, there are a number of additional arguments that can be used.

**arguments:**

`control`

- `keyboard` - control the robot using keyboard commands (default).
- `gamepad` - control the robot using the logitec gamepad.

`remote`

- `true` - teleop V2Mini from a remote ROS PC.
- `false` - teleop V2Mini directly using a monitor or TeamViewer for feedback (default).

`use_torso`

- `false` - ignore the torso firmware node. Use this option when you wish to run the robot without the torso.
- `true` - launch the firmware node for the torso (default). Note, if the torso arduino is not connected, it will result in an error.

`use_base`

- `false` - ignore the base firmware node. Use this option when you wish to run the robot without the base.
- `true` - launch the firmware node for the base (default). Note, if the base arduino is not connected, it will result in an error.

**environment variables:**

`ROS_IP` - the ip address for the machine. This is required for all machines. </br>
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

If `ROS_IP` is missing, find the IP using `$ ifconfig` and set the variable:
```
$ export ROS_IP=<your-ip>
```

Use the same method to set `V2MINI_ROS_IP`. Note, it is necessary to export environment variables each time a new terminal is opened.

Now, launch teleop with the `remote` argument set to true:
```
roslaunch v2mini_teleop teleop.launch remote:=true
```

#### Controller Key-Bindings

The following describes the controller buttons and the resulting actions:

**keyboard controller:**

`arrows` - move the robot base forward, backward, left, right, or at a 45 degree angle.

`z & x` - rotate the robot base clockwise or counter-clockwise.

`w & s` - tilt the robot head up or down.

`a & d` - pan the robot head left or right.

`e` - toggle robot emotions.

`r & f` - move the torso up or down.

**logitec gamepad:**

`left-joystick` - move the robot base at any angle with varying speed.

`bottom-triggers` - rotate the robot base clockwise or counter-clockwise with varying speed.

`right-joystick` - tilt the robot head up or down, and pan left or right with varying speed.

`B` - toggle robot emotions.

`A & Y` - move the torso up or down.

#### Troubleshooting

Assuming teleop launches successfully with no errors, the following steps can be used to isolate malfunctioning components:

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
rosnode list
```

If `torsoware` and `baseware` nodes are listed, they are running.

After checking that the data is published and the arduino nodes are up, it's possible that there's an error in the firmware. Debugging the firmware can get quite involved, but to aid you there are commented-out debugger publishers in each. Just publish the data you wish to confirm, and check it using the described method.
