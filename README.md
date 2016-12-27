# v2mini_robot

ROS stack for the V2Mini robot...

## 1. Setup

Install [ROS Indigo][ros-inst] on Ubuntu.

[ros-inst]:http://wiki.ros.org/indigo/Installation/Ubuntu

### Setting up the Environment

The steps in this section are required for v2mini's machine, or any machine used to directly interface with v2mini's external hardware.

#### Install udev Rules

asdfasdf ----------->  ----< TODO >-----

#### Install Apt Packages

install SDL2 .. what else????  ----< TODO >----- realsense camera drivers

### Creating a Workspace

A catkin workspace is required to build and run v2mini's packages. This section describes how to setup the workspace for both a dev machine and v2mini itself.

#### General Workspace Setup

1. Create a catkin workspace: </br>
    `$ mkdir -p ~/v2mini_ws/src && cd v2mini_ws/src` </br>
2. Initialize the workspace: </br>
    `$ catkin_init_workspace` </br>
3. Clone this repository: </br>
    `$ git clone <v2mini_robot>` </br>
4. Build the packages </br>
    `$ cd .. && catkin_make` </br>
5. Install package dependencies: </br>
    `$ ----< TODO >-----`


#### Additional Setup for V2Mini

asdf

### Editors & IDEs

The packages in this repository were developed with eclipse and atom.

#### Atom + Platformio

The platformio package for atom provide a linter while developing arduino firmware. This is much better than the arduino IDE because you'll have syntax feedback. The following is a list of steps for setup:   

1. Install the [atom][Atom] editor
2. Install the [Platformio][plat] package.
3. Create the config file: </br>
    `$ cd ~/v2mini_ws` </br>
    `$ touch platformio.ini` </br>
4. Install the 'ros_lib' package into the /lib directory: </br>
    `$ mkdir lib && cd lib` </br>
    `$ rosrun rosserial_arduino make_libraries.py .` </br>

Usage Notes:
- for platformio to work, you must open the v2mini_ws root directory in atom.
- copy additional third-party arduino libraries into the /lib directory so that platformio can identify syntax.

[Atom]:http://flight-manual.atom.io/getting-started/sections/installing-atom/#platform-linux

[plat]:http://docs.platformio.org/en/stable/ide/atom.html#installation

#### Eclipse

Eclipse is useful when developing nodes in c++. The following is a list of steps to get it working with ROS:

1. follow this tutorial to install [eclipse][ecli] on ubuntu.
2. run these [catkin elipse commands][ros-ecli] in section '2.2 Catkin-y approach' to enable ROS syntax in eclipse.
3. open eclipse and import the `v2mini_ws/build/` directory as a general existing project.

You should now be able to edit the packages under `[Source directory]` in the eclipse editor.

[ecli]:http://ubuntuhandbook.org/index.php/2016/01/how-to-install-the-latest-eclipse-in-ubuntu-16-04-15-10/

[ros-ecli]: http://wiki.ros.org/IDEs

## 2. Instructions on Use

All steps must be performed from workspace (ie. `$ cd ~/v2mini_ws` before running commands).

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

After opening a terminal, the workspace must be sourced to use v2mini's packages:

```
$ source devel/setup.bash
```

### Teleoping V2Mini

There are two methods of "teleoping" the V2Mini robot. The first method is to use TeamViewer, and the second method is to run the teleop nodes on a remote machine.

#### Control via TeamViewer

Using [TeamViewer][teamview] to remote into the V2Mini's PC is easier than setting up a secondary computer with ROS to control the robot. Moreover, it may be the only way if the user only has access to a Windows machine (besides running the robot headless).

Note, controllers such as the keyboard and gamepad must be directly attached to the V2Mini's PC, due to SDL not capturing the inputs from TeamViewer. This is basically remote control, which means the major limitation is range.

[teamview]:https://www.teamviewer.com/en/

#### Control via ROS-Enabled PC

Using a secondary linux machine with ROS, SDL2, and the v2mini_robot package installed allows for real teleop. That means the V2Mini can be controlled via wifi from any location.

Note, the setup for an additional control machine is the same as the V2Mini. Simply follow the setup provided above.  

#### Launching Teleop

To start Teleoping the V2Mini with a keyboard controller, use the following command:
```
roslaunch v2mini_teleop teleop.launch control:=keyboard
```
In addition to the `control` argument, there are a number of additional arguments that can be used:

`control`
- `keyboard` - control the robot using keyboard commands (default).
- `gamepad` - control the robot using the logitec gamepad.

`use_torso`
- `false` - ignore the torso firmware node. Use this option when you wish to run the robot without the torso.
- `true` - launch the firmware node for the torso (default). Note, if the torso arduino is not connected, it will result in an error.

`use_base`
- `false` - ignore the base firmware node. Use this option when you wish to run the robot without the base.
- `true` - launch the firmware node for the base (default). Note, if the base arduino is not connected, it will result in an error.

The following environment variables must be set to teleop the robot:

`ROS_IP` - the ip address for the machine. </br>
`USER` - the username for the machine.












d

d

d

d

d
aa
