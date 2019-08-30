# ENPH353 ROS & GAZEBO LAB

## Objectives
- Learn how to create simulated worlds
- Learn how to create simulated robots
- Understand how to launch basic ROS systems
- Control a simulated robot from sensor feedback

## Tasks
1. Setup a ROS workspace and clone ENPH353 lab resources.
2. Create a simple simulated track for line following.
3. Create a differential drive robot.
4. Launch a ROS system to test controlling the robot.
5. Integrate computer vision based line detection in a ROS node for a PID line follower.
6. Demonstrate the robot is able to line follow and complete three laps of your track.

## Steps

### Workspace setup

Setup a ROS workspace from the command line.
```
cd ~
mkdir -p enph353_ws/src
cd enph353_ws
catkin_make
```
The creating a workspace process is also documented on the ROS wiki's [create_a_workspace](http://wiki.ros.org/catkin/Tutorials/create_a_workspace) tutorial. For even more tutorials check the [ROS wiki](http://wiki.ros.org/ROS/Tutorials).

Whenever working with ROS, one must source the active workspace in each terminal to ensure path and environment variables are set. Source the relevant workspace when you open a new terminal window or tab.

`source ~/enph353_ws/devel/setup.bash`

While it is possible to add this command to your `./bashrc` file to ensure each new terminal is sourced, this can cause problems when switching between workspaces or after rebuilding a workspace. For convenience you could add the following alias to `./bashrc` which allows you to type `source353` to source this workspace.

```
sed -i -e "\$aalias source353=\'source ~\/enph353_ws\/devel\/setup.bash\`" ~/.bashrc
source ~/.bashrc
```

Check that this has added an alias equivalent to sourcing the workspace at the bottom of your `./bashrc` file.
```
cat ~/.bashrc
```

Clone the ENPH353 ROS & Gazebo lab repository.
```
cd ~/enph353_ws/src
git clone https://github.com/ENPH353/enph353_ROS_lab.git
```

### Creating a simulated world

ENPH353 uses the most common simulator for ROS: Gazebo. Although Gazebo and ROS are both maintained by [Open Robotics](https://www.openrobotics.org/) they are stand alone tools and follow different standards.

Gazebo uses [SDF](http://sdformat.org/) to configure simulated elements including robots, objects, and the environment. You can modify the appearance or physics of any object in a gazebo world through tuning parameters in an SDF file. ROS uses [URDF](http://wiki.ros.org/urdf) to model robots, which has common elements with SDF but has a larger focus on configuring robot kinematics instead of object physics. A typical ROS robot will be built using URDF and converted to SDF when launching a simulation while most other passive objects can be described and added to a simulated world with SDF alone.


A basic world can be constructed by including models in the world description file. A skeleton world file is provided. Check the file to see the basic structure and note that the world file simply includes several models defined elsewhere.
```
cat ~/enph353_ws/src/enph353_ROS_lab/worlds/353_ros_lab.world
```

It is also possible to use SDF to describe a model directly in the world file, however this reduces the reusability of generic models. The lab world contains two models: the sun and a track. The sun is a standard gazebo model, while the track is our custom surface which we texture with an image to create a basic 2D world for our robot to navigate.

View the track model to see how a planar surface is defined in SDF. Note that this model has single link with visual and collision element, both are required render the link. With URDF there is an additional requirement that each link has an inertial element. Both SDF and URDF do not require the visual and collision elements to match, this can be used to create objects that have complex visual appearance but simple collision geometery to reduce the computation required to detect collisions. Similary one can create visually large objects with infinitesimally small collision boxes if the purpose of the model is purely to provide a visual reference in the world.

```
cat ~/enph353_ws/src/enph353_ROS_lab/models/track/track.sdf
```

Note that the plane visual element is defined with a "track" texture. You will need to add an image to the texture definition. Several track images are provided in the media/materials/textures directory. Locate the `track.material` file and replace the <image_name> tag with the file name of your desired texture.

### Launching a ROS system

A ROS system consists of many nodes from self developed or off the shelf packages. It is possible to run a single node from the command line by using the [rosrun](http://wiki.ros.org/rosbash#rosrun) command, however in practice this is automated through launch files. This is done since nodes typically have parameters which are set at run time and managing parameters through the command line would be extremely difficult.

Many packages provide launch files to allow a user to conveniently run nodes by adjusting parameters within the launch file. To understand how to spawn nodes through a launch file using `roslaunch` read [section 1](http://wiki.ros.org/roslaunch/Commandline%20Tools) from the roslaunch command line tools wiki.

A key point to understand about launch files is they are a means of ensuring modularity in a ROS system. It is possible to include a launch file in another launch file, thus allowing for reuse of common nodes through a single launch file.

Read through Clearpath Robotics' [tutorial](http://www.clearpathrobotics.com/assets/guides/ros/Launch%20Files.html) which will explain how to create a launch file. Create a new directory: `~/enph353_ws/src/enph353_ROS_lab/launch` and write your own launch file to spawn a world with the correct track surface. 

Include the following snippet in your launch file to spawn your simualted world.
```
<include file="$(find gazebo_ros)/launch/empty_world.launch">
	<env name="GAZEBO_RESOURCE_PATH" value="$(find enph353_ROS_lab)"/>
	<env name="GAZEBO_MODEL_PATH" value="$(find enph353_ROS_lab)/models"/>
	<arg name="world_name" value="$(find enph353_ROS_lab)/worlds/353_ros_lab.world"/>
	<arg name="gui" value="true"/>
</include>
```
When construction more advanced launch files with conditionals and environment variables consult the official [roslaunch xml](http://wiki.ros.org/roslaunch/XML) documentation.

**Instructor Checkin** Demonstrate that you are able to launch a world file with a track texture.


### Creating a simulated robot

Our ROS robot is created using URDF; you can find the full specification is described on the ROS wiki [XML Spec](http://wiki.ros.org/urdf/XML). Specifically review the following:

- [robot](http://wiki.ros.org/urdf/XML/robot)
- [link](http://wiki.ros.org/urdf/XML/link)
- [joint](http://wiki.ros.org/urdf/XML/joint)
- [gazebo plugins](http://gazebosim.org/tutorials?tut=ros_gzplugins)

Essentially the robot consists of links such as wheels, arms, chassis, etc. in a tree structure where each link is connected to another through a joint that describes the frame of reference for the next joint and the allowable degrees of freedom.

Before you start building your own robot you should review the ROS wiki's [tutorial](http://wiki.ros.org/urdf/Tutorials/Building%20a%20Visual%20Robot%20Model%20with%20URDF%20from%20Scratch) for building the visual elements of a basic URDF robot. Following this, review the ROS wiki's [tutorial](http://wiki.ros.org/urdf/Tutorials/Adding%20Physical%20and%20Collision%20Properties%20to%20a%20URDF%20Model) for defining physical properties of your robot.

```
<include file="$(find enph353_ros_lab)/launch/robot.launch">
	<arg name="init_pose" value="-x 0.0 -y -0.673 -z 1.0 -R 0.0 -P 0.0 -Y 4.71" />
</include>
```