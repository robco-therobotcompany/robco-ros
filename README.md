# robco-ros
ROS packages for Robco robots

# Installation

1. Clone the repository into your catkin workspace: `git clone https://github.com/robco-therobotcompany/robco-ros.git ~/catkin_ws/src/robco-ros`
2. Install dependencies using `rosdep`: `cd ~/catkin_ws && rosdep install --from-paths src --ignore-src -r -y`
3. Install `librobcomm` according to its instructions [here](https://github.com/robco-therobotcompany/librobcomm)
4. Build the package: `cd ~/catkin_ws && catkin_make`

# Usage

The package provides the `robco_hw_node` which runs the hardware interface for a robot. See the launchfiles in the `robco_hw` package for an example of how to start it.

The `robco_description` package provides a URDF library for all of our robot modules. This way, any module configuration can be easily represented using the pre-defined macros. For example:

```xml
<?xml version="1.0" ?>

<robot name="robco" xmlns:xacro="http://www.ros.org/wiki/xacro">
    <xacro:include filename="$(find robco_description)/urdf/robco_modules.xacro"/>

    <xacro:robco_module_0106 name="base" next="drive0"/>
    <xacro:robco_module_0088 name="drive0" next="drive1"/>
    <xacro:robco_module_0089 name="drive1" next="link0"/>
    <xacro:robco_module_0025 name="link0" next="drive2"/>
    <xacro:robco_module_0085 name="drive2" next="link1"/>
    <xacro:robco_module_0022 name="link1" next="drive3"/>
    <xacro:robco_module_0097 name="drive3" next="drive4"/>
    <xacro:robco_module_0087 name="drive4" next="effector"/>
    <xacro:robco_module_0801 name="effector"/>
</robot>
```

This results in the following robot model:

![image](https://github.com/robco-therobotcompany/robco-ros/assets/131398638/203a5df6-f14e-4710-8ced-348054d7385c)
