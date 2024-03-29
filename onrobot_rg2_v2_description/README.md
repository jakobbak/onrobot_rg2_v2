This package is **Work in Progress**

This ROS2 package provides the core description data for the OnRobot RG2 gripper in its V2 iteration.

It is primarily intended to be used with UR robots, and as such it has not been checked if it fits with other types of robots.

The gripper model is developed in Fusion 360 from the STEP files of the gripper from the manufacturer's website. These has been segmented into functional parts, and from these parts relevant joint connections has been created. It is then exported to URDF using the [fusion2urdf](https://github.com/SpaceMaster85/fusion2urdf) script.

The current version of the gripper URDF was generated by an older version of the script (Version 1.1 if we go according to the date of the [commit](https://github.com/jakobbak/onrobot_rg2_v2/commit/247efa6a0224ab0c28a3fc585ab0e1187b96ec4c)). For that version it was necessary to make some changes to the URDF so that it would work with the intended use.

Therefore a copy of the exported URDF (`onrobot_rg2_v2.urdf.xacro`) was made, called `rg2_v2.urdf.xacro`, which avoid the "working" URDF to get overwritten upon new exports from the `fusion2urdf` script.

The changes made between the two files is illustrated below in screenshots.
Please be aware that the new file in this repo is called `rg2_v2_macro.xacro`, to follow the convention implicitly set out in the [`Universal_Robots_ROS2_Description`](https://github.com/UniversalRobots/Universal_Robots_ROS2_Description/tree/a8ac97b4cb6afa554af7f4928c934ce531f648f4/urdf) repo, and that `rg2_v2.urdf.xacro` is a convenience URDF to be loaded by Moveit Setup Assistant, to create a `moveit_config` for the gripper in isolation.

### Screenshots of changes from `onrobot_rg2_v2.urdf.xacro` -> `rg2_v2_macro.xacro`
![image](docs/images/onrobot_rg2_v2_xacro_vs_rg2_v2_macro_xacro%20Screenshot%20from%202022-09-09%2013-36-36.png) First we wrap the whole URDF in a Xacro Macro to make it easy to call from another file, without having to worry about argument name duplication, etc.
``` xml
<xacro:macro name="rg2_v2" params="prefix parent *origin">
```
Then we remove the trailing `_1` that Fusion added to all the links, and prefix these link names with `${prefix_}`, so that the links in the gripper can have the same `prefix` as the UR they are attached to (think `left` or `right`, for example).

With regards to using a `<xacro:macro` instead of just a `<xacro:include>`, then we must admit that the choice was as much based on confusion at the time it was decided, and it is possible it will make more sense to use the include version instead.

![image](docs/images/onrobot_rg2_v2_xacro_vs_rg2_v2_macro_xacro%20Screenshot%20from%202022-09-09%2013-37-26.png)

![image](docs/images/onrobot_rg2_v2_xacro_vs_rg2_v2_macro_xacro%20Screenshot%20from%202022-09-09%2013-37-39.png)

![image](docs/images/onrobot_rg2_v2_xacro_vs_rg2_v2_macro_xacro%20Screenshot%20from%202022-09-09%2013-37-53.png)

![image](docs/images/onrobot_rg2_v2_xacro_vs_rg2_v2_macro_xacro%20Screenshot%20from%202022-09-09%2013-38-19.png) Once we get to the `<joint>` elements we add the same `${prefix}_` and introduce the `<mimic>` tag to specify passive joints that are not actuated directly. Please note that the `multiplier` property of the the `<mimic>` tag is sometimes set to `1` and sometimes `-1`, due to the geometry. It's a bit of a headscratcher to get the links and joints positioned in a way where we would not need to be careful with putting `1` and `-1` there. I think we did try to minimise any confusion there, but ultimately it can be hard to see the pattern in it. Just make the changes exactly like here.

![image](docs/images/onrobot_rg2_v2_xacro_vs_rg2_v2_macro_xacro%20Screenshot%20from%202022-09-09%2013-38-33.png) Add the `<joint>` that fixes the gripper to the parent manipulator (passed in as parameters to the macro), and close out the `<xacro:macro />` element.
