# Perception-based-robotic-arm-grasping
Hello, I am Nitin, and I am currently documenting a project on Perception-Based Robot Manipulation and Grasping. In this project, I am utilizing the Panda Emika robotic arm in conjunction with the Intel RealSense D435 Depth camera. Initially, I implemented a Depth camera Plugin for simulation, attaching the camera to the world frame space. The primary objective is to use this camera to aid in detecting the grasp pose for object manipulation. Motion planning will be performed using the MoveIt API.

### **Steps to Follow for executions:**
* Install Moveit on your system from this link : [Moveit](https://ros-planning.github.io/moveit_tutorials/doc/getting_started/getting_started.html)
   * You can try checking if Moveit install correctly by running this command in terminal, after sourcing the workspace:
     `roslaunch panda_moveit_config demo_gazebo.launch` this should open rviz and gazebo with moveit configuration for           panda emika robotic arm:
     This should open the Gazebo with robotic arm but without rgbd camera and rviz with moveit configuration:
     
     <img src="Images/GazeboPandaEmika.png" alt="Gazebo Moveit Config" width="500">
     <img src="Images/RvizPandaEmika.png" alt="Gazebo Moveit Config" width="500">
     
* Fork this Repo: `https://github.com/thakkarnitin63/Perception-based-robotic-arm-grasping.git`
* In Repo, I have got Intel Realsense Plugin get that folder outside this folder(Perception-based-robotic-arm-grasping):`realsense_ws`
  * Open terminal follow this command `cd realsense_ws` then `catkin make` this will build the catkin package       and your intel realsense plugin will start working to simulate the depth camera in gazebo since we are not      using real hardware
  * Now, `cd rgbd_ws` and do catkin make using `catkin make` after that you will have working workspace for realsense and rgbd workspace.
  * Installation for Franka ros robotic arm files `sudo apt install ros-noetic-libfranka ros-noetic-franka-ros` run this command to get install franka ros files so we can later on change this files to attch camera module to robotic arm.
  * Locate `franka_description` in your `computer/opt/ros/noetic/share/` go to `franka_description/robots/panda/` and perform this changes to given file by the name `panda.urdf.xacro`. This change will enable to add camera to your Gazebo file for Franka arm.
  ```<?xml version='1.0' encoding='utf-8'?>
  <robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="panda">

  <xacro:include filename="$(find franka_description)/robots/common/franka_robot.xacro"/>

  <xacro:arg name="arm_id" default="panda" />

  <xacro:franka_robot arm_id="$(arg arm_id)"
                      joint_limits="${xacro.load_yaml('$(find franka_description)/robots/panda/joint_limits.yaml')}">
  </xacro:franka_robot>

  <xacro:include filename="/your address where you clone this/realsense_ws/src/realsense2_description/urdf/_d435.urdf.xacro"/> 
  <xacro:sensor_d435 name="camera" topics_ns="camera" parent="panda_link0" publish_pointcloud="true">
  <origin xyz="0.65 -0.05 0.75" rpy="0  1.5708 0.0"/>
  </xacro:sensor_d435>

  </robot>```

After performing these step when you launch your panda config file you can see the camera on the given distance to robotic arm and ready to provide feed with gazebo plugin of intel realsense `roslaunch panda_moveit_config demo_gazebo.launch` in the file I am also attaching the urdf model of cad designed bottle which can be used for grasping you can simply put in `gazebo/models` and your model will be ready to import in gazebo.

**Steps to perform grasping**
* All the prerequisites are completed. Launch the gazebo model `roslaunch panda_moveit_config demo_gazebo.launch` then go to rviz and you can add the camera feed in rviz by clicking on `Add` and move to `By topic` select `/camera/color/image_raw` to see the feed of camera. 
* Import the bottle in gazebo within camera vision by clicking `Insert` and getting the `BottleModel`. This will enale to add bottle to world.
* Lastly run this command after sourcing the bash `rosrun rgbd_ws rgbd_process.py` and in another terminal `rosrun rgbd_ws PicknPlace` This will run the program to pick the bottle with perception knowledge of robotic arm.  

    

**Author-Nitin Thakkar**
