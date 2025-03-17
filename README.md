## Rangefinder plugin package 

Plugin library to handle and read information from rangefinder sensor in MuJoco Simulation.

After compiling the package in your ros2 workspace: 

## REQUIREMENTS

To use in a simulation involving a Franka Emika Panda robot

-Requires to use the following package: https://github.com/tenfoldpaper/multipanda_ros2

## USING THE PACKAGE 

To work in a single manipulator, in multipanda_ros2 packages, inside franka_bringup/config/sim, modify `single_sim_controllers.yaml` as follows:

mujoco_server:
  ros__parameters:
    MujocoPlugins:
      names:
        - mujoco_rangefinder    <-Line to find the rangefinder plugin package
        - mujoco_ros2_control
      mujoco_rangefinder:       
        type: mujoco_ros::sensors::MujocoRosRangeFinder    <-Line to define the type of namespace referred to the rangefinder plugin
      mujoco_ros2_control:
        type: mujoco_ros2_control::MujocoRos2ControlPlugin
        hardware:
          type: mujoco_ros_control/RobotHW
          control_period: 0.001
        params:
          namespace: ""
          robot_description_node: "robot_state_publisher"
          robot_description: "robot_description"
          
          
mujoco_rangefinder:     <-Configure the plugin
  ros__parameters:
    test_name: MujocoRos2SensorsName
    rangefinder: 
      - rfsensor <-rangefinder site name to define in the Franka Emika Panda mujoco .xml file
    rfsensor:
      rf_count: 4 <-Number of sensor attached to the robot
      
## MODIFY ROBOT MUJOCO .XML FILE

Inside the multipanda_ros2 packages, go to franka_description/mujoco/franka/panda.xml, and insert configure the rangefinder sensors

## EXAMPLE: ATTACH 4 RANGEFINDER SENSOR IN ROBOT 5TH LINK

Search in panda.xml file the `panda_link5` body name, under this line add the rangefinder site name as follows:

         <site name="rfsensor0" pos="0.05 -0.05 -0.192" quat="0.7071 0.7071 0 0"/>
         <site name="rfsensor1" pos="0.05 -0.05 -0.182" quat="0.7071 0.7071 0 0"/>
         <site name="rfsensor2" pos="0.05 -0.05 -0.172" quat="0.7071 0.7071 0 0"/>
         <site name="rfsensor3" pos="0.05 -0.05 -0.162" quat="0.7071 0.7071 0 0"/>
 
Finally, define the rangefinder sensor as follows

 <sensor>
    <rangefinder site="rfsensor0"/>
    <rangefinder site="rfsensor1"/>
    <rangefinder site="rfsensor2"/>
    <rangefinder site="rfsensor3"/>
  </sensor>



      
      
      
      
      
      
      
      
      
      
      
      
