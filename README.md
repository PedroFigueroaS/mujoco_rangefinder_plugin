## Rangefinder plugin package 

Plugin library to handle and read information from rangefinder sensor in MuJoco Simulation.

After compiling the package in your ros2 workspace: 

## REQUIREMENTS

To use in a simulation involving a Franka Emika Panda robot

-Requires to use the following package: https://github.com/tenfoldpaper/multipanda_ros2

## USING THE PACKAGE 

To work in a single manipulator, in multipanda_ros2 packages, inside franka_bringup/config/sim, modify `single_sim_controllers.yaml` as follows:

Under label names:
- mujoco_rangefinder

Configuring mujoco_rangefinder:

mujoco_rangefinder:       
        type: mujoco_ros::sensors::MujocoRosRangeFinder

To configure the plugin:

mujoco_rangefinder:

  ros__parameters:
    test_name: MujocoRos2SensorsName
    rangefinder: 
      - rfsensor
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



      
      
      
      
      
      
      
      
      
      
      
      
