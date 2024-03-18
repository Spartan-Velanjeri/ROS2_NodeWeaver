## Folders in this package
- doc
- launch
- rviz
- world

### doc
- This folder contains the bautiro simulation model drawing and the README document.
- The drawing provides an overall idea of how the robot is implemented.

### launch
- This folder contains the file required to launch the robot in a gazebo environment.
- The launch file spawns the robot in a user-built gazebo world (sdf file) and launches all the nodes required to control the robot.
- This is the only launch file required for the whole simulation.

### rviz
- This folder contains the user-defined rviz configuration.

### world
- This folder contains the gazebo world sdf file and meshes to be launched in the gazebo.
- lab_mesh.stl file represents the complete building model.
- ramp.stl is used to test the imu sensor.
- All other meshes are just used for testing and are not required.

    _gazebo_world.sdf_
    - Other than just loading the lab_mesh.stl and ramp.stl, the world file serves more important functions.
    - The world file contains **marker_n** links, where 'n' is a natural number (marker_1, marker_2,...)
    - The poses of the markers (with respect to the gazebo origin) are published through the **libLocalPosePublisher.so** plugin, included at the end of the sdf file.
    - These marker positions are used by a ros2 service, which calculates the position of the marker with respect to leica.
    - The marker's poses are then used to figure out the accurate position of the robot in the world.
    <br><br>
    - The marker_n is also published through static_transform_publisher node in the launch file. This is only done to publish the marker_n in the tf tree and traverse through it.