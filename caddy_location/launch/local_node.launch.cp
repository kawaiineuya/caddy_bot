<launch>
  <include file="$(find turtlebot3_bringup)/launch/turtlebot3_robot.launch"/>
  <node pkg="tf2_ros" type="static_transform_publisher" name="base_to_imu" args="0 0 0 0 0 0 1 base_link imu_link"/>
  <node pkg="tf2_ros" type="static_transform_publisher" name="base_to_gps" args="0 0 0 0 0 0 1 base_link gps_link"/>
  <!-- <node pkg="tf2_ros" type="static_transform_publisher" name="base_to_camera" args="0 0 0 0 0 0 0 base_link camera_link"/> -->

  <!-- LOCAL_EKF_LOCALIZATION -->
  <node pkg="robot_localization" type="ekf_localization_node" name="ekf_localization_local" clear_params="true">
    <remap from="/odometry/filtered" to="/odometry/filtered"/>
    <param name="frequency" value="30"/>
    <!-- <param name="frequency" value="30"/> -->

    <param name="world_frame" value="odom"/>
    <param name="map_frame" value="map"/>
    <param name="odom_frame" value="odom"/>
    <param name="base_link_frame" value="base_link"/>

    <!-- IMU -->
    <param name="imu0" value="/imu"/>
    <rosparam param="imu0_config">[false, false, false,
                                   true,  true,  true,
                                   false, false, false,
                                   true,  true,  true,
                                   <!-- true,  true,  true]</rosparam> -->
                                   false,  false,  false]</rosparam>
    <!-- <param name="imu0_nodelay" value="true"/> -->

    <!-- VISUAL ODOMETRY -->
    <param name="odom0" value="/odom"/>
    <!-- <param name="odom0" value="/odom_source/realsense/odom"/> -->
    <rosparam param="odom0_config">[true,  true,  false,
                                    false, false, false,
                                    false, false, false,
                                    false, false, true,
                                    false, false, false]</rosparam>
    <!-- <param name="odom0_nodelay" value="true"/> -->
  </node>
</launch>