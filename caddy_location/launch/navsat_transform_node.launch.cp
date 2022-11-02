<launch>
  <!-- IMU, GPS topic launch -->
  <include file="$(find turtlebot3_bringup)/launch/turtlebot3_robot.launch"/>
  <include file="$(find ublox_gps)/launch/ublox_device.launch"/>
  
  <!-- STATIC_TRANSFORM_PUBLISHER -->
  <node pkg="tf2_ros" type="static_transform_publisher" name="base_to_imu" args="0 0 0 0 0 0 1 base_link imu_link"/>
  <node pkg="tf2_ros" type="static_transform_publisher" name="base_to_gps" args="0 0 0 0 0 0 1 base_link gps_link"/>
  
  <!-- NAVSAT_TRANSFORM_NODE -->
  <node pkg="robot_localization" type="navsat_transform_node" name="navsat_transform_node" respawn="true">
    <param name="magnetic_declination_radians" value="0.23788838"/>
    <param name="yaw_offset" value="0"/>
    <param name="zero_altitude" value="false"/>
    <param name="broadcast_utm_transform" value="true"/>
    <param name="publish_filtered_gps" value="true"/>
    <param name="use_odometry_yaw" value="false"/>
    <param name="wait_for_datum" value="false"/>
    <!-- OUTPUT TOPICS -->
    <remap from="/odometry/gps" to="/odometry/filtered_gps"/>
    <remap from="/gps/filtered" to="/gps/filtered"/>
    <!-- INPUT TOPICS -->
    <remap from="/imu/data" to="/imu" />
    <remap from="/gps/fix" to="/ublox/fix" />
    <remap from="/odometry/filtered" to="/odom" />
    <!-- <remap from="/odometry/filtered" to="/odometry/filtered_global" /> -->

  </node>
  <!-- GLOBAL_EKF_LOCALIZATION -->
  <node pkg="robot_localization" type="ekf_localization_node" name="ekf_localization_global" clear_params="true">
    <!-- OUT PUT -->
    <remap from="/odometry/filtered" to="/odometry/filtered_global"/>
    <!-- <param name="frequency" value="30"/> -->
    <param name="frequency" value="30"/>
    
    <param name="world_frame" value="map"/>
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
    <!-- <param name="imu0_remove_gravitational_acceleration" value="true"/> -->
    <!-- <param name="imu0_nodelay" value="true"/> -->

    <!-- VISUAL ODOMETRY -->
    <param name="odom0" value="/odom"/>
    <rosparam param="odom0_config">[false, false, false,
                                    false, false, false,
                                    true,  true,  true,
                                    false, false, false,
                                    false, false, false]</rosparam>
    <!-- <param name="odom0_nodelay" value="true"/> -->

    <!-- GPS ODOMETRY -->
    <!-- <param name="odom1_nodelay" value="true"/> -->
    <param name="odom1" value="/odometry/gps"/>
    <rosparam param="odom1_config">[true,  true, false,
                                   false, false, false,
                                   false, false, false,
                                   false, false, false,
                                   false, false, false]</rosparam>
    <!-- <param name="odom1_nodelay" value="true"/> -->
  </node>

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
    <rosparam param="odom0_config">[true,  true,  false,
                                    false, false, false,
                                    false, false, false,
                                    false, false, true,
                                    false, false, false]</rosparam>
    <!-- <param name="odom0_nodelay" value="true"/> -->
  </node>
</launch>