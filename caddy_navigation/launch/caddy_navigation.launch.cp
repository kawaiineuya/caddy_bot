<launch>
    <!-- Arguments -->
    <arg name="model" default="$(env TURTLEBOT3_MODEL)" doc="model type [burger, waffle, waffle_pi]"/>
    <arg name="open_rviz" default="true"/>
    <arg name="move_forward_only" default="false"/>

    <node pkg="tf2_ros" type="static_transform_publisher" name="base_link_to_camera_link"
        args="0.7 0 0.6 0 0 0 base_link camera_link"/>

    <include file="$(find turtlebot3_bringup)/launch/turtlebot3_robot.launch"/>
    <include file="$(find realsense2_camera)/launch/rs_camera.launch">
        <arg name="align_depth" value="true"/>
        <arg name="depth_width" value="424"/>
        <arg name="depth_height" value="240"/>
        <arg name="depth_fps" value="30"/>
        <arg name="color_width" value="424"/>
        <arg name="color_height" value="240"/>
        <arg name="color_fps" value="30"/>
    </include>
    <!-- <include file="$(find realsense2_camera)/launch/rs_camera.launch"/> -->
    
    <include file="$(find depthimage_to_laserscan)/launch/launchfile_sample.launch"/>

    <!-- Turtlebot3 -->
    <include file="$(find turtlebot3_bringup)/launch/turtlebot3_remote.launch">
        <arg name="model" value="$(arg model)" />
    </include>

    <!-- Map server -->
    <include file="$(find caddy_navigation)/launch/start_map_server.launch"/>
    <include file="$(find caddy_navigation)/launch/slam_gmapping_pr2.launch"/>

    <!-- AMCL -->
    <include file="$(find caddy_navigation)/launch/amcl.launch"/>

    <!-- move_base -->
    <include file="$(find caddy_navigation)/launch/move_base.launch">
        <arg name="model" value="$(arg model)" />
        <arg name="move_forward_only" value="$(arg move_forward_only)"/>
    </include>

    <!-- rviz -->
    <!-- <group if="$(arg open_rviz)"> 
        <node pkg="rviz" type="rviz" name="rviz" required="true"
            args="-d $(find caddy_navigation)/rviz/turtlebot3_navigation.rviz"/>
    </group> -->
</launch>