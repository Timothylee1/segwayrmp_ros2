<launch>
  <include
    file="$(find gazebo_ros)/launch/empty_world.launch" />
  <node
    name="tf_footprint_base"
    pkg="tf"
    type="static_transform_publisher"
    args="0 0 0 0 0 0 base_link base_footprint 40" />
  <node
    name="spawn_model"
    pkg="gazebo_ros"
    type="spawn_model"
    args="-file $(find 401pius)/urdf/401pius.urdf -urdf -model 401pius"
    output="screen" />
  <node
    name="fake_joint_calibration"
    pkg="rostopic"
    type="rostopic"
    args="pub /calibrated std_msgs/Bool true" />
</launch>