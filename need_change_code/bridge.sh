gnome-terminal -- bash -c "source ~/robot_ros_application/catkin_ws/devel/setup.bash && roslaunch rosbridge_server rosbridge_websocket.launch; exec bash"

gnome-terminal -- bash -c "source ~/robot_ros_application/catkin_ws/devel/setup.bash && rosrun bodyhub cmd_executor.py; exec bash"