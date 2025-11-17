gnome-terminal -- bash -c "roslaunch rosbridge_server rosbridge_websocket.launch; exec bash"

gnome-terminal -- bash -c "rosrun bodyhub cmd_executor.py; exec bash"