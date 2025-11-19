#! /bin/sh

### BEGIN INIT INFO
# Provides:          app_start.sh
# Required-Start:    $all
# Required-Stop:
# Default-Start:     2 3 4 5
# Default-Stop:
# Short-Description: Run /etc/app_start.sh if it exist
### END INIT INFO

export ROS_HOSTNAME=127.0.0.1
cd ~/robot_ros_application/scripts/check_routine
python3 stop_buzzer.py
file="`readlink "$0"`"
if [ ! $? -o "$file" = "" ] ; then
   file="$0"
fi
cd $(dirname "$file")
# ([ `ps -aux | grep x11vnc | awk '{print $2}' | wc -l` -lt 2 ] && which x11vnc && nohup x11vnc -auth guess -once -loop -noxdamage -repeat -rfbauth /home/lemon/.vnc/passwd -rfbport 5900 -shared) > /tmp/vnc.log 2>&1 &
setserial /dev/usb_servo low_latency
amixer -D pulse set Master 90%

# run mqtt broker
bash ~/robot_ros_application/scripts/run_emqx.sh
# run ros_bridge
bash ~/bridge.sh
cd /home/lemon/robot_ros_application/scripts
play -q start.mp3
python detect_network.py
cd servo_scan/ && python3 servo_check.py
. /home/lemon/robot_ros_application/catkin_ws/devel/setup.sh
cd ~/robot_ros_application/scripts/
touch /tmp/launch_ros_flag
echo "softdev" | sudo -S bash bodyhub.sh &

while ! rosnode list
do
    sleep 1s
done 
arecord -l | grep -qE "(AC108)|(Bothlent)"
if [ $? -eq 0 ] ; then
    roslaunch ros_socket_node run_hlw_mic.launch
else
    roslaunch ros_socket_node run.launch
fi
