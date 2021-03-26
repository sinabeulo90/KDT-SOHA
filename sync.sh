IP=10.42.0.1
# rsync -avu src/sliding_window "nvidia@$IP:/home/nvidia/smbyun0214_ws/src/"
rsync -avu --delete src/rally_3rd "nvidia@$IP:/home/nvidia/smbyun0214_ws/src/"
# rsync -avu --delete src "nvidia@$IP:/home/nvidia/smbyun0214_ws/"
# rsync -avu --exclude="src/darknet_ros" "nvidia@$IP:/home/nvidia/xycar_ws/src/xycar_device" /Users/sinabeulo90/Downloads/
# rsync -avu --exclude="src/darknet_ros" "nvidia@$IP:/home/nvidia/xycar_ws/src" /Users/sinabeulo90/Downloads/

#  kill -9 `ps -ef|grep python|awk '{print $2}'`