IP=10.42.0.1

# smbyun0214 workspace
# rsync -avu src/sliding_window "nvidia@$IP:/home/nvidia/smbyun0214_ws/src/"
# rsync -avu --delete src/rally_3rd "nvidia@$IP:/home/nvidia/smbyun0214_ws/src/"
# rsync -avu --delete "nvidia@$IP:/home/nvidia/smbyun0214_ws/src/rally_3rd/src" src/rally_3rd/
# rsync -avu --delete src "nvidia@$IP:/home/nvidia/smbyun0214_ws/"

# xycar workspace
# rsync -avu --exclude="src/darknet_ros" "nvidia@$IP:/home/nvidia/xycar_ws/src/xycar_device" /Users/sinabeulo90/Downloads/
# rsync -avu --exclude="src/darknet_ros" "nvidia@$IP:/home/nvidia/xycar_ws/src" /Users/sinabeulo90/Downloads/
# rsync -avu src/rally_3rd/src "nvidia@$IP:/home/nvidia/xycar_ws/src/rally_3rd/"
scp -r src/rally_3rd/src "nvidia@$IP:/home/nvidia/xycar_ws/src/rally_3rd/"
# scp -r src/usb_cam/calibration "nvidia@$IP:/home/nvidia/xycar_ws/src/usb_cam/"
# kill
# kill -9 `ps -ef|grep python|awk '{print $2}'`
