IP=10.42.0.1
# rsync -avu --delete "nvidia@$IP:/home/nvidia/smbyun0214_ws/src/sliding_window/src/image" image
rsync -avu --delete --exclude="src/backup" --exclude="src/sliding_window/src/image" src "nvidia@$IP:/home/nvidia/smbyun0214_ws/"
