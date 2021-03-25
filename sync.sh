IP=10.42.0.1
rsync -avu src/sliding_window "nvidia@$IP:/home/nvidia/smbyun0214_ws/src/"
# rsync -avu "nvidia@$IP:/home/nvidia/smbyun0214_ws/src/sliding_window/" src/sliding_window