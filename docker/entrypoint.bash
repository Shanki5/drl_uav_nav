### Source ROS1
source /opt/ros/noetic/setup.bash

### Source workspace
source /root/drl_drone_ws/devel/setup.bash


## Appending source command to ~/.bashrc enables autocompletion (ENTRYPOINT alone does not support that)
grep -qxF '. "${DRL_DRONE_NAVDIR}/entrypoint.bash"' ${HOME}/.bashrc || echo '. "${DRL_DRONE_NAVDIR}/entrypoint.bash"' >>${HOME}/.bashrc
