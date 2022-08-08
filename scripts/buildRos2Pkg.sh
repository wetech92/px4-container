#! /bin/bash

rosWorksapce=/root/ros_ws

colcon build \
  --build-base ${rosWorksapce}build \
  --install-base ${rosWorksapce}install \
  --base-paths ${rosWorksapce}src \
  --event-handlers console_direct+

# entry="${entry%/}"
# echo ${entry##*/}

#   if [ "${SHELL}" = "/usr/bin/zsh" ]; then
#     echo "ZSH Detected"
#     echo "source ${entry}/install/install.sh" >> /root/.zshrc
#   elif [ "${SHELL}" = "/usr/bin/bash" ]; then
#     echo "BASH Detected"
#     echo "source ${entry}/install/install.sh" >> /root/.bashrc
#   else
#     echo "Unidentified shell! Exiing script."
#     exit 1
#   fi

# for entry in "${rosWorksapce}/src/*/"
# do
#   entry="${entry%/}"    
#   echo "ROS2 package ${entry##*/} processed!"
# done
