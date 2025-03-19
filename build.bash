#!/bin/bash

set -e

CURRENT_PATH=$(cd $(dirname $0); pwd)
CLIENT_SDK_PATH=${CURRENT_PATH}/src/seyond_lidar_ros/src/seyond_sdk/

echo -e "\n\033[1;32m-- (a). Clean the old build files...\033[0m"
rm -rf ${CURRENT_PATH}/build ${CURRENT_PATH}/install ${CURRENT_PATH}/devel ${CURRENT_PATH}/log
rm -rf ${CURRENT_PATH}/src/CMakeLists.txt

echo -e "\n\033[1;32m-- (b). Check the Seyond SDK...\033[0m"
if [[ -e "${CLIENT_SDK_PATH}/lib/libinnolidarsdkclient.so" ]]; then
  echo "The inno_sdk has already been compiled, but please verify it is compatible with the current platform"
else
  # build cilent sdk
  echo "The inno_sdk has not been compiled, start to compile the inno_sdk"
  cd ${CLIENT_SDK_PATH}/build
  shared=1 ./build_unix.sh
  echo "build status $?"
  cd -
fi


echo -e "\n\033[1;32m-- (c). Build the driver...\033[0m"
if [[ "$ROS_VERSION" -eq 1 ]]; then
  catkin_make install
elif [[ "$ROS_VERSION" -eq 2 ]]; then
  colcon build
else
    echo "Can't find ROS_VERSION or ROS2_VERSION"
fi
echo -e "\n\033[1;32m-- (d). Build Done!\033[0m"
