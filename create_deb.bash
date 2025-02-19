#!/bin/bash

if [ -z "$ROS_VERSION" ] || [ -z "$ROS_DISTRO" ]; then
  echo "ROS_VERSION or ROS_DISTRO is not set. Please source the ROS environment."
  exit 1
fi

echo "ROS_VERSION: $ROS_VERSION"
echo "ROS_DISTRO: $ROS_DISTRO"

CURRENT_DIR=$(cd $(dirname $0); pwd)
PROJECT_DIR=${CURRENT_DIR}
DEB_ROS_ROOT=opt/ros/${ROS_DISTRO}

# result output
OUTPUT_DIR=${CURRENT_DIR}/output
mkdir -p ${OUTPUT_DIR}
cd ${OUTPUT_DIR}

# build in local
DEB_VERSION=1.0.0
SOFT_VER=${DEB_VERSION}

# build deb
pushd ${OUTPUT_DIR}
rm -fr deb_root
mkdir -p deb_root/DEBIAN
mkdir -p deb_root/${DEB_ROS_ROOT}

if [ "$ROS_VERSION" == "1" ]; then
  cp -rp ${PROJECT_DIR}/install/share deb_root/$DEB_ROS_ROOT/
  cp -rp ${PROJECT_DIR}/install/lib deb_root/$DEB_ROS_ROOT/
elif [ "$ROS_VERSION" == "2" ]; then
  cp -rp ${PROJECT_DIR}/install/seyond/share deb_root/$DEB_ROS_ROOT/
  cp -rp ${PROJECT_DIR}/install/seyond/lib deb_root/$DEB_ROS_ROOT/
else
  echo "the ros version has not support yet!"
  exit -1
fi

echo Package: seyond-${ROS_DISTRO}-driver > deb_root/DEBIAN/control
echo Version: ${SOFT_VER} >> deb_root/DEBIAN/control

# For better clarity and differentiation, use 'amd64' or 'arm64' instead of 'all'.
echo Architecture: all >> deb_root/DEBIAN/control

echo Maintainer: Seyond >> deb_root/DEBIAN/control
echo Description: Seyond ROS Driver >> deb_root/DEBIAN/control
echo "Build-Time: $(date -R)" >> deb_root/DEBIAN/control

chmod -R 755 deb_root
DEB_RESULT_FILE_PREFIX_PARTIAL=seyond-ros${ROS_VERSION}-${ROS_DISTRO}-${SOFT_VER}
dpkg-deb -z 0 --build deb_root ${DEB_RESULT_FILE_PREFIX_PARTIAL}.deb
rm -rf deb_root
rm -rf ${CURRENT_DIR}/deb_control
ls -lrt
popd
