#!/bin/bash

ROOT_PATH=$1

if [ -z ${ROOT_PATH} ]; then
  ROOT_PATH="./include"
fi

SIM_PROXY_PATH=${ROOT_PATH}"/../"
PROTOC="protoc"
MSGS_PATH=${SIM_PROXY_PATH}"/msgs"
TARGET=${ROOT_PATH}"/protobuf/"

mkdir -p $TARGET

MSG="any param param_v color "
MSG+="time vector2d vector3d quaternion pose pose_animation "
MSG+="imu image images_stamped image_stamped camerasensor distortion "
MSG+="laserscan laserscan_stamped "
MSG+="micom battery pointcloud gps "

for i in $MSG
do
  echo "Generate $i.proto"
  eval "$PROTOC -I $MSGS_PATH --cpp_out=$TARGET $MSGS_PATH/$i.proto"
done
