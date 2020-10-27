#!/bin/bash

PROTOC="protoc"

ROOT_PATH=$1
MSGS_PATH=${ROOT_PATH}"/msgs"
TARGET=${ROOT_PATH}"/include/protobuf/"
TARGET_SOURCE=${ROOT_PATH}"/src/protobuf"

mkdir -p $TARGET
mkdir -p $TARGET_SOURCE

MSG="any param param_v color "
MSG+="time vector2d vector3d quaternion pose "
MSG+="imu image images_stamped image_stamped camerasensor distortion "
MSG+="laserscan laserscan_stamped "
MSG+="micom battery pointcloud gps "

for i in $MSG
do
  echo "Generate $i.proto"
  eval "$PROTOC -I $MSGS_PATH --cpp_out=$TARGET $MSGS_PATH/$i.proto"
done

mv $TARGET/*.pb.cc $TARGET_SOURCE