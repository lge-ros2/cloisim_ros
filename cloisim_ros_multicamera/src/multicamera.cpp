/**
 *  @file   multicamera.cpp
 *  @date   2021-01-14
 *  @author hyunseok Yang
 *  @author Sungkyu Kang
 *  @brief
 *        ROS2 Multi Camera class for simulator
 *  @remark
 *  @warning
 *       LGE Advanced Robotics Laboratory
 *         Copyright(C) 2019 LG Electronics Co., LTD., Seoul, Korea
 *         All Rights are Reserved.
 */

#include "cloisim_ros_multicamera/multicamera.hpp"
#include <sensor_msgs/fill_image.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <cloisim_ros_base/camera_helper.h>
#include <cloisim_msgs/param.pb.h>

using namespace std;
using namespace chrono_literals;
using namespace cloisim_ros;
using namespace cloisim;

MultiCamera::MultiCamera(const rclcpp::NodeOptions &options_, const string node_name, const std::string namespace_)
    : Base(node_name, namespace_, options_)
{
  Start();
}

MultiCamera::MultiCamera(const std::string namespace_)
    : MultiCamera(rclcpp::NodeOptions(), "cloisim_ros_multicamera", namespace_)
{
}

MultiCamera::~MultiCamera()
{
  Stop();
}

void MultiCamera::Initialize()
{
  uint16_t portInfo, portData;
  get_parameter_or("bridge.Data", portData, uint16_t(0));
  get_parameter_or("bridge.Info", portInfo, uint16_t(0));

  const auto hashKeyData = GetTargetHashKey("Data");
  const auto hashKeyInfo = GetTargetHashKey("Info");
  DBG_SIM_INFO("hash Key: data(%s), info(%s)", hashKeyData.c_str(), hashKeyInfo.c_str());

  auto data_bridge_ptr = CreateBridge();
  auto info_bridge_ptr = CreateBridge();

  if (info_bridge_ptr != nullptr)
  {
    info_bridge_ptr->Connect(zmq::Bridge::Mode::CLIENT, portInfo, hashKeyInfo);

    GetRos2Parameter(info_bridge_ptr);

    image_transport::ImageTransport it(GetNode());
    for (auto frame_id : frame_id_list_)
    {
      const auto transform = GetObjectTransform(info_bridge_ptr, frame_id);
      SetStaticTf2(transform, GetPartsName() + "_" + frame_id);

      // Image publisher
      const auto topic_base_name_ = GetPartsName() + "/" + frame_id;
      DBG_SIM_INFO("topic_base_name:%s", topic_base_name_.c_str());

      sensor_msgs::msg::Image msg_img;
      msg_img.header.frame_id = frame_id;

      msg_imgs_[msg_imgs_.size()] = msg_img;

      pubs_.push_back(it.advertiseCamera(topic_base_name_ + "/image_raw", 1));

      // handling parameters for image_transport plugin
      const auto format_value = get_parameter("format");
      const auto jpeg_quality_value = get_parameter("jpeg_quality");
      const auto png_level_value = get_parameter("png_level");

      undeclare_parameter("format");
      undeclare_parameter("jpeg_quality");
      undeclare_parameter("png_level");

      declare_parameters(frame_id,
                         map<string, string>{
                             {"format", format_value.as_string()}});
      declare_parameters(frame_id,
                         map<string, int>{
                             {"jpeg_quality", jpeg_quality_value.as_int()},
                             {"png_level", png_level_value.as_int()}});

      camera_info_manager_.push_back(std::make_shared<camera_info_manager::CameraInfoManager>(GetNode().get()));
      const auto camSensorMsg = GetCameraSensorMessage(info_bridge_ptr, frame_id);
      SetCameraInfoInManager(camera_info_manager_.back(), camSensorMsg, frame_id);
    }

    data_bridge_ptr->Connect(zmq::Bridge::Mode::SUB, portData, hashKeyData);
    AddPublisherThread(data_bridge_ptr, bind(&MultiCamera::PublishData, this, std::placeholders::_1));
  }
}

void MultiCamera::Deinitialize()
{
  for (auto pub_ : pubs_)
  {
    pub_.shutdown();
  }
}

void MultiCamera::PublishData(const string &buffer)
{
  if (!pb_buf_.ParseFromString(buffer))
  {
    DBG_SIM_ERR("Parsing error, size(%d)", buffer.length());
    return;
  }

  SetTime(pb_buf_.time());

  for (auto i = 0; i < pb_buf_.image_size(); i++)
  {
    auto img = &pb_buf_.image(i);
    const auto encoding_arg = GetImageEncondingType(img->pixel_format());
    const uint32_t cols_arg = img->width();
    const uint32_t rows_arg = img->height();
    const uint32_t step_arg = img->step();

    auto const msg_img = &msg_imgs_[i];
    msg_img->header.stamp = GetTime();
    sensor_msgs::fillImage(*msg_img, encoding_arg, rows_arg, cols_arg, step_arg,
                           reinterpret_cast<const void *>(img->data().data()));

    // Publish camera info
    auto camera_info_msg = camera_info_manager_[i]->getCameraInfo();
    camera_info_msg.header.stamp = GetTime();
    pubs_.at(i).publish(*msg_img, camera_info_msg);
  }
}