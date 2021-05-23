/**
 *  @file   realsense.cpp
 *  @date   2021-01-14
 *  @author hyunseok Yang
 *  @author Sungkyu Kang
 *  @brief
 *        ROS2 realsense class for simulator
 *  @remark
 *  @warning
 *       LGE Advanced Robotics Laboratory
 *         Copyright(C) 2019 LG Electronics Co., LTD., Seoul, Korea
 *         All Rights are Reserved.
 */

#include "cloisim_ros_realsense/realsense.hpp"
#include <cloisim_ros_base/camera_helper.h>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/fill_image.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <cloisim_msgs/param.pb.h>

using namespace std;
using namespace chrono_literals;
using namespace cloisim;
using namespace cloisim_ros;

RealSense::RealSense(const rclcpp::NodeOptions &options_, const string node_name, const string namespace_)
    : Base(node_name, namespace_, options_)
{
  Start();
}

RealSense::RealSense(const string namespace_)
    : RealSense(rclcpp::NodeOptions(), "cloisim_ros_realsense", namespace_)
{
}

RealSense::~RealSense()
{
  // DBG_SIM_INFO("Delete");
  Stop();
}

void RealSense::Initialize()
{
  uint16_t portInfo;
  get_parameter_or("bridge.Info", portInfo, uint16_t(0));

  const auto hashKeyInfo = GetTargetHashKey("Info");

  auto info_bridge_ptr = CreateBridge(hashKeyInfo);
  // int bridgeIndex = 0;
  // auto info_bridge_ptr = GetBridge(bridgeIndex);

  string header_frame_id = "_camera_link";
  if (info_bridge_ptr != nullptr)
  {
    info_bridge_ptr->Connect(zmq::Bridge::Mode::CLIENT, portInfo, hashKeyInfo);
    GetActivatedModules(info_bridge_ptr);

    const auto transform = GetObjectTransform(info_bridge_ptr);
    header_frame_id = GetPartsName() + header_frame_id;
    SetupStaticTf2(transform, header_frame_id);
  }

  image_transport::ImageTransport it(GetNode());
  for (auto module_name : activated_modules_)
  {
    uint16_t portCamData, portCamInfo;
    get_parameter_or("bridge." + module_name + "Data", portCamData, uint16_t(0));
    get_parameter_or("bridge." + module_name + "Info", portCamInfo, uint16_t(0));

    const auto topic_base_name_ = GetPartsName() + "/" + module_name;

    const auto hashKeyData = GetTargetHashKey(module_name + "Data");
    const auto hashKeyInfo = GetTargetHashKey(module_name + "Info");
    DBG_SIM_INFO("topic_name: %s, hash Key: data(%s), info(%s)", topic_base_name_.c_str(), hashKeyData.c_str(), hashKeyInfo.c_str());

    auto pBridgeCamData = CreateBridge(hashKeyData);
    auto pBridgeCamInfo = CreateBridge(hashKeyInfo);
    if (pBridgeCamInfo != nullptr)
    {
      pBridgeCamInfo->Connect(zmq::Bridge::Mode::CLIENT, portCamInfo, hashKeyInfo);
      const auto transform = GetObjectTransform(pBridgeCamInfo, module_name);
      const auto child_frame_id = GetPartsName() + "_camera_" + module_name + "_frame";
      SetupStaticTf2(transform, child_frame_id, header_frame_id);

      const auto camInfoManager = std::make_shared<camera_info_manager::CameraInfoManager>(GetNode().get());

      const auto camSensorMsg = GetCameraSensorMessage(pBridgeCamInfo);
      SetCameraInfoInManager(camInfoManager, camSensorMsg, module_name);

      camera_info_managers_[pBridgeCamData] = camInfoManager;
    }

    const auto topic_name = (module_name.find("depth") == string::npos)? "image_raw":"image_rect_raw";
    pubs_[pBridgeCamData] = it.advertiseCamera(topic_base_name_ + "/" + topic_name, 1);

    // handling parameters for image_transport plugin
    const auto format_value = get_parameter("format");
    const auto jpeg_quality_value = get_parameter("jpeg_quality");
    const auto png_level_value = get_parameter("png_level");

    undeclare_parameter("format");
    undeclare_parameter("jpeg_quality");
    undeclare_parameter("png_level");

    declare_parameters(module_name,
                       map<string, string>{
                           {"format", format_value.as_string()}});
    declare_parameters(module_name,
                       map<string, int>{
                           {"jpeg_quality", jpeg_quality_value.as_int()},
                           {"png_level", png_level_value.as_int()}});

    sensor_msgs::msg::Image msg_img;
    msg_img.header.frame_id = module_name;

    msg_imgs_[pBridgeCamData] = msg_img;

    if (pBridgeCamData != nullptr)
    {
      pBridgeCamData->Connect(zmq::Bridge::Mode::SUB, portCamData, hashKeyData);
      CreatePublisherThread(pBridgeCamData);
    }
  }
}

void RealSense::Deinitialize()
{
  for (auto pub_ : pubs_)
  {
    pub_.second.shutdown();
  }
}

void RealSense::GetActivatedModules(zmq::Bridge* const bridge_ptr)
{
  if (bridge_ptr == nullptr)
  {
    return;
  }

  string moduleListStr;
  const auto reply = RequestReplyMessage(bridge_ptr, "request_module_list");

  if (reply.ByteSize() <= 0)
  {
    DBG_SIM_ERR("Faild to get activated module info, length(%ld)", reply.ByteSize());
  }
  else
  {
    if (reply.IsInitialized() &&
        reply.name() == "activated_modules")
    {
      for (auto i = 0; i < reply.children_size(); i++)
      {
        auto param = reply.children(i);
        if (param.name() == "module" && param.has_value() &&
            param.value().type() == msgs::Any_ValueType_STRING &&
            !param.value().string_value().empty())
        {
          const auto module = param.value().string_value();
          activated_modules_.push_back(module);
          moduleListStr.append(module + ", ");
        }
      }
    }
  }

  DBG_SIM_INFO("activated_modules: %s", moduleListStr.c_str());
}

void RealSense::UpdatePublishingData(const zmq::Bridge* const bridge_ptr, const std::string &buffer)
{
  cloisim::msgs::ImageStamped pb_buf_;
  if (!pb_buf_.ParseFromString(buffer))
  {
    DBG_SIM_ERR("Parsing error, size(%d)", buffer.length());
    return;
  }

  SetSimTime(pb_buf_.time());

  auto const msg_img = &msg_imgs_[bridge_ptr];
  msg_img->header.stamp = GetSimTime();

  const auto encoding_arg = GetImageEncondingType(pb_buf_.image().pixel_format());
  const uint32_t cols_arg = pb_buf_.image().width();
  const uint32_t rows_arg = pb_buf_.image().height();
  const uint32_t step_arg = pb_buf_.image().step();

  // Copy from src to image_msg
  sensor_msgs::fillImage(*msg_img, encoding_arg, rows_arg, cols_arg, step_arg,
                         reinterpret_cast<const void *>(pb_buf_.image().data().data()));

  if ((encoding_arg.compare(sensor_msgs::image_encodings::TYPE_16UC1) == 0) ||
      (encoding_arg.compare(sensor_msgs::image_encodings::TYPE_16SC1) == 0) ||
      (encoding_arg.compare(sensor_msgs::image_encodings::TYPE_32FC1) == 0))
  {
    msg_img->is_bigendian = true;
  }

  // Publish camera info
  auto camera_info_msg = camera_info_managers_[bridge_ptr]->getCameraInfo();
  camera_info_msg.header.stamp = GetSimTime();

  pubs_[bridge_ptr].publish(*msg_img, camera_info_msg);
}