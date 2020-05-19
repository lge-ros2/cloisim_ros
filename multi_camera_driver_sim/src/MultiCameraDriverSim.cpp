/**
 *  @file   MultiCameraDriverSim.cpp
 *  @date   2020-05-20
 *  @author hyunseok Yang
 *  @author Sungkyu Kang
 *  @brief
 *        ROS2 Multi Camera Driver class for simulator
 *  @remark
 *  @warning
 *       LGE Advanced Robotics Laboratory
 *         Copyright(C) 2019 LG Electronics Co., LTD., Seoul, Korea
 *         All Rights are Reserved.
 */
#include "multi_camera_driver_sim/MultiCameraDriverSim.hpp"
#include <unistd.h>
#include <math.h>
#include <string>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/fill_image.hpp>
#include <tf2/LinearMath/Quaternion.h>

using namespace std;
using namespace chrono_literals;

MultiCameraDriverSim::MultiCameraDriverSim()
    : Node("multi_camera_driver_sim",
           rclcpp::NodeOptions()
               .allow_undeclared_parameters(true)
               .automatically_declare_parameters_from_overrides(true)),
      m_pSimBridge(new SimBridge()),
      m_bRunThread(true),
      node_handle(shared_ptr<rclcpp::Node>(this))
{
  string sim_ip("");
  int sim_manager_port(0);
  std::string robot_name_;
  std::string part_name_;
  string camera_name_;
  vector<double> transform_;
  vector<string> camera_list_;

  get_parameter("sim.ip_address", sim_ip);
  get_parameter("sim.manage_port", sim_manager_port);
  get_parameter_or("sim.model", robot_name_, string("cloi"));
  get_parameter_or("sim.parts", part_name_, string("multi_camera"));

  get_parameter_or("camera_name", camera_name_, string("multi_camera"));
  get_parameter_or("transform", transform_, vector<double>({0.0, 0.0, 0.0, 0.0, 0.0, 0.0}));

  DBG_SIM_INFO("[CONFIG] sim manage ip:%s, port:%d ..", sim_ip.c_str(), sim_manager_port);
  DBG_SIM_INFO("[CONFIG] sim.model:%s", robot_name_.c_str());
  DBG_SIM_INFO("[CONFIG] sim.part:%s", part_name_.c_str());

  get_parameter("camera_list", camera_list_);

  tf2::Quaternion fixed_rot;
  for (auto item : camera_list_)
  {
    string item_name;
    vector<double> transform_offset;
    string frame_id;

    get_parameter_or(item + ".name", item_name, string("noname"));
    get_parameter_or(item + ".transform_offset", transform_offset, vector<double>({0.0, 0.0, 0.0, 0.0, 0.0, 0.0}));
    get_parameter_or(item + ".frame_id", frame_id, string("noname_link"));

    transform_offset[0] += transform_[0];
    transform_offset[1] += transform_[1];
    transform_offset[2] += transform_[2];
    transform_offset[3] += transform_[3];
    transform_offset[4] += transform_[4];
    transform_offset[5] += transform_[5];

    geometry_msgs::msg::TransformStamped camera_tf;
    camera_tf.header.frame_id = "base_footprint";
    camera_tf.child_frame_id = frame_id;
    camera_tf.transform.translation.x = transform_offset[0];
    camera_tf.transform.translation.y = transform_offset[1];
    camera_tf.transform.translation.z = transform_offset[2];

    fixed_rot.setRPY(transform_offset[3], transform_offset[4], transform_offset[5]);

    camera_tf.transform.rotation.x = fixed_rot.x();
    camera_tf.transform.rotation.y = fixed_rot.y();
    camera_tf.transform.rotation.z = fixed_rot.z();
    camera_tf.transform.rotation.w = fixed_rot.w();

    camera_tf_list.push_back(camera_tf);

    // Image publisher
    auto topic_name_ = camera_name_ + "/" + item_name + "/image_raw";
    DBG_SIM_INFO("[CONFIG] topic_name:%s", topic_name_.c_str());
    image_pubs_.push_back(image_transport::create_publisher(node_handle.get(), topic_name_));
  }

  msg_img = sensor_msgs::msg::Image();

  static_tf_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(node_handle);

  m_hashKeySub = robot_name_ + part_name_;
  DBG_SIM_INFO("hash Key sub: %s", m_hashKeySub.c_str());

  m_pSimBridge->SetSimMasterAddress(sim_ip);
  m_pSimBridge->SetPortManagerPort(sim_manager_port);

  Start();
}

MultiCameraDriverSim::~MultiCameraDriverSim()
{
  Stop();
  delete m_pSimBridge;
}

void MultiCameraDriverSim::Start()
{
  m_pSimBridge->Connect(SimBridge::Mode::SUB, m_hashKeySub);

  m_bRunThread = true;
  m_thread = thread([=]() { UpdateImage(); });

  auto callback_pub = [this]() -> void {
    UpdateStaticTF(m_simTime);
  };

  // ROS2 timer for Publisher
  timer = create_wall_timer(1s, callback_pub);
}

void MultiCameraDriverSim::Stop()
{
  m_bRunThread = false;

  usleep(500);

  if (m_thread.joinable())
  {
    m_thread.join();
    DBG_SIM_INFO("ReadProc() thread finished");
  }

  for (auto pub : image_pubs_)
  {
    pub.shutdown();
  }

  m_pSimBridge->Disconnect();
}

void MultiCameraDriverSim::UpdateImage()
{
  void *pBuffer = nullptr;
  int bufferLength = 0;

  while (m_bRunThread)
  {
    const bool succeeded = m_pSimBridge->Receive(&pBuffer, bufferLength, false);
    if (!succeeded || bufferLength < 0)
    {
      DBG_SIM_ERR("zmq receive error return size(%d): %s",
              bufferLength, zmq_strerror(zmq_errno()));

      // try reconnect1ion
      if(m_bRunThread) {
        m_pSimBridge->Reconnect(SimBridge::Mode::SUB, m_hashKeySub);
      }

      continue;
    }

    if (!m_pbBuf.ParseFromArray(pBuffer, bufferLength))
    {
      DBG_SIM_ERR("Parsing error, size(%d)", bufferLength);
      continue;
    }

    m_simTime = rclcpp::Time(m_pbBuf.time().sec(), m_pbBuf.time().nsec());

    for (auto i = 0; i < m_pbBuf.image_size(); i++)
    {
      auto img = &m_pbBuf.image(i);

      const auto encoding_arg = GetEncondingType(img->pixel_format());
      const uint32_t cols_arg = img->width();
      const uint32_t rows_arg = img->height();
      const uint32_t step_arg = img->step();

      msg_img.header.stamp = m_simTime;
      sensor_msgs::fillImage(msg_img, encoding_arg, rows_arg, cols_arg, step_arg,
                             reinterpret_cast<const void *>(img->data().data()));

      image_pubs_.at(i).publish(msg_img);
    }
  }
}

void MultiCameraDriverSim::UpdateStaticTF(const rclcpp::Time timestamp)
{
  for (auto camera_tf : camera_tf_list)
  {
    camera_tf.header.stamp = timestamp;
    static_tf_broadcaster_->sendTransform(camera_tf);
  }
}

string MultiCameraDriverSim::GetEncondingType(const uint32_t pixel_format)
{
// 	UNKNOWN_PIXEL_FORMAT = 0, L_INT8, L_INT16,
  // 	RGB_INT8 = 3, RGBA_INT8, BGRA_INT8, RGB_INT16, RGB_INT32,
  // 	BGR_INT8 = 8, BGR_INT16, BGR_INT32,
  // 	R_FLOAT16 = 11, RGB_FLOAT16 = 12, R_FLOAT32 = 13, RGB_FLOAT32,
  // 	BAYER_RGGB8, BAYER_RGGR8, BAYER_GBRG8, BAYER_GRBG8,
  // 	PIXEL_FORMAT_COUNT
  string encoding;
  switch (pixel_format)
  {
    case 3:
      encoding = sensor_msgs::image_encodings::RGB8;
      break;

    case 6:
      encoding = sensor_msgs::image_encodings::RGB16;
      break;

    case 8:
      encoding = sensor_msgs::image_encodings::BGR8;
      break;

    case 9:
      encoding = sensor_msgs::image_encodings::BGR16;
      break;

    case 13:
      encoding = sensor_msgs::image_encodings::TYPE_32FC1;
      break;

    default:
      encoding = sensor_msgs::image_encodings::RGB8;
      break;
  }

  return encoding;
}