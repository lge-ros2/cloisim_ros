/**
 *  @file   CameraDriverSim.cpp
 *  @date   2020-03-20
 *  @author hyunseok Yang
 *  @author Sungkyu Kang
 *  @brief
 *        ROS2 Depth Camera Driver class for simulator
 *  @remark
 *  @warning
 *       LGE Advanced Robotics Laboratory
 *         Copyright(C) 2019 LG Electronics Co., LTD., Seoul, Korea
 *         All Rights are Reserved.
 */
#include "camera_driver_sim/CameraDriverSim.hpp"
#include <unistd.h>
#include <math.h>
// #include <string>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/fill_image.hpp>
#include <tf2/LinearMath/Quaternion.h>

using namespace std;
using namespace chrono_literals;


CameraDriverSim::CameraDriverSim()
  : Node("camera_driver_sim", rclcpp::NodeOptions()
      .allow_undeclared_parameters(true)
      .automatically_declare_parameters_from_overrides(true))
  , m_bRun(true)
{
  node_handle = shared_ptr<rclcpp::Node>(this);

  string sim_ip("");
  int sim_manager_port(0);
  vector<double> transform_;

  get_parameter("sim.ip_address", sim_ip);
  get_parameter("sim.manage_port", sim_manager_port);
  get_parameter_or("sim.model", robot_name_, string("cloi"));
  get_parameter_or("sim.parts", part_name_, string("camera"));
  get_parameter_or("frame_name", frame_name_, string("camera_link"));
  get_parameter_or("topic", topic_name_, string("camera/rgb/image_raw"));
  get_parameter_or("transform", transform_, vector<double>({0.0, 0.0, 0.0, 0.0, 0.0, 0.0}));

  DBG_SIM_INFO("[CONFIG] sim manage ip:%s, port:%d ", sim_ip.c_str(), sim_manager_port);
  DBG_SIM_INFO("[CONFIG] sim.model:%s", robot_name_.c_str());
  DBG_SIM_INFO("[CONFIG] sim.part:%s", part_name_.c_str());
  DBG_SIM_INFO("[CONFIG] topic_name:%s", topic_name_.c_str());

  m_hashKeySub = robot_name_ + part_name_;
  DBG_SIM_INFO("hash Key sub: %s", m_hashKeySub.c_str());

  m_pSimBridge = new SimBridge();

  if (m_pSimBridge)
  {
    m_pSimBridge->SetSimMasterAddress(sim_ip);
    m_pSimBridge->SetPortManagerPort(sim_manager_port);
  }

  tf2::Quaternion fixed_rot;
  fixed_rot.setRPY(transform_[3], transform_[4], transform_[5]);

  camera_tf.header.frame_id = "base_footprint";
  camera_tf.child_frame_id = frame_name_;
  camera_tf.transform.translation.x = transform_[0];
  camera_tf.transform.translation.y = transform_[1];
  camera_tf.transform.translation.z = transform_[2];
  camera_tf.transform.rotation.x = fixed_rot.x();
  camera_tf.transform.rotation.y = fixed_rot.y();
  camera_tf.transform.rotation.z = fixed_rot.z();
  camera_tf.transform.rotation.w = fixed_rot.w();

  msg_img = sensor_msgs::msg::Image();
  msg_img.header.frame_id = frame_name_;

  static_tf_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(node_handle);

  pubImage = image_transport::create_publisher(node_handle.get(), topic_name_);

  Start();
}

CameraDriverSim::~CameraDriverSim()
{
  Stop();
  delete m_pSimBridge;
}

void CameraDriverSim::Start()
{
  m_pSimBridge->Connect(SimBridge::Mode::SUB, m_hashKeySub);
  m_bRun = true;
  m_thread = thread([=]() { ReadProc(); });

  auto callback_pub = [this]() -> void {
    UpdateStaticTF(m_simTime);
  };

  // ROS2 timer for Publisher
  timer = this->create_wall_timer(0.5s, callback_pub);
}

void CameraDriverSim::Stop()
{
  m_bRun = false;

  usleep(100);

  if (m_thread.joinable())
  {
    m_thread.join();
    DBG_SIM_INFO("ReadProc() thread finished");
  }

  m_pSimBridge->Disconnect();
}

void CameraDriverSim::ReadProc()
{
  void *pBuffer = nullptr;
  int bufferLength = 0;

  while (m_bRun)
  {
    // :x: at first time recv buf in blocking mode
    const bool succeeded = m_pSimBridge->Receive(&pBuffer, bufferLength, false);
    if (!succeeded || bufferLength < 0)
    {
      DBG_SIM_ERR("zmq receive error return size(%d): %s",
              bufferLength, zmq_strerror(zmq_errno()));

      // try reconnect1ion
      if(m_bRun) {
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

    // UpdateImage();
    msg_img.header.stamp = m_simTime;

    const auto encoding_arg = GetEncondingType(m_pbBuf.image().pixel_format());
    const uint32_t cols_arg = m_pbBuf.image().width();
    const uint32_t rows_arg = m_pbBuf.image().height();
    const uint32_t step_arg = m_pbBuf.image().step();

    // Copy from src to image_msg
    sensor_msgs::fillImage(msg_img, encoding_arg, rows_arg, cols_arg, step_arg,
                           reinterpret_cast<const void *>(m_pbBuf.image().data().data()));

    pubImage.publish(msg_img);
  }
}

void CameraDriverSim::UpdateImage()
{
  // Publish image
  // msg_img.header.stamp = m_simTime;

  // const auto encoding_arg = GetEncondingType(m_pbBuf.image().pixel_format());
  // const uint32_t cols_arg = m_pbBuf.image().width();
  // const uint32_t rows_arg = m_pbBuf.image().height();
  // const uint32_t step_arg = m_pbBuf.image().step();

  // // Copy from src to image_msg
  // sensor_msgs::fillImage(msg_img, encoding_arg, rows_arg, cols_arg, step_arg,
  //                        reinterpret_cast<const void *>(_image));

  // impl_->image_pub_.publish(msg_img);

  // const auto numCh = sensor_msgs::image_encodings::numChannels(msg_img.encoding);
  // if (numCh > 0)
  // {
  //   msg_img.width = cols_arg;
  //   msg_img.height = rows_arg;
  //   msg_img.step = numCh * cols_arg;
  //   msg_img.data.resize(rows_arg * cols_arg * numCh);
  //   msg_img.is_bigendian = 0;

  //   // unsigned int* dest = (unsigned int*)(&(msg_depth.data[0]));
  //   uint8_t *toCopyFrom = (uint8_t *)m_pbBuf.image().data().data();
  //   uint32_t index = 0;
  //   // cout << m_pbBuf.image().data().size() << endl;
  //   // cout << rows_arg * cols_arg * 3 << endl;
  //   for (uint32_t j = 0; j < rows_arg; j++)
  //   {
  //     for (uint32_t i = 0; i < cols_arg; i++)
  //     {
  //       for (uint32_t k = 0; k < (uint32_t)numCh; k++)
  //       {
  //         // cout << index << endl;
  //         if (index < m_pbBuf.image().data().size() && index < msg_img.data.size())
  //         {
  //           uint8_t value = (uint8_t)toCopyFrom[index];
  //           msg_img.data[index] = value;
  //         }
  //         index++;
  //       }
  //     }
  //   }
  // }
}

void CameraDriverSim::UpdateStaticTF(const rclcpp::Time timestamp)
{
  camera_tf.header.stamp = timestamp;
  static_tf_broadcaster_->sendTransform(camera_tf);
}


string CameraDriverSim::GetEncondingType(const uint32_t pixel_format)
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