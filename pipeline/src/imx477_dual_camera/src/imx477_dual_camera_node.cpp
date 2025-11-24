#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>

#include <vector>
#include <string>
#include <thread>
#include <chrono>

extern "C" {
#include <sp_vio.h>
#include <sp_sys.h>
}

class Imx477DualCameraNode : public rclcpp::Node
{
public:
  Imx477DualCameraNode()
  : Node("imx477_dual_camera_node")
  {
    this->declare_parameter<int>("full_width", 4000);
    this->declare_parameter<int>("full_height", 3000);
    this->declare_parameter<std::string>("full_topic", "/imx477/image_raw_4k");

    full_width_ = this->get_parameter("full_width").as_int();
    full_height_ = this->get_parameter("full_height").as_int();
    full_topic_ = this->get_parameter("full_topic").as_string();

    RCLCPP_INFO(
      this->get_logger(),
      "Starting IMX477 camera node: full=%dx%d, topic='%s'",
      full_width_, full_height_, full_topic_.c_str()
    );

    vio_handle_ = sp_init_vio_module();
    if (!vio_handle_) {
      RCLCPP_FATAL(this->get_logger(), "sp_init_vio_module() failed");
      throw std::runtime_error("sp_init_vio_module failed");
    }

    sp_sensors_parameters parms;
    parms.fps = -1;
    parms.raw_height = full_height_;
    parms.raw_width = full_width_;

    int w = full_width_;
    int h = full_height_;

    int ret = sp_open_camera_v2(
      vio_handle_,
      0,
      -1,
      1,
      &parms,
      &w,
      &h
    );
    if (ret != 0) {
      RCLCPP_FATAL(this->get_logger(), "sp_open_camera_v2() failed, ret=%d", ret);
      throw std::runtime_error("sp_open_camera_v2 failed");
    }

    full_width_ = w;
    full_height_ = h;

    RCLCPP_INFO(
      this->get_logger(),
      "Camera opened: actual full size = %dx%d",
      full_width_, full_height_
    );

    std::this_thread::sleep_for(std::chrono::seconds(2));

    int full_size = FRAME_BUFFER_SIZE(full_width_, full_height_);
    full_buffer_.resize(static_cast<size_t>(full_size));

    auto qos = rclcpp::SensorDataQoS().reliable();
    pub_full_ = this->create_publisher<sensor_msgs::msg::Image>(
      full_topic_,
      qos
    );

    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(33),
      std::bind(&Imx477DualCameraNode::capture_loop, this)
    );
  }

  ~Imx477DualCameraNode() override
  {
    if (vio_handle_) {
      sp_vio_close(vio_handle_);
      sp_release_vio_module(vio_handle_);
      vio_handle_ = nullptr;
    }
  }

private:
  void capture_loop()
  {
    int ret = sp_vio_get_yuv(
      vio_handle_,
      reinterpret_cast<char*>(full_buffer_.data()),
      full_width_,
      full_height_,
      2000
    );

    if (ret != 0) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(),
        *this->get_clock(),
        2000,
        "sp_vio_get_yuv() failed, ret=%d",
        ret
      );
      return;
    }

    sensor_msgs::msg::Image msg;
    msg.header.stamp = this->get_clock()->now();
    msg.header.frame_id = "imx477_4k";
    msg.width = static_cast<uint32_t>(full_width_);
    msg.height = static_cast<uint32_t>(full_height_);
    msg.encoding = "nv12";
    msg.is_bigendian = 0;
    msg.step = static_cast<sensor_msgs::msg::Image::_step_type>(full_width_);
    msg.data.assign(full_buffer_.begin(), full_buffer_.end());

    pub_full_->publish(std::move(msg));
  }

  void* vio_handle_{nullptr};

  int full_width_{4000};
  int full_height_{3000};
  std::string full_topic_;

  std::vector<uint8_t> full_buffer_;

  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_full_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  try {
    auto node = std::make_shared<Imx477DualCameraNode>();
    rclcpp::spin(node);
  } catch (const std::exception &e) {
    RCLCPP_FATAL(rclcpp::get_logger("imx477_dual_camera_main"), "Exception: %s", e.what());
  }
  rclcpp::shutdown();
  return 0;
}
