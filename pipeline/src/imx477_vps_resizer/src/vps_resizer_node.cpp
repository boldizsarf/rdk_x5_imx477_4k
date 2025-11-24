#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>

#include <string>
#include <vector>

extern "C" {
#include <sp_vio.h>
#include <sp_sys.h>
}

class Imx477VpsResizerNode : public rclcpp::Node
{
public:
  Imx477VpsResizerNode()
  : Node("imx477_vps_resizer_node")
  {
    this->declare_parameter<int>("input_width", 4000);
    this->declare_parameter<int>("input_height", 3000);
    this->declare_parameter<int>("output_width", 1920);
    this->declare_parameter<int>("output_height", 1080);
    this->declare_parameter<std::string>("input_topic", "/imx477/image_raw_4k");
    this->declare_parameter<std::string>("output_topic", "/imx477/image_raw_1080");

    input_width_ = this->get_parameter("input_width").as_int();
    input_height_ = this->get_parameter("input_height").as_int();
    output_width_ = this->get_parameter("output_width").as_int();
    output_height_ = this->get_parameter("output_height").as_int();
    input_topic_ = this->get_parameter("input_topic").as_string();
    output_topic_ = this->get_parameter("output_topic").as_string();

    RCLCPP_INFO(
      this->get_logger(),
      "Starting VPS resizer: %dx%d -> %dx%d, input_topic='%s', output_topic='%s'",
      input_width_, input_height_, output_width_, output_height_,
      input_topic_.c_str(), output_topic_.c_str()
    );

    vps_handle_ = sp_init_vio_module();
    if (!vps_handle_) {
      RCLCPP_FATAL(this->get_logger(), "sp_init_vio_module() failed for VPS");
      throw std::runtime_error("sp_init_vio_module failed");
    }

    int ow = output_width_;
    int oh = output_height_;
    int ret = sp_open_vps(
      vps_handle_,
      0,
      1,
      SP_VPS_SCALE,
      input_width_,
      input_height_,
      &ow,
      &oh,
      nullptr, nullptr, nullptr, nullptr, nullptr
    );
    if (ret != 0) {
      RCLCPP_FATAL(this->get_logger(), "sp_open_vps() failed, ret=%d", ret);
      throw std::runtime_error("sp_open_vps failed");
    }

    if (ow != output_width_ || oh != output_height_) {
      RCLCPP_WARN(
        this->get_logger(),
        "VPS output size adjusted by hardware: requested %dx%d, got %dx%d",
        output_width_, output_height_, ow, oh
      );
      output_width_ = ow;
      output_height_ = oh;
    }

    int out_size = FRAME_BUFFER_SIZE(output_width_, output_height_);
    down_buffer_.resize(static_cast<size_t>(out_size));

    auto qos = rclcpp::SensorDataQoS().reliable();

    pub_down_ = this->create_publisher<sensor_msgs::msg::Image>(
      output_topic_,
      qos
    );

    sub_full_ = this->create_subscription<sensor_msgs::msg::Image>(
      input_topic_,
      qos,
      std::bind(&Imx477VpsResizerNode::image_callback, this, std::placeholders::_1)
    );
  }

  ~Imx477VpsResizerNode() override
  {
    if (vps_handle_) {
      sp_vio_close(vps_handle_);
      sp_release_vio_module(vps_handle_);
      vps_handle_ = nullptr;
    }
  }

private:
  void image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
  {
    if (msg->width != static_cast<uint32_t>(input_width_) ||
        msg->height != static_cast<uint32_t>(input_height_)) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(),
        *this->get_clock(),
        2000,
        "Unexpected input size: %ux%u, expected %dx%d. Skipping frame.",
        msg->width, msg->height, input_width_, input_height_
      );
      return;
    }

    if (msg->data.empty()) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(),
        *this->get_clock(),
        2000,
        "Input image data is empty, skipping frame."
      );
      return;
    }

    const int input_size = static_cast<int>(msg->data.size());

    int ret = sp_vio_set_frame(
      vps_handle_,
      reinterpret_cast<char*>(msg->data.data()),
      input_size
    );
    if (ret != 0) {
      RCLCPP_WARN(
        this->get_logger(),
        "sp_vio_set_frame() failed, ret=%d",
        ret
      );
      return;
    }

    ret = sp_vio_get_frame(
      vps_handle_,
      reinterpret_cast<char*>(down_buffer_.data()),
      output_width_,
      output_height_,
      2000
    );
    if (ret != 0) {
      RCLCPP_WARN(
        this->get_logger(),
        "sp_vio_get_frame() failed, ret=%d",
        ret
      );
      return;
    }

    sensor_msgs::msg::Image out_msg;
    out_msg.header = msg->header;
    out_msg.header.frame_id = "imx477_downscaled";
    out_msg.width = static_cast<uint32_t>(output_width_);
    out_msg.height = static_cast<uint32_t>(output_height_);
    out_msg.encoding = "nv12";
    out_msg.is_bigendian = 0;
    out_msg.step = static_cast<sensor_msgs::msg::Image::_step_type>(output_width_);
    out_msg.data.assign(down_buffer_.begin(), down_buffer_.end());

    pub_down_->publish(std::move(out_msg));
  }

  void* vps_handle_{nullptr};

  int input_width_{4000};
  int input_height_{3000};
  int output_width_{1920};
  int output_height_{1080};

  std::string input_topic_;
  std::string output_topic_;

  std::vector<uint8_t> down_buffer_;

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_full_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_down_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  try {
    auto node = std::make_shared<Imx477VpsResizerNode>();
    rclcpp::spin(node);
  } catch (const std::exception &e) {
    RCLCPP_FATAL(rclcpp::get_logger("imx477_vps_resizer_main"), "Exception: %s", e.what());
  }
  rclcpp::shutdown();
  return 0;
}
