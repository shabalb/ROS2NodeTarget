#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

class ImageViewer : public rclcpp::Node {
public:
  ImageViewer() : Node("image_viewer") {
    // Параметр на случай, если топик другой
    this->declare_parameter<std::string>("image_topic", "/camera/image");
    const auto topic = this->get_parameter("image_topic").as_string();

    auto qos = rclcpp::QoS(rclcpp::KeepLast(5)).reliable();
    sub_ = this->create_subscription<sensor_msgs::msg::Image>(
      topic, qos,
      std::bind(&ImageViewer::cb, this, std::placeholders::_1));

    //sub_ = this->create_subscription<sensor_msgs::msg::Image>(
    //  topic, rclcpp::SensorDataQoS(),
    //  std::bind(&ImageViewer::cb, this, std::placeholders::_1));

    cv::namedWindow("camera", cv::WINDOW_NORMAL);
    RCLCPP_INFO(get_logger(), "Subscribed to: %s", topic.c_str());
  }

  ~ImageViewer() override {
    cv::destroyAllWindows();
  }

private:

  size_t n_ = 0;
  rclcpp::Clock steady_{RCL_STEADY_TIME};

  void cb(const sensor_msgs::msg::Image::SharedPtr msg) {
    n_++;
    if (n_ % 30 == 0) {
      RCLCPP_INFO(get_logger(), "frames=%zu stamp=%u.%u encoding=%s",
        n_, msg->header.stamp.sec, msg->header.stamp.nanosec, msg->encoding.c_str());
    }
    try {
      // В Gazebo часто приходит RGB8, но бывает BGR8 — обработаем оба случая
      cv_bridge::CvImageConstPtr cv_ptr;

      if (msg->encoding == "rgb8") {
        cv_ptr = cv_bridge::toCvShare(msg, "rgb8");
        cv::Mat bgr;
        cv::cvtColor(cv_ptr->image, bgr, cv::COLOR_RGB2BGR);
        cv::imshow("camera", bgr);
      } else {
        // Попробуем как BGR8 или как есть
        cv_ptr = cv_bridge::toCvShare(msg, msg->encoding);
        cv::imshow("camera", cv_ptr->image);
      }

      // Без waitKey окно не обновляется
      cv::waitKey(1);
    } catch (const cv_bridge::Exception &e) {
      RCLCPP_ERROR(get_logger(), "cv_bridge exception: %s", e.what());
    } catch (const cv::Exception &e) {
      RCLCPP_ERROR(get_logger(), "OpenCV exception: %s", e.what());
    }
  }

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ImageViewer>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}