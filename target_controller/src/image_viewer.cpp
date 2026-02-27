#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/opencv.hpp>

#include <cmath>
#include <limits>

class ImageViewer : public rclcpp::Node
{
public:
  ImageViewer() : Node("image_viewer")
  {

    this->declare_parameter<std::string>("image_topic", "/camera/image");
    const auto topic = this->get_parameter("image_topic").as_string();

    auto qos = rclcpp::QoS(rclcpp::KeepLast(5)).reliable();
    sub_ = this->create_subscription<sensor_msgs::msg::Image>(
        topic, qos,
        std::bind(&ImageViewer::cb, this, std::placeholders::_1));

    // sub_ = this->create_subscription<sensor_msgs::msg::Image>(
    //   topic, rclcpp::SensorDataQoS(),
    //   std::bind(&ImageViewer::cb, this, std::placeholders::_1));

    cv::namedWindow("camera", cv::WINDOW_NORMAL);
    RCLCPP_INFO(get_logger(), "Subscribed to: %s", topic.c_str());
  }

  ~ImageViewer() override
  {
    cv::destroyAllWindows();
  }

private:
  size_t n_ = 0;
  rclcpp::Clock steady_{RCL_STEADY_TIME};

  void cb(const sensor_msgs::msg::Image::SharedPtr msg)
  {
    n_++;
    if (n_ % 30 == 0)
    {
      RCLCPP_INFO(get_logger(), "frames=%zu stamp=%u.%u encoding=%s",
                  n_, msg->header.stamp.sec, msg->header.stamp.nanosec, msg->encoding.c_str());
    }
    try
    {

      cv_bridge::CvImageConstPtr cv_ptr;

      if (msg->encoding == "rgb8")
      {
        cv_ptr = cv_bridge::toCvShare(msg, "rgb8");
        cv::Mat bgr;
        cv::cvtColor(cv_ptr->image, bgr, cv::COLOR_RGB2BGR);
        cv::imshow("camera", bgr);
      }
      else
      {

        cv_ptr = cv_bridge::toCvShare(msg, msg->encoding);
        cv::imshow("camera", cv_ptr->image);
      }

      cv::waitKey(1);
    }
    catch (const cv_bridge::Exception &e)
    {
      RCLCPP_ERROR(get_logger(), "cv_bridge exception: %s", e.what());
    }
    catch (const cv::Exception &e)
    {
      RCLCPP_ERROR(get_logger(), "OpenCV exception: %s", e.what());
    }
  }

  

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_;
};

class LidarViewer : public rclcpp::Node
{
public:
  LidarViewer() : Node("lidar_viewer")
  {
    scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "/scan", rclcpp::SensorDataQoS(),
        std::bind(&LidarViewer::onScan, this, std::placeholders::_1));

    cv::namedWindow(win_, cv::WINDOW_AUTOSIZE);
  }

  ~LidarViewer() override
  {
    cv::destroyWindow(win_);
  }

private:
  void onScan(const sensor_msgs::msg::LaserScan::ConstSharedPtr msg)
  {
    // Картинка 600x600, центр — робот
    const int W = 600, H = 600;
    cv::Mat img(H, W, CV_8UC3, cv::Scalar(15, 15, 15));

    const cv::Point center(W / 2, H / 2);

    // Масштаб: сколько пикселей на метр (подстрой под range_max)
    const float meters_span = 12.0f; // “радиус” в метрах, который хотим видеть
    const float px_per_m = (std::min(W, H) * 0.45f) / meters_span;

    // сетка (опционально)
    drawGrid(img, center, px_per_m);

    // Собираем точки контура
    std::vector<cv::Point> poly;
    poly.reserve(msg->ranges.size());

    float angle = msg->angle_min;
    for (size_t i = 0; i < msg->ranges.size(); ++i, angle += msg->angle_increment)
    {
      float r = msg->ranges[i];
      if (!std::isfinite(r))
        continue;
      if (r < msg->range_min || r > msg->range_max)
        continue;

      float x = r * std::cos(angle);
      float y = r * std::sin(angle);

      // экранные координаты: +x вправо, +y вверх (поэтому y инвертируем)
      int u = static_cast<int>(center.x + x * px_per_m);
      int v = static_cast<int>(center.y - y * px_per_m);

      // отсекаем, если за пределами
      if (u < 0 || u >= W || v < 0 || v >= H)
        continue;

      poly.emplace_back(u, v);
    }

    // Рисуем контур: полилиния + точки
    if (poly.size() >= 2)
    {
      cv::polylines(img, poly, false, cv::Scalar(0, 255, 0), 1, cv::LINE_AA);
    }
    for (const auto &p : poly)
    {
      cv::circle(img, p, 1, cv::Scalar(0, 200, 255), -1, cv::LINE_AA);
    }

    // Робот в центре
    cv::circle(img, center, 4, cv::Scalar(255, 255, 255), -1, cv::LINE_AA);

    // Показ + обновление
    cv::imshow(win_, img);
    cv::waitKey(1);
  }

  static void drawGrid(cv::Mat &img, const cv::Point &c, float px_per_m)
  {
    // окружности каждые 1м и оси
    for (int m = 1; m <= 10; ++m)
    {
      int r = static_cast<int>(m * px_per_m);
      cv::circle(img, c, r, cv::Scalar(40, 40, 40), 1, cv::LINE_AA);
    }
    cv::line(img, cv::Point(0, c.y), cv::Point(img.cols, c.y), cv::Scalar(60, 60, 60), 1, cv::LINE_AA);
    cv::line(img, cv::Point(c.x, 0), cv::Point(c.x, img.rows), cv::Scalar(60, 60, 60), 1, cv::LINE_AA);
  }

  std::string win_ = "Lidar 2D (contour)";
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LidarViewer>());
  auto node = std::make_shared<ImageViewer>();
  
  rclcpp::spin(node);
  
  rclcpp::shutdown();
  return 0;
}