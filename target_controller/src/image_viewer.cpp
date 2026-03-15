#include <opencv2/core/types.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/opencv.hpp>

#include <cmath>
#include <limits>

struct Detection
{
  bool found = false;
  cv::Rect bbox;
  cv::Point2f center;
  int cell_x = -1;
  int cell_y = -1;
};

struct LidarPoint
{//  в системе лидара
    float x = 0.0f;      
    float y = 0.0f;      
    float range = 0.0f;  
    float angle = 0.0f;  
};

class ImageViewer : public rclcpp::Node
{
public:
  ImageViewer() : Node("image_viewer")
  {

    this->declare_parameter<std::string>("image_topic", "/camera/image");
    const auto topic = this->get_parameter("image_topic").as_string();
    
    lidar_sub_.subscribe(this, "/scan");
    camera_sub_.subscribe(this, topic);
    sync_ = std::make_shared<Synchronizer>(
        SyncPolicy(10), lidar_sub_, camera_sub_);
    sync_->registerCallback(
        std::bind(&ImageViewer::fusionCallback, this,
                  std::placeholders::_1,
                  std::placeholders::_2));
    /*
    auto qos = rclcpp::QoS(rclcpp::KeepLast(5)).reliable();
    sub_ = this->create_subscription<sensor_msgs::msg::Image>(
        topic, qos,
        std::bind(&ImageViewer::cb, this, std::placeholders::_1));
    scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "/scan", rclcpp::SensorDataQoS(),
        std::bind(&ImageViewer::onScan, this, std::placeholders::_1));
    */
    
    cv::namedWindow("camera", cv::WINDOW_NORMAL);
    RCLCPP_INFO(get_logger(), "Subscribed to: %s", topic.c_str());

    cv::namedWindow(win_, cv::WINDOW_AUTOSIZE);
  }

  ~ImageViewer() override
  {
    cv::destroyAllWindows();
  }

private:
  size_t n_ = 0;
  rclcpp::Clock steady_{RCL_STEADY_TIME};
  using SyncPolicy = message_filters::sync_policies::ApproximateTime<
        sensor_msgs::msg::LaserScan,
        sensor_msgs::msg::Image>;
  using Synchronizer = message_filters::Synchronizer<SyncPolicy>;

  message_filters::Subscriber<sensor_msgs::msg::LaserScan> lidar_sub_;
  message_filters::Subscriber<sensor_msgs::msg::Image> camera_sub_;
  std::shared_ptr<Synchronizer> sync_;

  void fusionCallback(
        const sensor_msgs::msg::LaserScan::ConstSharedPtr & lidar_msg,
        const sensor_msgs::msg::Image::ConstSharedPtr & camera_msg)
  {
      auto lidar_data = onScan(lidar_msg);
      auto camera_data = cb(camera_msg);
      processTogether(lidar_data, camera_data);
  }

  void processTogether(std::vector<LidarPoint> lidar_data, Detection camera_data)
  {
      RCLCPP_INFO(this->get_logger(),
                  "Получили синхронизированную пару и обработали её");
  }

  Detection cb(const sensor_msgs::msg::Image::ConstSharedPtr msg)
  {
    Detection defDetect;
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

        //cv::imshow("camera", bgr);

        // ищем квадрат и клетку (например, сетка 8x6)
        auto det = detectRedSquareAndCell(bgr, 30, 30);
        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 500,
                               "maxAspect %.1f",
                                this->maxAspect);
        if (det.found)
        {
          cv::rectangle(bgr, det.bbox, cv::Scalar(0, 255, 0), 2);
          cv::circle(bgr, det.center, 3, cv::Scalar(255, 255, 255), -1);

          // подпись
          std::string txt = "cell=(" + std::to_string(det.cell_x) + "," + std::to_string(det.cell_y) + ")";
          cv::putText(bgr, txt, cv::Point(det.bbox.x, std::max(0, det.bbox.y - 8)),
                      cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 255, 0), 2);

          // чтобы не спамить лог
          RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 500,
                               "Red square at px=(%.1f,%.1f) cell=(%d,%d)",
                               det.center.x, det.center.y, det.cell_x, det.cell_y);
          
          cv::rectangle(bgr,det.bbox,cv::Scalar(0,255,0),2);
          cv::imshow("camera", bgr);
          return det;
          
        }
      }
      else
      {

        cv_ptr = cv_bridge::toCvShare(msg, msg->encoding);
        cv::imshow("camera", cv_ptr->image);
        return defDetect;
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

  Detection detectRedSquareAndCell(
      const cv::Mat &bgr,
      int grid_cols,
      int grid_rows)
  {
    Detection d;
    if (bgr.empty())
      return d;

    cv::Mat hsv;
    cv::cvtColor(bgr, hsv, cv::COLOR_BGR2HSV);

    // Красный: два диапазона Hue
    cv::Mat mask1, mask2, mask;
    cv::inRange(hsv, cv::Scalar(0, 80, 80), cv::Scalar(10, 255, 255), mask1);
    cv::inRange(hsv, cv::Scalar(170, 80, 80), cv::Scalar(180, 255, 255), mask2);
    mask = mask1 | mask2;

    // Убираем шум
    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5, 5));
    cv::morphologyEx(mask, mask, cv::MORPH_OPEN, kernel);
    cv::morphologyEx(mask, mask, cv::MORPH_CLOSE, kernel);

    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    double bestScore = 0.0;
    cv::Rect bestRect;
    std::vector<cv::Point> bestApprox;
    double maxAspect = 0.;
    for (const auto &c : contours)
    {
      double area = cv::contourArea(c);
      if (area < 400.0)
        continue; // фильтр по площади (подстрой)

      cv::Rect r = cv::boundingRect(c);
      double aspect = (double)r.width / (double)r.height;
      if (aspect>maxAspect){
        maxAspect = aspect;
      }
      if (aspect < 0.5 || aspect > 1.5)
        continue; // близко к квадрату

      // Аппроксимация контура -> квадрат обычно даёт 4 вершины
      //*
      std::vector<cv::Point> approx;
      double peri = cv::arcLength(c, true);
      cv::approxPolyDP(c, approx, 0.02 * peri, true);
      if ((int)approx.size() >= 8)
        continue;
      if (!cv::isContourConvex(approx))//выпуклость
        continue;
      //*/
      // Скор: площадь * “квадратность”
      double fill = area / (double)(r.area() + 1);
      double score = area * fill;

      if (score > bestScore)
      {
        bestScore = score;
        bestRect = r;
        bestApprox = approx;
      }
    }
    this->maxAspect = maxAspect;
    //if (bestScore <= 0.0)
    //  return d;

    d.found = true;
    d.bbox = bestRect;
    d.center = cv::Point2f(bestRect.x + bestRect.width * 0.5f,
                           bestRect.y + bestRect.height * 0.5f);

    // Определяем “клетку” сетки grid_cols x grid_rows
    const int W = bgr.cols;
    const int H = bgr.rows;

    int cellW = std::max(1, W / grid_cols);
    int cellH = std::max(1, H / grid_rows);

    d.cell_x = std::clamp((int)(d.center.x / cellW), 0, grid_cols - 1);
    d.cell_y = std::clamp((int)(d.center.y / cellH), 0, grid_rows - 1);

    return d;
  }

  std::vector<LidarPoint> onScan(const sensor_msgs::msg::LaserScan::ConstSharedPtr msg)
  {
    std::vector<LidarPoint> points;
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
    points.reserve(msg->ranges.size());

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
      points.push_back({x,y,r,angle});
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
    return points;
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


  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_;
  std::string win_ = "Lidar 2D (contour)";
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  double maxAspect = 0.;
};




int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  //rclcpp::spin(std::make_shared<LidarViewer>());
  auto node = std::make_shared<ImageViewer>();

  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}