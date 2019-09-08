#ifndef CAMERA_ROS_BASE_H_
#define CAMERA_ROS_BASE_H_

#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv-3.3.1-dev/opencv2/opencv.hpp>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>
#include <camera_info_manager/camera_info_manager.h>
#include <diagnostic_updater/publisher.h>
#include <diagnostic_updater/diagnostic_updater.h>

namespace camera_base {

/**
 * @brief getParam Util function for getting ros parameters under nodehandle
 * @param nh Node handle
 * @param name Parameter name
 * @return Parameter value
 */
template <typename T>
T getParam(const ros::NodeHandle& nh, const std::string& name) {
  T value{};
  if (!nh.getParam(name, value)) {
    ROS_ERROR("Cannot find parameter: %s", name.c_str());
  }
  return value;
}

/**
 * @brief The CameraRosBase class
 * This class implements a ros camera
 */
class CameraRosBase {
 public:
  explicit CameraRosBase(const ros::NodeHandle& pnh,
                         const std::string& prefix = std::string())
      : pnh_(pnh),
        cnh_(pnh, prefix),
        it_(cnh_),
        camera_pub_(it_.advertiseCamera("image_raw", 1)),
        cinfo_mgr_(cnh_, getParam<std::string>(cnh_, "camera_name"),
                   getParam<std::string>(cnh_, "calib_url")),
        fps_(10.0),
        diagnostic_updater_(pnh_, cnh_),
        topic_diagnostic_(
            prefix.empty() ? "image_raw" : (prefix + "/image_raw"),
            diagnostic_updater_,
            diagnostic_updater::FrequencyStatusParam(&fps_, &fps_, 0.1, 10),
            diagnostic_updater::TimeStampStatusParam(-0.01, 0.1)) {
    cnh_.param<std::string>("frame_id", frame_id_, cnh_.getNamespace());
    cnh_.param<std::string>("identifier", identifier_, "");
  }

  CameraRosBase() = delete;
  CameraRosBase(const CameraRosBase&) = delete;
  CameraRosBase& operator=(const CameraRosBase&) = delete;
  virtual ~CameraRosBase() = default;

  const std::string& identifier() const { return identifier_; }
  const std::string& frame_id() const { return frame_id_; }

  double fps() const { return fps_; }
  void set_fps(double fps) { fps_ = fps; }

  /**
   * @brief SetHardwareId Set hardware id for diagnostic updater
   * @param id harware id
   */
  void SetHardwareId(const std::string& id) {
    diagnostic_updater_.setHardwareID(id);
  }

void HistogramEqualize(boost::shared_ptr<sensor_msgs::Image>& image_msg) {
  cv_bridge::CvImagePtr cv_ptr;
  cv::Mat img, src, dst;
  std::vector<cv::Mat> hsv, hsv_hist(3);
  try {
    cv_ptr = cv_bridge::toCvCopy(image_msg, sensor_msgs::image_encodings::BGR8);  
  }
  catch(cv_bridge::Exception &e) {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }
  img = cv_ptr->image;
  cv::cvtColor(img, src, CV_BGR2HSV);
  cv::split(src, hsv);
  hsv_hist = hsv;
  cv::equalizeHist(hsv.at(1), hsv_hist.at(1));
  cv::equalizeHist(hsv.at(2), hsv_hist.at(2));
  cv::merge(hsv_hist, dst);
  cv::cvtColor(dst, img, CV_HSV2BGR);
  image_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", img).toImageMsg();
}

void Undistort(boost::shared_ptr<sensor_msgs::Image>& image_msg, const boost::shared_ptr<sensor_msgs::CameraInfo>& cinfo_msg){
  cv_bridge::CvImagePtr cv_ptr;
  cv::Mat img, dst;
  std::vector<double> camMatrix;
  std::vector<cv::Mat> hsv, hsv_hist(3);
  try { cv_ptr = cv_bridge::toCvCopy(image_msg, sensor_msgs::image_encodings::BGR8); }
  catch(cv_bridge::Exception &e) {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }
  img = cv_ptr->image;
  for(int i = 0; i < 9; i++){ camMatrix.push_back(cinfo_msg->K[i]); }
  cv::undistort(img, dst, camMatrix, cinfo_msg->D);
  image_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", img).toImageMsg();
}

  /**
   * @brief PublishCamera Publish a camera topic with Image and CameraInfo
   * @param time Acquisition time stamp
   */
  void PublishCamera(const ros::Time& time) {
    auto image_msg = boost::make_shared<sensor_msgs::Image>();
    const auto cinfo_msg =
        boost::make_shared<sensor_msgs::CameraInfo>(cinfo_mgr_.getCameraInfo());
    image_msg->header.frame_id = frame_id_;
    image_msg->header.stamp = time;
    if (Grab(image_msg, cinfo_msg)) {
      // Update camera info header
      cinfo_msg->header = image_msg->header;
      Undistort(image_msg, cinfo_msg);
      camera_pub_.publish(image_msg, cinfo_msg);
      topic_diagnostic_.tick(image_msg->header.stamp);
    }
    diagnostic_updater_.update();
  }

  void Publish(const sensor_msgs::ImagePtr& image_msg) {
    const auto cinfo_msg =
        boost::make_shared<sensor_msgs::CameraInfo>(cinfo_mgr_.getCameraInfo());
    // Update camera info header
    image_msg->header.frame_id = frame_id_;
    cinfo_msg->header = image_msg->header;
    camera_pub_.publish(image_msg, cinfo_msg);
    topic_diagnostic_.tick(image_msg->header.stamp);
    diagnostic_updater_.update();
  }

  /**
   * @brief Grab Fill image_msg and cinfo_msg from low level camera driver
   * @param image_msg Ros message ImagePtr
   * @return True if successful
   */
  virtual bool Grab(const sensor_msgs::ImagePtr& image_msg,
                    const sensor_msgs::CameraInfoPtr& cinfo_msgs = nullptr) = 0;

 private:
  ros::NodeHandle pnh_;
  ros::NodeHandle cnh_;
  image_transport::ImageTransport it_;
  image_transport::CameraPublisher camera_pub_;
  camera_info_manager::CameraInfoManager cinfo_mgr_;
  double fps_;
  diagnostic_updater::Updater diagnostic_updater_;
  diagnostic_updater::TopicDiagnostic topic_diagnostic_;
  std::string frame_id_;
  std::string identifier_;
};

}  // namespace camera_base

#endif  // ROS_CAMERA_BASE_H_
