#include <zivid_camera/Capture.h>
#include <zivid_camera/CaptureAssistantSuggestSettings.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <ros/ros.h>

//include service header
#include <zivid_capture/ZividCaptureSuggested.h>


#define CHECK(cmd)                                                                                                     \
  do                                                                                                                   \
  {                                                                                                                    \
    if (!cmd)                                                                                                          \
    {                                                                                                                  \
      throw std::runtime_error{ "\"" #cmd "\" failed!" };                                                              \
    }                                                                                                                  \
  } while (false)

namespace
{
const ros::Duration default_wait_duration{ 30 };
constexpr auto ca_suggest_settings_service_name = "/zivid_camera/capture_assistant/suggest_settings";

void capture_assistant_suggest_settings()
{
  zivid_camera::CaptureAssistantSuggestSettings cass;
  cass.request.max_capture_time = ros::Duration{ 1.20 };
  cass.request.ambient_light_frequency =
      zivid_camera::CaptureAssistantSuggestSettings::Request::AMBIENT_LIGHT_FREQUENCY_NONE;

  ROS_INFO_STREAM("Calling " << ca_suggest_settings_service_name
                             << " with max capture time = " << cass.request.max_capture_time << " sec");
  CHECK(ros::service::call(ca_suggest_settings_service_name, cass));
}

void capture()
{
  ROS_INFO("Calling capture service");
  zivid_camera::Capture capture;
  CHECK(ros::service::call("/zivid_camera/capture", capture));
}

}  // namespace

bool execute_capture(zivid_capture::ZividCaptureSuggested::Request  &req,
                     zivid_capture::ZividCaptureSuggested::Response &res)
{
  ROS_INFO("Get suggested settings...");

  capture_assistant_suggest_settings();

  ROS_INFO("Capture...");

  capture();

  ROS_INFO("Capture...OK");

  return true;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "capture_assistant");
  ros::NodeHandle n;
  ros::ServiceServer service = n.advertiseService("zivid_capture/zivid_capture_suggested", execute_capture);
  
  CHECK(ros::service::waitForService(ca_suggest_settings_service_name, default_wait_duration));

  ROS_INFO("Ready to capture with suggested settings.");

  ros::spin();

  return 0;
}
