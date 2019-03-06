/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include <cstdio>
#include <string>
#include <thread>
#include <vector>
#include "GripCargoPipeline.h"
#include "GripStripPipeline.h"
#include "GripHatchPipeline.h"
#include <networktables/NetworkTableInstance.h>
#include <vision/VisionPipeline.h>
#include <vision/VisionRunner.h>
#include <wpi/StringRef.h>
#include <wpi/json.h>
#include <wpi/raw_istream.h>
#include <wpi/raw_ostream.h>
#include <opencv2/core/core.hpp>
#include <opencv2/core/types.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "cameraserver/CameraServer.h"
#include <networktables/NetworkTableEntry.h>
#include <networktables/NetworkTable.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/video/video.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>


/*
   JSON format:
   {
       "team": <team number>,
       "ntmode": <"client" or "server", "client" if unspecified>
       "cameras": [
           {
               "name": <camera name>
               "path": <path, e.g. "/dev/video0">
               "pixel format": <"MJPEG", "YUYV", etc>   // optional
               "width": <video mode width>              // optional
               "height": <video mode height>            // optional
               "fps": <video mode fps>                  // optional
               "brightness": <percentage brightness>    // optional
               "white balance": <"auto", "hold", value> // optional
               "exposure": <"auto", "hold", value>      // optional
               "properties": [                          // optional
                   {
                       "name": <property name>
                       "value": <property value>
                   }
               ],
               "stream": {                              // optional
                   "properties": [
                       {
                           "name": <stream property name>
                           "value": <stream property value>
                       }
                   ]
               }
           }
       ]
   }
 */

 const int k_HResolution = 320;
 const int k_VResolution = 240;
 const int k_WCameraHFOV = 128;
 const int k_WCameraVFOV = 96;
 const int k_LightCamCameraHFOV = 52;
 const int k_LightCamCameraVFOV = 39;

static const char* configFile = "/boot/frc.json";

namespace {

unsigned int team;
bool server = false;

struct CameraConfig {
  std::string name;
  std::string path;
  wpi::json config;
  wpi::json streamConfig;
};

std::vector<CameraConfig> cameraConfigs;

wpi::raw_ostream& ParseError() {
  return wpi::errs() << "config error in '" << configFile << "': ";
}

bool ReadCameraConfig(const wpi::json& config) {
  CameraConfig c;

  // name
  try {
    c.name = config.at("name").get<std::string>();
  } catch (const wpi::json::exception& e) {
    ParseError() << "could not read camera name: " << e.what() << '\n';
    return false;
  }

  // path
  try {
    c.path = config.at("path").get<std::string>();
  } catch (const wpi::json::exception& e) {
    ParseError() << "camera '" << c.name
                 << "': could not read path: " << e.what() << '\n';
    return false;
  }

  // stream properties
  if (config.count("stream") != 0) c.streamConfig = config.at("stream");

  c.config = config;

  cameraConfigs.emplace_back(std::move(c));
  return true;
}

bool ReadConfig() {
  // open config file
  std::error_code ec;
  wpi::raw_fd_istream is(configFile, ec);
  if (ec) {
    wpi::errs() << "could not open '" << configFile << "': " << ec.message()
                << '\n';
    return false;
  }

  // parse file
  wpi::json j;
  try {
    j = wpi::json::parse(is);
  } catch (const wpi::json::parse_error& e) {
    ParseError() << "byte " << e.byte << ": " << e.what() << '\n';
    return false;
  }

  // top level must be an object
  if (!j.is_object()) {
    ParseError() << "must be JSON object\n";
    return false;
  }

  // team number
  try {
    team = j.at("team").get<unsigned int>();
  } catch (const wpi::json::exception& e) {
    ParseError() << "could not read team number: " << e.what() << '\n';
    return false;
  }

  // ntmode (optional)
  if (j.count("ntmode") != 0) {
    try {
      auto str = j.at("ntmode").get<std::string>();
      wpi::StringRef s(str);
      if (s.equals_lower("client")) {
        server = false;
      } else if (s.equals_lower("server")) {
        server = true;
      } else {
        ParseError() << "could not understand ntmode value '" << str << "'\n";
      }
    } catch (const wpi::json::exception& e) {
      ParseError() << "could not read ntmode: " << e.what() << '\n';
    }
  }

  // cameras
  try {
    for (auto&& camera : j.at("cameras")) {
      if (!ReadCameraConfig(camera)) return false;
    }
  } catch (const wpi::json::exception& e) {
    ParseError() << "could not read cameras: " << e.what() << '\n';
    return false;
  }

  return true;
}

cs::UsbCamera StartCamera(const CameraConfig& config) {
  wpi::outs() << "Starting camera '" << config.name << "' on " << config.path
              << '\n';
  auto inst = frc::CameraServer::GetInstance();
  cs::UsbCamera camera{config.name, config.path};
  auto server = inst->StartAutomaticCapture(camera);

  camera.SetConfigJson(config.config);
  // camera.SetConnectionStrategy(cs::VideoSource::kConnectionKeepOpen);

  if (config.streamConfig.is_object())
    server.SetConfigJson(config.streamConfig);
    return camera;
}

// example pipeline
/*
class MyPipeline : public frc::VisionPipeline {
 public:
  int val = 0;

  void Process(cv::Mat& mat) override {
    ++val;
  }
};
*/
}  // namespace


int main(int argc, char* argv[]) {
  if (argc >= 2) configFile = argv[1];

  // read configuration
  if (!ReadConfig()) return EXIT_FAILURE;

  // start NetworkTables
  auto ntinst = nt::NetworkTableInstance::GetDefault();
  if (server) {
    wpi::outs() << "Setting up NetworkTables server\n";
    ntinst.StartServer();
  } else {
    wpi::outs() << "Setting up NetworkTables client for team " << team << '\n';
    ntinst.StartClientTeam(team);
  }

      std::vector<cs::VideoSource> cameras;
      for (auto&& cameraConfig : cameraConfigs)
        cameras.emplace_back(StartCamera(cameraConfig));
      // NetworkTableEntry frontOrBack;
      // NetworkTable visionTable = ntinst.getTable("visionTable");

      // std::thread([&] {
      //     cameras[2].SetResolution(320,240);
      //     cs::CvSink lifeCamSink = frc::CameraServer::GetInstance()->GetVideo(cameras[1]);
      //     cs::CvSource crosshairsOutput =
      //         frc::CameraServer::GetInstance()->PutVideo("Crosshairs", 320, 80);
      //
      //     cv::Mat crosshairsMat;
      //
      //     while (true){
      //         if (lifeCamSink.GrabFrame(crosshairsMat) == 0) {
      //           // Send the output the error.
      //           crosshairsOutput.NotifyError(lifeCamSink.GetError());
      //           // skip the rest of the current iteration
      //           continue;
      //         }
      //
      //         int xCrosshairOffset = 0;
      //         int yCrosshairOffset = 0;
      //         // add the crosshairs
      //         cv::line(crosshairsMat, cv::Point(160 + xCrosshairOffset, 80 + yCrosshairOffset), cv::Point(160 + xCrosshairOffset,105 + yCrosshairOffset), CV_RGB(255,0,0));    // vertical
      //         cv::line(crosshairsMat, cv::Point(160 + xCrosshairOffset, 135 + yCrosshairOffset), cv::Point(160 + xCrosshairOffset,160 + yCrosshairOffset), CV_RGB(255,0,0));   // vertical
      //         cv::line(crosshairsMat, cv::Point(120 + xCrosshairOffset, 120 + yCrosshairOffset), cv::Point(145 + xCrosshairOffset,120 + yCrosshairOffset), CV_RGB(255,0,0));   // horizontal
      //         cv::line(crosshairsMat, cv::Point(175 + xCrosshairOffset, 120 + yCrosshairOffset), cv::Point(200 + xCrosshairOffset,120 + yCrosshairOffset), CV_RGB(255,0,0));   // horizontal
      //         // Give the output stream a new image to display
      //         crosshairsOutput.PutFrame(crosshairsMat);
      //     }
      // }).detach();

      std::thread([&] {
          cameras[1].SetResolution(320,240);
          cs::CvSink backSink = frc::CameraServer::GetInstance()->GetVideo(cameras[2]);
        //   cs::CvSource rotateOutput =
        //         frc::CameraServer::GetInstance()->PutVideo("rotated", 320, 240);
        //  cv::Mat src;
        //  double angle = 90;
        //   while(true){
        //       backSink.GrabFrame(src);
        //     // get rotation matrix for rotating the image around its center in pixel coordinates
        //     cv::Point2f center((src.cols-1)/2.0, (src.rows-1)/2.0);
        //     cv::Mat rot = cv::getRotationMatrix2D(center, angle, 1.0);
        //     // determine bounding rectangle, center not relevant
        //     cv::Rect2f bbox = cv::RotatedRect(cv::Point2f(), src.size(), angle).boundingRect2f();
        //     // adjust transformation matrix
        //     rot.at<double>(0,2) += bbox.width/2.0 - src.cols/2.0;
        //     rot.at<double>(1,2) += bbox.height/2.0 - src.rows/2.0;
        //
        //     cv::Mat dst;
        //     cv::warpAffine(src, dst, rot, bbox.size());
        //
        //     rotateOutput.PutFrame(dst);
        // }

      }).detach();

      std::thread([&] {
          cameras[0].SetResolution(320,240);
          cs::CvSink frontSink = frc::CameraServer::GetInstance()->GetVideo(cameras[0]);


      }).detach();
      // std::thread t0 (frontCamera);
      // std::thread t1 (backCamera);
      // lifeCam();
      // std::thread t2 (lifeCam);

      // t0.detach();
      // t1.detach();
      // t2.detach();

  // loop forever
  for (;;) std::this_thread::sleep_for(std::chrono::seconds(10));
}
