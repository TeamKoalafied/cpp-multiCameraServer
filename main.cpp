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

  // start cameras
  std::vector<cs::VideoSource> cameras;
  for (auto&& cameraConfig : cameraConfigs)
    cameras.emplace_back(StartCamera(cameraConfig));

  // start image processing on camera 0 if present
  if (cameras.size() >= 1) {
    std::thread([&] {
      // frc::VisionRunner<MyPipeline> runner(cameras[0], new MyPipeline(),
      //                                      [&](MyPipeline &pipeline) {
      //
      // });
      /* something like this for GRIP:
      frc::VisionRunner<MyPipeline> runner(cameras[0], new grip::GripPipeline(),
                                           [&](grip::GripPipeline& pipeline) {
        ...
      });
       */

      //runner.RunForever();
    const int kWidth = 320;
    const int kHeight = 240;
    int object_X_Max=0;
    int object_Y_Max=0;
    int object_Y_Min=0;
    int object_X_Min=0;
    int centreX; //centre x
    int centreY; //centre y
    int objectWidth = 0;
    int objectArea = 0;

    nt::NetworkTableEntry xMaxEntry;
    nt::NetworkTableEntry xMinEntry;
    nt::NetworkTableEntry yMaxEntry;
    nt::NetworkTableEntry yMinEntry;
    nt::NetworkTableEntry xLenEntry;
    nt::NetworkTableEntry yLenEntry;
    nt::NetworkTableEntry areaEntry;
    nt::NetworkTableEntry objectWidthEntry;

    auto inst = nt::NetworkTableInstance::GetDefault();
    auto table = inst.GetTable("GripOutputValues");

    xLenEntry = table -> GetEntry("xLen");
    yLenEntry = table -> GetEntry("yLen");
    xMaxEntry = table -> GetEntry("xMax");
    xMinEntry = table -> GetEntry("xMin");
    yMaxEntry = table -> GetEntry("yMax");
    yMinEntry = table -> GetEntry("yMin");
    areaEntry = table -> GetEntry("area");
    objectWidthEntry = table -> GetEntry("objectWidth");

    cameras[0].SetResolution(kWidth, kHeight);
    cameras[1].SetResolution(kWidth, kHeight);
    cs::CvSink croppedSink = frc::CameraServer::GetInstance()->GetVideo(cameras[0]);
    cs::CvSink crosshairsSink = frc::CameraServer::GetInstance()->GetVideo(cameras[1]);
    // Setup a CvSource. This will send images back to the Dashboard
    cs::CvSource croppedOutput =
        frc::CameraServer::GetInstance()->PutVideo("Cropped", kWidth, 80);
    cs::CvSource crosshairsOutput =
        frc::CameraServer::GetInstance()->PutVideo("Crosshairs", kWidth, kHeight);
    cs::CvSource pipelineOutput =
        frc::CameraServer::GetInstance()->PutVideo("GripPipeline", kWidth, kHeight);

    // Mats are very memory expensive. Lets reuse this Mat.
    cv::Mat wideFovMat;
    cv::Mat crosshairsMat;
    cv::Mat pipelineMat;
//declaring grip pipelines
    grip::GripCargoPipeline* pipeline = new grip::GripCargoPipeline();

    while (true) {
      const int thresh = 10;
      // Tell the CvSink to grab a frame from the camera and put it
      // in the source mat.  If there is an error notify the output.
      if (croppedSink.GrabFrame(wideFovMat) == 0) {
        // Send the output the error.
        croppedOutput.NotifyError(croppedSink.GetError());
        // skip the rest of the current iteration
        continue;
      }
      if (crosshairsSink.GrabFrame(crosshairsMat) == 0) {
        // Send the output the error.
        crosshairsOutput.NotifyError(crosshairsSink.GetError());
        // skip the rest of the current iteration
        continue;
      }
      // Put a rectangle on the image (x, y, width, height)
      cv::Rect rectangle = cv::Rect(0,80,kWidth,80);
      wideFovMat = wideFovMat(rectangle);
      // add the crosshairs
      cv::line(crosshairsMat, cv::Point(160, 80), cv::Point(160,105), CV_RGB(255,0,0));
      cv::line(crosshairsMat, cv::Point(160, 135), cv::Point(160,160), CV_RGB(255,0,0));
      cv::line(crosshairsMat, cv::Point(120, 120), cv::Point(145,120), CV_RGB(255,0,0));
      cv::line(crosshairsMat, cv::Point(175, 120), cv::Point(200,120), CV_RGB(255,0,0));
      // Give the output stream a new image to display
      croppedOutput.PutFrame(wideFovMat);
      crosshairsOutput.PutFrame(crosshairsMat);

      //Vision process
      pipeline->Process(wideFovMat);
      pipelineMat = *(pipeline->GetRgbThresholdOutput());
      //Vision pixel process
      struct Pixbgr
      {
        unsigned char b:8;
      };
      object_X_Max=0;
      object_Y_Max=0;
      object_Y_Min=pipelineMat.rows-1;
      object_X_Min=pipelineMat.cols-1;
      for(int i = 0; i < pipelineMat.rows; i++)
      {
        const struct Pixbgr* Mi = pipelineMat.ptr<struct Pixbgr>(i);
        for(int j = 0; j < pipelineMat.cols; j++)
        {
          if (Mi[j].b > thresh)
          {
            if (j < object_X_Min)
              object_X_Min = j;
            if (j > object_X_Max)
              object_X_Max = j;
            if (i < object_Y_Min)
              object_Y_Min = i;
            if (i > object_Y_Max)
              object_Y_Max = i;
          }
        }
      }

      //Send values to NetworkTables
      xMaxEntry.SetDouble(object_X_Max);
      xMinEntry.SetDouble(object_X_Min);
      yMaxEntry.SetDouble(object_Y_Max);
      yMinEntry.SetDouble(object_Y_Min);
      //Calculate area
      objectArea = (object_X_Max-object_X_Min) * (object_Y_Max-object_Y_Min);
      std::cout << objectArea << std::endl;
      std::cout << object_X_Max << std::endl;
      std::cout << object_X_Min << std::endl;
      objectWidth = sqrt(objectArea);
      //it is the average of the centres of object
      centreX = object_X_Max - object_X_Min;
      centreY = object_Y_Max - object_Y_Min;
      std::cout << centreX << std::endl;
      std::cout << centreY << std::endl;
      pipelineOutput.PutFrame(pipelineMat);
      xLenEntry.SetDouble(centreX);
      yLenEntry.SetDouble(centreY);
      areaEntry.SetDouble(objectArea);
      objectWidthEntry.SetDouble(objectWidth);
     }
   }).detach();
  }

  // loop forever
  for (;;) std::this_thread::sleep_for(std::chrono::seconds(10));
}
