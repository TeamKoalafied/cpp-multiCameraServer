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

#include <vision/VisionPipeline.h>
#include <vision/VisionRunner.h>
#include <wpi/StringRef.h>
#include <wpi/json.h>
#include <wpi/raw_istream.h>
#include <wpi/raw_ostream.h>

#include "cameraserver/CameraServer.h"
#include <opencv2/core/core.hpp>
#include <opencv2/core/types.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <iostream>

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
                server = false;
                auto str = j.at("ntmode").get<std::string>();
                wpi::StringRef s(str);
                if (s.equals_lower("client")) {
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
        wpi::outs() << "Starting camera '" << config.name << "' on " << config.path << '\n';
        auto inst = frc::CameraServer::GetInstance();
        cs::UsbCamera camera{config.name, config.path};
        auto server = inst->StartAutomaticCapture(camera);

        camera.SetConfigJson(config.config);
        //  // It takes a while to open a new connection so keep it open
        //  camera.SetConnectionStrategy(cs::VideoSource::kConnectionKeepOpen);

        if (config.streamConfig.is_object())
        server.SetConfigJson(config.streamConfig);

        return camera;
    }

    // example pipeline
    class MyPipeline : public frc::VisionPipeline {
        public:
        int val = 0;

        void Process(cv::Mat& mat) override {
            ++val;
        }
    };
}  // namespace

int main(int argc, char* argv[]) {

    if (argc >= 2) configFile = argv[1];

    // read configuration
    if (!ReadConfig()) return EXIT_FAILURE;

    // start cameras
    std::vector<cs::VideoSource> cameras;
    for (auto&& cameraConfig : cameraConfigs)
    cameras.emplace_back(StartCamera(cameraConfig));

    //
    // On a Raspberry Pi 3B+, if all the USB ports connect to USB cameras then the
    // cameras can be uniquely identified by the USB device pathnames as follows:
    //
    //	/----------------------------\
    //	| |      | | USB1 | | USB3 | |
    //	| |  IP  |  ======   ======  |
    //	| |      | | USB2 | | USB4 | |
    //	\----------------------------/
    //
    //  USB1: /dev/v4l/by-path/platform-3f980000.usb-usb-0:1.1.2:1.0-video-index0
    //  USB2: /dev/v4l/by-path/platform-3f980000.usb-usb-0:1.1.3:1.0-video-index0
    //  USB3: /dev/v4l/by-path/platform-3f980000.usb-usb-0:1.3:1.0-video-index0
    //  USB4: /dev/v4l/by-path/platform-3f980000.usb-usb-0:1.2:1.0-video-index0
    //

    //UsbCamera frontCamera = CameraServer.getInstance().startAutomaticCapture("Front",
    //  "/.dev/v4l/by-path/platform-3f980000.usb-usb-0:1.2:1.0-video-index0");
    //frontCamera.setVideoMode(PixelFormat.kMJPEG, 320, 240, 30);
    //frontCamera.setBrightness(50);
    //frontCamera.setWhiteBalanceHoldCurrent();
    //frontCamera.setExposureManual(15);

    // start separate image processing threads for each camera if present
    if (cameras.size() >= 1) {

    // c.name = config.at("name").get<std::string>();
    // auto str = cameras.at("name").get<std::string>();
    // wpi::StringRef s(str);
    // if (s.equals_lower("front")) {

        std::thread([&] {
            // Control bandwidth by defining output resolution and camera frame rate divider
            const double kWidth = 320.0;
            const double kHeight = 240.0;
            const int kFrameRateDivider = 2;

            // Front facing drive camera. We just want to draw cross hairs on this.
            cs::CvSink FrontCam = frc::CameraServer::GetInstance()->GetVideo(cameras[0]);
            // Setup a CvSource. This will send images back to the Dashboard
            cs::CvSource FrontSvr =
            frc::CameraServer::GetInstance()->PutVideo("FrontCam", kWidth, kHeight);
            // FrontSvr.SetFPS(10); // This does not seem to work

            // Create mats to hold images
            cv::Mat frontMat;
            cv::Mat frontView;
            int counter = 0;

            while (true) {
                    // Tell the CvSink to grab a frame from the camera and put it
                    // in the source mat.  If there is an error notify the output.
                    if (FrontCam.GrabFrame(frontMat) == 0) {
                    // Send error to the output
                    FrontSvr.NotifyError(FrontCam.GetError());
                    // skip the rest of the current iteration
                    continue;
                }

                // Skip frames (when counter is not 0) to reduce bandwidth
                counter = (counter + 1) % kFrameRateDivider;
                if (!counter) {
                    // Scale the image (if needed) to reduce bandwidth
                    cv::resize(frontMat, frontView, cv::Size(kWidth, kHeight), 0.0, 0.0, cv::INTER_AREA);
                    // Give the output stream a new image to display
                    FrontSvr.PutFrame(frontView);
                }
            }
        }).detach();
    }

    // start separate image processing threads for each camera if present
    if (cameras.size() >= 2) {

        // c.name = config.at("name").get<std::string>();
        // auto str = cameras.at("name").get<std::string>();
        // wpi::StringRef s(str);
        // if (s.equals_lower("back")) {

        std::thread([&] {
            // Control bandwidth by defining output resolution and camera frame rate divider
            const double kWidth = 320.0;
            const double kHeight = 240.0;
            const int kFrameRateDivider = 2;

            // Back facing drive camera. We just want to draw cross hairs on this.
            cs::CvSink BackCam = frc::CameraServer::GetInstance()->GetVideo(cameras[1]);
            // Setup a CvSource. This will send images back to the Dashboard
            cs::CvSource BackSvr =
            frc::CameraServer::GetInstance()->PutVideo("BackCam", kWidth, kHeight);
            // BackSvr.SetFPS(10); // This does not seem to work

            // Create mats to hold images
            cv::Mat backMat;
            cv::Mat backView;
            int counter = 0;

            while (true) {
                // Tell the CvSink to grab a frame from the camera and put it
                // in the source mat.  If there is an error notify the output.
                if (BackCam.GrabFrame(backMat) == 0) {
                    // Send error to the output
                    BackSvr.NotifyError(BackCam.GetError());
                    // skip the rest of the current iteration
                    continue;
                }
                // Skip frames (when counter is not 0) to reduce bandwidth
                counter = (counter + 1) % kFrameRateDivider;
                if (!counter) {
                    // Scale the image (if needed) to reduce bandwidth
                    cv::resize(backMat, backView, cv::Size(kWidth, kHeight), 0.0, 0.0, cv::INTER_AREA);
                    // Give the output stream a new image to display
                    BackSvr.PutFrame(backView);
                }
            }
        }).detach();
    }

    // loop forever
    for (;;) std::this_thread::sleep_for(std::chrono::seconds(10));
}
