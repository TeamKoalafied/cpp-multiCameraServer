# Team Koalafied FRCVision Code
### For Team Koalafied
## Building locally on rPi
1. Run ```make```
2. Run ```make install``` (replaces /home/pi/runCamera)
3. Run ```./runInteractive"``` in /home/pi or ```"sudo svc -t /service/camera``` to
   restart service.

## Building on desktop
### One time Setup
Install the Raspbian compiler [1] as well as GNU make [2].
1. https://github.com/wpilibsuite/raspbian-toolchain/releases
2. (windows) http://gnuwin32.sourceforge.net/packages/make.htm
3. Set the CXX variable in the makefile to the path to ```C:\PATH_TO_DIR\raspbian9\bin\arm-raspbian9-linux-gnueabihf-g++.exe```

### Building
Run ```make```
### Deploying
On the rPi web dashboard:

1. Make the rPi writable by selecting the "Writable" tab
2. In the rPi web dashboard Application tab, select the
   "Uploaded C++ executable" option for Application
3. Click "Browse..." and select the "multiCameraServerExample" executable in
   your desktop project directory
4. Click Save

The application will be automatically started.  Console output can be seen by
enabling console output in the Vision Status tab.
