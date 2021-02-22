equi2omni: warps an equirectangular to an omnidirectional image

February 2021
Author: G. Caron
Contact: guillaume.caron@u-picardie.fr

Prerequisities
0. CMake (version 3.14.5 tested)
2. ViSP (version 3.2.0 tested)
3. libPeR_base (version 0.0.2 tested, https://github.com/PerceptionRobotique/libPeR_base)

Configure and prepare equi2omni to build with catkin
0. export PER_DIR=/path/to/libPeR_base/build/
1. create a "build" directory in the same directory than CMakeLists.txt and cd in
2. run ccmake .., configure and generate
3. make

Run equi2omni
0. Create a "media" directory next to the "build" directory
1. Download examples of equirectangular images in the media directory: http://mis.u-picardie.fr/~g-caron/pub/data/SVMIS_extension/SVMIS_Disco_er_1FPS_94_104.zip
1. run from the command line
  ./equi2omni ../data/omni_400_400.xml ../media/ 94 104 400 400 0
command line arguments are:
* xmlFic the omnidirectional camera calibration xml file (the data directory stores an example of an omnidirectional camera xml file)
* imagesDir directory where equirectangular images to read (with 6 digits before the extension) are and where the output omni images will be written (with character 'o' before the 6 digits)
* iFirst the number of the first image to transform
* iLast the number of the last image to transform
* width the width (pixels) of the output omnidirectional image
* height the height (pixels) of the output omnidirectional image
* camOri omnidirectional camera axis orientation (degrees): 0 (default) downward, 180 upward

