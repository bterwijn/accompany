

sudo ls

sudo apt-get -y install aptitude emacs git gitk mercurial libopencv2.3-dev cmake libgstreamer0.10-dev libgstreamer-plugins-base0.10-dev gstreamer-tools gstreamer0.10-x gtk2-engines-pixbuf ros-electric-openni-kinect

cd
mkdir -p ros
export ROS_WORKSPACE=~/ros
export ROS_PACKAGE_PATH=$ROS_WORKSPACE:$ROS_PACKAGE_PATH
echo "export ROS_WORKSPACE=~/ros" >> ~/.bashrc
echo "export ROS_PACKAGE_PATH=$ROS_WORKSPACE:$ROS_PACKAGE_PATH" >> ~/.bashrc
cd ros

# accompany
git clone git://basterwijn.nl/home/bterwijn/git/accompany.git

cd ~
mkdir -p programs


# cmnGwenn
cd ~/programs
git clone git://basterwijn.nl/home/bterwijn/git/cmnGwenn.git
cd cmnGwenn
mkdir build
cd build
cmake ../src
make
sudo make install

# yaml-cpp
cd ~/programs
hg clone https://code.google.com/p/yaml-cpp.new-api yaml-cpp
cd yaml-cpp
mkdir build
cd build
cmake ../
make -j 4
sudo make install

# LogProbOp
cd ~/programs
git clone git://basterwijn.nl/home/bterwijn/git/LogProbOp.git
cd LogProbOp
mkdir build
cd build
cmake ../
make -j 4
sudo make install

# GaussianMixture
cd ~/programs
git clone git://basterwijn.nl/home/bterwijn/git/GaussianMixture.git
cd GaussianMixture
mkdir build
cd build
cmake ../
make -j 4
sudo make install

# KalmanFilter
cd ~/programs
git clone git://basterwijn.nl/home/bterwijn/git/KalmanFilter.git
cd KalmanFilter
mkdir build
cd build
cmake ../
make -j 4
sudo make install

# skeleton_marker
cd ~/ros
svn checkout http://pi-robot-ros-pkg.googlecode.com/svn/trunk/skeleton_markers
rosdep install skeleton_markers
rosmake skeleton_markers

# cob_perception_common
cd ~/ros
git clone git://github.com/ipa320/cob_perception_common.git
rosdep install cob_perception_common
rosmake cob_perception_common

# cob_people_perception
cd ~/ros
git clone https://github.com/ipa320/cob_people_perception.git
rosdep install cob_people_perception
rosmake cob_people_perception

# gscam
rosdep install gscam
rosmake gscam

# accompany
rosdep install accompany_uva
rosmake accompany_uva

# Test
# downloads prerecorded video and does detection and tracking
roscd accompany_uva/scripts
./startTestRobotHouse.sh

# tracks humans and identities using artificial data
roslaunch accompany_human_tracker testTracker.launch
