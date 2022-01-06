#!/bin/sh
sudo apt-get update -y # To get the latest package lists
sudo apt-get upgrade -y # To install the latest package 
sudo apt-get install xorg -y
sudo apt-get install build-essential -y
sudo apt-get install g++ libglu1-mesa-dev freeglut3 freeglut3-dev mesa-common-dev qt5-default -y
sudo cp libmyelin.so /usr/lib/aarch64-linux-gnu/

unzip eigen-3.3.9.zip
cd eigen-3.3.9
mkdir build
cd build
cmake ..
sudo make install
cd ../../

