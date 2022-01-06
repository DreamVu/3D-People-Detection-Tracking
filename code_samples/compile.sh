g++ 001_people_tracking.cpp /usr/src/tensorrt/bin/common/logger.o  ../lib/libPAL.so  ../lib/libPAL_DE.so ../lib/libPAL_DEPTH_128.so  ../lib/libPAL_DEPTH_HQ.so ../lib/libPAL_Track.so `pkg-config --libs --cflags opencv python3 libusb-1.0`   -O3  -o 001_people_tracking.out -I../include/ -lv4l2 -lpthread -lcudart -L/usr/local/cuda/lib64 -lnvinfer -I../../monolith/inc -w -std=c++11

g++ 002_people_following.cpp /usr/src/tensorrt/bin/common/logger.o  ../lib/libPAL.so  ../lib/libPAL_DE.so ../lib/libPAL_DEPTH_128.so  ../lib/libPAL_DEPTH_HQ.so ../lib/libPAL_Track.so `pkg-config --libs --cflags opencv python3 libusb-1.0`   -O3  -o 002_people_following.out -I../include/ -lv4l2 -lpthread -lcudart -L/usr/local/cuda/lib64 -lnvinfer -I../../monolith/inc -w -std=c++11

