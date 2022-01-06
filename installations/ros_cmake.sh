cat ../dreamvu_pal_tracking/cmake_template/header.txt > ../dreamvu_pal_tracking/CMakeLists.txt
echo "set(PAL_INCLUDE_DIR" `pwd`/../include ")" >> ../dreamvu_pal_tracking/CMakeLists.txt
echo "set(PAL_LIBRARY" `pwd`/../lib/libPAL.so ")" >> ../dreamvu_pal_tracking/CMakeLists.txt
cat ../dreamvu_pal_tracking/cmake_template/footer.txt >> ../dreamvu_pal_tracking/CMakeLists.txt
