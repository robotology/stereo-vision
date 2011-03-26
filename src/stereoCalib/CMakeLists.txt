project(stereoCalib)

file(GLOB source_mod *.cpp)
file(GLOB header_mod *.h)
file(GLOB source_aux ../auxialiryFunctions/*.cpp ../auxialiryFunctions/*.c)
file(GLOB header_aux ../auxialiryFunctions/*.h)

source_group("Source Files" FILES ${source_mod} ${source_aux})
source_group("Header Files" FILES ${header_mod} ${header_aux})

add_executable(stereoCalib ${source_mod} ${source_aux} ${header_mod} ${header_aux})
target_link_libraries(stereoCalib ${ICUB_LIBRARIES} ${YARP_LIBRARIES} ${OPENCV_LIBRARIES})
install(TARGETS stereoCalib DESTINATION ${ICUB_DIR}/bin)

