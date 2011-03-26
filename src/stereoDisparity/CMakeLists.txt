
file(GLOB source_mod *.cpp)
file(GLOB header_mod *.h)
file(GLOB source_aux ../auxialiryFunctions/*.cpp ../auxialiryFunctions/*.c)
file(GLOB header_aux ../auxialiryFunctions/*.h)

source_group("Source Files" FILES ${source_mod} ${source_aux})
source_group("Header Files" FILES ${header_mod} ${header_aux})

add_executable(stereoDisparity ${source_mod} ${source_aux} ${header_mod} ${header_aux})
target_link_libraries(stereoDisparity ${ICUB_LIBRARIES} ${YARP_LIBRARIES} ${OPENCV_LIBRARIES})
install(TARGETS stereoDisparity DESTINATION ${ICUB_DIR}/bin)

