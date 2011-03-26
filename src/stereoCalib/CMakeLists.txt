project(stereoCalib)

file(GLOB source *.cpp)
file(GLOB header *.h)

source_group("Source Files" FILES ${source})
source_group("Header Files" FILES ${header})

include_directories(${PROJECT_SOURCE_DIR}/../auxiliaryFunctions)

add_executable(stereoCalib ${source} ${header} ${AUXFCN})
target_link_libraries(stereoCalib ${ICUB_LIBRARIES} ${YARP_LIBRARIES} ${OPENCV_LIBRARIES})
install(TARGETS stereoCalib DESTINATION ${ICUB_DIR}/bin)

