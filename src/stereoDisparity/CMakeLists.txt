project(stereoDisparity)

file(GLOB source *.cpp)
file(GLOB header *.h)

source_group("Source Files" FILES ${source})
source_group("Header Files" FILES ${header})

include_directories(${PROJECT_SOURCE_DIR}/../auxiliaryFunctions)

add_executable(stereoDisparity ${source} ${header} ${AUXFCN})
target_link_libraries(stereoDisparity ${ICUB_LIBRARIES} ${YARP_LIBRARIES} ${OPENCV_LIBRARIES})
install(TARGETS stereoDisparity DESTINATION ${ICUB_DIR}/bin)

