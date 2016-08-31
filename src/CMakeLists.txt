
FIND_PACKAGE(OpenCV REQUIRED)


#INCLUDE_DIRECTORIES( ${PROJECT_SOURCE_DIR}/include )

LIST(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules)

FIND_PACKAGE( CSparse)
INCLUDE_DIRECTORIES( ${G2O_INCLUDE_DIR} ${CSPARSE_INCLUDE_DIR})

SET(optical_flow_src_list main_optical_flow.cpp optical_flow_data_class.cpp)
ADD_EXECUTABLE(Optical_Flow ${optical_flow_src_list})
TARGET_LINK_LIBRARIES( Optical_Flow ${OpenCV_LIBS} ${PCL_LIBRARIES})