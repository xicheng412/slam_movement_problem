
FIND_PACKAGE(OpenCV REQUIRED)


#INCLUDE_DIRECTORIES( ${PROJECT_SOURCE_DIR}/include )

LIST(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules)

FIND_PACKAGE( CSparse)
INCLUDE_DIRECTORIES( ${G2O_INCLUDE_DIR} ${CSPARSE_INCLUDE_DIR})


ADD_EXECUTABLE(optical_flow optical_flow.cpp)
TARGET_LINK_LIBRARIES( optical_flow  
	${OpenCV_LIBS}  
	${PCL_LIBRARIES})

ADD_EXECUTABLE(optical_flow_LK optical_flow_LK.cpp)
TARGET_LINK_LIBRARIES( optical_flow_LK  
	${OpenCV_LIBS}  
	${PCL_LIBRARIES})


ADD_EXECUTABLE(optical_movement  optical_movement.cpp)
TARGET_LINK_LIBRARIES( optical_movement
	${OpenCV_LIBS}  
	${PCL_LIBRARIES})

ADD_EXECUTABLE(opticalFlow  opticalFlow.cpp)
TARGET_LINK_LIBRARIES( opticalFlow
	${OpenCV_LIBS}  
	${PCL_LIBRARIES})