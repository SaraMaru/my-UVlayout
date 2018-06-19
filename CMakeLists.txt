CMAKE_MINIMUM_REQUIRED(VERSION 3.9)

SET(TARGET_NAME myUVlayout)

FIND_PACKAGE(OpenGL)
FIND_PACKAGE(GLUT)

INCLUDE_DIRECTORIES(
  ${Assimp_SOURCE_DIR}/include
  ${Assimp_SOURCE_DIR}/code
  ${OPENGL_INCLUDE_DIR}
  ${GLUT_INCLUDE_DIR}
)

LINK_DIRECTORIES(
  ${Assimp_BINARY_DIR}
  ${Assimp_BINARY_DIR}/lib
)

ADD_EXECUTABLE(
  ${TARGET_NAME}
  viewer.cpp
  gen_info.cpp
  segmentation.cpp
  parameterization.cpp
  packing.cpp
  gen_obj.cpp
)

TARGET_LINK_LIBRARIES(
  ${TARGET_NAME}
  ${OPENGL_LIBRARIES}
  ${GLUT_LIBRARIES}
  assimp
  freeimage 
)

INSTALL( 
  TARGETS ${TARGET_NAME}
  DESTINATION `pwd` 
  COMPONENT assimp-dev
)
