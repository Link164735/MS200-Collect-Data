
cmake_minimum_required(VERSION 3.5s)
PROJECT(oradar_test)
#add_compile_options(-std=c++14) # Use C++14
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14")
#set(Open3D_DIR /usr/local/lib/cmake/Open3D)

find_package(OpenGL REQUIRED)
find_package(GLUT REQUIRED)
find_package(GLEW REQUIRED)
find_package(glfw3 3.3 REQUIRED)
find_package(Open3D REQUIRED)

message(STATUS "Open3D include dirs: ${Open3D_INCLUDE_DIRS}")

#Include directories
INCLUDE_DIRECTORIES(
     ${CMAKE_SOURCE_DIR}
     ${CMAKE_SOURCE_DIR}/../
     ${CMAKE_CURRENT_BINARY_DIR}
     ${OPENGL_INCLUDE_DIRS} 
     ${GLUT_INCLUDE_DIRS} 
     ${GLEW_INCLUDE_DIRS} 
     ${GLFW_INCLUDE_DIRS}
     ${Open3D_INCLUDE_DIRS}
)

SET(EXECUTABLE_OUTPUT_PATH ${CMAKE_BINARY_DIR})


set(curdir ${CMAKE_CURRENT_SOURCE_DIR})
FILE(GLOB APP_LIST "${curdir}/*.cpp")
foreach(child ${APP_LIST})
  string(REPLACE "${curdir}/" "" app_main ${child})
  string(REPLACE ".cpp" "" APP_NAME ${app_main})
  ADD_EXECUTABLE(${APP_NAME} ${app_main})
  TARGET_LINK_LIBRARIES(${APP_NAME} oradar_sdk ${OPENGL_LIBRARIES} ${GLUT_LIBRARIES} ${GLEW_LIBRARIES} glfw Open3D::Open3D)

  INSTALL(TARGETS ${APP_NAME}
    RUNTIME DESTINATION bin
  )
endforeach()

FILE(GLOB C_APP_LIST "${curdir}/*.c")
foreach(child ${C_APP_LIST})
  string(REPLACE "${curdir}/" "" app_main ${child})
  string(REPLACE ".c" "" APP_NAME ${app_main})
  ADD_EXECUTABLE(${APP_NAME} ${app_main})
  TARGET_LINK_LIBRARIES(${APP_NAME} oradar_sdk)
  target_link_libraries(${APP_NAME} -lstdc++ -lm)
  INSTALL(TARGETS ${APP_NAME}
    RUNTIME DESTINATION bin
  )
endforeach()

