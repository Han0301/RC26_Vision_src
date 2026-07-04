SET(PROJECT_NAME vikit_common)
PROJECT(${PROJECT_NAME})
CMAKE_MINIMUM_REQUIRED (VERSION 2.8.3)
SET(CMAKE_BUILD_TYPE Release) # Release, RelWithDebInfo
SET(CMAKE_VERBOSE_MAKEFILE OFF)
SET(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${PROJECT_SOURCE_DIR}/CMakeModules/")
SET(USE_ROS TRUE) # Set False if you want to build this package without Catkin

# Set build flags. Set IS_ARM on odroid board as environment variable
SET(CMAKE_CXX_FLAGS "-Wall -D_LINUX -D_REENTRANT -march=native -Wno-unused-variable -Wno-unused-but-set-variable -Wno-unknown-pragmas")
message("Current CPU archtecture: ${CMAKE_SYSTEM_PROCESSOR}")
if(CMAKE_SYSTEM_PROCESSOR MATCHES "(x86)|(X86)|(amd64)|(AMD64)" )
  SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -mmmx -msse -msse -msse2 -msse3 -mssse3")
else()
  SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -march=armv8-a")
endif()

IF(CMAKE_COMPILER_IS_GNUCC)
  SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
ELSE()
  SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
ENDIF()
SET(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS} -O3 -fsee -fomit-frame-pointer -fno-signed-zeros -fno-math-errno -funroll-loops")

# Add plain cmake packages 
FIND_PACKAGE(OpenCV REQUIRED)
FIND_PACKAGE(Eigen REQUIRED)

# 🔴 核心修改：手动强制指定 Sophus，不依赖 find_package 变量！
# 直接写死路径，100% 能找到
SET(SOPHUS_INCLUDE_DIR "/usr/local/include")
SET(SOPHUS_LIBRARY "/usr/local/lib/libSophus.so")

# Include dirs
INCLUDE_DIRECTORIES(
  include
  ${EIGEN_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${SOPHUS_INCLUDE_DIR}
)

IF(USE_ROS)
  FIND_PACKAGE(catkin REQUIRED COMPONENTS roscpp cmake_modules)
  INCLUDE_DIRECTORIES(${catkin_INCLUDE_DIRS})
  catkin_package(
    DEPENDS Eigen OpenCV
    CATKIN_DEPENDS roscpp
    INCLUDE_DIRS include
    LIBRARIES ${PROJECT_NAME}
  )
ELSE()
  SET(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
  SET(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)
ENDIF()

# Set Sourcefiles
LIST(APPEND SOURCEFILES src/atan_camera.cpp
                        src/omni_camera.cpp
                        src/math_utils.cpp
                        src/vision.cpp
                        src/performance_monitor.cpp
                        src/robust_cost.cpp
                        src/user_input_thread.cpp
                        src/pinhole_camera.cpp
                        src/equidistant_camera.cpp
                        src/polynomial_camera.cpp
                        src/homography.cpp
                        src/img_align.cpp)

# Create vikit library
ADD_LIBRARY(${PROJECT_NAME} SHARED ${SOURCEFILES})
TARGET_LINK_LIBRARIES(${PROJECT_NAME}
  ${OpenCV_LIBS}
  ${SOPHUS_LIBRARY}  # 直接链接 .so 文件
  ${catkin_LIBRARIES})

# Tests
ADD_EXECUTABLE(test_vk_common_camera test/test_camera.cpp)
TARGET_LINK_LIBRARIES(test_vk_common_camera ${PROJECT_NAME} ${SOPHUS_LIBRARY})

ADD_EXECUTABLE(test_vk_common_triangulation test/test_triangulation.cpp)
TARGET_LINK_LIBRARIES(test_vk_common_triangulation ${PROJECT_NAME} ${SOPHUS_LIBRARY})

ADD_EXECUTABLE(test_vk_common_patch_score test/test_patch_score.cpp)
TARGET_LINK_LIBRARIES(test_vk_common_patch_score ${PROJECT_NAME} ${SOPHUS_LIBRARY})


################################################################################
# Create the vikit_commonConfig.cmake file for other cmake projects.
IF(NOT USE_ROS)
  GET_TARGET_PROPERTY( FULL_LIBRARY_NAME ${PROJECT_NAME} LOCATION )
  SET(vikit_common_LIBRARIES ${FULL_LIBRARY_NAME} )
  SET(vikit_common_LIBRARY_DIR ${PROJECT_BINARY_DIR} )
  SET(vikit_common_INCLUDE_DIR "${PROJECT_SOURCE_DIR}/include")
  CONFIGURE_FILE( ${CMAKE_CURRENT_SOURCE_DIR}/vikit_commonConfig.cmake.in
      ${CMAKE_CURRENT_BINARY_DIR}/vikit_commonConfig.cmake @ONLY IMMEDIATE )
  export( PACKAGE vikit_common )

  INSTALL(DIRECTORY include/vikit DESTINATION ${CMAKE_INSTALL_PREFIX}/include FILES_MATCHING PATTERN "*.h" )
  INSTALL(TARGETS ${PROJECT_NAME} DESTINATION ${CMAKE_INSTALL_PREFIX}/lib )
ENDIF()
