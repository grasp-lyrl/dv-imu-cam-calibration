# Init CMake (require at least version 3.16)
CMAKE_MINIMUM_REQUIRED(VERSION 3.16)

PROJECT(dv-imu-cam-calibration
	VERSION 1.1.0
	LANGUAGES C CXX)
#SET(IMU_CAM_CALIB_LIBRARY_NAME kalibr_imu_camera_calibration) # ${CMAKE_PROJECT_NAME}

ADD_DEFINITIONS(-march=native)

OPTION(ICC_ENABLE_SAMPLES "Build usage samples" ON)

OPTION(ENABLE_IMU "Compile code to calibrate IMU" OFF)
IF(ENABLE_IMU)
	ADD_DEFINITIONS(-DWITH_IMU_CALIBRATION=1)
ELSE()
	ADD_DEFINITIONS(-DWITH_IMU_CALIBRATION=0)
ENDIF()


# Define installation paths
INCLUDE(GNUInstallDirs)

# Basic setup
FIND_PACKAGE(dv 1.5.2 REQUIRED)


SET(CMAKE_C_VISIBILITY_PRESET default)
SET(CMAKE_CXX_VISIBILITY_PRESET default)

# Set full RPATH, modules are libraries for DV
SET(CMAKE_INSTALL_RPATH ${USER_LOCAL_PREFIX}/${DV_MODULES_DIR}/imu-calibration)

add_subdirectory(src)
add_subdirectory(thirdparty)
#
# Enable samples
IF(ICC_ENABLE_SAMPLES)
	MESSAGE(STATUS "Imu camera calibration samples enabled")
	ADD_SUBDIRECTORY(samples)
ENDIF()
