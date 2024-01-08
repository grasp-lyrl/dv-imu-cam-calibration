# Init CMake (require at least version 3.16)
CMAKE_MINIMUM_REQUIRED(VERSION 3.16)

PROJECT(
	dv-imu-cam-calibration
	VERSION 1.1.0
	LANGUAGES C CXX)
# SET(IMU_CAM_CALIB_LIBRARY_NAME kalibr_imu_camera_calibration) # ${CMAKE_PROJECT_NAME}

ADD_DEFINITIONS(-march=native)

OPTION(IMU_CAM_CALIB_ENABLE_SAMPLES "Build usage samples" ON)
OPTION(IMU_CAM_CALIB_ENABLE_TESTS "Build unit tests for the library" OFF)

OPTION(ENABLE_IMU "Compile code to calibrate IMU. If false, only camera calibration code will be compiled" OFF)
IF(ENABLE_IMU)
	ADD_DEFINITIONS(-DWITH_IMU_CALIBRATION=1)
ELSE()
	ADD_DEFINITIONS(-DWITH_IMU_CALIBRATION=0)
ENDIF()

OPTION(
	ENABLE_KALIBR_FILTERING
	"During intrinsics computation, enable intrinsics refinement by filtering detection and using incremental optimization on filtered data"
	ON)
IF(ENABLE_KALIBR_FILTERING)
	ADD_DEFINITIONS(-DWITH_KALIBR_FILTERING=1)
ELSE()
	ADD_DEFINITIONS(-DWITH_KALIBR_FILTERING=0)
ENDIF()

# Note: keep this find package before flags visibility presets to avoid linking error in dv runtime for kalibr .so files
FIND_PACKAGE(dv 1.5.2 REQUIRED)

# Define installation paths
INCLUDE(GNUInstallDirs)

SET(CMAKE_C_VISIBILITY_PRESET default)
SET(CMAKE_CXX_VISIBILITY_PRESET default)

# Set full RPATH, modules are libraries for DV
SET(CMAKE_INSTALL_RPATH ${USER_LOCAL_PREFIX}/${DV_MODULES_DIR}/imu-calibration)

IF(Git_FOUND AND EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/.git")
	# Update submodules as needed
	OPTION(GIT_SUBMODULE "Check-out submodules during build" ON)

	IF(GIT_SUBMODULE)
		MESSAGE(STATUS "Git submodule update")

		EXECUTE_PROCESS(
			COMMAND ${GIT_EXECUTABLE} submodule update --init --recursive
			WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
			RESULT_VARIABLE GIT_SUBMOD_RESULT)
		IF(NOT
		   GIT_SUBMOD_RESULT
		   EQUAL
		   "0")
			MESSAGE(
				WARNING
					"${GIT_EXECUTABLE} submodule update --init --recursive failed with ${GIT_SUBMOD_RESULT}, please check-out submodules manually."
			)
		ENDIF()
	ENDIF()
ENDIF()

ADD_SUBDIRECTORY(external/cli11 EXCLUDE_FROM_ALL)

# Basic setup

ADD_SUBDIRECTORY(src)
ADD_SUBDIRECTORY(thirdparty)

IF(IMU_CAM_CALIB_ENABLE_TESTS)
	MESSAGE(STATUS "Imu camera calibration tests enabled")
	FIND_PACKAGE(GTest REQUIRED)
	ENABLE_TESTING()
	ADD_SUBDIRECTORY(tests)
ENDIF()

# Enable samples
IF(IMU_CAM_CALIB_ENABLE_SAMPLES)
	MESSAGE(STATUS "Imu camera calibration samples enabled")
	ADD_SUBDIRECTORY(samples)
ENDIF()
