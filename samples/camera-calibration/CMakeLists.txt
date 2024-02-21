CMAKE_MINIMUM_REQUIRED(VERSION 3.22)

PROJECT(
	camera-calibration
	VERSION 0.0.1
	DESCRIPTION "Camera-calibration using kalibr"
	LANGUAGES C CXX)

SET(CMAKE_CXX_STANDARD 20)

# We call FIND_PACKAGE only when this sample is used as a stand-alone project outside of this repository. FIND_PACKAGE
# will fail if this directory is added as a subdirectory of main project, so adding this "if-guard" makes it possible to
# compile it stand-alone and as part of library.
IF(NOT TARGET kalibr_imu_camera_calibration)
	FIND_PACKAGE(kalibr_imu_camera_calibration REQUIRED)
ENDIF()

ADD_EXECUTABLE(run-camera-calibration run_camera_calibration.cpp)
TARGET_LINK_LIBRARIES(run-camera-calibration PRIVATE kalibr_imu_camera_calibration dv::processing CLI11::CLI11)

ADD_EXECUTABLE(run-camera-calibration-all-methods run_camera_calibration_all_methods.cpp)
TARGET_LINK_LIBRARIES(run-camera-calibration-all-methods PRIVATE kalibr_imu_camera_calibration dv::processing
																 CLI11::CLI11)

ADD_EXECUTABLE(run-camera-calibration-all-recordings run_camera_calibration_all_recordings.cpp)
TARGET_LINK_LIBRARIES(run-camera-calibration-all-recordings PRIVATE kalibr_imu_camera_calibration dv::processing
																	CLI11::CLI11)

ADD_EXECUTABLE(run-camera-calibration-all-patterns run_camera_calibration_all_patterns.cpp)
TARGET_LINK_LIBRARIES(run-camera-calibration-all-patterns PRIVATE kalibr_imu_camera_calibration dv::processing
																  CLI11::CLI11)

ADD_EXECUTABLE(run-camera-calibration-all-devices run_camera_calibration_all_devices.cpp)
TARGET_LINK_LIBRARIES(run-camera-calibration-all-devices PRIVATE kalibr_imu_camera_calibration dv::processing
																 CLI11::CLI11)
