ADD_LIBRARY(kalibr_imu_camera_calibration SHARED kalibr_common.cpp)
TARGET_LINK_LIBRARIES(
	kalibr_imu_camera_calibration
	aslam_backend
	aslam_cameras
	aslam_cameras_april
	aslam_cv_backend
	aslam_splines
	bsplines
	incremental_calibration
	sm_kinematics
	kalibr_errorterms
	aslam_cv_error_terms
	${DV_LIBRARIES})
TARGET_INCLUDE_DIRECTORIES(kalibr_imu_camera_calibration PUBLIC ../../include/dv-imu-cam-calibration)

INSTALL(TARGETS kalibr_imu_camera_calibration DESTINATION ${DV_MODULES_DIR}/imu-calibration)
