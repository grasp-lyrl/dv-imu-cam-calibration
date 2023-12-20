ADD_EXECUTABLE(test_calibrator test_all_calibration.cpp)

TARGET_LINK_LIBRARIES(
	test_calibrator
	kalibr_imu_camera_calibration
	gtest
	pthread)

# define tests
ADD_TEST(NAME test_calibrator COMMAND $<TARGET_FILE:test_calibrator>)

INSTALL(TARGETS test_calibrator DESTINATION ${DV_MODULES_DIR}/imu-calibration)
