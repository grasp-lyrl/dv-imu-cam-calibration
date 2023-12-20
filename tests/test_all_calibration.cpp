#define ENABLE_STEREO_CALIBRATION_TESTS  false
#define ENABLE_IMU_CAM_CALIBRATION_TESTS false

#include "test_mono_cam_calibration.hpp"

#if ENABLE_IMU_CAM_CALIBRATION_TESTS
#	include "test_imu_cam_calibration.hpp"
#endif

#if ENABLE_STEREO_CALIBRATION_TESTS
#	include "test_stereo_cam_calibration.hpp"
#endif

int main(int argc, char **argv) {
	testing::InitGoogleTest(&argc, argv);
	return RUN_ALL_TESTS();
}
