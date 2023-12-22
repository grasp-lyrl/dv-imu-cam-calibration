#pragma once

namespace IccCalibratorUtils {
struct ErrorInfo {
	double meanReprojectionError;
	double meanGyroscopeError;
	double meanAccelerometerError;

	ErrorInfo(const double repr, const double gyr, const double acc) :
		meanReprojectionError(repr),
		meanGyroscopeError(gyr),
		meanAccelerometerError(acc) {
	}
};

struct CalibrationResult {
	double t_cam_imu;
	Eigen::Matrix4d T_cam_imu;
	bool converged;
	ErrorInfo error_info;

	CalibrationResult(
		const double timeShift, const Eigen::Matrix4d &transformation, bool conv, const ErrorInfo &err_info) :
		t_cam_imu(timeShift),
		T_cam_imu(transformation),
		converged(conv),
		error_info(err_info) {
	}
};

void printResult(const CalibrationResult &result, std::ostream &ss) {
	ss << "Optimization converged:" << std::endl;
	ss << "  " << (result.converged ? "true" : "false") << std::endl;
	ss << "Transformation T_cam_imu:" << std::endl;
	ss << result.T_cam_imu << std::endl;
	ss << "Camera to imu time: [s] (t_imu = t_cam + shift):" << std::endl;
	ss << "  " << result.t_cam_imu << std::endl;
}

} // namespace IccCalibratorUtils
