#pragma once

#include <aslam/cameras.hpp>
#include <aslam/cameras/GridCalibrationTargetAprilgrid.hpp>
#include <aslam/cameras/GridCalibrationTargetCheckerboard.hpp>
#include <aslam/cameras/GridCalibrationTargetCirclegrid.hpp>
#include <aslam/cameras/GridDetector.hpp>

#include <sm/boost/JobQueue.hpp>

#include "camera_calibration/kalibr_camera_calibrator.hpp"
#include "imu_camera_calibration/kalibr_iccCalibrator.hpp"
#include "imu_camera_calibration/kalibr_iccCamera.hpp"
#include "imu_camera_calibration/kalibr_iccImu.hpp"

#include <dv-processing/exception/exception.hpp>
#include <dv-processing/kinematics/transformation.hpp>

#include <opencv2/opencv.hpp>

#include <Eigen/Eigen>
#include <atomic>
#include <iostream>
#include <mutex>
#include <string>
#include <tbb/parallel_for_each.h>

/**
 * IMU camera calibration.
 */
class CalibratorBase {
public:
	virtual ~CalibratorBase() = default;

	std::vector<CameraCalibrationInfo> mCameraCalibrationInfo;

	/**
	 * Add IMU measurement to the calibration buffer.
	 */
	virtual void addImu(const int64_t timestamp, const Eigen::Vector3d &gyro, const Eigen::Vector3d &acc) = 0;

	/**
	 * Add a stamped image to the calibration buffer.
	 *
	 * @param CalibratorUtils::StampedImage
	 */
	virtual void addImages(const std::vector<CalibratorUtils::StampedImage> &stampedImages) = 0;

	/**
	 * Return the last images and grid observations
	 * @return a pair containing a vector of CalibratorUtils::StampedImage and a vector of pointers to
	 * aslam::cameras::GridCalibrationTargetObservation
	 */
	virtual std::pair<std::vector<CalibratorUtils::StampedImage>,
		std::vector<boost::shared_ptr<aslam::cameras::GridCalibrationTargetObservation>>>
		getLatestObservations() = 0;

	/**
	 * @return preview image visualizing the current status of the calibration
	 */
	virtual std::vector<CalibratorUtils::StampedImage> getPreviewImages() = 0;

	/**
	 * Calibrate the camera intrinsics (monocular).
	 */
	virtual std::optional<std::vector<CameraCalibrationUtils::CalibrationResult>> calibrateCameraIntrinsics() = 0;

	/**
	 * Build optimization problem. Needs to be called before calibrate().
	 */
	virtual void buildProblem() = 0;

	/**
	 * Begin calibration procedure on the collected data.
	 *
	 * @return result of the calibration
	 */
	[[nodiscard]] virtual IccCalibratorUtils::CalibrationResult calibrate() = 0;

	/**
	 * Start collecting data from the camera and IMU.
	 */
	virtual void startCollecting() = 0;

	/**
	 * Stop collecting data from the camera and IMU.
	 */
	virtual void stopCollecting() = 0;

	/**
	 * Discard the collected data and reset the calibrator.
	 */
	virtual void reset() = 0;

	/**
	 * Get a string containing DV log information before optimization.
	 *
	 * @param ss string stream into which log will be output
	 */
	virtual void getDvInfoBeforeOptimization(std::ostream &ss) = 0;

	/**
	 * Get a string containing DV log information after optimization.
	 *
	 * @param ss string stream into which log will be output
	 */
	virtual void getDvInfoAfterOptimization(std::ostream &ss) = 0;

	virtual std::ostream &print(std::ostream &os) = 0;

	std::vector<CameraCalibrationInfo> getCalibrationInfo() {
		return mCameraCalibrationInfo;
	}

protected:
	/**
	 * Detect the calibration pattern on the given stamped image.
	 *
	 * @param stampedImage
	 */
	virtual void detectPattern(const std::vector<CalibratorUtils::StampedImage> &frames) = 0;
};
