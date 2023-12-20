#pragma once

#include "test_utils.hpp"

#include <boost/filesystem.hpp>

#include <gtest/gtest.h>

/**
 * Imu-Camera Calibration, single camera
 */
TEST(ImuCamCalibrationTest, EquidistantDistortionModel) {
	////
	/// Prepare the calibrator
	////
	CalibratorUtils::Options options;
	options.cols                                           = 6;
	options.rows                                           = 6;
	options.spacingMeters                                  = 0.088;
	options.patternSpacing                                 = 0.3;
	options.pattern                                        = CalibratorUtils::PatternType::APRIL_GRID;
	options.cameraInitialSettings.emplace_back().imageSize = cv::Size(346, 260);
	options.maxIter                                        = 50;
	options.timeCalibration                                = false;

	Calibrator<aslam::cameras::EquidistantDistortedPinholeCameraGeometry, aslam::cameras::EquidistantDistortion>
		calibrator(options);
	calibrator.startCollecting();

	////
	/// Useful paths
	////

	fs::path thisPath(__FILE__);
	fs::path imgsPath = thisPath.parent_path() / "test_files" / "event_mono" / "img";
	fs::path imusPath = thisPath.parent_path() / "test_files" / "event_mono" / "imu";

	////
	/// Add images
	////

	std::vector<fs::path> imgPaths;
	for (fs::directory_iterator itr(imgsPath); itr != fs::directory_iterator(); ++itr) {
		imgPaths.push_back(itr->path());
	}
	std::sort(imgPaths.begin(), imgPaths.end());

	// We don't need many images for testing
	bool useAll = true;
	if (!useAll) {
		const size_t startIdx = 1000;
		const size_t nIdx     = 25;
		assert(imgPaths.size() > startIdx + nIdx);
		imgPaths = std::vector<fs::path>(imgPaths.begin() + startIdx, imgPaths.begin() + startIdx + nIdx);
	}

	std::cout << "Testing using " << imgPaths.size() << " images" << std::endl;

	addImagesToCalibrator<aslam::cameras::EquidistantDistortedPinholeCameraGeometry,
		aslam::cameras::EquidistantDistortion>(calibrator, imgPaths);

	////
	/// Add IMU measurements
	////

	std::vector<fs::path> imuPaths;
	for (fs::directory_iterator itr(imusPath); itr != fs::directory_iterator(); ++itr) {
		imuPaths.push_back(itr->path());
	}
	std::sort(imuPaths.begin(), imuPaths.end());

	std::cout << "Testing using " << imuPaths.size() << " IMU measurements" << std::endl;

	addImuToCalibrator<aslam::cameras::EquidistantDistortedPinholeCameraGeometry,
		aslam::cameras::EquidistantDistortion>(calibrator, imuPaths);

	////
	/// Try to calibrate - this is the actual test
	////

	calibrator.stopCollecting();

	calibrator.print(std::cout);

	auto intrResult = calibrator.calibrateCameraIntrinsics();

	EXPECT_TRUE(intrResult.has_value());
	if (!intrResult.has_value()) {
		return;
	}
	const auto res = intrResult.value();

	CameraCalibrationUtils::printResult(res[0], std::cout);

	const auto cameraCalib = res[0];
	const auto distortion  = cameraCalib.distortion;

	EXPECT_NEAR(distortion[0], -0.01857137341235418, 0.01);
	EXPECT_NEAR(distortion[1], 0.012092548557359837, 0.001);
	EXPECT_NEAR(distortion[2], 0.0025734782201408792, 0.001);
	EXPECT_NEAR(distortion[3], -0.0052093652806526778, 0.001);

	calibrator.buildProblem();

	// Print the info before optimization
	calibrator.getDvInfoBeforeOptimization(std::cout);

	// Run the optimization problem
	try {
		IccCalibratorUtils::CalibrationResult result = calibrator.calibrate();
		// Print the info after optimization
		calibrator.getDvInfoAfterOptimization(std::cout);

		// Print the result
		IccCalibratorUtils::printResult(result, std::cout);

		EXPECT_TRUE(result.converged);
	}
	catch (std::exception &ex) {
		std::cout << ex.what() << std::endl;
		std::cout << "Optimization failed. Please make sure that the pattern is detected on all frames in your "
					 "dataset and repeat the calibration"
				  << std::endl;
	}
}

TEST(ImuCamCalibrationTest, RadTanDistortionModel) {
	////
	/// Prepare the calibrator
	////
	CalibratorUtils::Options options;
	options.cols                                           = 6;
	options.rows                                           = 6;
	options.spacingMeters                                  = 0.088;
	options.patternSpacing                                 = 0.3;
	options.pattern                                        = CalibratorUtils::PatternType::APRIL_GRID;
	options.cameraInitialSettings.emplace_back().imageSize = cv::Size(346, 260);
	options.maxIter                                        = 50;
	options.timeCalibration                                = false;

	Calibrator<aslam::cameras::DistortedPinholeCameraGeometry, aslam::cameras::RadialTangentialDistortion> calibrator(
		options);
	calibrator.startCollecting();

	////
	/// Useful paths
	////

	fs::path thisPath(__FILE__);
	fs::path imgsPath = thisPath.parent_path() / "test_files" / "img";
	fs::path imusPath = thisPath.parent_path() / "test_files" / "imu";

	////
	/// Add images
	////

	std::vector<fs::path> imgPaths;
	for (fs::directory_iterator itr(imgsPath); itr != fs::directory_iterator(); ++itr) {
		imgPaths.push_back(itr->path());
	}
	std::sort(imgPaths.begin(), imgPaths.end());

	// We don't need many images for testing
	bool useAll = true;
	if (!useAll) {
		const size_t startIdx = 1000;
		const size_t nIdx     = 25;
		assert(imgPaths.size() > startIdx + nIdx);
		imgPaths = std::vector<fs::path>(imgPaths.begin() + startIdx, imgPaths.begin() + startIdx + nIdx);
	}

	std::cout << "Testing using " << imgPaths.size() << " images" << std::endl;

	addImagesToCalibrator<aslam::cameras::DistortedPinholeCameraGeometry, aslam::cameras::RadialTangentialDistortion>(
		calibrator, imgPaths);

	////
	/// Add IMU measurements
	////

	std::vector<fs::path> imuPaths;
	for (fs::directory_iterator itr(imusPath); itr != fs::directory_iterator(); ++itr) {
		imuPaths.push_back(itr->path());
	}
	std::sort(imuPaths.begin(), imuPaths.end());

	std::cout << "Testing using " << imuPaths.size() << " IMU measurements" << std::endl;

	addImuToCalibrator<aslam::cameras::DistortedPinholeCameraGeometry, aslam::cameras::RadialTangentialDistortion>(
		calibrator, imuPaths);

	////
	/// Try to calibrate - this is the actual test
	////

	calibrator.stopCollecting();

	calibrator.print(std::cout);

	auto intrResult = calibrator.calibrateCameraIntrinsics();

	EXPECT_TRUE(intrResult.has_value());
	if (!intrResult.has_value()) {
		return;
	}
	const auto res = intrResult.value();

	CameraCalibrationUtils::printResult(res[0], std::cout);

	const auto cameraCalib = res[0];
	const auto distortion  = cameraCalib.distortion;

	EXPECT_NEAR(distortion[0], -0.32509976545609448, 0.001);
	EXPECT_NEAR(distortion[1], 0.096499640038551257, 0.001);
	EXPECT_NEAR(distortion[2], 0.00041809700179432394, 0.001);
	EXPECT_NEAR(distortion[3], -0.0001080197069080138, 0.001);

	calibrator.buildProblem();

	// Print the info before optimization
	calibrator.getDvInfoBeforeOptimization(std::cout);

	// Run the optimization problem
	try {
		IccCalibratorUtils::CalibrationResult result = calibrator.calibrate();
		// Print the info after optimization
		calibrator.getDvInfoAfterOptimization(std::cout);

		// Print the result
		IccCalibratorUtils::printResult(result, std::cout);

		EXPECT_TRUE(result.converged);
	}
	catch (std::exception &ex) {
		std::cout << ex.what() << std::endl;
		std::cout << "Optimization failed. Please make sure that the pattern is detected on all frames in your "
					 "dataset and repeat the calibration"
				  << std::endl;
	}
}
