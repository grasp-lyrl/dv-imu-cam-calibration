#pragma once

#include "test_utils.hpp"

#include <boost/filesystem.hpp>

#include <gtest/gtest.h>

/**
 * Stereo Imu-Camera Calibration
 */
TEST(StereoCalibrationTest, EquidistantDistortionModel) {
	////
	/// Prepare the calibrator
	////
	CalibratorUtils::Options options;
	options.cols                                           = 6;
	options.rows                                           = 6;
	options.spacingMeters                                  = 0.031;
	options.patternSpacing                                 = 0.29;
	options.pattern                                        = CalibratorUtils::PatternType::APRIL_GRID;
	options.cameraInitialSettings.emplace_back().imageSize = cv::Size(346, 260);
	options.cameraInitialSettings.emplace_back().imageSize = cv::Size(346, 260);
	options.maxIter                                        = 50;
	options.timeCalibration                                = false;
	options.imuParameters.updateRate                       = 200.0;

	Calibrator<aslam::cameras::EquidistantDistortedPinholeCameraGeometry, aslam::cameras::EquidistantDistortion>
		calibrator(options);

	calibrator.startCollecting();

	////
	/// Useful paths
	////

	fs::path thisPath(__FILE__);
	fs::path imgsLeftPath  = thisPath.parent_path() / "test_files" / "stereo" / "img_left";
	fs::path imgsRightPath = thisPath.parent_path() / "test_files" / "stereo" / "img_right";
	fs::path imusPath      = thisPath.parent_path() / "test_files" / "stereo" / "imu";

	////
	/// Add images
	////

	std::vector<fs::path> imgLeftPaths, imgRightPaths;
	for (fs::directory_iterator itr(imgsLeftPath); itr != fs::directory_iterator(); ++itr) {
		imgLeftPaths.push_back(itr->path());
	}
	std::sort(imgLeftPaths.begin(), imgLeftPaths.end());

	for (fs::directory_iterator itr(imgsRightPath); itr != fs::directory_iterator(); ++itr) {
		imgRightPaths.push_back(itr->path());
	}
	std::sort(imgRightPaths.begin(), imgRightPaths.end());

	std::cout << "Testing using " << imgLeftPaths.size() << " images" << std::endl;

	addImagesToCalibrator<aslam::cameras::EquidistantDistortedPinholeCameraGeometry,
		aslam::cameras::EquidistantDistortion>(calibrator, imgLeftPaths, imgRightPaths);

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
	CameraCalibrationUtils::printResult(res[1], std::cout);

	const auto cameraCalib0 = res[0];
	const auto distortion0  = cameraCalib0.distortion;

	EXPECT_NEAR(distortion0[0], -0.026196177439606966, 0.01);
	EXPECT_NEAR(distortion0[1], 0.011470443712235325, 0.01);
	EXPECT_NEAR(distortion0[2], -0.031772586000478914, 0.01);
	EXPECT_NEAR(distortion0[3], 0.021363785963385253, 0.001);

	const auto cameraCalib1 = res[1];
	const auto distortion1  = cameraCalib1.distortion;

	EXPECT_NEAR(distortion1[0], -0.014088983574702143, 0.001);
	EXPECT_NEAR(distortion1[1], -0.011596027262493244, 0.001);
	EXPECT_NEAR(distortion1[2], 0.0080855729069350436, 0.001);
	EXPECT_NEAR(distortion1[3], -0.0036355859914529395, 0.001);

	std::cout << "Baseline: " << cameraCalib0.baseline << std::endl;
	//    EXPECT_LT(cameraCalib0.baseline)

	// CAM-IMU calibration

	calibrator.buildProblem();

	calibrator.getDvInfoBeforeOptimization(std::cout);

	try {
		const auto result = calibrator.calibrate();
		calibrator.getDvInfoAfterOptimization(std::cout);

		IccCalibratorUtils::printResult(result, std::cout);
		EXPECT_TRUE(result.converged);
	}
	catch (std::exception &ex) {
		EXPECT_TRUE(false);
		std::cout << ex.what() << std::endl;
		std::cout << "Optimization failed. Please make sure that the pattern is detected on all frames in your "
					 "dataset and repeat the calibration"
				  << std::endl;
	}
}

TEST(StereoCalibrationTest, RadTanDistortionModel) {
	////
	/// Prepare the calibrator
	////
	CalibratorUtils::Options options;
	options.cols                                           = 6;
	options.rows                                           = 6;
	options.spacingMeters                                  = 0.031;
	options.patternSpacing                                 = 0.29;
	options.pattern                                        = CalibratorUtils::PatternType::APRIL_GRID;
	options.cameraInitialSettings.emplace_back().imageSize = cv::Size(346, 260);
	options.cameraInitialSettings.emplace_back().imageSize = cv::Size(346, 260);
	options.maxIter                                        = 50;
	options.timeCalibration                                = false;
	options.imuParameters.updateRate                       = 200.0;

	Calibrator<aslam::cameras::DistortedPinholeCameraGeometry, aslam::cameras::RadialTangentialDistortion> calibrator(
		options);

	calibrator.startCollecting();

	////
	/// Useful paths
	////

	fs::path thisPath(__FILE__);
	fs::path imgsLeftPath  = thisPath.parent_path() / "test_files" / "stereo" / "img_left";
	fs::path imgsRightPath = thisPath.parent_path() / "test_files" / "stereo" / "img_right";
	fs::path imusPath      = thisPath.parent_path() / "test_files" / "stereo" / "imu";

	////
	/// Add images
	////

	std::vector<fs::path> imgLeftPaths, imgRightPaths;
	for (fs::directory_iterator itr(imgsLeftPath); itr != fs::directory_iterator(); ++itr) {
		imgLeftPaths.push_back(itr->path());
	}
	std::sort(imgLeftPaths.begin(), imgLeftPaths.end());
	for (fs::directory_iterator itr(imgsRightPath); itr != fs::directory_iterator(); ++itr) {
		imgRightPaths.push_back(itr->path());
	}
	std::sort(imgRightPaths.begin(), imgRightPaths.end());

	std::cout << "Testing using " << imgLeftPaths.size() << " images" << std::endl;

	addImagesToCalibrator<aslam::cameras::DistortedPinholeCameraGeometry, aslam::cameras::RadialTangentialDistortion>(
		calibrator, imgLeftPaths, imgRightPaths);

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
	CameraCalibrationUtils::printResult(res[1], std::cout);

	const auto cameraCalib0 = res[0];
	const auto distortion0  = cameraCalib0.distortion;

	EXPECT_NEAR(distortion0[0], -0.319038, 0.001);
	EXPECT_NEAR(distortion0[1], 0.0971273, 0.001);
	EXPECT_NEAR(distortion0[2], 0.000252449, 0.001);
	EXPECT_NEAR(distortion0[3], 0.000304249, 0.001);

	const auto cameraCalib1 = res[1];
	const auto distortion1  = cameraCalib1.distortion;

	EXPECT_NEAR(distortion1[0], -0.319845, 0.001);
	EXPECT_NEAR(distortion1[1], 0.0958489, 0.001);
	EXPECT_NEAR(distortion1[2], 0.000538449, 0.001);
	EXPECT_NEAR(distortion1[3], 0.000623019, 0.001);

	std::cout << "Baseline: " << cameraCalib0.baseline << std::endl;
	// EXPECT_LT(cameraCalib0.baseline)

	// CAM-IMU calibration

	calibrator.buildProblem();

	calibrator.getDvInfoBeforeOptimization(std::cout);

	try {
		const auto result = calibrator.calibrate();
		calibrator.getDvInfoAfterOptimization(std::cout);

		IccCalibratorUtils::printResult(result, std::cout);
		EXPECT_TRUE(result.converged);
	}
	catch (std::exception &ex) {
		EXPECT_TRUE(false);
		std::cout << ex.what() << std::endl;
		std::cout << "Optimization failed. Please make sure that the pattern is detected on all frames in your "
					 "dataset and repeat the calibration"
				  << std::endl;
	}
}
