#pragma once

#include "test_utils.hpp"

#include <boost/filesystem.hpp>

#include <gtest/gtest.h>

/**
 * Images Calibration only
 */
TEST(MonoCameraCalibrationNoImuTest, RadTanDistortionModel) {
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

	// Check that the calibration converged (less than 1 pixel reprojection error)
	const auto errorInfo = res[0].err_info;
	EXPECT_LE(errorInfo.errorNormMean, 1.0);

	// Check that results are consistent
	EXPECT_NEAR(errorInfo.mean.x(), -1.22081e-06, 1e-3);
	EXPECT_NEAR(errorInfo.mean.y(), -2.66866e-08, 1e-3);
	EXPECT_NEAR(errorInfo.std.x(), 0.347521, 0.1);
	EXPECT_NEAR(errorInfo.std.y(), 0.323904, 0.1);

	const auto cameraCalib = res[0];
	const auto distortion  = cameraCalib.distortion;

	EXPECT_NEAR(distortion[0], -0.369442, 0.1);
	EXPECT_NEAR(distortion[1], 0.2267, 0.1);
	EXPECT_NEAR(distortion[2], 0.000758743, 0.1);
	EXPECT_NEAR(distortion[3], -0.00172444, 0.1);
}

/*
TEST(MonoCameraCalibrationNoImuTest, EquidistantDistortionModel) {
	////
	/// Prepare the calibrator
	////
	CalibratorUtils::Options options;
	options.cols                                           = 6;
	options.rows                                           = 6;
	options.spacingMeters                                  = 0.088;
	options.patternSpacing                                 = 0.3;
	options.pattern                                        = CalibratorUtils::PatternType::APRIL_GRID;
	options.cameraInitialSettings.emplace_back().imageSize = cv::Size(1280, 1024);
	options.maxIter                                        = 50;
	options.timeCalibration                                = false;

Calibrator<aslam::cameras::EquidistantDistortedPinholeCameraGeometry, aslam::cameras::EquidistantDistortion>
	calibrator(options);
calibrator.startCollecting();

////
/// Useful paths
////
fs::path thisPath(__FILE__);
fs::path imgsPath = thisPath.parent_path() / "test_files" / "img";

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

EXPECT_NEAR(distortion[0], NaN, 0.01);
EXPECT_NEAR(distortion[1], NaN, 0.01);
EXPECT_NEAR(distortion[2], NaN, 0.01);
EXPECT_NEAR(distortion[3], NaN, 0.01);
}
*/
