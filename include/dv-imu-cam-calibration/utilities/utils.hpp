#pragma once

#include <dv-processing/camera/calibrations/camera_calibration.hpp>

#include <Eigen/Eigen>
#include <ostream>
#include <string>

struct PatternInfo {
	std::string mName;
	cv::Size mShape;
	float mTagSize;
	float mTagSpacing;

	PatternInfo(std::string name, cv::Size shape, float tagSize, float tagSpacing) :
		mName(name),
		mShape(shape),
		mTagSize(tagSize),
		mTagSpacing(tagSpacing) {
	}
};

namespace ImuCamModelTypes {
const std::string RADTAN      = "Pinhole-RadialTangential";
const std::string EQUIDISTANT = "Pinhole-Equidistant";
} // namespace ImuCamModelTypes

namespace QualityStrings {
const std::string EXCELLENT = "excellent";
const std::string GOOD      = "good";
const std::string POOR      = "poor";
const std::string BAD       = "bad";
} // namespace QualityStrings

namespace CameraCalibrationUtils {
struct ErrorInfo {
	Eigen::Vector2d mean;
	Eigen::Vector2d std;

	ErrorInfo(const Eigen::Vector2d &_mean, const Eigen::Vector2d &_std) : mean(_mean), std(_std) {
	}
};

struct CalibrationResult {
	const std::vector<double> projection;
	const std::vector<double> distortion;
	const ErrorInfo err_info;
	Eigen::Matrix4d baseline;

	CalibrationResult(const std::vector<double> &_projection, const std::vector<double> &_distortion,
		const ErrorInfo &_err_info, const Eigen::Matrix4d &_baseline) :
		projection(_projection),
		distortion(_distortion),
		err_info(_err_info),
		baseline(_baseline) {
	}
};

static void printResult(const CameraCalibrationUtils::CalibrationResult &result, std::ostream &ss) {
	ss << "Intrinsic calibration results:" << std::endl;
	ss << "\n  projection: \n\t\t";
	for (const auto val : result.projection) {
		ss << val << " ";
	}
	ss << std::endl;
	ss << "\n  distortion: \n\t\t";
	for (const auto val : result.distortion) {
		ss << val << " ";
	}
	ss << std::endl;

	ss << "\n  reprojection error: \n\t\t[" << result.err_info.mean.x() << ", " << result.err_info.mean.y() << "] +- ["
	   << result.err_info.std.x() << ", " << result.err_info.std.y() << "]" << std::endl;

	ss << "\n  baseline: \n" << result.baseline.block<1, 4>(0, 0) << std::endl;
	ss << result.baseline.block<1, 4>(1, 0) << std::endl;
	ss << result.baseline.block<1, 4>(2, 0) << std::endl;
	ss << result.baseline.block<1, 4>(3, 0) << std::endl;
}
} // namespace CameraCalibrationUtils

std::string getCalibrationQuality(float std) {
	std::string quality;
	if (std < 0.1) {
		quality = QualityStrings::EXCELLENT;
	}
	else if (std < 0.4) {
		quality = QualityStrings::GOOD;
	}
	else if (std < 0.5) {
		quality = QualityStrings::POOR;
	}
	else {
		quality = QualityStrings::BAD;
	}

	return quality;
}

dv::camera::calibrations::CameraCalibration::Metadata getCameraCalibrationMetadata(
	const CameraCalibrationUtils::CalibrationResult &intrinsicResult, const PatternInfo &pattern,
	std::string comment = "", int64_t timestamp = 0) {
	auto meanStd       = (intrinsicResult.err_info.std.x() + intrinsicResult.err_info.std.y()) / 2.0;
	const auto quality = getCalibrationQuality(meanStd);

	return {pattern.mShape, pattern.mShape, pattern.mName, pattern.mTagSize, pattern.mTagSpacing, meanStd,
		std::to_string(timestamp), quality, comment, std::nullopt};
}

dv::camera::calibrations::CameraCalibration getIntrinsicCalibrationData(
	const CameraCalibrationUtils::CalibrationResult &res, PatternInfo patternInfo, const std::string &position,
	const std::string &inputName, std::string comment = "", cv::Size resolution = cv::Size(640, 480),
	bool isMaster = true) {
	const Eigen::Matrix<float, 4, 4, Eigen::RowMajor> floatTransform = res.baseline.cast<float>().eval();

	// todo(giovanni) add equidistant model
	dv::camera::DistortionModel distortionModel = dv::camera::DistortionModel::RadTan;

	dv::camera::calibrations::CameraCalibration cal(inputName, position, isMaster, resolution,
		cv::Point2f(static_cast<float>(res.projection.at(2)), static_cast<float>(res.projection.at(3))),
		cv::Point2f(static_cast<float>(res.projection.at(0)), static_cast<float>(res.projection.at(1))),
		std::vector<float>(res.distortion.begin(), res.distortion.end()), distortionModel,
		std::vector<float>(floatTransform.data(), floatTransform.data() + 16),
		getCameraCalibrationMetadata(res, patternInfo, comment));

	return cal;
}
